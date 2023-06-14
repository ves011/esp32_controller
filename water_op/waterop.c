/*
 * water_op.c
 *
 *  Created on: Jun 8, 2023
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include <string.h>
#include "driver/pcnt.h"
#include "driver/timer.h"

#include "sdkconfig.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_adc_cal.h"

#include "esp_log.h"

#include "common_defines.h"
#include "mqtt_ctrl.h"
#include "utils.h"
#include "adc_op.h"
#include "waterop.h"

#if ACTIVE_CONTROLLER == WATER_CONTROLLER

extern int adc_raw[2][NR_SAMPLES], adc_mv[2][NR_SAMPLES];
extern esp_adc_cal_characteristics_t adc1_chars;
extern int sample_count;

static const char *TAG = "WATER OP";
static xQueueHandle gpio_evt_queue = NULL;
static TaskHandle_t water_task_handle; //, gpio_dv_cmd_handle;
static int watering_status;
dvconfig_t dvconfig[2];
int activeDV;

static void get_dv_state();
static int read_program(dvprogram_t *param_val);
static int write_program(dvprogram_t *param_val);

typedef struct
		{
		uint32_t source;
		uint32_t val;
		}msg_t;

static void IRAM_ATTR gpiodv_isr_handler(void* arg)
	{
	msg_t msg;
    uint32_t gpio_num = (uint32_t) arg;
    msg.source = gpio_num;
	xQueueSendFromISR(gpio_evt_queue, &msg, NULL);
	}

static void config_dv_gpio(void)
	{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = OUTPUTPINS_DV1 | OUTPUTPINS_DV2;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_set_drive_capability(PINOP_DV1, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(PINLED_DV1, GPIO_DRIVE_CAP_2);
    gpio_set_drive_capability(PINOP_DV2, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(PINLED_DV2, GPIO_DRIVE_CAP_2);
    gpio_config(&io_conf);
    gpio_set_level(PINOP_DV1, 0);
    gpio_set_level(PINLED_DV1, 0);
    gpio_set_level(PINOP_DV2, 0);
    gpio_set_level(PINLED_DV2, 0);

    io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PINCMD_DV1 | 1ULL << PINCMD_DV2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PINCMD_DV1, gpiodv_isr_handler, (void*) PINCMD_DV1);
    gpio_isr_handler_add(PINCMD_DV2, gpiodv_isr_handler, (void*) PINCMD_DV2);
	}
void open_dv(int dvnum)
	{
	int dv_current = 0, stdev = 0;
	if(dvnum == DV0)
		{
		gpio_set_level(PINOP_DV1, 1);
		activeDV = DV0;
		get_dv_current(&dv_current, &stdev);
		ESP_LOGI(TAG, "current = %d, stdev = %d", dv_current, stdev);
		}
	else if(dvnum == DV1)
		{
		gpio_set_level(PINOP_DV2, 1);
		activeDV = DV1;
		}
	else
		ESP_LOGI(TAG, "DV %d out of range - valid is 0 or 1", dvnum);
	}
void close_dv(int dvnum)
	{
	if(dvnum == DV0)
		{
		gpio_set_level(PINOP_DV1, 0);
		activeDV = -1;
		}
	else if(dvnum == DV1)
		{
		gpio_set_level(PINOP_DV2, 0);
		activeDV = -1;
		}
	else
		ESP_LOGI(TAG, "DV %d out of range - valid is 0 or 1", dvnum);
	}
void parse_devstr(int argc, char **argv)
	{
	ESP_LOGI(TAG, "mqtt topic: %s", argv[0]);
	if(strstr(argv[0], DEVICE_TOPIC_R))
		{
		if(strstr(argv[1], WATER_PUMP_DESC)) // water pump is available
			{
			subscribe(WATER_PUMP_DESC"/monitor");
			subscribe(WATER_PUMP_DESC"/state");
			}
		}
	else if(strstr(argv[0], WATER_PUMP_DESC"/monitor"))
		{
		}
	else if(strstr(argv[0], WATER_PUMP_DESC"/state"))
		{
		}
	}
int do_dvop(int argc, char **argv)
	{
	dvprogram_t pval;
	int dv_current0, stdev0, dv_current1, stdev1;
	int nerrors = arg_parse(argc, argv, (void **)&waterop_args);
    if (nerrors != 0)
    	{
        arg_print_errors(stderr, waterop_args.end, argv[0]);
        return 1;
    	}
    if(strcmp(waterop_args.op->sval[0], "open") == 0)
    	{
    	if(waterop_args.dv->count == 1)
			open_dv(waterop_args.dv->ival[0]);
		else
			ESP_LOGI(TAG, "open: no #dv provided");
    	}
    else if(strcmp(waterop_args.op->sval[0], "close") == 0)
    	{
    	if(waterop_args.dv->count == 1)
			close_dv(waterop_args.dv->ival[0]);
		else
			ESP_LOGI(TAG, "close: no #dv provided");
    	}
    else if(strcmp(waterop_args.op->sval[0], "cal") == 0)
    	get_dv_state();
    else if(strcmp(waterop_args.op->sval[0], "program") == 0)
    	{
    	if(waterop_args.dv->count == 0) // no params -> just show DVs program
    		{
    		read_program(&pval);
    		ESP_LOGI(TAG, "DV0 program:  %d %d %d %d %d %d", pval.p[0].dv, pval.p[0].starth, pval.p[0].startm, pval.p[0].stoph, pval.p[0].stopm, pval.p[0].cs);
    		ESP_LOGI(TAG, "DV1 program:  %d %d %d %d %d %d", pval.p[1].dv, pval.p[1].starth, pval.p[1].startm, pval.p[1].stoph, pval.p[1].stopm, pval.p[1].cs);
    		}
    	else
    		{
    		if(waterop_args.start->count == 0)
    			{
    			ESP_LOGI(TAG, "program: no start time provided");
    			}
    		else
    			{
    			if(waterop_args.stop->count == 0)
					{
					ESP_LOGI(TAG, "program: no stop time provided");
					}
				else
					{
					if(waterop_args.dv->ival[0] >=0 && waterop_args.dv->ival[0] < DVCOUNT)
						{
						pval.p[0].dv = waterop_args.dv->ival[0];
						pval.p[0].cs = 0;
						sscanf(waterop_args.start->sval[0], "%d:%d", &pval.p[0].starth, &pval.p[0].startm);
						sscanf(waterop_args.stop->sval[0], "%d:%d", &pval.p[0].stoph, &pval.p[0].stopm);
						if(pval.p[0].starth >= 0 && pval.p[0].starth <= 23 && pval.p[0].stoph >= 0 && pval.p[0].stoph <= 23 &&
								pval.p[0].startm >= 0 && pval.p[0].startm <= 59 && pval.p[0].stopm >= 0 && pval.p[0].stopm <= 59)
							{
							write_program(&pval);
							}
						}
					}
    			}
    		}
    	}
    else if(strcmp(waterop_args.op->sval[0], "state") == 0)
    	{
    	int saved_actdv = activeDV;
    	activeDV = 0;
    	dv_current0 = stdev0 = 0;
    	get_dv_current(&dv_current0, &stdev0);
    	activeDV = 1;
    	dv_current1 = stdev1 = 0;
    	get_dv_current(&dv_current1, &stdev1);
    	activeDV = saved_actdv;
    	ESP_LOGI(TAG, "active DV = %d / current0 = %d / stdev0 = %d / current1 = %d / stdev1 = %d", activeDV, dv_current0, stdev0, dv_current1, stdev1);
    	}
    else
    	ESP_LOGI(TAG, "Invalid op: %s", waterop_args.op->sval[0]);
    return 0;
    }
void water_mon_task(void *pvParameters)
	{
	while(1)
		{
		if(watering_status != WATER_ON)
			{
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			}
		else
			{
			vTaskDelay(100 / portTICK_PERIOD_MS);
			}
		}
	}
static void get_dv_state()
	{
	int saved_state, saved_actdv, dv_current, stdev;
	saved_actdv = activeDV;
	//test both dvs
	for(int i = 0; i < DVCOUNT; i++)
		{
		activeDV = i;
		saved_state = dvconfig[activeDV].state;
		// 1. get current for dv off
		gpio_set_level(dvconfig[activeDV].pin_op, WATER_OFF);
		dvconfig[activeDV].off_current = 0;
		get_dv_current(&dv_current, &stdev);
		if(dv_current == 0) // cannot take measurement with good stdev
			esp_restart();
		dvconfig[activeDV].off_current = dv_current;
		gpio_set_level(dvconfig[activeDV].pin_op, WATER_ON);
		get_dv_current(&dv_current, &stdev);
		ESP_LOGI(TAG, "\n--> %5d: %5d / %5d\n", activeDV, dvconfig[activeDV].off_current, dv_current);
		if(dv_current == 0) // cannot take measurement with good stdev
			esp_restart();
		if(dv_current <= 10 || dv_current > 100) //error when dv is on
			dvconfig[activeDV].status = DVSTATE_FAULT;
		else
			dvconfig[activeDV].status = DVSTATE_OK;
		gpio_set_level(dvconfig[activeDV].pin_op, saved_state);
		}
	activeDV = saved_actdv;
	}
void register_waterop()
	{
	water_task_handle = NULL;
	dvconfig[0].pin_cmd = PINCMD_DV1;
	dvconfig[0].pin_current = PINCURRENT_DV1;
	dvconfig[0].pin_led = PINLED_DV1;
	dvconfig[0].pin_op = PINOP_DV1;

	dvconfig[1].pin_cmd = PINCMD_DV2;
	dvconfig[1].pin_current = PINCURRENT_DV2;
	dvconfig[1].pin_led = PINLED_DV2;
	dvconfig[1].pin_op = PINOP_DV2;
	config_dv_gpio();
	adc_calibration_init();
	config_adc_timer();
   	activeDV = -1;
   	//get_dv_state();
	watering_status = WATER_OFF;
	waterop_args.op = arg_str1(NULL, NULL, "<op>", "type of operation");
	waterop_args.dv = arg_int0(NULL, NULL, "<#DV>", "DV number");
	waterop_args.start = arg_str0(NULL, NULL, "hh:mm", "start time");
	waterop_args.stop = arg_str0(NULL, NULL, "hh:mm", "end time");
    waterop_args.end = arg_end(1);
    const esp_console_cmd_t dvop_cmd =
    	{
        .command = "dv",
        .help = "dv # open | close",
        .hint = NULL,
        .func = &do_dvop,
        .argtable = &waterop_args
    	};
    ESP_ERROR_CHECK(esp_console_cmd_register(&dvop_cmd));
    publish_reqID();
    /*
     * not needed with existing DVs
     */
    /*
    xTaskCreate(gpio_dv_cmd, "gpio_pump_cmd", 8192, NULL, 5, &gpio_dv_cmd_handle);
	if(!gpio_dv_cmd_handle)
		{
		ESP_LOGE(TAG, "Unable to start gpio cmd dv task");
		esp_restart();
		}
	*/
	xTaskCreate(water_mon_task, "pump task", 8192, NULL, 5, &water_task_handle);
	if(!water_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start watering monitor task");
		esp_restart();
		}
	}

void get_dv_current(int *dv_current, int *stdev)
	{
	//minmax_t min[10] = {0}, max[10] = {0};
    uint32_t i, l;
    int *d_val;
    int sd = 0, ssd = 0;
    *dv_current = 0;
    for(l = 0; l < 20; l++)
    	{
		sample_count = 0;
		timer_set_counter_value(ADC_TIMER_GROUP, ADC_TIMER_INDEX, 0);
		timer_start(ADC_TIMER_GROUP, ADC_TIMER_INDEX);
		while(sample_count < NR_SAMPLES)
			vTaskDelay(pdMS_TO_TICKS(2));
		for(i = 0; i < NR_SAMPLES; i++)
			{
			adc_mv[0][i] = esp_adc_cal_raw_to_voltage(adc_raw[0][i], &adc1_chars);
			//ESP_LOGI(TAG, "%5d raw = %5d / mv = %5d", i, adc_raw[0][i], adc_mv[0][i]);
			}
		d_val = adc_mv[0];
		sd = 0, ssd = 0;
		for(i = 10; i < NR_SAMPLES; i++)
			sd += d_val[i];

		sd /= NR_SAMPLES - 10;
		for(i = 10; i < NR_SAMPLES; i++)
			ssd += (d_val[i] - sd) * (d_val[i] - sd);
		ssd /= NR_SAMPLES - 10;
		for(i = 1; i < ssd; i++)
			{
			if(i * i > ssd)
				break;
			}
		*stdev = i -1;
		ESP_LOGI(TAG, "%4d stdev: %4d\n", l, *stdev);
		if(*stdev < STDEV_MAX)
			break;
		}
	*dv_current = abs(2500 - sd) - dvconfig[activeDV].off_current;
    }

static int read_program(dvprogram_t *param_val)
	{
	char buf[64];
	esp_err_t ret;
    struct stat st;
    ret = ESP_FAIL;
    param_val->p[0].cs = param_val->p[1].cs = -1;
	param_val->p[0].dv = param_val->p[1].dv = -1;
	if (stat(BASE_PATH"/"PROGRAM_FILE, &st) != 0)
		{
		// file does no exists
		ret = ESP_FAIL;
		}
	else
		{
		FILE *f = fopen(BASE_PATH"/"PROGRAM_FILE, "r");
		if (f != NULL)
			{
			int dv, starth, startm, stoph, stopm, cs;
			if(fgets(buf, 64, f))
				{
				sscanf(buf, "%d %d %d %d %d %d", &dv, &starth, &startm, &stoph, &stopm, &cs);
				param_val->p[0].dv = dv;
				param_val->p[0].starth = starth;
				param_val->p[0].startm = startm;
				param_val->p[0].stoph = stoph;
				param_val->p[0].stopm = stopm;
				param_val->p[0].cs = cs;
				if(fgets(buf, 64, f))
					{
					sscanf(buf, "%d %d %d %d %d %d", &dv, &starth, &startm, &stoph, &stopm, &cs);
					param_val->p[1].dv = dv;
					param_val->p[1].starth = starth;
					param_val->p[1].startm = startm;
					param_val->p[1].stoph = stoph;
					param_val->p[1].stopm = stopm;
					param_val->p[1].cs = cs;
					}
				}
			fclose(f);
			ret = ESP_OK;
			}
		else
			{
			ESP_LOGE(TAG, "Failed to open program file for reading");
			return ret;
			}
		}
    return ret;
	}
static int write_program(dvprogram_t *param_val)
	{
	char buf[64];
	esp_err_t ret;
    ret = ESP_FAIL;
	dvprogram_t pval;
	read_program(&pval);
	if(pval.p[0].dv == -1 && pval.p[1].dv == -1) //no program file or program file is empty
		{
		pval.p[0].dv = ((dvprogram_t *)param_val)->p[0].dv;
		pval.p[0].starth = ((dvprogram_t *)param_val)->p[0].starth;
		pval.p[0].startm = ((dvprogram_t *)param_val)->p[0].startm;
		pval.p[0].stoph = ((dvprogram_t *)param_val)->p[0].stoph;
		pval.p[0].stopm = ((dvprogram_t *)param_val)->p[0].stopm;
		pval.p[0].cs = ((dvprogram_t *)param_val)->p[0].cs;
		}
	else
		{
		if(pval.p[0].dv == ((dvprogram_t *)param_val)->p[0].dv)
			{
			pval.p[0].starth = ((dvprogram_t *)param_val)->p[0].starth;
			pval.p[0].startm = ((dvprogram_t *)param_val)->p[0].startm;
			pval.p[0].stoph = ((dvprogram_t *)param_val)->p[0].stoph;
			pval.p[0].stopm = ((dvprogram_t *)param_val)->p[0].stopm;
			pval.p[0].cs = ((dvprogram_t *)param_val)->p[0].cs;
			}
		else if(pval.p[1].dv == ((dvprogram_t *)param_val)->p[0].dv)
			{
			pval.p[1].starth = ((dvprogram_t *)param_val)->p[0].starth;
			pval.p[1].startm = ((dvprogram_t *)param_val)->p[0].startm;
			pval.p[1].stoph = ((dvprogram_t *)param_val)->p[0].stoph;
			pval.p[1].stopm = ((dvprogram_t *)param_val)->p[0].stopm;
			pval.p[1].cs = ((dvprogram_t *)param_val)->p[0].cs;
			}
		else
			{
			if(pval.p[0].dv == -1)
				{
				pval.p[0].dv = ((dvprogram_t *)param_val)->p[0].dv;
				pval.p[0].starth = ((dvprogram_t *)param_val)->p[0].starth;
				pval.p[0].startm = ((dvprogram_t *)param_val)->p[0].startm;
				pval.p[0].stoph = ((dvprogram_t *)param_val)->p[0].stoph;
				pval.p[0].stopm = ((dvprogram_t *)param_val)->p[0].stopm;
				pval.p[0].cs = ((dvprogram_t *)param_val)->p[0].cs;
				}
			else if(pval.p[1].dv == -1)
				{
				pval.p[1].dv = ((dvprogram_t *)param_val)->p[0].dv;
				pval.p[1].starth = ((dvprogram_t *)param_val)->p[0].starth;
				pval.p[1].startm = ((dvprogram_t *)param_val)->p[0].startm;
				pval.p[1].stoph = ((dvprogram_t *)param_val)->p[0].stoph;
				pval.p[1].stopm = ((dvprogram_t *)param_val)->p[0].stopm;
				pval.p[1].cs = ((dvprogram_t *)param_val)->p[0].cs;
				}
			}
		}
	FILE *f = fopen(BASE_PATH"/"PROGRAM_FILE, "w");
	if (f == NULL)
		{
		ESP_LOGE(TAG, "Failed to create dv program file");
		}
	else
		{
		if(pval.p[0].starth * 60 +  pval.p[0].startm > pval.p[0].stoph * 60 + pval.p[0].stopm)
			pval.p[0].cs = INVALID;
		else
			pval.p[0].cs = NOT_STARTED;
		sprintf(buf, "%2d %2d %2d %2d %2d %2d\n", pval.p[0].dv, pval.p[0].starth, pval.p[0].startm, pval.p[0].stoph, pval.p[0].stopm, pval.p[0].cs);
		if(fputs(buf, f) >= 0)
			{
			if(pval.p[1].starth * 60 +  pval.p[1].startm > pval.p[1].stoph * 60 + pval.p[1].stopm)
				pval.p[1].cs = INVALID;
			else
				pval.p[1].cs = NOT_STARTED;
			sprintf(buf, "%2d %2d %2d %2d %2d %2d\n", pval.p[1].dv, pval.p[1].starth, pval.p[1].startm, pval.p[1].stoph, pval.p[1].stopm, pval.p[1].cs);
			if(fputs(buf, f) >= 0)
				ret = ESP_OK;
			}
		fclose(f);
		}
	return ret;
	}
#endif
