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
#include "mqtt_client.h"
#include "mqtt_ctrl.h"
#include "utils.h"
#include "adc_op.h"
#include "pumpop.h"
#include "waterop.h"
#include "external_defs.h"

#if ACTIVE_CONTROLLER == WATER_CONTROLLER

extern int adc_raw[2][NR_SAMPLES], adc_mv[2][NR_SAMPLES];
extern esp_adc_cal_characteristics_t adc1_chars;
extern int sample_count;

static const char *TAG = "WATER OP";
static xQueueHandle gpio_evt_queue = NULL;
static TaskHandle_t water_task_handle; //, gpio_dv_cmd_handle;
static int watering_status;
static int pump_present, pstate, pstatus, ppressure, pcurrent, pminlim, pmaxlim;
static time_t last_pump_state, last_pump_mon;
static dvprogram_t dv_program;
dvconfig_t dvconfig[DVCOUNT];
int activeDV;

static int read_program(dvprogram_t *param_val);
static int write_program(dvprogram_t *param_val);
static int start_watering(int idx);
static int stop_watering(int idx, int reason);
static int write_status(int idx);

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
	if(dvnum >= 0 && dvnum < DVCOUNT)
		{
		gpio_set_level(dvconfig[dvnum].pin_op, 1);
		activeDV =  dvconfig[dvnum].dvno;
		dvconfig[dvnum].state = DVOPEN;
		}
	else
		ESP_LOGI(TAG, "DV %d out of range - valid is 0 or 1", dvnum);
	}
void close_dv(int dvnum)
	{
	if(dvnum >= 0 && dvnum < DVCOUNT)
		{
		gpio_set_level(dvconfig[dvnum].pin_op, 0);
		activeDV =  -1;
		dvconfig[dvnum].state = DVCLOSE;
		}
	else
		ESP_LOGI(TAG, "DV %d out of range - valid is 0 or 1", dvnum);
	}
void parse_devstr(int argc, char **argv)
	{
	//ESP_LOGI(TAG, "mqtt topic: %s", argv[0]);
	if(strstr(argv[0], DEVICE_TOPIC_R))
		{
		if(strstr(argv[1], WATER_PUMP_DESC)) // water pump is available
			{
			pump_present = 1;
			publish_topic(PUMP_CMD_TOPIC, "state", 0, 0);
			}
		}
	else if(strstr(argv[0], WATER_PUMP_DESC"/monitor"))
		{
		char *tok = strtok(argv[1], "\1");
		last_pump_mon = -1;
		pstate = pstatus = pcurrent = ppressure = -1;
		if(tok)
			{
			last_pump_mon = time(NULL);
			tok = strtok(NULL, "\1");
			if(tok)
				{
				tok = strtok(NULL, "\1");
				if(tok)
					{
					pstate = atoi(tok);
					tok = strtok(NULL, "\1");
					if(tok)
						{
						pstatus = atoi(tok);
						tok = strtok(NULL, "\1");
						if(tok)
							{
							pcurrent = atoi(tok);
							tok = strtok(NULL, "\1");
							if(tok)
								ppressure = atoi(tok);

			}	}	}	}	}
		//ESP_LOGI(TAG, "pump monitor: %ld / %d / %d / %d / %d", last_pump_mon, pstate, pstatus, pcurrent, ppressure);
		}
	else if(strstr(argv[0], WATER_PUMP_DESC"/state"))
		{
		char *tok = strtok(argv[1], "\1");
		last_pump_state = -1;
		pstate = pstatus= pcurrent = ppressure = pminlim = pmaxlim = -1;
		if(tok)
			{
			last_pump_state = time(NULL);
			tok = strtok(NULL, "\1");
			if(tok)
				{
				tok = strtok(NULL, "\1");
				if(tok)
					{
					pstate = atoi(tok);
					tok = strtok(NULL, "\1");
					if(tok)
						{
						pstatus = atoi(tok);
						tok = strtok(NULL, "\1");
						if(tok)
							{
							pcurrent = atoi(tok);
							tok = strtok(NULL, "\1");
							if(tok)
								{
								ppressure = atoi(tok);
								tok = strtok(NULL, "\1");
								if(tok)
									{
									tok = strtok(NULL, "\1");
									if(tok)
										{
										pminlim = atoi(tok);
										tok = strtok(NULL, "\1");
										if(tok)
											pmaxlim = atoi(tok);
			}	}	}	}	}	}	}	}
		ESP_LOGI(TAG, "pump state: %ld / %d / %d / %d / %d / %d / %d", last_pump_state, pstate, pstatus, pcurrent, ppressure, pminlim, pmaxlim);

		}
	}
int do_dvop(int argc, char **argv)
	{
	dvprogram_t pval;
	char mqttbuf[256];
	char buf[50], bufs[20];
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
    else if(strcmp(waterop_args.op->sval[0], "state") == 0)
    	{
    	ESP_LOGI(TAG, "Watering status: %-d / activeDV: %d", watering_status, activeDV);
    	read_program(&dv_program);
    	buf[0] = 0;
    	for(int i = 0; i < DVCOUNT; i++)
			{
			ESP_LOGI(TAG, "DV%d state: %d", dvconfig[i].dvno, dvconfig[i].state);
			sprintf(bufs, "%d\1%d\1", dvconfig[i].dvno, dvconfig[i].state);
			strcat(buf, bufs);
			}
    	if(watering_status == WATER_ON)
    		ESP_LOGI(TAG, "Watering ON on DV%d started @%02d:%02d", activeDV, dv_program.p[activeDV].starth, dv_program.p[activeDV].startm);
		for(int i = 0; i < DVCOUNT; i++)
			{
			if(dv_program.p[i].cs == NOT_STARTED)
				ESP_LOGI(TAG, "Next program to start for DV%d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
			}
		sprintf(mqttbuf, "%d\1%d\1", watering_status, activeDV);
		strcat(mqttbuf, buf);
		publish_state(mqttbuf, 0, 0);
    	}
    else if(strcmp(waterop_args.op->sval[0], "resetps") == 0)
    	{
    	read_program(&pval);
    	for(int i = 0; i < DVCOUNT; i++)
    		pval.p[i].cs = 0;
    	write_program(&pval);
    	read_program(&dv_program);
    	}
    else if(strcmp(waterop_args.op->sval[0], "program") == 0)
    	{
    	int starth, startm, stoph, stopm, i;
    	read_program(&pval);
    	if(waterop_args.dv->count == 0) // no params -> just show DVs program
    		{
    		mqttbuf[0] = 0;
    		for(int i = 0; i < DVCOUNT; i++)
    			{
    			sprintf(buf, "%d\1%d\1%d\1%d\1%d\1%d\1", pval.p[i].dv, pval.p[i].starth, pval.p[i].startm, pval.p[i].stoph, pval.p[i].stopm, pval.p[i].cs);\
    			strcat(mqttbuf, buf);
    			}
    		ESP_LOGI(TAG, "DV0 program:  %d %d %d %d %d %d", pval.p[0].dv, pval.p[0].starth, pval.p[0].startm, pval.p[0].stoph, pval.p[0].stopm, pval.p[0].cs);
    		ESP_LOGI(TAG, "DV1 program:  %d %d %d %d %d %d", pval.p[1].dv, pval.p[1].starth, pval.p[1].startm, pval.p[1].stoph, pval.p[1].stopm, pval.p[1].cs);
    		publish_state(mqttbuf, 0, 0);
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
						sscanf(waterop_args.start->sval[0], "%d:%d", &starth, &startm);
						sscanf(waterop_args.stop->sval[0], "%d:%d", &stoph, &stopm);
						if(starth >= 0 && starth <= 23 && stoph >= 0 && stoph <= 23 &&
								startm >= 0 && startm <= 59 && stopm >= 0 && stopm <= 59)
							{
							for(i = 0; i < DVCOUNT; i++)
								{
								if(pval.p[i].dv == waterop_args.dv->ival[0])
									{
									pval.p[i].starth = starth;
									pval.p[i].startm = startm;
									pval.p[i].stoph = stoph;
									pval.p[i].stopm = stopm;
									pval.p[i].cs = 0;
									pval.p[i].fault = 0;
									break;
									}
								}
							if(i == DVCOUNT)
								{
								for(i = 0; i < DVCOUNT; i++)
									{
									if(pval.p[i].dv == -1)
										break;
									}
								if(i < DVCOUNT)
									{
									pval.p[i].dv = waterop_args.dv->ival[0];
									pval.p[i].starth = starth;
									pval.p[i].startm = startm;
									pval.p[i].stoph = stoph;
									pval.p[i].stopm = stopm;
									pval.p[i].cs = 0;
									pval.p[i].fault = 0;
									}
								}
							if(write_program(&pval) == ESP_OK)
								read_program(&dv_program);
							}
						}
					}
    			}
    		}
    	}
    else
    	ESP_LOGI(TAG, "Invalid op: %s", waterop_args.op->sval[0]);
    return 0;
    }
void water_mon_task(void *pvParameters)
	{
	struct tm tminfo;
	time_t ltime;
	int i, timem, ret, reset_done = 0;
	int saved_status = -1, savedDV = -1;
	char mqttbuf[256], buf[50];
	while(1)
		{
		ltime = time(NULL);
		localtime_r(&ltime, &tminfo);
		for(i = 0; i < DVCOUNT; i++)
			{
			if(dv_program.p[i].dv >= 0 && dv_program.p[i].dv < DVCOUNT)
				{
				timem = tminfo.tm_hour * 60 + tminfo.tm_min;
				if(timem >= dv_program.p[i].starth * 60 + dv_program.p[i].startm &&
						timem < dv_program.p[i].stoph * 60 + dv_program.p[i].stopm &&
						dv_program.p[i].cs == NOT_STARTED)
					{
					ret = start_watering(i);
					if(ret != 0)
						{
						ESP_LOGI(TAG, "Watering program for DV%d could not be started", dv_program.p[i].dv);
						dv_program.p[i].cs = START_ERROR;
						dv_program.p[i].fault = ret;
						write_program(&dv_program);
						write_status(i);
						sprintf(mqttbuf, "err_start\1%d\1%d\1%d\1", dv_program.p[i].dv, dv_program.p[i].cs, dv_program.p[i].fault = ret);
						publish_monitor(mqttbuf, 0, 0);
						}
					}
				else if(timem >= dv_program.p[i].stoph * 60 + dv_program.p[i].stopm &&
						dv_program.p[i].cs == IN_PROGRESS)
					{
					ret = stop_watering(i, COMPLETED);
					if(ret != 0)
						{
						ESP_LOGI(TAG, "Watering program for DV%d could not be stopped", dv_program.p[i].dv);
						dv_program.p[i].cs = ABORTED;
						dv_program.p[i].fault = ret;
						write_program(&dv_program);
						write_status(i);
						sprintf(mqttbuf, "err_stop\1%d\1%d\1%d\1", dv_program.p[i].dv, dv_program.p[i].cs, dv_program.p[i].fault = ret);
						publish_monitor(mqttbuf, 0, 0);
						}
					}
				}
			}
		if(tminfo.tm_hour * 60 + tminfo.tm_min != RESET_PROGRAM_H * 60 + RESET_PROGRAM_M)
			reset_done = 0;
		if(tminfo.tm_hour == RESET_PROGRAM_H &&
				tminfo.tm_min == RESET_PROGRAM_M &&
				!reset_done)
			{
			read_program(&dv_program);
			for(i = 0; i < DVCOUNT; i++)
				dv_program.p[i].cs = NOT_STARTED;
			write_program(&dv_program);
			reset_done = 1;
			ESP_LOGI(TAG, "Watering program status reset");
			}
		if(watering_status == WATER_OFF)
			{
			if(saved_status != watering_status ||
					savedDV != activeDV)
				{
				ESP_LOGI(TAG, "Watering OFF");
				strcpy(mqttbuf, "woff\1");
				for(i = 0; i < DVCOUNT; i++)
					{
					if(dv_program.p[i].cs == NOT_STARTED)
						{
						ESP_LOGI(TAG, "Next program to start for DV%d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
						sprintf(buf, "np\1%d%d%d\1", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
						strcat(mqttbuf, buf);
						}
					}
				publish_monitor(mqttbuf, 0, 0);
				saved_status = watering_status;
				savedDV = activeDV;
				}
			vTaskDelay(5000 / portTICK_PERIOD_MS);
			}
		else
			{
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			if(saved_status != watering_status ||
					savedDV != activeDV)
				{
				ESP_LOGI(TAG, "Watering ON on DV%d started @%02d:%02d", activeDV, dv_program.p[activeDV].starth, dv_program.p[activeDV].startm);
				sprintf(mqttbuf, "won\1%d\1%d\1%d\1", activeDV, dv_program.p[activeDV].starth, dv_program.p[activeDV].startm);
				for(i = 0; i < DVCOUNT; i++)
					{
					if(dv_program.p[i].cs == NOT_STARTED)
						{
						ESP_LOGI(TAG, "Next program to start for DV%d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
						sprintf(buf, "np\1%d%d%d\1", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
						strcat(mqttbuf, buf);
						}
					}
				publish_monitor(mqttbuf, 0, 0);
				saved_status = watering_status;
				savedDV = activeDV;
				}
			}
		}
	}
	/*
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
*/
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
	dvconfig[0].dvno = DV0;
	dvconfig[0].state = DVCLOSE;
	dvconfig[0].pin_op = PINOP_DV1;
	dvconfig[1].dvno = DV1;
	dvconfig[1].state = DVCLOSE;
	dvconfig[1].pin_op = PINOP_DV2;
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
    pump_present = 0;
    publish_reqID();
    subscribe(WATER_PUMP_DESC"/monitor");
	subscribe(WATER_PUMP_DESC"/state");
    read_program(&dv_program);

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
    memset(param_val, 0, sizeof(dvprogram_t));
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
			int dv, starth, startm, stoph, stopm, cs, fault;
			for(int i = 0; i < DVCOUNT; i++)
				{
				if(fgets(buf, 64, f))
					{
					sscanf(buf, "%d %d %d %d %d %d %d", &dv, &starth, &startm, &stoph, &stopm, &cs, &fault);
					param_val->p[i].dv = dv;
					param_val->p[i].starth = starth;
					param_val->p[i].startm = startm;
					param_val->p[i].stoph = stoph;
					param_val->p[i].stopm = stopm;
					param_val->p[i].cs = cs;
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
    ret = ESP_OK;
	FILE *f = fopen(BASE_PATH"/"PROGRAM_FILE, "w");
	if (f == NULL)
		{
		ESP_LOGE(TAG, "Failed to create dv program file");
		ret = ESP_FAIL;
		}
	else
		{
		for(int i = 0; i < DVCOUNT; i++)
			{
			if(param_val->p[i].starth * 60 +  param_val->p[i].startm > param_val->p[i].stoph * 60 + param_val->p[i].stopm)
				{
				param_val->p[i].cs = INVALID;
				ESP_LOGI(TAG, "Start time is after stop time for DV%d program : INVALID", param_val->p[i].dv);
				}
			else
				{
				sprintf(buf, "%2d %2d %2d %2d %2d %2d %d\n", param_val->p[i].dv, param_val->p[i].starth, param_val->p[i].startm,
															 param_val->p[i].stoph, param_val->p[i].stopm, param_val->p[i].cs, param_val->p[i].fault);
				if(fputs(buf, f) == EOF)
					{
					ret = ESP_FAIL;
					break;
					}
				}
			}
		fclose(f);
		}
	return ret;
	}
static int write_status(int idx)
	{
	int ret;
	struct tm tminfo;
	char strtime[100], dvpbuf[50];
	time_t ltime;
	ltime = time(NULL);
	localtime_r(&ltime, &tminfo);
	strftime(strtime, sizeof(strtime), "%Y-%m-%dT%H:%M:%S", &tminfo);
	sprintf(dvpbuf, " %d %d %d\n", dv_program.p[idx].dv, dv_program.p[idx].cs, dv_program.p[idx].fault);
	strcat(strtime, dvpbuf);
	FILE *f = fopen(BASE_PATH"/"STATUS_FILE, "a");
	if (f == NULL)
		{
		ESP_LOGE(TAG, "Failed to open status file for append");
		ret = ESP_FAIL;
		}
	else
		{
		if(fputs(strtime, f) >= 0)
			ret = ESP_OK;
		else
			{
			ESP_LOGE(TAG, "Cannot update status file");
			ret = ESP_FAIL;
			}
		fclose(f);
		}
	return ret;
	}
static int start_watering(int idx)
	{
	int lc, retry_start = 0, ret = 0;
	time_t ltime;
	if(dv_program.p[idx].cs == NOT_STARTED) //open DV[idx]
		{
		while(retry_start < RETRY_OP_WATERING)
			{
			ESP_LOGI(TAG, "try to start watering on DV%d / try %d", dv_program.p[idx].dv, retry_start);
			publish_topic(PUMP_CMD_TOPIC, "online", 0, 0);
			vTaskDelay(1000 / portTICK_PERIOD_MS); //wait 1 sec to check pressure status
			ltime = last_pump_state;
			publish_topic(PUMP_CMD_TOPIC, "state", 0, 0);
			lc = 0;
			while(last_pump_state <= ltime) //wait 2 sec for pump to be online and pressure above pmaxlim
				{
				lc++;
				if(lc > 20)
					break;
				vTaskDelay(100 / portTICK_PERIOD_MS);
				}
			if(lc >= 20) // no response from pump
				{
				retry_start++;
				ret = NO_PUMP_RESPONSE;
				if(retry_start >= RETRY_OP_WATERING)
					break;
				continue;
				}
			if(pstatus == PUMP_ONLINE)// && ppressure > pmaxlim)
				{
				open_dv(dv_program.p[idx].dv);
				vTaskDelay(1000 / portTICK_PERIOD_MS); //wait 1 sec to check pressure status
				ltime = last_pump_state;
				publish_topic(PUMP_CMD_TOPIC, "state", 0, 0);
				lc = 0;
				while(last_pump_mon <= ltime) //wait 2 sec for pump monitor report
					{
					lc++;
					if(lc > 20)
						break;
					vTaskDelay(100 / portTICK_PERIOD_MS);
					}
				if(lc >= 20) // no response from pump
					{
					close_dv(dv_program.p[idx].dv);
					retry_start++;
					ret = NO_PUMP_RESPONSE;
					if(retry_start >= RETRY_OP_WATERING)
						break;
					continue;
					}
				if(ppressure < pmaxlim)
					{
					dv_program.p[idx].cs = IN_PROGRESS;
					watering_status = WATER_ON;
					ESP_LOGI(TAG, "Watering program for DV%d started rs:%d", dv_program.p[idx].dv, retry_start);
					activeDV = dv_program.p[idx].dv;
					ret = 0;
					break;
					}
				else
					{
					ESP_LOGI(TAG, "Error! DV%d - cannot open rs:%d", dv_program.p[idx].dv, retry_start);
					close_dv(dv_program.p[idx].dv);
					publish_topic(PUMP_CMD_TOPIC, "offline", 0, 0);
					ret = DV_ERROR;
					retry_start++;
					if(retry_start >= RETRY_OP_WATERING)
						break;
					continue;
					}
				}
			else
				{
				ESP_LOGI(TAG, "Pump not online or low pressure %d / %d rs:%d", pstatus, ppressure, retry_start);
				retry_start++;
				ret = PUMP_WRONG_STATE;
				if(retry_start >= RETRY_OP_WATERING)
					break;
				}
			}
		}
	if(ret > 0) //last try before exit
		{
		publish_topic(PUMP_CMD_TOPIC, "offline", 0, 0);
		close_dv(dv_program.p[idx].dv);
		}
	return ret;
	}

static int stop_watering(int idx, int reason)
	{
	int lc, retry_stop = 0, ret = 0;
	time_t ltime;
	if(dv_program.p[idx].cs == IN_PROGRESS)
		{
		while(retry_stop < RETRY_OP_WATERING)
			{
			ESP_LOGI(TAG, "Stop watering try %d", retry_stop);
			close_dv(dv_program.p[idx].dv);
			vTaskDelay(1000 / portTICK_PERIOD_MS); //wait 1 sec to check pressure status
			ltime = last_pump_state;
			publish_topic(PUMP_CMD_TOPIC, "state", 0, 0);
			lc = 0;
			while(last_pump_state <= ltime) //wait 2 sec for pump state report
				{
				lc++;
				if(lc > 20)
					break;
				vTaskDelay(100 / portTICK_PERIOD_MS);
				}
			if(lc >= 20) // no response from pump
				{
				ret = NO_PUMP_RESPONSE;
				retry_stop++;
				if(retry_stop >= RETRY_OP_WATERING)
					{
					ret = PUMP_NO_RESPONSE;
					break;
					}
				continue;
				}
			//if(ppressure >= pmaxlim) // dv closed successfully
			if(1 == 1)
				{
				publish_topic(PUMP_CMD_TOPIC, "offline", 0, 0);
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				ltime = last_pump_state;
				publish_topic(PUMP_CMD_TOPIC, "state", 0, 0);
				lc = 0;
				while(last_pump_state <= ltime) //wait 2 sec for pump state report
					{
					lc++;
					if(lc > 20)
						break;
					vTaskDelay(100 / portTICK_PERIOD_MS);
					}
				if(lc >= 20) // no response from pump
					{
					ret = NO_PUMP_RESPONSE;
					retry_stop++;
					if(retry_stop >= RETRY_OP_WATERING)
						break;
					continue;
					}
				if(pstatus == PUMP_OFFLINE)
					{
					dv_program.p[idx].fault = 0;
					dv_program.p[idx].cs = reason;
					watering_status = WATER_OFF;
					write_program(&dv_program);
					write_status(idx);
					ESP_LOGI(TAG, "Watering program for DV%d stopped", dv_program.p[idx].dv);
					activeDV = -1;
					break;
					}
				else
					{
					ret = PUMP_WRONG_STATE;
					retry_stop++;
					if(retry_stop >= RETRY_OP_WATERING)
						break;
					continue;
					}
				}
			else
				{
				ret = DV_CLOSE_FAIL;
				retry_stop++;
				if(retry_stop >= RETRY_OP_WATERING)
					break;
				}
			}
		}
	if(ret > 0) //last try before exit
		{
		publish_topic(PUMP_CMD_TOPIC, "offline", 0, 0);
		close_dv(dv_program.p[idx].dv);
		activeDV = -1;
		}
	return ret;
	}
#endif

