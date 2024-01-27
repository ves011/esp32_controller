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
#include "sys/stat.h"

#include "common_defines.h"
#include "mqtt_client.h"
#include "mqtt_ctrl.h"
#include "utils.h"
#include "adc_op.h"
#include "pumpop.h"
#include "waterop.h"
#include "external_defs.h"

#if ACTIVE_CONTROLLER == WATER_CONTROLLER

static struct {
    struct arg_str *op;
    struct arg_int *dv;
    struct arg_str *start;
    struct arg_str *stop;
    struct arg_end *end;
} waterop_args;

static const char *TAG = "WATER OP";
static TaskHandle_t water_task_handle;
static int watering_status;
static int pump_present, pstate, pstatus, ppressure, pcurrent, pminlim, pmaxlim;
static volatile uint64_t last_pump_state, last_pump_mon;
static dvprogram_t dv_program;
dvconfig_t dvconfig[DVCOUNT];
int activeDV;

static int read_program_status();
static int read_program(dvprogram_t *param_val);
static int write_program(dvprogram_t *param_val);
static int start_watering(int idx);
static int stop_watering(int idx, int reason);
static int write_status(int idx);
static int get_act_state(int dvnum);
/*
typedef struct
		{
		uint32_t source;
		uint32_t val;
		}msg_t;
*/

static int open_dv(int dvnum)
	{
	int dv_current, op_start = 0;
	int coff = 0, i;
	gpio_set_level(PINMOT_A1, PIN_ON);
	gpio_set_level(PINMOT_B1, PIN_OFF);
	if(dvnum == 0)
		gpio_set_level(PINEN_DV0, PIN_ON);
	else if(dvnum == 1)
		gpio_set_level(PINEN_DV1, PIN_ON);
	for(i = 0; i < 30; i++)
		{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		get_dv_adc_values(&dv_current);
		//ESP_LOGI(TAG, "sense voltage: %d (%04x)", dv_current, dv_current);
		if(dv_current < CURRENT_OFF_LIM)
			coff++;
		else
			{
			coff = 0;
			op_start = 1;
			}
		if(coff >= CURRENT_OFF_COUNT)
			break;
		}
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_OFF);
	gpio_set_level(PINEN_DV0, PIN_OFF);
	gpio_set_level(PINEN_DV1, PIN_OFF);
	if(op_start == 0)
		{
		ESP_LOGI(TAG, "DV%d already in DVOPEN state", dvnum);
		activeDV =  dvconfig[dvnum].dvno;
		dvconfig[dvnum].state = DVOPEN;
		return ESP_OK;
		}
	if(i >= 30) // operation aborted
		{
		ESP_LOGI(TAG, "open DV%d failed %d %d", dvnum, i, dv_current);
		return OP_ABORTED;
		}
	else
		{
		ESP_LOGI(TAG, "open DV%d OK %d %d", dvnum, i, dv_current);
		activeDV =  dvconfig[dvnum].dvno;
		dvconfig[dvnum].state = DVOPEN;
		return ESP_OK;
		}
	return ESP_OK;
	}
static int close_dv(int dvnum)
	{
	int dv_current, op_start = 0;
	int coff = 0, i;
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_ON);
	if(dvnum == 0)
		gpio_set_level(PINEN_DV0, PIN_ON);
	else if(dvnum == 1)
		gpio_set_level(PINEN_DV1, PIN_ON);
	for(i = 0; i < 30; i++)
		{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		get_dv_adc_values(&dv_current);
		//ESP_LOGI(TAG, "sense voltage: %d (%04x)", dv_current, dv_current);
		if(dv_current < CURRENT_OFF_LIM)
			coff++;
		else
			{
			op_start = 1;
			coff = 0;
			}

		if(coff >= CURRENT_OFF_COUNT)
			break;
		}
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_OFF);
	gpio_set_level(PINEN_DV0, PIN_OFF);
	gpio_set_level(PINEN_DV1, PIN_OFF);
	if(op_start == 0)
		{
		ESP_LOGI(TAG, "DV%d already in closed state", dvnum);
		activeDV =  -1;
		dvconfig[dvnum].state = DVCLOSE;
		return ESP_OK;
		}
	if(i >= 30) // operation aborted
		{
		ESP_LOGI(TAG, "close DV%d failed %d %d", dvnum, i, dv_current);
		return OP_ABORTED;
		}
	else
		{
		ESP_LOGI(TAG, "close DV%d OK %d %d", dvnum, i, dv_current);
		activeDV =  -1;
		dvconfig[dvnum].state = DVCLOSE;
		return ESP_OK;
		}
	}
static void config_dv_gpio(void)
	{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PINEN_DV0) | (1ULL << PINEN_DV1) | (1ULL << PINMOT_A1) | (1ULL << PINMOT_B1) | (1ULL << PINMOT_A2) | (1ULL << PINMOT_B2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(PINEN_DV0, PIN_OFF);
    gpio_set_level(PINEN_DV1, PIN_OFF);
    gpio_set_level(PINMOT_A1, PIN_OFF);
    gpio_set_level(PINMOT_B1, PIN_OFF);
	}

void parse_devstr(int argc, char **argv)
	{
	char mqttbuf[20];
	//ESP_LOGI(TAG, "mqtt topic: %s", argv[0]);
	if(strstr(argv[0], DEVICE_TOPIC_R))
		{
		if(strstr(argv[1], WATER_PUMP_DESC)) // water pump is available
			{
			pump_present = 1;
			publish_topic(PUMP_CMD_TOPIC, "state", 1, 0);
			}
		}
	else if(strstr(argv[0], WATER_PUMP_DESC"/monitor"))
		{
		char *tok = strtok(argv[1], "\1");
		last_pump_mon = 0;
		pstate = pstatus = pcurrent = ppressure = -1;
		if(tok)
			{
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
								last_pump_mon = esp_timer_get_time();
								sprintf(mqttbuf, "%s\1%d\1%d\1%d\1", PUMP_STATE, pstatus, pstate, ppressure);
								publish_monitor(mqttbuf, 1, 0);
								}

			}	}	}	}	}
		//ESP_LOGI(TAG, "pump monitor: %llu / %d / %d / %d / %d", last_pump_mon, pstate, pstatus, pcurrent, ppressure);
		}
	else if(strstr(argv[0], WATER_PUMP_DESC"/state"))
		{
		char *tok = strtok(argv[1], "\1");
		ESP_LOGI(TAG, "dev str: %s / %s", argv[0], argv[1]);
		pstate = pstatus= pcurrent = ppressure = pminlim = pmaxlim = -1;
		last_pump_state = 0;
		if(tok)
			{
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
											{
											pmaxlim = atoi(tok);
											last_pump_state = esp_timer_get_time();
											}
			}	}	}	}	}	}	}	}
		ESP_LOGI(TAG, "pump state: %llu / %d / %d / %d / %d / %d / %d", last_pump_state, pstate, pstatus, pcurrent, ppressure, pminlim, pmaxlim);

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
    	int st;
    	ESP_LOGI(TAG, "Watering status: %-d / activeDV: %d", watering_status, activeDV);
    	read_program(&dv_program);
    	buf[0] = 0;
    	for(int i = 0; i < DVCOUNT; i++)
			{
			st = get_act_state(dvconfig[i].dvno);
			ESP_LOGI(TAG, "DV%d state: %d / %d", dvconfig[i].dvno, dvconfig[i].state, st);
			sprintf(bufs, "%d\1%d\1", dvconfig[i].dvno, dvconfig[i].state);
			strcat(buf, bufs);
			}
    	if(watering_status == WATER_ON)
    		{
    		ESP_LOGI(TAG, "Watering ON on DV%d started @%02d:%02d", activeDV, dv_program.p[activeDV].starth, dv_program.p[activeDV].startm);
    		}
		/*for(int i = 0; i < DVCOUNT; i++)
			{
			if(dv_program.p[i].cs == NOT_STARTED)
				ESP_LOGI(TAG, "Next program to start for DV%d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
			}*/
		sprintf(mqttbuf, "%s\1%d\1%d\1", STATE_W, watering_status, activeDV);
		strcat(mqttbuf, buf);
		for(int i = 0; i < DVCOUNT; i++)
		    {
		    sprintf(buf, "%d\1%d\1%d\1%d\1%d\1%d\1%d\1",
		    		dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm, dv_program.p[i].stoph,
					dv_program.p[i].stopm, dv_program.p[i].cs, dv_program.p[i].fault);
		    strcat(mqttbuf, buf);
		    ESP_LOGI(TAG, "DV%d program:  %d %d %d %d %d %d %d", i, dv_program.p[i].dv, dv_program.p[i].starth,
		    			dv_program.p[i].startm, dv_program.p[i].stoph, dv_program.p[i].stopm,
						dv_program.p[i].cs, dv_program.p[i].fault);
		    }
		sprintf(buf, "%d\1%d\1%d\1", pstatus, pstate, ppressure);
		strcat(mqttbuf, buf);
		publish_state(mqttbuf, 1, 0);
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
    	read_program(&dv_program);
    	memcpy(&pval, &dv_program, sizeof(dvprogram_t));
    	sprintf(mqttbuf, "%s\1", STATE_P);
    	if(waterop_args.dv->count == 0) // no params -> just show DVs program
    		{
    		for(int i = 0; i < DVCOUNT; i++)
    			{
    			sprintf(buf, "%d\1%d\1%d\1%d\1%d\1%d\1", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm, dv_program.p[i].stoph, dv_program.p[i].stopm, dv_program.p[i].cs);
    			strcat(mqttbuf, buf);
    			ESP_LOGI(TAG, "DV%d program:  %d %d %d %d %d %d %d", i, dv_program.p[i].dv, dv_program.p[i].starth,
    					    			dv_program.p[i].startm, dv_program.p[i].stoph, dv_program.p[i].stopm,
    									dv_program.p[i].cs, dv_program.p[i].fault);
    			}
    		publish_state(mqttbuf, 1, 0);
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
    else if(strcmp(waterop_args.op->sval[0], "readps") == 0)
    	read_program_status();
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
						sprintf(mqttbuf, "%s\1%d\1%d\1%d\1", ERR_START, dv_program.p[i].dv, dv_program.p[i].cs, dv_program.p[i].fault);
						publish_monitor(mqttbuf, 1, 0);
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
						sprintf(mqttbuf, "%s\1%d\1%d\1%d\1", ERR_STOP, dv_program.p[i].dv, dv_program.p[i].cs, dv_program.p[i].fault);
						publish_monitor(mqttbuf, 1, 0);
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
				sprintf(mqttbuf, "%s\1woff\1", WATERING_STATE);
				for(i = 0; i < DVCOUNT; i++)
					{
					if(dv_program.p[i].cs == NOT_STARTED)
						{
						ESP_LOGI(TAG, "Next program to start for DV%d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
						sprintf(buf, "np\1%d%d%d\1", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
						strcat(mqttbuf, buf);
						}
					}
				publish_monitor(mqttbuf, 1, 0);
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
				sprintf(mqttbuf, "%s\1won\1%d\1%d\1%d\1", WATERING_STATE, activeDV, dv_program.p[activeDV].starth, dv_program.p[activeDV].startm);
				for(i = 0; i < DVCOUNT; i++)
					{
					if(dv_program.p[i].cs == NOT_STARTED)
						{
						ESP_LOGI(TAG, "Next program to start for DV%d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
						sprintf(buf, "np\1%d%d%d\1", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
						strcat(mqttbuf, buf);
						}
					}
				publish_monitor(mqttbuf, 1, 0);
				saved_status = watering_status;
				savedDV = activeDV;
				}
			}
		}
	}

static int get_act_state(int dvnum)
	{
	char op[20];
	int i, dv_current, c_med, ret_state;
	//try close
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_ON);
	strcpy(op, "close");

	if(dvnum == 0)
		gpio_set_level(PINEN_DV0, PIN_ON);
	else if(dvnum == 1)
		gpio_set_level(PINEN_DV1, PIN_ON);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	c_med = 0;
	for(i = 0; i < 3; i++)
		{
		get_dv_adc_values(&dv_current);
		ESP_LOGI(TAG, "DV%d, %s %d", dvnum, op, dv_current);
		c_med += dv_current;
		vTaskDelay(100 / portTICK_PERIOD_MS);
		}
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_OFF);
	gpio_set_level(PINEN_DV0, PIN_OFF);
	gpio_set_level(PINEN_DV1, PIN_OFF);
	c_med /= 3;
	if(c_med < CURRENT_OFF_LIM) // DV already closed
		ret_state = DVCLOSE;
	else // DV OPEN
		{
		ret_state = DVOPEN;
		//revert op
		open_dv(dvnum);
		}
	return ret_state;
	}

void register_waterop()
	{
	water_task_handle = NULL;
	dvconfig[0].pin_current = PINSENSE_MOT;
	dvconfig[1].pin_current = PINSENSE_MOT;

	config_dv_gpio();
	adc_calibration_init();
	config_adc_timer();
	dvconfig[0].dvno = DV0;
	dvconfig[0].state = DVCLOSE;
	dvconfig[1].dvno = DV1;
	dvconfig[1].state = DVCLOSE;

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
	xTaskCreate(water_mon_task, "pump task", 8192, NULL, 5, &water_task_handle);
	if(!water_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start watering monitor task");
		esp_restart();
		}
	}
static int read_program_status()
	{
	char bufr[64];
	char mqttbuf[128];
	esp_err_t ret;
	struct stat st;
	ret = ESP_FAIL;
	if (stat(BASE_PATH"/"STATUS_FILE, &st) == 0)
		{
		FILE *f = fopen(BASE_PATH"/"STATUS_FILE, "r");
		if (f != NULL)
			{
			int i = 0;
			sprintf(mqttbuf, "%s\1start\1", PROG_HISTORY);
			publish_state(mqttbuf, 1, 0);
			while(!feof(f))
				{
				fgets(bufr, 64, f);
				if(feof(f))
					break;
				sprintf(mqttbuf, "%s\1%s\1", PROG_HISTORY, bufr);
				publish_state(mqttbuf, 1, 0);
				i++;
				}
			sprintf(mqttbuf, "%s\1end\1%d\1", PROG_HISTORY, i);
			publish_state(mqttbuf, 1, 0);
			ret = ESP_OK;
			fclose(f);
			}
		else
			{
			sprintf(mqttbuf, "%s\1err_open\1", PROG_HISTORY);
			publish_state(mqttbuf, 1, 0);
			}
		}
	else
		{
		sprintf(mqttbuf, "%s\1not_found\1", PROG_HISTORY);
		publish_state(mqttbuf, 1, 0);
		}
	return ret;
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
					param_val->p[i].fault = fault;
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
	int lc, retry_start = 0, ret = START_WATERING_ERROR;
	int dvop;
	uint64_t ltime;

	close_dv(0);
	close_dv(1);
	//ensure both DVs are in DVCLOSED state
	/*
	if(get_act_state(0) == DVOPEN)
		{
		ESP_LOGI(TAG, "DV0 in DVOPEN state -> closing");
		if(close_dv(0) != ESP_OK)
			return ret;
		}
	if(get_act_state(1) == DVOPEN)
		{
		ESP_LOGI(TAG, "DV1 in DVOPEN state -> closing");
		if(close_dv(1)!= ESP_OK)
			return ret;
		}
	*/
	//set pump online
	retry_start = 0;
	while (retry_start < RETRY_OP_WATERING)
		{
		ESP_LOGI(TAG, "Start watering on DV%d - set pump online / try %d /last_pump_state: %llu", dv_program.p[idx].dv, retry_start, last_pump_state);
		publish_topic(PUMP_CMD_TOPIC, "online", 1, 0);
		lc = 0;
		ltime = esp_timer_get_time();
		while(last_pump_state <= ltime) //wait 10 sec for pump to be online and pressure above pmaxlim
			{
			vTaskDelay(1000 / portTICK_PERIOD_MS); //wait 1 sec to check pressure status
			publish_topic(PUMP_CMD_TOPIC, "state", 1, 0);
			lc++;
			if(lc > 10)
				break;
			}
		if(lc > 10)
			{
			ESP_LOGI(TAG, "No response from pump #1");
			}
		else
			{
			if(pstatus == PUMP_ONLINE && ppressure > pmaxlim)
				break;
			else
				ESP_LOGI(TAG, "Pump not online or low pressure");
			}
		retry_start++;
		}
	if(retry_start < RETRY_OP_WATERING) // pump condition OK
		{
		dvop = open_dv(dv_program.p[idx].dv);
		if(dvop == ESP_OK)
			{
			ESP_LOGI(TAG, "open DV%d OK", dv_program.p[idx].dv);
			//now wait for pressure to fall between max and min limit
			ltime = esp_timer_get_time();
			lc = 0;
			do  //wait 10 sec for pump monitor report
				{
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				lc++;
				if(lc > 10)
					break;
				ESP_LOGI(TAG, "ltime %llu, last_pump_mon %llu", ltime, last_pump_mon);
				}
			while(last_pump_mon <= ltime);
			if(lc <= 10)
				{
				if(ppressure > pmaxlim)
					{
					ESP_LOGI(TAG, "Error! DV%d - pump pressure too high: %d", dv_program.p[idx].dv, ppressure);
					}
				else if(ppressure < MINPRES)
					{
					ESP_LOGI(TAG, "Error! pump pressure too low: %d", ppressure);
					}
				else
					{
					dv_program.p[idx].cs = IN_PROGRESS;
					watering_status = WATER_ON;
					ESP_LOGI(TAG, "Watering program for DV%d started rs:%d", dv_program.p[idx].dv, retry_start);
					activeDV = dv_program.p[idx].dv;
					ret = 0;
					}
				}
			else
				{
				ESP_LOGI(TAG, "No response from pump #2");
				}
			}
		else
			{
			ESP_LOGI(TAG, "Error open! DV%d: %d", dv_program.p[idx].dv, dvop);
			//try to close back the DV
			dvop = close_dv(dv_program.p[idx].dv);
			ESP_LOGI(TAG, "Closing DV%d because error on open / close err: %d", dv_program.p[idx].dv, dvop);
			}

		}
	if(ret > 0) //last try before exit
		{
		publish_topic(PUMP_CMD_TOPIC, "offline", 1, 0);
		dvop = close_dv(dv_program.p[idx].dv);
		}
	return ret;
	}

static int stop_watering(int idx, int reason)
	{
	int lc, retry_stop = 0, ret = STOP_WATERING_ERROR;
	uint64_t ltime = esp_timer_get_time();
	if(dv_program.p[idx].cs == IN_PROGRESS)
		{
		while(retry_stop < RETRY_OP_WATERING)
			{
			ESP_LOGI(TAG, "Stop watering try %d", retry_stop);
			close_dv(dv_program.p[idx].dv);
			vTaskDelay(1000 / portTICK_PERIOD_MS); //wait 1 sec to check pressure status
			lc = 0;
			ltime = esp_timer_get_time();
			while(last_pump_state <= ltime) //wait 5 sec for pump state report
				{
				publish_topic(PUMP_CMD_TOPIC, "state", 1, 0);
				lc++;
				if(lc > 10)
					break;
				vTaskDelay(500 / portTICK_PERIOD_MS);
				}
			if(lc >= 10) // no response from pump
				{
				ret = NO_PUMP_RESPONSE;
				retry_stop++;
				ESP_LOGI(TAG, "No pump response for DV off: %llu / %llu", ltime, last_pump_state);
				if(retry_stop >= RETRY_OP_WATERING)
					{
					ret = PUMP_NO_RESPONSE;
					break;
					}
				continue;
				}
			if(ppressure >= pminlim) // dv closed successfully
				{
				ESP_LOGI(TAG, "close1 ok %llu", last_pump_state);
				ltime = esp_timer_get_time();
				lc = 0;
				//while(last_pump_state <= ltime) //wait 2 sec for pump state report
				while(pstatus != PUMP_OFFLINE)
					{
					publish_topic(PUMP_CMD_TOPIC, "offline", 1, 0);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					publish_topic(PUMP_CMD_TOPIC, "state", 1, 0);
					lc++;
					if(lc > 10)
						break;
					vTaskDelay(500 / portTICK_PERIOD_MS);
					}
				if(lc >= 10) // no response from pump
					{
					ret = NO_PUMP_RESPONSE;
					ESP_LOGI(TAG, "No pump response for offline: %llu / %llu", ltime, last_pump_state);
					retry_stop++;
					if(retry_stop >= RETRY_OP_WATERING)
						break;
					continue;
					}
				if(pstatus == PUMP_OFFLINE)
					{
					ESP_LOGI(TAG, "close2 ok %llu", last_pump_state);
					dv_program.p[idx].fault = 0;
					dv_program.p[idx].cs = reason;
					watering_status = WATER_OFF;
					write_program(&dv_program);
					write_status(idx);
					ESP_LOGI(TAG, "Watering program for DV%d stopped", dv_program.p[idx].dv);
					activeDV = -1;
					ret = ESP_OK;
					break;
					}
				else
					{
					ret = PUMP_WRONG_STATE;
					ESP_LOGI(TAG, "pump wrong state: %llu / %llu", ltime, last_pump_state);
					retry_stop++;
					if(retry_stop >= RETRY_OP_WATERING)
						break;
					continue;
					}
				}
			else
				{
				ESP_LOGI(TAG, "DV%d not closed. Pump pressure still low", dv_program.p[idx].dv);
				ret = DV_CLOSE_FAIL;
				retry_stop++;
				if(retry_stop >= RETRY_OP_WATERING)
					break;
				}
			}
		}
	if(ret > 0) //last try before exit
		{
		publish_topic(PUMP_CMD_TOPIC, "offline", 1, 0);
		close_dv(dv_program.p[idx].dv);
		activeDV = -1;
		}
	return ret;
	}
#endif

