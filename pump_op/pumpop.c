/*
 * pumpop.c
 *
 *  Created on: Jan 28, 2023
 *      Author: viorel_serbu
 */


#include <stdio.h>
#include <string.h>
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "mqtt_client.h"
#include "common_defines.h"
#include "external_defs.h"
#include "hal/adc_types.h"
#include "driver/timer.h"
#include "mqtt_ctrl.h"
#include "esp_adc_cal.h"
#include "utils.h"
#include "adc_op.h"
#include "pumpop.h"

#if ACTIVE_CONTROLLER == PUMP_CONTROLLER
static SemaphoreHandle_t pumpop_mutex;

static int pump_state, pump_status, pump_pressure_kpa, pump_current, kpa0_offset, pump_min_lim, pump_max_lim, pump_current_limit, psensor_mv;
static TaskHandle_t pump_task_handle, gpio_pump_cmd_handle;

int pump_pres_stdev;
static time_t start_overp_time;
uint32_t overp_time_limit = 10;


static const char *TAG = "PUMP OP";
static xQueueHandle gpio_evt_queue = NULL;

static uint32_t testModeCurrent;
uint32_t loop;

typedef struct
		{
		uint32_t source;
		uint32_t val;
		}msg_t;

static void IRAM_ATTR gpio_isr_handler(void* arg)
	{
	msg_t msg;
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == PUMP_ONLINE_CMD)
    	{
		msg.source = 1;
		xQueueSendFromISR(gpio_evt_queue, &msg, NULL);
		}
	}
static void gpio_pump_cmd(void* arg)
	{
    msg_t msg;
    int i;
    while(1)
    	{
        if(xQueueReceive(gpio_evt_queue, &msg, portMAX_DELAY))
        	{
        	if(msg.source == 1)
        		{
				i = gpio_get_level(PUMP_ONLINE_CMD);
				if(i == 0)
					{
					timer_set_counter_value(CMD_TIMER_GROUP, CMD_TIMER_INDEX, 0);
					timer_set_alarm_value(CMD_TIMER_GROUP, CMD_TIMER_INDEX, PUSH_TIME_MS * TIMER_SCALE);
					//timer_enable_intr(CMD_TIMER_GROUP, CMD_TIMER_INDEX);
					timer_start(CMD_TIMER_GROUP, CMD_TIMER_INDEX);
					//ESP_LOGI(TAG, "GPIO CMD timer start");
					}
				else
					{
					timer_pause(CMD_TIMER_GROUP, CMD_TIMER_INDEX);
					//timer_disable_intr(CMD_TIMER_GROUP, CMD_TIMER_INDEX);
					//ESP_LOGI(TAG, "GPIO CMD timer stop");
					}
				}
			else if(msg.source == 2)
				{
				timer_pause(CMD_TIMER_GROUP, CMD_TIMER_INDEX);
				ESP_LOGI(TAG, "timer expired");
				if(pump_status == PUMP_ONLINE)
					pump_operational(PUMP_OFFLINE);
				else if(pump_status == PUMP_OFFLINE)
					pump_operational(PUMP_ONLINE);
				else
					ESP_LOGI(TAG, "Uknown pump status %d", pump_status);
				}
        	}
    	}
	}
static bool IRAM_ATTR cmd_timer_callback(void *args)
	{
	msg_t msg;
    BaseType_t high_task_awoken = pdFALSE;
    msg.source = 2;
	xQueueSendFromISR(gpio_evt_queue, &msg, NULL);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
	}

static void config_cmd_timer()
	{
	timer_config_t config = {	.divider = TIMER_DIVIDER,
	        					.counter_dir = TIMER_COUNT_UP,
								.counter_en = TIMER_PAUSE,
								.alarm_en = TIMER_ALARM_EN,
								.auto_reload = true,}; // default clock source is APB
	timer_init(CMD_TIMER_GROUP, CMD_TIMER_INDEX, &config);
	timer_set_counter_value(CMD_TIMER_GROUP, CMD_TIMER_INDEX, 0);
	timer_enable_intr(CMD_TIMER_GROUP, CMD_TIMER_INDEX);
	timer_isr_callback_add(CMD_TIMER_GROUP, CMD_TIMER_INDEX, cmd_timer_callback, NULL, 0);
	}
static void config_pump_gpio(void)
	{
	/*
	 * configure GPIO for pressure sensor: SENSOR_PIN IO3
	 * ADC1 - chn 2
	 */

	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PUMP_ONOFF_PIN | 1ULL << PUMP_ONOFF_LED | 1ULL << PUMP_ONLINE_LED | 1ULL << PUMP_FAULT_LED);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    /*
     * GPIO + relay (relay datasheet -> 40mA@5V)
     * GPIO_DRIVE_CAP_3 dI ~120 mA ->
     * GPIO_DRIVE_CAP_0 dI ~45mA
     */
    gpio_set_drive_capability(PUMP_ONOFF_PIN, GPIO_DRIVE_CAP_0);
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PUMP_ONLINE_CMD);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PUMP_ONLINE_CMD, gpio_isr_handler, (void*) PUMP_ONLINE_CMD);

    //gpio_set_level(PUMP_ONLINE_CMD, PIN_ON);
    gpio_set_level(PUMP_ONOFF_LED, PIN_OFF);
    gpio_set_level(PUMP_ONLINE_LED, PIN_OFF);
    gpio_set_level(PUMP_FAULT_LED, PIN_OFF);
    gpio_set_level(PUMP_ONOFF_PIN, PIN_OFF);

	}

int get_pump_state(void)
	{
	esp_err_t ret = ESP_FAIL;
	minmax_t min[10] = {0}, max[10] = {0};
    pump_limits_t plimits;
    psensor_offset_t offset;
    int ps;
    char msg[80];
    pump_status = kpa0_offset = pump_min_lim = pump_max_lim = pump_current_limit = -1;
    overp_time_limit = 10;
    if(rw_params(PARAM_READ, PARAM_V_OFFSET, &offset) == ESP_OK)
    	{
    	kpa0_offset = offset.v_offset;
    	}
   	if(rw_params(PARAM_READ, PARAM_LIMITS, &plimits) == ESP_OK)
    	{
    	pump_min_lim = plimits.min_val;
		pump_max_lim = plimits.max_val;
		pump_current_limit = plimits.faultc;
		pump_pres_stdev = plimits.stdev;
		overp_time_limit = plimits.overp_lim;
		}
	if(rw_params(PARAM_READ, PARAM_OPERATIONAL, &ps) == ESP_OK)
		pump_status = ps;
	if(!pump_task_handle)
		{
		if(get_pump_adc_values(min, max, &psensor_mv) ==ESP_OK)
			{
			pump_pressure_kpa = ((psensor_mv - kpa0_offset) * 400) / 1000;
			if(pump_pressure_kpa < 0)
				pump_pressure_kpa = 0;
			process_adc_current(min, max);
			}
		else
			{
			pump_pressure_kpa = -1;
			psensor_mv = -1;
			pump_current = -1;
			}
		}
	char opstate[20];
	if(pump_status == PUMP_ONLINE)
		{
		strcpy(opstate, "ONLINE");
		gpio_set_level(PUMP_ONLINE_LED, PIN_ON);
		}
	else if(pump_status == PUMP_OFFLINE)
		{
		strcpy(opstate, "OFFLINE");
		gpio_set_level(PUMP_ONLINE_LED, PIN_OFF);
		}
	else
		strcpy(opstate, "UNKNOWN");
	ESP_LOGI(TAG, "\n \
running state\t\t= %5d\n \
Operational state\t\t= %s\n \
pressure (kPa/mV)        = %5d/%5d\n \
stdev current			 = %5d\n \
stdev pressure			 = %5d\n \
min pressure (kPa)       = %5d\n \
max pressure (kPa)       = %5d\n \
fault current limit (mA) = %5d\n \
0 kPa offset (mV)   	 = %5d\n \
current (mA)             = %5d\n \
max STDEV				 = %5d\n \
timeout P max (sec)		 = %5d\n",
		pump_state, opstate, pump_pressure_kpa, psensor_mv, stdev_c, stdev_p, pump_min_lim, pump_max_lim, pump_current_limit, kpa0_offset, pump_current, pump_pres_stdev, overp_time_limit);
	sprintf(msg, "%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d",
		pump_state, pump_status, pump_current, pump_pressure_kpa, kpa0_offset, pump_min_lim, pump_max_lim, pump_current_limit, pump_pres_stdev, overp_time_limit, stdev_c, stdev_p);
	publish_state(msg, 0, 0);
    return ret;
    }

int start_pump(int from)
	{
	esp_err_t ret = ESP_OK;
	minmax_t min[10] = {0}, max[10] = {0};
    if(pump_status == PUMP_ONLINE && from == 0)
    	{
    	ESP_LOGI(TAG, "Pump ONLINE. Start/Stop controlled by monitor task");
    	ret = ESP_FAIL;
    	}
    else
    	{
		gpio_set_level(PUMP_ONOFF_PIN, PIN_ON);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		if(get_pump_adc_values(min, max, &psensor_mv) == ESP_OK)
			{
			process_adc_current(min, max);
			if(pump_current > 100 && pump_current <= pump_current_limit)
				{
				pump_pressure_kpa = ((psensor_mv - kpa0_offset) * 250) / 1000;
				gpio_set_level(PUMP_ONOFF_LED, PIN_ON);
				gpio_set_level(PUMP_FAULT_LED, PIN_OFF);
				pump_state = PUMP_ON;
				ESP_LOGI(TAG, "Pump ON   start %d", pump_pressure_kpa);
				}
			else if(pump_current > pump_current_limit)
				{
				ESP_LOGI(TAG, "Pump overcurrent %d / %d", pump_current, pump_current_limit);
				gpio_set_level(PUMP_ONOFF_PIN, PUMP_OFF);
				gpio_set_level(PUMP_ONOFF_LED, PIN_OFF);
				gpio_set_level(PUMP_FAULT_LED, PIN_ON);
				gpio_set_level(PUMP_ONLINE_LED, PIN_OFF);
				pump_state = PUMP_FAULT;
				pump_status = PUMP_OFFLINE;
				//if(pump_task_handle)
				//	vTaskDelete(pump_task_handle);
				//pump_task_handle = NULL;
				rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &pump_status);
				ret = ESP_OK;
				}
			else
				{
				ESP_LOGI(TAG, "Pump doesn't start");
				gpio_set_level(PUMP_ONOFF_PIN, PUMP_OFF);
				gpio_set_level(PUMP_ONOFF_LED, PIN_OFF);
				gpio_set_level(PUMP_FAULT_LED, PIN_OFF);
				pump_state = PUMP_FAULT;
				ret = ESP_OK;
				}
			//get_pump_state();
			}
		}
	return ret;
	}

int stop_pump(int from)
	{
	esp_err_t ret = ESP_OK;
	minmax_t min[10] = {0}, max[10] = {0};
	if(pump_status == PUMP_ONLINE && from == 0)
    	{
    	ESP_LOGI(TAG, "Pump ONLINE. Start/Stop controlled by monitor task");
    	ret = ESP_FAIL;
    	}
    else
    	{
		gpio_set_level(PUMP_ONOFF_PIN, PUMP_OFF);
		if(get_pump_adc_values(min, max, &psensor_mv) == ESP_OK)
			{
			pump_pressure_kpa = ((psensor_mv - kpa0_offset) * 250) / 1000;
			process_adc_current(min, max);
			if(pump_current < 100)
				{
				pump_state = PUMP_OFF;
				gpio_set_level(PUMP_ONOFF_LED, PIN_OFF);
				ESP_LOGI(TAG, "Pump OFF  stop %d", pump_pressure_kpa);
				}
			else
				{
				ESP_LOGI(TAG, "Pump doesn't stop");
				pump_state = PUMP_ON;
				gpio_set_level(PUMP_ONOFF_LED, PIN_ON);
				ret = ESP_FAIL;
				}
			}
		//get_pump_state();
		}
	return ret;
	}
int pump_operational(int po)
	{
	esp_err_t ret = ESP_OK;
	loop = 0;
	if(pump_status == po)
		ESP_LOGI(TAG, "Pump already in desired mode");
	else
		{
		if(xSemaphoreTake(pumpop_mutex, ( TickType_t ) 100 )) // 1 sec wait
    		{
			if(po == PUMP_OFFLINE)
				{
				pump_status = po;
				stop_pump(1);
				gpio_set_level(PUMP_ONLINE_LED, PIN_OFF);
				}
			else if(po == PUMP_ONLINE)
				{
				start_pump(1);
				if(pump_state == PUMP_ON)
					{
					pump_status = po;
					gpio_set_level(PUMP_ONLINE_LED, PIN_ON);
					}
				else
					{
					gpio_set_level(PUMP_ONLINE_LED, PIN_OFF);
					ret = ESP_FAIL;
					}
				}
			xSemaphoreGive(pumpop_mutex);
			}
		else
			ret = ESP_FAIL;
		if(ret == ESP_OK) // update status in pump_status.txt
			{
			ret = rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &pump_status);
			if(ret != ESP_OK)
				{
				ESP_LOGI(TAG, "Operational status could not be updated: current value: %d", pump_status);
				ret = ESP_FAIL;
				}
			}
		}
	return ret;
	}
int set_pump_0_offset()
	{
	esp_err_t ret = ESP_FAIL;
	minmax_t min[10] = {0}, max[10] = {0};
    int old_value;
    psensor_offset_t v_offset;

	//printf("\nSet 0 offset for pressure sensor");
	//printf("\nSensor should be already at 0 kPa pressure\n");
	ret = get_pump_adc_values(min, max, &psensor_mv);
    if(ret == ESP_OK)
    	{
    	ESP_LOGI(TAG, " %d /", psensor_mv);
    	v_offset.v_offset = 0;
		ret = rw_params(PARAM_READ, PARAM_V_OFFSET, &v_offset);
		if(ret == ESP_OK)
			{
			old_value = v_offset.v_offset;
			v_offset.v_offset = psensor_mv;
			ret = rw_params(PARAM_WRITE, PARAM_V_OFFSET, &v_offset);
			if(ret == ESP_OK)
				{
				v_offset.v_offset = 0;
				ret = rw_params(PARAM_READ, PARAM_V_OFFSET, &v_offset);
				if(ret != ESP_OK)
					{
					ESP_LOGI(TAG, "Error reading 0 offset value: %d / %s", ret, esp_err_to_name(ret));
					}
				else
					ESP_LOGI(TAG, "0 offset value updated; new value = %d / old value = %d", v_offset.v_offset, old_value);
				}
			else
				ESP_LOGI(TAG, "Error writing 0 offset value: %d / %s", ret, esp_err_to_name(ret));
			}
		else
			{
			ESP_LOGI(TAG, "Error getting current value");
			}
    	}
	return ret;
	}

int do_pumpop(int argc, char **argv)
	{
	uint32_t minp, maxp, fc, stdev, overpt;
	int nerrors = arg_parse(argc, argv, (void **)&pumpop_args);
    if (nerrors != 0)
    	{
        arg_print_errors(stderr, pumpop_args.end, argv[0]);
        return 1;
    	}
    if(strcmp(pumpop_args.op->sval[0], "start") == 0)
    	{
		start_pump(0);
    	}
    else if(strcmp(pumpop_args.op->sval[0], "stop") == 0)
    	{
		stop_pump(0);
    	}
    else if(strcmp(pumpop_args.op->sval[0], "state") == 0)
    	{
    	get_pump_state();
    	}
    else if(strcmp(pumpop_args.op->sval[0], "set0") == 0)
    	{
    	set_pump_0_offset();
    	}
    else if(strcmp(pumpop_args.op->sval[0], "set_limits") == 0)
    	{
    	minp = pump_min_lim;
    	maxp = pump_max_lim;
    	fc = pump_current_limit;
    	stdev = pump_pres_stdev;
    	overpt = overp_time_limit;
    	if(pumpop_args.minP->count)
    		{
    		minp = pumpop_args.minP->ival[0];
    		if(pumpop_args.maxP->count)
    			{
    			maxp = pumpop_args.maxP->ival[0];
    			if(pumpop_args.faultC->count)
    				{
    				fc = pumpop_args.faultC->ival[0];
    				if(pumpop_args.stdev->count)
    					{
    					stdev = pumpop_args.stdev->ival[0];
    					if(pumpop_args.overpt->count)
    						overpt = pumpop_args.overpt->ival[0];
    					}

    				}
    			}
    		}
    	if(minp > 0 && maxp > 0 && maxp > minp)
    		{
    		pump_limits_t plimits;
    		plimits.max_val = maxp;
    		plimits.min_val = minp;
    		plimits.faultc = fc;
    		plimits.stdev = stdev;
    		plimits.overp_lim = overpt;
    		rw_params(PARAM_WRITE, PARAM_LIMITS, &plimits);
    		get_pump_state();
    		}
    	else
    		printf("pumpop: [%s]: invalid parameters\n", pumpop_args.op->sval[0]);
    	}
    else if(strcmp(pumpop_args.op->sval[0], "offline") == 0)
    	{
    	return pump_operational(PUMP_OFFLINE);
    	}
     else if(strcmp(pumpop_args.op->sval[0], "online") == 0)
    	{
    	return pump_operational(PUMP_ONLINE);
    	}
    else if(strcmp(pumpop_args.op->sval[0], "test") == 0)
    	{
    	if(pumpop_args.minP->count)
    		testModeCurrent = pumpop_args.minP->ival[0];
    	}
    else
		{
		printf("pumpop: [%s] unknown option\n", pumpop_args.op->sval[0]);
		}
	return 0;
	}

void register_pumpop()
	{
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	config_pump_gpio();
	config_cmd_timer();

	adc_calibration_init();
	config_adc_timer();
	testModeCurrent = 0;

	pump_task_handle = NULL;
	pumpop_mutex = xSemaphoreCreateMutex();
	if(!pumpop_mutex)
		{
		ESP_LOGE(TAG, "cannot create pumpop_mutex");
		esp_restart();
		}


	pumpop_args.op = arg_str1(NULL, NULL, "<op>", "type of operation: start | stop | offline | online | set0 --> set 0 kPa sensor offset | set_limits --> set the interval pump is running ");
	pumpop_args.minP = arg_int0(NULL, NULL, "<min pressure (kPa)>", "at this pressure pump will start");
	pumpop_args.maxP = arg_int0(NULL, NULL, "<max pressure (kPa)>", "at this pressure pump will stop");
	pumpop_args.faultC = arg_int0(NULL, NULL, "<max current (mA))>", "at this current pump will stop");
	pumpop_args.stdev = arg_int0(NULL, NULL, "<#>", "max accepted STDEV");
	pumpop_args.overpt = arg_int0(NULL, NULL, "<#>", "timeout la presiune maxima");
	pumpop_args.end = arg_end(1);
    const esp_console_cmd_t pumpop_cmd =
    	{
        .command = "pump",
        .help = "pump start|stop|online|offline|set0|set_limits <min_pressure kPa> <max_pressure (kPa)> <fault_current (mA)",
        .hint = NULL,
        .func = &do_pumpop,
        .argtable = &pumpop_args
    	};
    ESP_ERROR_CHECK(esp_console_cmd_register(&pumpop_cmd));
    get_pump_state();

    xTaskCreate(gpio_pump_cmd, "gpio_pump_cmd", 8192, NULL, 5, &gpio_pump_cmd_handle);
	if(!gpio_pump_cmd_handle)
		{
		ESP_LOGE(TAG, "Unable to start gpio cmd pump task");
		esp_restart();
		}
	xTaskCreate(pump_mon_task, "pump task", 8192, NULL, 5, &pump_task_handle);
	if(!pump_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start pump monitor task");
		esp_restart();
		}
		/*
	if(pump_status == PUMP_ONLINE)
		{
		xTaskCreate(pump_mon_task, "pump task", 8192, NULL, 5, &pump_task_handle);
		if(!pump_task_handle)
			{
			ESP_LOGE(TAG, "Unable to start pump monitor task");
			esp_restart();
			}
		gpio_set_level(PUMP_ONLINE_LED, PIN_ON);
		}
		*/
	}

void pump_mon_task(void *pvParameters)
	{
	minmax_t min[10], max[10];
	int saved_pump_state = -1, saved_pump_status = -1, saved_pump_current = -1, saved_pump_pressure_kpa = -1;
	char msg[80];
	uint32_t pcount = 20;
	while(1)
		{
		memset(min, 0, sizeof(min));
		memset(max, 0, sizeof(max));
		if(get_pump_adc_values(min, max, &psensor_mv) == ESP_OK)
			{
			pump_pressure_kpa = ((psensor_mv - kpa0_offset) * 250) / 1000;
			if(pump_pressure_kpa < 0)
				pump_pressure_kpa = 0;
			process_adc_current(min, max);
			if(pump_current > pump_current_limit)
				{
				pump_status = PUMP_OFFLINE;
				stop_pump(1);
				pump_state = PUMP_FAULT;
				gpio_set_level(PUMP_FAULT_LED, PIN_ON);
				gpio_set_level(PUMP_ONLINE_LED, PIN_OFF);
				rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &pump_status);
				}
			else
				{
				gpio_set_level(PUMP_FAULT_LED, PIN_OFF);
				if(pump_current > 100)
					{
					gpio_set_level(PUMP_ONOFF_LED, PIN_ON);
					pump_state = PUMP_ON;
					}
				else
					{
					gpio_set_level(PUMP_ONOFF_LED, PIN_OFF);
					if(pump_state != PUMP_FAULT)
						pump_state = PUMP_OFF;
					}
				if(pump_status == PUMP_ONLINE)
					{
					if(xSemaphoreTake(pumpop_mutex, ( TickType_t ) 100 )) // 1 sec wait
						{
						if(pump_pressure_kpa > pump_max_lim)
							{
							if(start_overp_time == 0)
								start_overp_time = time(NULL);
							else
								{
								if(time(NULL) - start_overp_time >= overp_time_limit)
									{
									if(pump_state == PUMP_ON)
										{
										ESP_LOGI(TAG, "Pump OFF  mon %d", pump_pressure_kpa);
										stop_pump(1);
										}
									}
								}
							}
						else if((pump_pressure_kpa < pump_max_lim) && (pump_state == PUMP_OFF))
							{
							ESP_LOGI(TAG, "Pump ON  mon %d", pump_pressure_kpa);
							start_pump(1);
							start_overp_time = 0;
							}
						xSemaphoreGive(pumpop_mutex);
						}
					}
				}
			//ESP_LOGI(TAG, "Pump state running:%d, pressure:%d(kPa), current:%d(mA)", pump_state, pump_pressure_kpa, pump_current);
			if((loop % pcount) == 0)
				{
				if(pump_state != saved_pump_state ||
					pump_status != saved_pump_status ||
						pump_current != saved_pump_current ||
							pump_pressure_kpa != saved_pump_pressure_kpa)
					{
					sprintf(msg, "%d\1%d\1%d\1%d\1%d\1%d", pump_state, pump_status, pump_current, pump_pressure_kpa, stdev_c, stdev_p);
					publish_monitor(msg, 0, 0);
					ESP_LOGI(TAG, "Pump state running:%d, pressure:%d(kPa), current:%d(mA), stdev p:%d, loop:%d", pump_state, pump_pressure_kpa, pump_current, stdev_p, loop);
					saved_pump_state = pump_state;
					saved_pump_status = pump_status;
					saved_pump_current = pump_current;
					saved_pump_pressure_kpa = pump_pressure_kpa;
					}
				}
			}
		// run loop every 2 sec if !PUMP_ONLINE
		// elese loop immediately
		if(pump_status != PUMP_ONLINE)
			{
			pcount = 1;
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			}
		else
			{
			pcount = 10;
			vTaskDelay(20 / portTICK_PERIOD_MS);
			}
		loop++;
		}
	}
void process_adc_current(minmax_t *min, minmax_t *max)
	{
	int smin, smax, smed;
    int i, k;
    int dmin, dmax, med, delta;
	smin = smax = smed = 0;
	k = 0;
	if(testModeCurrent > 0)
		{
		pump_current = testModeCurrent;
		return;
		}
	for(i = 0; i < 10; i++)
		{
		if(min[i].index == 0 || max[i].index == 0)
			break;
		med = (max[i].mv + min[i].mv)/2;
		dmin = med - min[i].mv;
		dmax = max[i].mv - med;
		delta = max[i].mv - min[i].mv;
		/*
		 * check the period
		 * 50 Hz @2 kHz sampling rate it requires 20 samples between min and max
		 * accepted valid is 18 - 22
		 */
		if(abs(min[i].index - max[i].index) >= 18 && abs(min[i].index - max[i].index) <= 22)
			{
			if(delta >= 25 && dmin > 10 && dmax > 10) // valid measurement
				{
				smin += min[i].mv;
				smax += max[i].mv;
				smed += med;
				k++;
				}
			}
		}
	if(k)
		{
		smin /= k;
		smax /= k;
		smed /= k;
		if(smed < 2000 || smed > 3000)
			pump_current = 0;
		else
			{
			int cmin = (smed - smin) * 10000;
			int cmax = (smax - smed) * 10000;
			pump_current = (cmin + cmax) / 2 / 1414;
			//ESP_LOGI(TAG, "current measurements: %d   %d   %d", cmin, cmax, pump_current);
			}
		}
	else
		{
		pump_current = 0;
		}
	}

#endif
