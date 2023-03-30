/*
 * gateop.c
 *
 *  Created on: Sep 28, 2022
 *      Author: viorel_serbu
 */

 /**
 * @file gateop.c
 * @brief implementation logic for slifing gate controller
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

#include "esp_log.h"

#include "common_defines.h"
#include "mqtt_ctrl.h"
#include "gateop.h"

//#define TIMER_BASE_CLK   (APB_CLK_FREQ)  /*!< Frequency of the clock on the input of the timer groups */


#if ACTIVE_CONTROLLER == AGATE_CONTROLLER
static const char *TAG = "\nGateOp";

static uint32_t signal_value;
static int16_t pulse_count_input;
static uint32_t cmd_state;
static uint32_t gate_state;
static uint32_t gate_moving_state;
static uint32_t op_response;
static uint32_t open_count, close_count;
static xQueueHandle gate_evt_queue;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

static bool IRAM_ATTR pw_timer_callback(void *args)
	{
    BaseType_t high_task_awoken = pdFALSE;
    gpio_set_level(GATE_PIN_OPEN, PIN_OFF);
    gpio_set_level(GATE_PIN_CLOSE, PIN_OFF);
    timer_pause(PW_TIMER_GROUP, PW_TIMER_INDEX);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
	}

static bool IRAM_ATTR steady_timer_callback(void *args)
	{
	gate_evt_t gevt;
    BaseType_t high_task_awoken = pdFALSE;
    BaseType_t xHigherPriorityTaskWoken;
    portENTER_CRITICAL_ISR(&timerMux);

	gevt.source = STEADY_TIMER_SOURCE;
	//pcnt_counter_pause(PCNT_UNIT_0);
	pcnt_get_counter_value(PCNT_UNIT_0, &pulse_count_input);
    pcnt_counter_clear(PCNT_UNIT_0);

    //pcnt_counter_resume(PCNT_UNIT_0);
    gevt.pulse_count = pulse_count_input;
    gevt.op_response = op_response;
    gevt.moving_state = gate_moving_state;
    gevt.time = esp_timer_get_time();
    //pulse_count_input = 0;
    portEXIT_CRITICAL_ISR(&timerMux);
	xQueueSendFromISR(gate_evt_queue, &gevt, &xHigherPriorityTaskWoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
	}

static void config_pw_timer()
	{
	timer_config_t config = {	.divider = TIMER_DIVIDER,
	        					.counter_dir = TIMER_COUNT_UP,
								.counter_en = TIMER_PAUSE,
								.alarm_en = TIMER_ALARM_EN,
								.auto_reload = true,}; // default clock source is APB
	timer_init(PW_TIMER_GROUP, PW_TIMER_INDEX, &config);
	timer_set_counter_value(PW_TIMER_GROUP, PW_TIMER_INDEX, 0);
	timer_enable_intr(PW_TIMER_GROUP, PW_TIMER_INDEX);
	timer_isr_callback_add(PW_TIMER_GROUP, PW_TIMER_INDEX, pw_timer_callback, NULL, 0);
	}

static void config_steady_wdog_timer()
	{
	timer_config_t config = {	.divider = TIMER_DIVIDER,
	        					.counter_dir = TIMER_COUNT_UP,
								.counter_en = TIMER_PAUSE,
								.alarm_en = TIMER_ALARM_EN,
								.auto_reload = true,}; // default clock source is APB
	timer_init(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX, &config);
	timer_set_counter_value(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX, 0);
	timer_enable_intr(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);
	timer_isr_callback_add(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX, steady_timer_callback, NULL, 0);
	}

static void config_gate_gpio(void)
	{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_set_drive_capability(GATE_PIN_OPEN, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(GATE_PIN_CLOSE, GPIO_DRIVE_CAP_3);
    gpio_config(&io_conf);
    gpio_set_level(GATE_PIN_OPEN, 0);
    gpio_set_level(GATE_PIN_CLOSE, 0);
    gpio_set_level(GATE_PIN_SIGNAL, GATECLOSED);
    signal_value = GATECLOSED;

    io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = GATE_PIN_SENSE,
        .ctrl_gpio_num = PCNT_PIN_CTRL,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_DIS,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_INC,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .hctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .lctrl_mode = PCNT_MODE_KEEP, //PCNT_MODE_DISABLE,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(PCNT_UNIT_0);
    /*
     * Filter value in APB clock units
     * 1 / 80000000 = 12.5 ns
     */
    pcnt_set_filter_value(PCNT_UNIT_0, 80);  // 1us glitch filter
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
	}

int open_gate()
	{
	if(cmd_state == CMD_CLOSE)	//close command is in progress -> no open command is accepted
		{
		op_response = OPEN_GATE_NACK;
		}
	else
		{
		op_response = OPEN_GATE_ACK;
		//pcnt_counter_pause(PCNT_UNIT_0);
		pcnt_counter_clear(PCNT_UNIT_0);
		//pcnt_counter_resume(PCNT_UNIT_0);

		timer_pause(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);
		timer_disable_intr(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);
		cmd_state = CMD_OPEN;
		open_count++;
		timer_set_counter_value(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX, 0);
		timer_enable_intr(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);
		timer_start(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);

		// pulse command
		gpio_set_level(GATE_PIN_OPEN, PIN_ON);
		timer_set_counter_value(PW_TIMER_GROUP, PW_TIMER_INDEX, 0);
		timer_set_alarm_value(PW_TIMER_GROUP, PW_TIMER_INDEX, CMD_PW * TIMER_SCALE);
		timer_start(PW_TIMER_GROUP, PW_TIMER_INDEX);
		ESP_LOGI(TAG, "open command %llu", esp_timer_get_time());
		}
	return op_response;
	}


int close_gate()
	{
	if(cmd_state == CMD_OPEN)	//open command is in progress -> no close command is accepted
		{
		op_response = CLOSE_GATE_NACK;
		}
	else
		{
		op_response = CLOSE_GATE_ACK;
		//if(cmd_state == CMD_COMPLETE)
			{
			//pcnt_counter_pause(PCNT_UNIT_0);
			pcnt_counter_clear(PCNT_UNIT_0);
			//pcnt_counter_resume(PCNT_UNIT_0);
			timer_pause(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);
			timer_disable_intr(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);
			cmd_state = CMD_CLOSE;
			close_count++;
			timer_set_counter_value(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX, 0);
			timer_enable_intr(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);
			timer_start(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);
			}
		gpio_set_level(GATE_PIN_CLOSE, PIN_ON);
		timer_set_counter_value(PW_TIMER_GROUP, PW_TIMER_INDEX, 0);
		timer_set_alarm_value(PW_TIMER_GROUP, PW_TIMER_INDEX, CMD_PW * TIMER_SCALE );
		timer_start(PW_TIMER_GROUP, PW_TIMER_INDEX);
		ESP_LOGI(TAG, "close command %llu", esp_timer_get_time());
		}
	return op_response;
	}

int get_gate_state()
	{
	char msg[100];
	ESP_LOGI(TAG, "\n gate_state = %d / gate_moving_state = %d / pulse_count = %d / cmd_state = %d / signal_value = %d\n",
		gate_state, gate_moving_state, pulse_count_input, cmd_state, signal_value);
	sprintf(msg, "%d\1%d\1%d\1%d\1%d",
		gate_state, gate_moving_state,  pulse_count_input, cmd_state, signal_value);
	publish_state(msg, 0, 0);
	return 1;
	}

void gate_task()
	{
	gate_evt_t gevt;
	char msg[100];
	int saved_gate_state = -1, saved_gate_moving_state = -1,  saved_cmd_state = -1;
	while(1)
		{
		xQueueReceive(gate_evt_queue, &gevt, portMAX_DELAY);

		if(gevt.source == STEADY_TIMER_SOURCE)
			{
			if(cmd_state == CMD_OPEN)
				{
				if(gevt.pulse_count > 1800) //that means steady
					{
					if(open_count == 1)
						gate_state = STATE_OPEN;
					else
						gate_state = STATE_OPENP;
					cmd_state = CMD_COMPLETE;
					open_count = 0;
					gate_moving_state = STEADY_STATE;
					}
				else if(gevt.pulse_count < 100)
					{
					gate_moving_state = STEADY_STATE;
					gate_state = STATE_CLOSED;
					}
				else
					{
					gate_moving_state = MOVING_STATE;
					gate_state = STATE_OPENP;
					}
				}
			else if(cmd_state == CMD_CLOSE)
				{
				if(gevt.pulse_count > 100 && gevt.pulse_count < 1800)
					{
					gate_moving_state = MOVING_STATE;
					gate_state = STATE_OPENP;
					}
				else
					{
					if(gevt.pulse_count < 100)
						gate_state = STATE_CLOSED;
					else
						gate_state = STATE_OPENP;
					cmd_state = CMD_COMPLETE;
					close_count = 0;
					gate_moving_state = STEADY_STATE;
					}
				}
			else // CMD_COMPLETE
				{
				if(gevt.pulse_count < 1800 && gevt.pulse_count > 100)
					{
					ESP_LOGI(TAG, "Illegal CMD_COMPLETE sate %4d", gevt.pulse_count);
					}
				else
					{
					if(gate_state == 0)
						{
						if(gevt.pulse_count < 100)
							gate_state = STATE_CLOSED;
						else
							gate_state = STATE_OPENP;
						}
					gate_moving_state = STEADY_STATE;
					}
				}
			//ESP_LOGI(TAG, "mv state: %2d / pc: %5d / op response %2d / cmd state: %2d / gate state %2d / %llu", gevt.moving_state, gevt.pulse_count, gevt.op_response, cmd_state, gate_state, gevt.time);
			if(saved_gate_state != gate_state ||
					saved_gate_moving_state != gate_moving_state ||
						saved_cmd_state != cmd_state)
				{
				/*
				 * save and publish the event in TOPIC_STATE
				 */
				saved_gate_state = gate_state;
				saved_gate_moving_state = gate_moving_state;
				saved_cmd_state = cmd_state;
				sprintf(msg, "%d\1%d\1%d\1%d\1%d",
							gate_state, gate_moving_state,  pulse_count_input, cmd_state, signal_value);
				publish_state(msg, 0, 0);
				}
			}
		}
	}

int do_gateop(int argc, char **argv)
	{
	int nerrors = arg_parse(argc, argv, (void **)&gateop_args);
    if (nerrors != 0)
    	{
        arg_print_errors(stderr, gateop_args.end, argv[0]);
        return 1;
    	}
    if(strcmp(gateop_args.op->sval[0], "open") == 0)
    	{
		open_gate();
    	}
    else if(strcmp(gateop_args.op->sval[0], "close") == 0)
    	{
		close_gate();
    	}
    else if(strcmp(gateop_args.op->sval[0], "open1") == 0)
    	{
		gpio_set_level(GATE_PIN_OPEN, 1);
    	}
    else if(strcmp(gateop_args.op->sval[0], "open0") == 0)
    	{
		gpio_set_level(GATE_PIN_OPEN, 0);
    	}
    else if(strcmp(gateop_args.op->sval[0], "state") == 0)
    	{
    	get_gate_state();
    	}
    else if(strcmp(gateop_args.op->sval[0], "signal") == 0)
    	{
    	ESP_LOGI(TAG, "param: %d", gateop_args.t->ival[0]);
    	}
    else
		{
		printf("gateop: [%s] unknown option\n", gateop_args.op->sval[0]);
		}
	return 0;
	}
void register_gateop()
	{
	gateop_args.op = arg_str1(NULL, NULL, "<op>", "type of operation");
	gateop_args.t = arg_int0(NULL, NULL, "<t(msec)>", "pulse duration");
    gateop_args.end = arg_end(1);
    const esp_console_cmd_t gateop_cmd =
    	{
        .command = "gate",
        .help = "gate open | close | state | stop | openp | closep",
        .hint = NULL,
        .func = &do_gateop,
        .argtable = &gateop_args
    	};
    ESP_ERROR_CHECK(esp_console_cmd_register(&gateop_cmd));

    config_gate_gpio();
    config_pw_timer();
    config_steady_wdog_timer();
    timer_set_counter_value(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX, 0);
	timer_set_alarm_value(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX, 1000 * TIMER_SCALE);
	timer_start(STEADY_TIMER_GROUP, STEADY_TIMER_INDEX);
    gate_evt_queue = xQueueCreate(10, sizeof(gate_evt_t));
    open_count = close_count = 0;

    xTaskCreate(gate_task, "gate task", 4096, NULL, 5, NULL);
    signal_value = gpio_get_level(GATE_PIN_SENSE);

	pulse_count_input = 0;
	gate_state = 0;
	cmd_state = CMD_COMPLETE;
	}

#endif
