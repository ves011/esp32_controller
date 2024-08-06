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
//#include "driver/pcnt.h"
#include "driver/pulse_cnt.h"
//#include "driver/timer.h"
#include "driver/gptimer.h"
#include "driver/rmt_rx.h"

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
#include "mqtt_client.h"

#include "utils.h"
#include "common_defines.h"
#include "external_defs.h"
#include "mqtt_ctrl.h"
#include "gateop.h"

//#define TIMER_BASE_CLK   (APB_CLK_FREQ)  /*!< Frequency of the clock on the input of the timer groups */

struct {
    struct arg_str *op;
    struct arg_end *end;
} gateop_args;

#if ACTIVE_CONTROLLER == AGATE_CONTROLLER
static const char *TAG = "GateOp";

static int pulse_count_input;
static uint32_t cmd_state;
static uint32_t gate_state;
static uint32_t gate_moving_state;
static QueueHandle_t gate_evt_queue;

static pcnt_unit_handle_t hf_pcnt_unit = NULL;
static pcnt_channel_handle_t hf_pcnt_chan = NULL;
static esp_timer_handle_t pw_timer;
static gptimer_handle_t steady_wdog_timer;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

static void pw_timer_callback(void *args)
	{
    gpio_set_level(GATE_PIN_OPEN, PIN_OFF);
    gpio_set_level(GATE_PIN_CLOSE, PIN_OFF);
    esp_timer_stop(pw_timer);
	}

static bool IRAM_ATTR steady_timer_callback(gptimer_handle_t steady_wdog_timer, const gptimer_alarm_event_data_t *edata, void *args)
	{
	gate_evt_t gevt;
	int pc_count;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	pcnt_unit_get_count(hf_pcnt_unit, &pc_count);
	if(pc_count > 0)
		{
		if(pc_count == pulse_count_input)// || pulse_count_input == -1)
			{
			pcnt_unit_clear_count(hf_pcnt_unit);
			gevt.source = PC_TIMER_SOURCE;
			gevt.pulse_count = pc_count;
			gevt.moving_state = gate_moving_state;
			gevt.time = esp_timer_get_time();
			xQueueSendFromISR(gate_evt_queue, &gevt, &xHigherPriorityTaskWoken);
			}
		pulse_count_input = pc_count;
		}
	else
		{
		if(pulse_count_input == 0)
			{
			pulse_count_input = -1;
			gevt.source = PC_TIMER_SOURCE;
			gevt.pulse_count = pulse_count_input;
			gevt.moving_state = gate_moving_state;
			gevt.time = esp_timer_get_time();
			xQueueSendFromISR(gate_evt_queue, &gevt, &xHigherPriorityTaskWoken);

			}
		else if(pulse_count_input < 0)
			{
			pulse_count_input--;
			if(pulse_count_input == PULSE_COUNT_STEADY_CLOSE)
				{
				gevt.source = PC_TIMER_SOURCE;
				gevt.pulse_count = pulse_count_input;
				gevt.moving_state = gate_moving_state;
				gevt.time = esp_timer_get_time();
				xQueueSendFromISR(gate_evt_queue, &gevt, &xHigherPriorityTaskWoken);
				pulse_count_input = -1;
				}
			}
		else if(pulse_count_input > 0)
			{
			pulse_count_input = pc_count;
			}
		}

	return xHigherPriorityTaskWoken == pdTRUE; // return whether we need to yield at the end of ISR
	}

static bool hf_pcnt_isr(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
	{
	gate_evt_t gevt;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int pc_count;

	pcnt_unit_get_count(hf_pcnt_unit, &pc_count);
	if(pc_count >= PULSE_COUNT_STEADY_OPEN)
		{
		pcnt_unit_clear_count(hf_pcnt_unit);
		gevt.source = PC_TIMER_SOURCE;
		gevt.pulse_count = pc_count;
		gevt.time = esp_timer_get_time();
		xQueueSendFromISR(gate_evt_queue, &gevt, &xHigherPriorityTaskWoken);
		}
    return (xHigherPriorityTaskWoken == pdTRUE);
	}


static void config_pw_timer()
	{
	const esp_timer_create_args_t pw_timer_args =
		{
         .callback = &pw_timer_callback,
         .name = "",
		};
    ESP_ERROR_CHECK(esp_timer_create(&pw_timer_args, &pw_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pw_timer, 10000)); //make a first start with 10 msec timeout
	}

static void config_steady_wdog_timer()
	{

	steady_wdog_timer = NULL;
	gptimer_config_t gptconf = 	{
								.clk_src = GPTIMER_CLK_SRC_DEFAULT,
								.direction = GPTIMER_COUNT_UP,
								.resolution_hz = 1000000,					//1 usec resolution
								};
	gptimer_alarm_config_t al_config = 	{
										.reload_count = 0,
										.alarm_count = 10000,				// 10 msec alarm time
										.flags.auto_reload_on_alarm = true,
										};
	gptimer_event_callbacks_t cbs = {.on_alarm = &steady_timer_callback,}; // register user callback
	ESP_ERROR_CHECK(gptimer_new_timer(&gptconf, &steady_wdog_timer));
	ESP_ERROR_CHECK(gptimer_set_alarm_action(steady_wdog_timer, &al_config));
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(steady_wdog_timer, &cbs, NULL));
	ESP_ERROR_CHECK(gptimer_enable(steady_wdog_timer));
	gptimer_start(steady_wdog_timer);
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


    io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    pcnt_unit_config_t unit_config =
    	{
        .high_limit = 30000,
        .low_limit = -30000,
		.flags.accum_count = 0,
    	};
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &hf_pcnt_unit));

    pcnt_chan_config_t chan_config =
    	{
		.edge_gpio_num = GATE_PIN_SENSE,
		.level_gpio_num = -1,
    	};
	ESP_ERROR_CHECK(pcnt_new_channel(hf_pcnt_unit, &chan_config, &hf_pcnt_chan));

	pcnt_channel_set_edge_action(hf_pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
	pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 10000,};
	pcnt_unit_set_glitch_filter(hf_pcnt_unit, &filter_config);
	ESP_ERROR_CHECK(pcnt_unit_add_watch_point(hf_pcnt_unit, PULSE_COUNT_STEADY_OPEN));
	pcnt_event_callbacks_t cbs = {.on_reach = hf_pcnt_isr,};
	ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(hf_pcnt_unit, &cbs, NULL));
	ESP_ERROR_CHECK(pcnt_unit_enable(hf_pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(hf_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(hf_pcnt_unit));
	}

static void open_gate()
	{
	cmd_state = CMD_OPEN;
	// pulse command
	gpio_set_level(GATE_PIN_OPEN, PIN_ON);
	esp_timer_start_once(pw_timer, CMD_PW);
	ESP_LOGI(TAG, "open command %llu", esp_timer_get_time());
	}


static void close_gate()
	{
	cmd_state = CMD_CLOSE;

	gpio_set_level(GATE_PIN_CLOSE, PIN_ON);
	esp_timer_start_once(pw_timer, CMD_PW);
	ESP_LOGI(TAG, "close command %llu", esp_timer_get_time());
	}

static void openped_gate()
	{
	while(gate_moving_state == 0)
		vTaskDelay(pdMS_TO_TICKS(15));
	if(gate_state == STATE_CLOSED && gate_moving_state == STEADY_STATE) // command allowed only if gate is closed
		{
		cmd_state = CMD_CLOSE;
		// pulse command
		gpio_set_level(GATE_PIN_CLOSE, PIN_ON);
		esp_timer_start_once(pw_timer, CMD_PW);
		ESP_LOGI(TAG, "open pedestrian command %llu", esp_timer_get_time());
		}
	}

static void get_gate_state()
	{
	char msg[100];
	ESP_LOGI(TAG, "\n gate_state = %lu / gate_moving_state = %lu / pulse_count = %d / cmd_state = %lu\n",
		gate_state, gate_moving_state, pulse_count_input, cmd_state);
	sprintf(msg, "%lu\1%lu\1%d\1%lu",
		gate_state, gate_moving_state,  pulse_count_input, cmd_state);
	publish_topic(TOPIC_STATE, msg, 0, 0);
	}

void gate_task()
	{
	gate_evt_t gevt;
	char msg[100];
	int saved_gate_state = -1, saved_gate_moving_state = -1,  saved_cmd_state = -1;
	int p_count[5], idx = 0, spc = 0;
	while(1)
		{
		xQueueReceive(gate_evt_queue, &gevt, portMAX_DELAY);

		if(gevt.source == PC_TIMER_SOURCE)
			{
			//ESP_LOGI(TAG, "PC_TIMER pc: %d / %llu", gevt.pulse_count, gevt.time);
			if(gevt.pulse_count > PULSE_COUNT_STEADY_OPEN - 100) // steady open
				{
				gate_moving_state = STEADY_STATE;
				gate_state = STATE_OPEN;
				cmd_state = CMD_COMPLETE;
				}
			else if(gevt.pulse_count == PULSE_COUNT_STEADY_CLOSE) //steady closed
				{
				gate_moving_state = STEADY_STATE;
				gate_state = STATE_CLOSED;
				cmd_state = CMD_COMPLETE;
				}
			else // moving state
				{
				//gate_moving_state = MOVING_STATE;
				if(gevt.pulse_count == -1)
					{
					for(int i = 0; i < idx; i++)
						spc += p_count[i];
					if(spc > PULSE_COUNT_OPEN - 20)
						gate_moving_state = OPEN_IN_PROGRESS;
					else if(spc > PULSE_COUNT_CLOSE - 20 && spc < PULSE_COUNT_CLOSE + 20)
						gate_moving_state = CLOSE_IN_PROGRESS;
					else
						gate_moving_state = MOVING_STATE;
					gate_state = STATE_OPEN;
					ESP_LOGI(TAG, "in progress: %lu / %d(%d)", gate_moving_state, spc, idx);
					idx = spc = 0;
					}
				else
					{
					p_count[idx++] = gevt.pulse_count;
					}
				}
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
				sprintf(msg, "%lu\1%lu\1%d\1%lu",
							gate_state, gate_moving_state,  gevt.pulse_count, cmd_state);
				//publish_topic(TOPIC_STATE, msg, 0, 0);
				ESP_LOGI(TAG, "gate state %2lu / mv state: %2lu / pc: %5d / cmd state: %2lu / %llu", gate_state, gate_moving_state, gevt.pulse_count, cmd_state, gevt.time);
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
    else if(strcmp(gateop_args.op->sval[0], "openped") == 0)
    	{
		openped_gate();
    	}
    else if(strcmp(gateop_args.op->sval[0], "state") == 0)
    	{
    	get_gate_state();
    	}
    else
		{
		my_printf("gateop: [%s] unknown option\n", gateop_args.op->sval[0]);
		}
	return 0;
	}
void register_gateop()
	{
	gateop_args.op = arg_str1(NULL, NULL, "<op>", "type of operation");
    gateop_args.end = arg_end(1);
    const esp_console_cmd_t gateop_cmd =
    	{
        .command = "gate",
        .help = "gate open | close | state",
        .hint = NULL,
        .func = &do_gateop,
        .argtable = &gateop_args
    	};
    ESP_ERROR_CHECK(esp_console_cmd_register(&gateop_cmd));

    gate_evt_queue = xQueueCreate(10, sizeof(gate_evt_t));

    config_gate_gpio();
    config_pw_timer();
    pulse_count_input = 0;
    gate_moving_state = 0;
    config_steady_wdog_timer();
    xTaskCreate(gate_task, "gate task", 4096, NULL, 5, NULL);

    //wait for gate state update
    while(gate_moving_state == 0)
    	vTaskDelay(pdMS_TO_TICKS(15));
    ESP_LOGI(TAG, "gate_state: %lu / gate_moving_state: %lu", gate_state, gate_moving_state);
    if(gate_moving_state == OPEN_IN_PROGRESS)
    	{
    	open_gate(); // just stop the operation
    	while(gate_moving_state != STEADY_STATE) // wait for gate to be steady
    		vTaskDelay(pdMS_TO_TICKS(15));
    	}
    if(gate_moving_state == STEADY_STATE && gate_state == STATE_OPEN)
    	close_gate();
	}

#endif
