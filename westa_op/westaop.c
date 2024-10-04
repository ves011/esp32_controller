/*
 * westaop.c
 *
 *  Created on: Apr 2, 2023
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "esp_timer.h"
#include "hal/gpio_types.h"
#include "math.h"
#include "errno.h"
#include "ctype.h"
#include "esp_netif.h"
//#include "esp_spi_flash.h"
#include "esp_spiffs.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "driver/gptimer.h"
#include "mqtt_client.h"
#include "common_defines.h"
#include "external_defs.h"
#include "mqtt_ctrl.h"
#include "utils.h"
#include "bmp2_defs.h"
#include "bmp2.h"
#include "dht22.h"
#include "westaop.h"

#if ACTIVE_CONTROLLER == WESTA_CONTROLLER

struct
	{
    struct arg_str *dst;
    struct arg_str *op;
    struct arg_int *os_mode;
    struct arg_int *filter;
    struct arg_int *odr;
    struct arg_int *power_mode;
    struct arg_end *end;
	} westaop_args;

static bmp2_dev_t bmpdev;
static uint8_t osrs_t, osrs_p;

static QueueHandle_t westa_cmd_queue = NULL;

static const char *TAG = "WESTA OP";
static double psl;
static double hmp;
static double pnorm;
static double ml_b;
static int sq_cmp;

// no need sice data is saved in the remote database
//SemaphoreHandle_t pthfile_mutex;

static int get_bmp_data(bmp_data_t *bmpdata)
	{
	int res = BMP2_E_COM_FAIL;
	uint8_t reg_addr = 0xf4, reg_data;
	struct bmp2_status bmps;
	if(bmpdev.power_mode == BMP2_POWERMODE_FORCED)
		{
		reg_data = 0;
		reg_data = osrs_t << 5 | osrs_p << 2 | 1;
		res = bmp2_set_regs(&reg_addr, &reg_data, 1, &bmpdev);
		}
	else
		res = BMP2_OK;

	if(res == BMP2_OK)
		{
		if(bmpdev.power_mode == BMP2_POWERMODE_FORCED)
			{
			while((res = bmp2_get_status(&bmps, &bmpdev) == BMP2_OK) && bmps.measuring == 1)
				{
				vTaskDelay(pdMS_TO_TICKS(10));
				//ESP_LOGI(TAG, "bmp280 busy");
				}
			}
		res = bmp2_get_sensor_data(bmpdata, &bmpdev);
		bmpdata->pressure /= 100.;
		if(res == BMP2_OK)
			{
			char buf[50];
			ESP_LOGI(TAG, "Temperature = %8.3f", bmpdata->temperature);
			ESP_LOGI(TAG, "Pressure    = %8.3f", bmpdata->pressure);
			sprintf(buf, "BMP\1%.3f\1%.3f", bmpdata->temperature, bmpdata->pressure);
			publish_topic(TOPIC_STATE, buf, 0, 0);
			}
		}
	return res;
	}

static void get_bmp_status(void)
	{
	int res;
	char buf[250];
	struct bmp2_config conf;
	uint8_t pmode = 0xff;
	res = bmp2_get_config(&conf, &bmpdev);
	if(res == BMP2_OK)
		{
		res = bmp2_get_power_mode(&pmode, &bmpdev);
		if(res == BMP2_OK)
			{
			ESP_LOGI(TAG, "filter: %d, os_pres: %d, os_temp: %d, pmode: %d",  conf.filter, conf.os_pres, conf.os_temp, pmode);
			sprintf(buf, "BMP%d\1%d\1%d\1%d", conf.filter, conf.os_pres, conf.os_temp, pmode);
			publish_topic(TOPIC_STATE, buf, 0, 0);
			}
		}
	}
/*
 * @brief wrapper function required by Bosch API.
 */
static void my_usleep(uint32_t period, void *intf_ptr)
	{
	usleep(period);
	}
/*
 * @brief wrapper function required by Bosch API.
 */
static BMP2_INTF_RET_TYPE bmp280_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
	{
	int ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_I2C_ADDRESS, &reg_addr, 1, reg_data, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
	if(ret == ESP_OK)
		ret = BMP2_OK;
	else
		ret = BMP2_E_COM_FAIL;
	return ret;
	}
/*
 * @brief wrapper function required by Bosch API.
 */
static BMP2_INTF_RET_TYPE bmp280_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
	{
	uint8_t *wr_buf = calloc(length + 1, 1);
	int ret;
	if(wr_buf)
		{
		wr_buf[0] = reg_addr;
		memcpy(wr_buf + 1, reg_data, length);
		ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_I2C_ADDRESS, wr_buf, length + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
		if(ret == ESP_OK)
			ret = BMP2_OK;
		else
			ret = BMP2_E_COM_FAIL;

		free(wr_buf);
		}
	else
		{
		ret = BMP2_E_NULL_PTR;
		}

	return ret;
	}

static esp_err_t i2c_master_init(void)
	{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    	};

    i2c_param_config(I2C_MASTER_NUM, &conf);

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	}

int do_westaop(int argc, char **argv)
	{
	int res = BMP2_OK;
	struct bmp2_config conf;
	bmp_data_t bmpdata;
	dht_data_t dhtdata;
	int nerrors = arg_parse(argc, argv, (void **)&westaop_args);
    if (nerrors != 0)
    	{
        arg_print_errors(stderr, westaop_args.end, argv[0]);
        return 1;
    	}
    if(!strcmp(westaop_args.dst->sval[0], "bmp"))
    	{
   		if(!strcmp(westaop_args.op->sval[0], "read"))
   			get_bmp_data(&bmpdata);
   		else if(!strcmp(westaop_args.op->sval[0], "state"))
			get_bmp_status();
		else if(!strcmp(westaop_args.op->sval[0], "reset"))
			bmp2_soft_reset(&bmpdev);
   		else if(!strcmp(westaop_args.op->sval[0], "set"))
   			{
   			if(bmp2_get_config(&conf, &bmpdev) == BMP2_OK)
   				{
   				if(westaop_args.os_mode->count)
   					conf.os_mode = westaop_args.os_mode->ival[0];
   				if(westaop_args.filter->count)
   					conf.filter = westaop_args.filter->ival[0];
   				if(westaop_args.odr->count)
   					conf.odr = westaop_args.odr->ival[0];
   				if((res = bmp2_set_config(&conf, &bmpdev)) != BMP2_OK)
   					ESP_LOGI(TAG, "Could not set new configuration. err = %d", res);
   				if(westaop_args.power_mode->count)
   					res = bmp2_set_power_mode(westaop_args.power_mode->ival[0], &conf, &bmpdev);
   				else
   					res = bmp2_set_config(&conf, &bmpdev);
   				if(res!= BMP2_OK)
   					ESP_LOGI(TAG, "Could not set new configuration. err = %d", res);
   				}
   			else
   				ESP_LOGI(TAG, "Could not read configuration. err = %d", res);
   			}
   		else if(!strcmp(westaop_args.op->sval[0], "pset"))
   			{
   			pnorm_param_t param = {0, 0};
   			char buf[100];
   			if(westaop_args.os_mode->count == 0) // no arguments just display values
   				{
   				if(rw_params(PARAM_READ, PARAM_PNORM, &param) == ESP_OK)
					{
					ESP_LOGI(TAG, "pnorm calculation parameters: psl = %.3lf, hmp = %.3lf", param.psl, param.hmp);
					sprintf(buf, "BMP\1 1\1%.3lf\1%.3lf", param.psl, param.hmp);
					}
				else
					{
					ESP_LOGI(TAG, "Error reading parameters pnorm\nReverting to default values: psl = %.3lf, hmp = %.3lf", DEFAULT_PSL, DEFAULT_ELEVATION);
					sprintf(buf, "BMP\1 0\1%.3lf\1%.3lf", DEFAULT_PSL, DEFAULT_ELEVATION);
					}
				publish_topic(TOPIC_STATE, buf, 0, 0);
   				}
   			else
   				{
   				if(westaop_args.filter->count == 0)
   					{
   					ESP_LOGI(TAG, "You need to provide both arguments for <pset>");
   					}
   				else
   					{
   					param.psl = westaop_args.os_mode->ival[0] / 1000.;
   					param.hmp = westaop_args.filter->ival[0] / 100.;
   					if(param.psl <= 0. || param.hmp <= 0.)
   						ESP_LOGI(TAG, "parameters error: %.3lf %.2lf", param.psl, param.hmp);
   					else
   						{
   						if(rw_params(PARAM_WRITE, PARAM_PNORM, &param) == ESP_OK)
							{
							psl = param.psl;
							hmp = param.hmp;
							pnorm = psl * pow((1 - hmp/44330), 5.255);
							sprintf(buf, "BMP\1 1\1%.3lf\1%.3lf\1%.3lf", param.psl, param.hmp, pnorm);
							}
						else
							{
							ESP_LOGI(TAG, "Eror updating pnorm parameters.\nKeeping old values: %.3lf, %.3lf", psl, hmp);
							sprintf(buf, "BMP\1 0\1%.3lf\1%.3lf", psl, hmp);
							}
						publish_topic(TOPIC_STATE, buf, 0, 0);
   						}
   					}
   				}
   			}
    	}
    else if(!strcmp(westaop_args.dst->sval[0], "dht"))
    	{
    	if(!strcmp(westaop_args.op->sval[0], "read"))
    		get_dht_data(&dhtdata);
    	}
	else if(!strcmp(westaop_args.dst->sval[0], "rg"))
    	{
    	if(!strcmp(westaop_args.op->sval[0], "cal"))
    		{
			pgcal_t rgcal;
			if(argc == 3) // no parameter just read the calibration
				{
				if(rw_params(PARAM_READ, PARAM_RGCAL, &rgcal) == ESP_OK)
					ESP_LOGI(TAG, "rg cal: %.1lf ml/bucket  %d cmp area", rgcal.mlb, rgcal.sqcmp);	
				}
			else
				{
				if(westaop_args.os_mode->count == 1)
					{
					rgcal.mlb = westaop_args.os_mode->ival[0] / 10.;
					rgcal.sqcmp = westaop_args.filter->ival[0];
					if(rw_params(PARAM_WRITE, PARAM_RGCAL, &rgcal) == ESP_OK)
						{
						ml_b = rgcal.mlb;
						sq_cmp = rgcal.sqcmp;
						}
					}
				}
			}
    	}
    else
    	{
    	ESP_LOGI(TAG, "Unknown operand: %s", westaop_args.dst->sval[0]);
    	return 1;
    	}
    return 0;
	}

static bool IRAM_ATTR poll_timer_callback(gptimer_handle_t c_timer, const gptimer_alarm_event_data_t *edata, void *args)
	{
	msg_t msg;
    BaseType_t high_task_awoken = pdFALSE;
    msg.source = 1;
	xQueueSendFromISR(westa_cmd_queue, &msg, NULL);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
	}
static void config_poll_timer()
	{
	gptimer_handle_t poll_timer;
	poll_timer = NULL;
	gptimer_config_t gptconf = 	{
								.clk_src = GPTIMER_CLK_SRC_DEFAULT,
								.direction = GPTIMER_COUNT_UP,
								.resolution_hz = 1000000,					//1 usec resolution

								};
	gptimer_alarm_config_t al_config = 	{
										.reload_count = 0,
										.alarm_count = 1000000,
										.flags.auto_reload_on_alarm = true,
										};

	gptimer_event_callbacks_t cbs = {.on_alarm = &poll_timer_callback,}; // register user callback
	ESP_ERROR_CHECK(gptimer_new_timer(&gptconf, &poll_timer));
	ESP_ERROR_CHECK(gptimer_set_alarm_action(poll_timer, &al_config));
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(poll_timer, &cbs, NULL));
	ESP_ERROR_CHECK(gptimer_enable(poll_timer));
	gptimer_start(poll_timer);
	}
static void IRAM_ATTR rg_isr_handler(void* arg)
	{
	msg_t msg;
    //uint32_t gpio_num = (uint32_t) arg;
    //if(gpio_num == RG_GPIO)
    	{
		msg.source = 2;
		msg.val = gpio_get_level(RG_GPIO);
		msg.vts.ts = esp_timer_get_time();
		xQueueSendFromISR(westa_cmd_queue, &msg, NULL);
		}
	}
static void config_rg_gpio()
	{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RG_GPIO);
    //io_conf.pull_down_en = 0;
    //io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_isr_handler_add(RG_GPIO, rg_isr_handler, (void*) RG_GPIO);
	}

static void pth_poll()
	{
	msg_t msg;
	bmp_data_t bmpdata;
	dht_data_t dhtdata;
	int mlmp;
	int resb, resd;
	struct tm timeinfo = { 0 };
	char bufd[40], strpub[200];
	int bucket_counter = 0;
	pnorm = psl * pow((1 - hmp/44330), 5.255);
	westa_cmd_queue = xQueueCreate(3, sizeof(msg_t));
	config_rg_gpio();
	uint64_t last_ts = 0;
	ESP_LOGI(TAG, "pth poll create - rg_state: %d", gpio_get_level(RG_GPIO));
	config_poll_timer();
	while(1)
		{
		if(xQueueReceive(westa_cmd_queue, &msg, portMAX_DELAY))
			{
			if(msg.source == 1)
				{
				time_t tm = time(NULL);
				if(tm % PTH_POLL_INT == 0 && tsync)
					{
					localtime_r(&tm, &timeinfo);
					strftime(bufd, sizeof(bufd), "%Y-%m-%dT%H:%M:%S", &timeinfo);
					resb = get_bmp_data(&bmpdata);
					resd = get_dht_data(&dhtdata);
					if(resb != BMP2_OK)
						bmpdata.temperature = bmpdata.pressure = 0;
					if(resd != ESP_OK)
						dhtdata.humidity = dhtdata.temperature = 0;
					mlmp = (ml_b * 10000 * bucket_counter) / sq_cmp;
					ESP_LOGI(TAG, "ml/mp: %d / bucket_counter: %d", mlmp, bucket_counter);
					sprintf(strpub, "%s\1%.2lf\1%.2lf\1%.2lf\1%.1lf\1%.1lf\1%d",
							bufd, bmpdata.temperature, bmpdata.pressure, pnorm, dhtdata.humidity, dhtdata.temperature, mlmp);
					publish_topic(TOPIC_MONITOR, strpub, 0, 0);
					//ESP_LOGI(TAG, "%s", strpub);
					bucket_counter = 0;
					}
				}
			else if(msg.source == 2)
				{
				ESP_LOGI(TAG, "rg interrupt: rg_state: %lu  ts: %15llu", msg.val, msg.vts.ts);
				if(/*last_rg_state == 0 &&*/ msg.val == 1)
					{
					if(msg.vts.ts - last_ts > RG_DEBOUNCE)
						{
						bucket_counter++;
						//ESP_LOGI(TAG, "bucket_counter: %d rg_state: %lu  ts: %15llu", bucket_counter, msg.val, msg.vts.ts - last_ts);
						}
					}
				last_ts = msg.vts.ts;
				}
			}
		}
	}

void register_westaop(void)
	{
	uint8_t pmode = 0xff;
	struct bmp2_config conf;
	pgcal_t rgcal;
	bmpdev.intf = BMP2_I2C_INTF;
	bmpdev.delay_us = my_usleep;
	bmpdev.read = bmp280_read;
	bmpdev.write = bmp280_write;
	bmpdev.intf_ptr = NULL;
	pnorm_param_t param = {0, 0};
	if(rw_params(PARAM_READ, PARAM_RGCAL, &rgcal) == ESP_OK)
		{
		ml_b = rgcal.mlb;
		sq_cmp = rgcal.sqcmp;
		}
	else 
		{
		ml_b = DEFAULT_RGCAL;
		sq_cmp = DEFAULT_SQCMP;
		}
	/*
	pthfile_mutex = xSemaphoreCreateMutex();
	if(!pthfile_mutex)
		{
		ESP_LOGE(TAG, "cannot create pthfile_mutex");
		esp_restart();
		}
	*/
	if(rw_params(PARAM_READ, PARAM_PNORM, &param) == ESP_OK)
		{
		psl = param.psl;
		hmp = param.hmp;
		}
	else
		{
		psl = DEFAULT_PSL;
		hmp = DEFAULT_ELEVATION;
		}

	int res = i2c_master_init();
	res = bmp2_init(&bmpdev);
	if(res == BMP2_OK)
		{
    	res = bmp2_get_config(&conf, &bmpdev);
    	res = bmp2_get_power_mode(&pmode, &bmpdev);
    	if(res == BMP2_OK)
    		{
    		ESP_LOGI(TAG, "bmp280 chip ID: %0x \n%0x %0x %0x %0x %0x %0x\nPower mode = %d",
    					bmpdev.chip_id, conf.filter, conf.odr, conf.os_mode, conf.os_pres, conf.os_temp, conf.spi3w_en, bmpdev.power_mode);
    		ESP_LOGI(TAG, "bmp280 cal param: %6d %6d \n%6d %6d %6d %6d %6d\n%6d %6d %6d %6d %6d", bmpdev.calib_param.dig_t1, bmpdev.calib_param.dig_t2
    								, bmpdev.calib_param.dig_p1, bmpdev.calib_param.dig_p2, bmpdev.calib_param.dig_p3, bmpdev.calib_param.dig_p4, bmpdev.calib_param.dig_p5
    								, bmpdev.calib_param.dig_p6, bmpdev.calib_param.dig_p7, bmpdev.calib_param.dig_p8, bmpdev.calib_param.dig_p9, bmpdev.calib_param.dig_p10);
    		conf.os_mode = BMP2_OS_MODE_ULTRA_HIGH_RESOLUTION;
    		conf.filter = BMP2_FILTER_COEFF_16;
    		conf.os_pres = 5; // BMP2_OS_MODE_ULTRA_HIGH_RESOLUTION;
    		conf.os_temp = 2; //BMP2_OS_MODE_ULTRA_HIGH_RESOLUTION;
    		conf.odr = BMP2_ODR_500_MS;
    		res = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &conf, &bmpdev);
    		if(res == BMP2_OK)
    			{
				res = bmp2_get_config(&conf, &bmpdev);
				ESP_LOGI(TAG, "bmp280 config: %0x %0x  %0x %0x  %0x\n", conf.filter, conf.odr, conf.os_pres, conf.os_temp, conf.spi3w_en);
				osrs_t = conf.os_temp;
				osrs_p = conf.os_pres;
				res = bmp2_get_power_mode(&pmode, &bmpdev);
				ESP_LOGI(TAG, "bmp280 power mode: %0x", pmode);
				}
			else
				ESP_LOGI(TAG, "error during configuration %d", res);
    		}
		}
	if(res != BMP2_OK)
		ESP_LOGI(TAG, "Cannot initialize i2c driver. Error = %d", res);
	dht_init();
// test for normal mode
//	bmp_data_t bmpdata;
//	while(1 && res == BMP2_OK)
//		{
//		get_bmp_data(&bmpdata);
//		vTaskDelay(2000 / portTICK_PERIOD_MS); //wait 2 seconds
//		}
// end test for normal mode

	westaop_args.dst = arg_str1(NULL, NULL, "<dest>", "bmp | dht");
	westaop_args.op = arg_str1(NULL, NULL, "<op>", "status | set | read");
	westaop_args.os_mode = arg_int0(NULL, NULL, "<os_mode>", "over sampling mode");
	westaop_args.filter = arg_int0(NULL, NULL, "<filter>", "filter coeffs");
	westaop_args.odr = arg_int0(NULL, NULL, "<odr>", "ouput data rate");
	westaop_args.power_mode = arg_int0(NULL, NULL, "<p_mode>", "power mode");
	westaop_args.end = arg_end(1);
	const esp_console_cmd_t westaop_cmd =
		{
		.command = "westa",
		.help = "westa bmp|dht status|set|read [set parameters]",
		.hint = NULL,
		.func = &do_westaop,
		.argtable = &westaop_args
		};
	ESP_ERROR_CHECK(esp_console_cmd_register(&westaop_cmd));
	if(xTaskCreate(pth_poll, "PTH poll task", 6144, NULL, USER_TASK_PRIORITY, NULL) != pdPASS)
		ESP_LOGI(TAG, "cannot create PTH poll task");
	}
#endif
