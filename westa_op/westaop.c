/*
 * westaop.c
 *
 *  Created on: Apr 2, 2023
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "math.h"
#include "common_defines.h"
#include "mqtt_ctrl.h"
#include "utils.h"
#include "bmp2_defs.h"
#include "bmp2.h"
#include "dht22.h"
#include "westaop.h"

#if ACTIVE_CONTROLLER == WESTA_CONTROLLER

static bmp2_dev_t bmpdev;
static uint8_t osrs_t, osrs_p;

static const char *TAG = "WESTA OP";
double psl;

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
		if(res == BMP2_OK)
			{
			char buf[50];
			ESP_LOGI(TAG, "Temperature = %8.3f", bmpdata->temperature);
			ESP_LOGI(TAG, "Pressure    = %8.3f", bmpdata->pressure);
			sprintf(buf, "%.3f\1%.3f", bmpdata->temperature, bmpdata->pressure);
			publish_state(buf, 0, 0);
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
			sprintf(buf, "%d\1%d\1%d\1%d", conf.filter, conf.os_pres, conf.os_temp, pmode);
			publish_state(buf, 0, 0);
			}
		}
	}

static void my_usleep(uint32_t period, void *intf_ptr)
	{
	usleep(period);
	}
static BMP2_INTF_RET_TYPE bmp280_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
	{
	int ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_I2C_ADDRESS, &reg_addr, 1, reg_data, length, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
	if(ret == ESP_OK)
		ret = BMP2_OK;
	else
		ret = BMP2_E_COM_FAIL;
	return ret;
	}
static BMP2_INTF_RET_TYPE bmp280_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
	{
	uint8_t *wr_buf = calloc(length + 1, 1);
	int ret;
	if(wr_buf)
		{
		wr_buf[0] = reg_addr;
		memcpy(wr_buf + 1, reg_data, length);
		ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_I2C_ADDRESS, wr_buf, length + 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
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
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    	};

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	}

int do_westaop(int argc, char **argv)
	{
	int res = BMP2_OK;
	struct bmp2_config conf;
	bmp_data_t bmpdata;
	int nerrors = arg_parse(argc, argv, (void **)&westaop_args);
    if (nerrors != 0)
    	{
        arg_print_errors(stderr, westaop_args.end, argv[0]);
        return 1;
    	}
    if(!strcmp(westaop_args.dst->sval[0], "bmp"))
    	{
    	if(westaop_args.op->count)
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
    		}

    	}
    else if(!strcmp(westaop_args.dst->sval[0], "dht"))
    	{
    	}
    else
    	{
    	ESP_LOGI(TAG, "Unknown operand: %s", westaop_args.dst->sval[0]);
    	return 1;
    	}
    return 0;
	}
static void pth_poll()
	{
	bmp_data_t bmpdata;
	dht_data_t dhtdata;
	double pnorm;
	int res;
	char bufb[100], bufd[100];
	res = dht_init();
	while(1)
		{
		res = get_bmp_data(&bmpdata);
		if(res == BMP2_OK)
			{
			pnorm = psl * pow((1 - bmpdata.pressure/psl), 5.255);
			sprintf(bufb, "%.3lf %.3lf %3lf", bmpdata.temperature, bmpdata.pressure, pnorm);
			}
		else
			strcpy(bufb, "0.0 0.0 0.0");
		res = get_dht_data(&dhtdata);
		if(res == ESP_OK)
			{
			sprintf(bufd, " %.1lf %.1lf", dhtdata.humidity, dhtdata.temperature);
			}
		else
			strcpy(bufd, " 0.0 0.0");
		strcat(bufb, bufd);
		rw_tpdata(PARAM_WRITE, bufb);
		vTaskDelay(PTH_POLL_INT);
		}
	}
void register_westaop(void)
	{
	uint8_t pmode = 0xff;
	struct bmp2_config conf;
	bmpdev.intf = BMP2_I2C_INTF;
	bmpdev.delay_us = my_usleep;
	bmpdev.read = bmp280_read;
	bmpdev.write = bmp280_write;
	bmpdev.intf_ptr = NULL;
	dht_data_t dhtd;
	psl = DEFAULT_PSL;
	int res = i2c_master_init();
	res = bmp2_init(&bmpdev);
	if(res == BMP2_OK)
		{
    	res = bmp2_get_config(&conf, &bmpdev);
    	if(res == BMP2_OK)
    		{
    		ESP_LOGI(TAG, "bmp280 chip ID: %0x %0x %0x %0x %0x %0x %0x", bmpdev.chip_id, conf.filter, conf.odr, conf.os_mode, conf.os_pres, conf.os_temp, conf.spi3w_en);
    		ESP_LOGI(TAG, "bmp280 cal param: %6d %6d \n%6d %6d %6d %6d %6d\n%6d %6d %6d %6d %6d", bmpdev.calib_param.dig_t1, bmpdev.calib_param.dig_t2
    								, bmpdev.calib_param.dig_p1, bmpdev.calib_param.dig_p2, bmpdev.calib_param.dig_p3, bmpdev.calib_param.dig_p4, bmpdev.calib_param.dig_p5
    								, bmpdev.calib_param.dig_p6, bmpdev.calib_param.dig_p7, bmpdev.calib_param.dig_p8, bmpdev.calib_param.dig_p9, bmpdev.calib_param.dig_p10);
    		conf.os_mode = BMP2_OS_MODE_ULTRA_HIGH_RESOLUTION;
    		conf.filter = BMP2_FILTER_COEFF_16;
    		conf.os_pres = 0; // BMP2_OS_MODE_ULTRA_HIGH_RESOLUTION;
    		conf.os_temp = 0; //BMP2_OS_MODE_ULTRA_HIGH_RESOLUTION;
    		conf.odr = BMP2_ODR_500_MS;
    		res = bmp2_set_power_mode(BMP2_POWERMODE_FORCED, &conf, &bmpdev);
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
// test for normal mode
//	bmp_data_t bmpdata;
//	while(1 && res == BMP2_OK)
//		{
//		get_bmp_data(&bmpdata);
//		vTaskDelay(2000 / portTICK_PERIOD_MS); //wait 2 seconds
//		}
// end test for normal mode
	//res = dht_init();
	//get_dht_data(&dhtd);
	westaop_args.op = arg_str1(NULL, NULL, "<dest>", "bmp | dht");
	westaop_args.dst = arg_str1(NULL, NULL, "<op>", "status | set | read");
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
	if(xTaskCreate(pth_poll, "PTH poll task", 4096, NULL, USER_TASK_PRIORITY, NULL) != pdPASS)
		ESP_LOGI(TAG, "cannot create PTH poll task");
	}
#endif
