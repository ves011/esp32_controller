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
#include "bmp2_defs.h"
#include "bmp2.h"
#include "westaop.h"

#if ACTIVE_CONTROLLER == WESTA_CONTROLLER

static const char *TAG = "WESTA OP";

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

static int do_westaop(int argc, char **argv)
	{
	int nerrors = arg_parse(argc, argv, (void **)&westaop_args);
    if (nerrors != 0)
    	{
        arg_print_errors(stderr, westaop_args.end, argv[0]);
        return 1;
    	}
    if(!strcmp(westaop_args.dst->sval[0], "bmp280"))
    	{

    	}
    else if(!strcmp(westaop_args.dst->sval[0], "dht22"))
    	{
    	}
    else if(!strcmp(westaop_args.dst->sval[0], "temp"))
    	{
    	}
    else if(!strcmp(westaop_args.dst->sval[0], "pres"))
    	{
    	}
    else if(!strcmp(westaop_args.dst->sval[0], "hum"))
    	{
    	}
    else
    	{
    	ESP_LOGI(TAG, "Unknown operand: %s", westaop_args.dst->sval[0]);
    	return 1;
    	}
    return 0;
	}
void register_westaop(void)
	{
	bmp2_dev_t bmpdev;
	uint8_t pmode = 0xff, reg_addr = 0xf4, reg_data = 0x55;
	struct bmp2_config conf;
	struct bmp2_data bmpdata;
	bmpdev.intf = BMP2_I2C_INTF;
	bmpdev.delay_us = my_usleep;
	bmpdev.read = bmp280_read;
	bmpdev.write = bmp280_write;
	bmpdev.intf_ptr = NULL;
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
				res = bmp2_get_power_mode(&pmode, &bmpdev);
				ESP_LOGI(TAG, "bmp280 power mode: %0x", pmode);
				while(1)
					{
					res = bmp2_set_regs(&reg_addr, &reg_data, 1, &bmpdev);
					res = bmp2_get_sensor_data(&bmpdata, &bmpdev);
					if(res == BMP2_OK)
						{
						ESP_LOGI(TAG, "Temperature = %f", bmpdata.temperature);
						ESP_LOGI(TAG, "Pressure = %f", bmpdata.pressure);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						}
					}
				}
			else
				ESP_LOGI(TAG, "error during configuration %d", res);
    		}
		}
	/*
	uint8_t reg_addr = 0x88, data[4] = {0};
	i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_I2C_ADDRESS, &reg_addr, 1, data, 4, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
	ESP_LOGI(TAG, "bmp280 chip ID: %0x %0x %0x %0x", data[0], data[1], data[2], data[3]);
	reg_addr = 0x8a;
	i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_I2C_ADDRESS, &reg_addr, 1, data, 4, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
	ESP_LOGI(TAG, "bmp280 chip ID: %0x %0x %0x %0x", data[0], data[1], data[2], data[3]);
	*/
	if(res == ESP_OK)
		{
		westaop_args.op = arg_str1(NULL, NULL, "<op>", "type of operation: read | write");
		westaop_args.dst = arg_str1(NULL, NULL, "<dest>|<param>", "destination: bmp280, dht22 | parameter: temp, pres, hum");
		westaop_args.reg_addr = arg_str0(NULL, NULL, "<reg_addr>", "register address in HEX | parameter: temp, pres, hum");
		westaop_args.end = arg_end(1);
		const esp_console_cmd_t westaop_cmd =
			{
			.command = "westa",
			.help = "westa read|write dest reg_addr",
			.hint = NULL,
			.func = &do_westaop,
			.argtable = &westaop_args
			};
		ESP_ERROR_CHECK(esp_console_cmd_register(&westaop_cmd));
		}
	else
		{
		ESP_LOGI(TAG, "Cannot initialize i2c driver. Error = %d", res);
		}
	}
#endif
