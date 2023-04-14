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
#include "errno.h"
#include "ctype.h"
#include "esp_netif.h"
#include "esp_spi_flash.h"
#include "esp_spiffs.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
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

static bmp2_dev_t bmpdev;
static uint8_t osrs_t, osrs_p;
static int publish_range(char *start_date, char *end_date, int nvals, int av_points);

static const char *TAG = "WESTA OP";
static double psl;
static double hmp;
static double pnorm;

SemaphoreHandle_t pthfile_mutex;

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
			publish_state(buf, 0, 0);
			}
		}
	return res;
	}
/*
static int get_bmp_data(bmp_data_t *bmpdata)
	{
	int res = BMP2_E_COM_FAIL;
	struct bmp2_status bmps;
	while((res = bmp2_get_status(&bmps, &bmpdev) == BMP2_OK) && bmps.measuring == 1)
		{
		vTaskDelay(pdMS_TO_TICKS(10));
		ESP_LOGI(TAG, "bmp280 busy");
		}
	res = bmp2_get_sensor_data(bmpdata, &bmpdev);
	if(res == BMP2_OK)
		{
		char buf[50];
		ESP_LOGI(TAG, "Temperature = %8.3f", bmpdata->temperature);
		ESP_LOGI(TAG, "Pressure    = %8.3f", bmpdata->pressure);
		//sprintf(buf, "BMP\1%.3f\1%.3f", bmpdata->temperature, bmpdata->pressure);
		//publish_state(buf, 0, 0);
		}
	return res;
	}
*/
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
			publish_state(buf, 0, 0);
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
	int ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_I2C_ADDRESS, &reg_addr, 1, reg_data, length, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
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
				publish_state(buf, 0, 0);
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
						publish_state(buf, 0, 0);
   						}
   					}
   				}
   			}
    	}
    else if(!strcmp(westaop_args.dst->sval[0], "dht"))
    	{
    	if(!strcmp(westaop_args.op->sval[0], "read"))
    		get_dht_data(&dhtdata);
    	if(!strcmp(westaop_args.op->sval[0], "state"))
    		{
    		res = get_dht_status();
    		ESP_LOGI(TAG, "DHT status: %d", res);
    		}
    	}
    else if(!strcmp(westaop_args.dst->sval[0], "range"))
    	{
    	char start_date[32], end_date[32], *b;
   		int i = 0, k = 0, nvals = 0;
   		b = start_date;
   		start_date[0] = end_date[0] = 0;
   		if(strstr(westaop_args.op->sval[0], ">#"))
   			nvals = 1;

   		while(i < strlen(westaop_args.op->sval[0]))
   			{
   			if(westaop_args.op->sval[0][i] != '>')
   				b[k] = westaop_args.op->sval[0][i];
   			else
   				{
   				if(nvals)
   					{
   					i += 2;
   					nvals = atoi(westaop_args.op->sval[0] + i);
   					break;
   					}
   				else
   					{
					b[k] = 0;
					b = end_date;
					k = -1;
					}
   				}
   			i++;
   			k++;
    		}
    	b[k] = 0;
    	if(westaop_args.os_mode->count)
    		i = westaop_args.os_mode->ival[0];
    	else
    		i = 1;
    	ESP_LOGI(TAG, "start %s", start_date);
    	ESP_LOGI(TAG, "start %s", end_date);
    	ESP_LOGI(TAG, "n vals %d", nvals);
    	publish_range(start_date, end_date, nvals, i);
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
	int res;
	uint32_t poll_int = (PTH_POLL_INT * portTICK_PERIOD_MS) / 1000;
	char bufb[100], bufd[100];
	TickType_t  tdelta;
	res = dht_init();
	pnorm = psl * pow((1 - hmp/44330), 5.255);
	// get the first measurement when time%PTH_POLL_INT = 0
	// and after tsync = 1 by ntp_sync
	while(1)
		{
		time_t tm = time(NULL);
		if(tm % poll_int == 0 && tsync)
			break;
		vTaskDelay(1000 / portTICK_PERIOD_MS); //wait 1 sec
		}
	while(1)
		{
		tdelta = xTaskGetTickCount();
		res = get_bmp_data(&bmpdata);
		if(res == BMP2_OK)
			sprintf(bufb, "%.3lf %.3lf %.3lf", bmpdata.temperature, bmpdata.pressure, pnorm);
		else
			strcpy(bufb, "0.0 0.0 0.0");

		res = get_dht_data(&dhtdata);
		if(res == ESP_OK)
			sprintf(bufd, " %.1lf %.1lf", dhtdata.humidity, dhtdata.temperature);
		else
			strcpy(bufd, " 0.0 0.0");

		strcat(bufb, bufd);
		write_tpdata(PARAM_WRITE, bufb);
		sprintf(bufb, "BMPDHT\1%.3lf\1%.3lf\1%.3lf\1%.1lf\1%.1lf", bmpdata.temperature, bmpdata.pressure, pnorm, dhtdata.humidity, dhtdata.temperature);
		publish_monitor(bufb, 0, 0);
		// tdelta is used to avoid time drift because of long read operation
		// wo tdelta the drift is almost 100msec per cycle
		tdelta = xTaskGetTickCount() - tdelta;
		vTaskDelay(PTH_POLL_INT - tdelta);
		}
	}
static int publish_range(char *start_date, char *end_date, int nvals, int av_points)
	{
	int syear = 0, smonth = 0, sday = 0, shour = 0, smin = 0, ssec = 0;
	int eyear = 0, emonth = 0, eday = 0, ehour = 0, emin = 0, esec = 0;
	char file_name[80], bread[128], bdate[80], bval[128], bdateval[80];
	double atb = 0, atd = 0, ahd = 0, apb = 0;
	double pb, tb, pn, hd, td;
	int k = 0, nval = 0;
	FILE *f;
	int ret = ESP_FAIL, i = 0;;
	//2023-04-09/09:09:21 23.137 101199.138 77.488641 45.0 22.6
	sscanf(start_date, "%d-%d-%dT%d:%d:%d", &syear, &smonth, &sday, &shour, &smin, &ssec);
	sscanf(end_date, "%d-%d-%dT%d:%d:%d", &eyear, &emonth, &eday, &ehour, &emin, &esec);
	sprintf(file_name, "%s/%d.tph", BASE_PATH, syear);
	ESP_LOGI(TAG, "%d %d %d %d %d %d", syear, smonth, sday, shour, smin, ssec);
	ESP_LOGI(TAG, "%d %d %d %d %d %d", eyear, emonth, eday, ehour, emin, esec);
	ESP_LOGI(TAG, "%s", file_name);
	if(xSemaphoreTake(pthfile_mutex, ( TickType_t ) 100 )) // 1 sec wait
		{
		f = fopen(file_name, "r");
		if(f)
			{
			while(!feof(f))
				{
				if(fgets(bread , sizeof(bread), f))
					{
					strncpy(bdate, bread, 19);
					bdate[19] = 0;
					if(strcmp(bdate, start_date) >= 0)
						{
						strcpy(bval, bread + 20);
						tb = pb = pn = hd = td = 0;
						sscanf(bval, "%lf %lf %lf %lf %lf", &tb, &pb, &pn, &hd, &td);
						if(k == 0)
							{
							atb = 0; atd = 0; ahd = 0; apb = 0;
							}
						if(k == av_points / 2)
							{
							strcpy(bdateval, bdate);
							}
						atb += tb; atd += td; ahd += hd; apb += pb;
						k++;
						if(k == av_points)
							{
							atb /= k; atd /= k; ahd /= k; apb /= k;
							k = 0;
							sprintf(bread, "WR %s %.3lf %.3lf %.3lf %.3lf %.3lf", bdateval, atb, apb, pn, ahd, atd);
							//ESP_LOGI(TAG, "%s", bread);
							publish_state(bread, 0, 0);
							nval++;
							}
						if(nvals > 0)
							{
							if(i >= nvals)
								break;
							}
						else
							{
							if(strlen(end_date) && strcmp(bdate, end_date) >= 0)
								break;
							}
						//ESP_LOGI(TAG, "%s", bread);
						//publish_state(bread, 0, 0);
						i++;
						}
					}
				}
			fclose(f);
			ret =  ESP_OK;
			}
		else
			{
			ESP_LOGI(TAG, "Cannot open %s for reading. errno = %d", file_name, errno);
			ret = ESP_FAIL;
			}
		xSemaphoreGive(pthfile_mutex);
		}
	else
		ret = ESP_FAIL;
	sprintf(bread, "WRS %d %d", i, nval);
	publish_state(bread, 0, 0);
	ESP_LOGI(TAG, "range: %d lines / n values: %d", i, nval);
	return ret;
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
	pnorm_param_t param = {0, 0};
	pthfile_mutex = xSemaphoreCreateMutex();
	if(!pthfile_mutex)
		{
		ESP_LOGE(TAG, "cannot create pthfile_mutex");
		esp_restart();
		}
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
