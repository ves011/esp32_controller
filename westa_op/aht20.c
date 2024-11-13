/*
 * aht20.c
 *
 *  Created on: Nov 9, 2024
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
//#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "esp_timer.h"
#include "freertos/projdefs.h"
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
#include "mqtt_ctrl.h"
#include "common_defines.h"
#include "project_specific.h"
#include "gpios.h"
#include "external_defs.h"
#include "westaop.h"
#include "aht20.h"


static const char *TAG = "AHT";

static i2c_master_dev_handle_t aht_handle;

int aht20_init(i2c_master_dev_handle_t handle)
	{
	uint8_t wr_buf[8];
	uint8_t rd_buf[8];
	int ret, i;
	aht_handle = handle;
	//soft reset
	wr_buf[0] = 0xbe; //Initialization;
	ret = i2c_master_transmit(aht_handle, wr_buf, 1, 500);
	rd_buf[0] = 0;
	for(i = 0; i < NRETRY; i++)
		{
		wr_buf[0] = GET_STATUS_WORD;
		ret = i2c_master_transmit_receive(aht_handle, wr_buf, 1, rd_buf, 1, 500);
		ESP_LOGI(TAG, "Perform calibration: %x - %d", rd_buf[0], i);
		if(ret != ESP_OK)
			break;
		if((rd_buf[0] & (COMPLETE_BIT)) == 0)
			{
			if((rd_buf[0] & (CALIBRATION_BIT)) == 0)  // calibration  not done
				{
				memcpy(wr_buf, CALIBRATION_CMD, sizeof(CALIBRATION_CMD));
				ret = i2c_master_transmit(aht_handle, wr_buf, sizeof(CALIBRATION_CMD), 500);
				if(ret != ESP_OK)
					break;
				}
			else
				break;
			}
		vTaskDelay(pdMS_TO_TICKS(20));
		}
	if(ret == ESP_OK && i < NRETRY)
 		{
		ESP_LOGI(TAG, "Calibration done. status word: %x / %d", rd_buf[0], i);
		return ESP_OK;
		}
	else 
		ESP_LOGI(TAG, "Init error (calibration failed) %d", ret);
	return ESP_FAIL;
	}

int get_aht_data(th_data_t *ahtdata)
	{
	int ret = ESP_FAIL, i, j;
	uint32_t temp, hum;
	uint8_t wr_buf[4], rd_buf[8], crc;
	if(aht_handle == NULL)
		{
		ESP_LOGI(TAG, "AHT20 senzor not initialized");
		return ESP_FAIL;
		}
	memcpy(wr_buf, GET_DATA_CMD, sizeof(GET_DATA_CMD));
	ret = i2c_master_transmit(aht_handle, wr_buf, sizeof(GET_DATA_CMD), 500);
	if(ret == ESP_OK)
		{
		for(i = 0; i < NRETRY; i++)
			{
			vTaskDelay(pdMS_TO_TICKS(80));
			ret = i2c_master_receive(aht_handle, rd_buf, 7, 500);
			if(ret != ESP_OK)
				return ret;
			//ESP_LOGI(TAG, "read data(%d / %d) %x %x %x %x %x %x %x", i, ret,
			//		rd_buf[0], rd_buf[1], rd_buf[2], rd_buf[3], rd_buf[4], rd_buf[5], rd_buf[5]);
			if((rd_buf[0] & (COMPLETE_BIT)) == 0)
				break;
			}
		if(i < NRETRY)
			{
			//calculate CRC
			//  poly =x8+x5+x4+1
			//  initial CRC 0xff
			crc = 0xFF;
			for (i = 0; i < 6; i++) 
				{
	  			crc ^= rd_buf[i];
	  			for (j = 8; j > 0; --j) 
	  				{
	    			if (crc & 0x80)
	      				crc = (crc << 1) ^ 0x31;
	    			else
	      				crc = (crc << 1);
	    			}
	  			}
	  		if(crc == rd_buf[6])
				{
				temp = rd_buf[3];
				temp = (temp << 8) & 0xf00; 
				temp |= rd_buf[4];
				temp = (temp << 8) & 0xfff00;
				temp |= rd_buf[5];	
				
				hum = rd_buf[1];
				hum = (hum << 8) & 0xff00;
				hum |= rd_buf[2];
				hum = (hum << 4) & 0xffff0;
				hum |= (rd_buf[3] >> 4) & 0xf;	
				
				ahtdata->temperature = temp * 200./1048576. - 50.;
				ahtdata->humidity = hum * 100. / 1048576.;		
				char buf[50];
				ESP_LOGI(TAG, "Temperature = %8.3lf Humidity = %8.3lf", ahtdata->temperature, ahtdata->humidity);
				sprintf(buf, "AHT\1%.3f\1%.3f", ahtdata->temperature, ahtdata->humidity);
				//ESP_LOGI(TAG, "raw data: %x %x %x %x %x %x CRC: %x", rd_buf[0], rd_buf[1], rd_buf[2], rd_buf[3], rd_buf[4], rd_buf[5], rd_buf[6]);
				publish_topic(TOPIC_STATE, buf, 0, 0);		
				ret = ESP_OK;
				}
			else
				{
				ESP_LOGI(TAG, "CRC failed: %x / %x", crc, rd_buf[6]);
				ret = ESP_FAIL;
				}		
			}
		}	
	return ret;
	}

int aht20_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length)
	{
		return 0;
		/*
	uint8_t *wr_buf = calloc(length + 1, 1);
	int ret = ESP_FAIL;
	if(wr_buf)
		{
		wr_buf[0] = reg_addr;
		if(length)
			memcpy(wr_buf + 1, reg_data, length);
		ret = i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_I2C_ADDRESS, wr_buf, length + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
		free(wr_buf);
		}
	return ret;
	*/
	}

