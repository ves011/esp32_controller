/*
 * dht22.c
 *
 *  Created on: Apr 5, 2023
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_err.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "mqtt_ctrl.h"
#include "dht22.h"

static const char *TAG = "DHT_RMT";

static RingbufHandle_t ringbuf_handle = NULL;
static SemaphoreHandle_t dhtd_mutex;

int dht_init()
	{
	int ret = ESP_FAIL;
	dhtd_mutex = xSemaphoreCreateMutex();
	if(!dhtd_mutex)
		{
		ESP_LOGE(TAG, "cannot create dhtd_mutex");
		esp_restart();
		}

	rmt_config_t rx_config = {
        .rmt_mode      = RMT_MODE_RX,
        .channel       = RMT_CHANNEL_4,  //found somewhere in the docs chn[0..3] are TX and chn[4..7] are RX ?!?
        .gpio_num      = DHT_DATA_PIN,
        .clk_div       = 80,			// 1us resolution
        .mem_block_num = 1,				// 1 memory block which can hold up to 64 symbols
        .rx_config = {
            .idle_threshold      = 32000, 	// a pulse longer than 400usec will turn rmt receiver idle
            .filter_ticks_thresh = 200, 	// any pulse < 2.5usec will be ignored
            .filter_en           = 1
        	}
    	};
    ret = rmt_config(&rx_config);
    if(ret == ESP_OK)
    	{
    	ret = rmt_driver_install(DHT_RMT_CHANNEL, 1024, 0);
    	if(ret == ESP_OK)
    		{
    		rmt_set_memory_owner(DHT_RMT_CHANNEL, RMT_MEM_OWNER_RX);
    		ret = rmt_get_ringbuf_handle(DHT_RMT_CHANNEL, &ringbuf_handle);
    		if(ret != ESP_OK)
    			{
    			ESP_LOGI(TAG, "error getting rmt channel ring buffer %d - %s", ret, esp_err_to_name(ret));
    			}
    		}
    	else
    		ESP_LOGI(TAG, "error installing rmt driver %d - %s", ret, esp_err_to_name(ret));
    	}
    else
    	ESP_LOGI(TAG, "error creating rmt channel %d - %s", ret, esp_err_to_name(ret));
    if(ret == ESP_OK)
    	{
    	gpio_config_t gpio = {
			.intr_type    = GPIO_INTR_DISABLE,
			.pin_bit_mask = (1ULL << DHT_DATA_PIN),
			.mode         = GPIO_MODE_INPUT_OUTPUT_OD,
			.pull_up_en   = true,
			.pull_down_en = false
			};
		gpio_config(&gpio);
		gpio_set_level(DHT_DATA_PIN, 1);
    	}
	return ret;
	}
static int dht_parse(rmt_item32_t *symbol, int n_symbols, dht_data_t * dhtd)
	{
	int k, i, bit, ret;
	int16_t temp = 0, hum = 0, cs = 0, csc = 0;
	/*
	 * useful data is 40 symbols but a read returns at least 42 symbols
	 * symbol[0] = 1 with duration of 20 to 200usec --> bus release by esp32
	 * symbol[1, 2] = 80usec low, 80 usec high --> dht22 response signal
	 * symbol[3...end] = data bit --> 50usec low + x usec high
	 */
	 if(n_symbols >= 42 && symbol[0].level0 == 1 &&
			 symbol[0].duration1 >=75 && symbol[0].duration1 <= 85 && symbol[0].level1 == 0 &&
			 symbol[1].duration0 >=75 && symbol[1].duration0 <= 85 && symbol[1].level0 == 1)
	 	 {
	 	 // start of transmission OK, parse further symbols
	 	 k = 1;
	 	 // humidity first: 16 bits
	 	 for(i = 15; i >= 0; i--)
	 	 	 {
	 	 	 if(symbol[k].duration1 >= 48 && symbol[k].duration1 <= 55 && symbol[k++].level1 == 0) //start bit
	 	 	 	 {
	 	 	 	 if(symbol[k].duration0 >= 22 && symbol[k].duration0 <= 30)bit = 0;
	 	 	 	 else if(symbol[k].duration0 >= 68 && symbol[k].duration0 <= 75)bit = 1;
	 	 	 	 else return ESP_ERR_INVALID_STATE;
	 	 	 	 hum |= bit << i;
	 	 	 	 }
	 	 	 }
	 	 // temp: 16 bits
	 	 for(i = 15; i >= 0; i--)
	 	 	 {
	 	 	 if(symbol[k].duration1 >= 48 && symbol[k].duration1 <= 55 && symbol[k++].level1 == 0) //start bit
	 	 	 	 {
	 	 	 	 if(symbol[k].duration0 >= 22 && symbol[k].duration0 <= 30)bit = 0;
	 	 	 	 else if(symbol[k].duration0 >= 68 && symbol[k].duration0 <= 75)bit = 1;
	 	 	 	 else return ESP_ERR_INVALID_STATE;
	 	 	 	 temp |= bit << i;
	 	 	 	 }
	 	 	 }
	 	 // check sum: 8 bits
	 	 for(i = 7; i >= 0; i--)
	 	 	 {
	 	 	 if(symbol[k].duration1 >= 48 && symbol[k].duration1 <= 55 && symbol[k++].level1 == 0) //start bit
	 	 	 	 {
	 	 	 	 if(symbol[k].duration0 >= 22 && symbol[k].duration0 <= 30)bit = 0;
	 	 	 	 else if(symbol[k].duration0 >= 68 && symbol[k].duration0 <= 75)bit = 1;
	 	 	 	 else return ESP_ERR_INVALID_STATE;
	 	 	 	 cs |= bit << i;
	 	 	 	 }
	 	 	 }
	 	 csc = (((hum >> 8) & 0xff) + (hum & 0xff) + ((temp >> 8) & 0xff) + (temp & 0xff)) & 0xff;
	 	 if(csc == cs)
	 	 	 {
	 	 	 dhtd->humidity = hum / 10.;
	 	 	 dhtd->temperature = temp / 10.;
	 	 	 ret = ESP_OK;
	 	 	 }
	 	 else
	 	 	 ret = ESP_ERR_INVALID_CRC;
	 	 //ESP_LOGI(TAG, "humidity: %d, temp: %d, check sum: %x, %x", hum, temp, cs, csc);
	 	 }
	else
		ret = ESP_FAIL;
	return ret;
	}
int get_dht_data(dht_data_t * dhtd)
	{
	size_t n_symbol = 0;
	int ret = ESP_FAIL;
	if(xSemaphoreTake(dhtd_mutex, ( TickType_t ) 100 )) // 1 sec wait
		{
		gpio_set_level(DHT_DATA_PIN, 0);
		rmt_rx_memory_reset(DHT_RMT_CHANNEL);
		ret = rmt_rx_start(DHT_RMT_CHANNEL, true);
		if(ret == ESP_OK)
			{
			ret = ESP_FAIL;
			usleep(1000);
			gpio_set_level(DHT_DATA_PIN, 1);
			rmt_item32_t *symbol = (rmt_item32_t *)xRingbufferReceive(ringbuf_handle, &n_symbol, 10);
			rmt_rx_stop(DHT_RMT_CHANNEL);

			if(symbol && n_symbol)
				{
				ret = ESP_OK;
				n_symbol /= 4;
				ret = dht_parse(symbol, n_symbol, dhtd);
				char buf[50];
				ESP_LOGI(TAG, "Temperature = %8.1f", dhtd->temperature);
				ESP_LOGI(TAG, "Humidity    = %8.1f (%d)", dhtd->humidity, ret);
				sprintf(buf, "DHT\1%.1f\1%.1f", dhtd->temperature, dhtd->humidity);
				publish_state(buf, 0, 0);
				}
			else
				ret = ESP_ERR_INVALID_SIZE;
			vRingbufferReturnItem(ringbuf_handle, (void*)symbol);
			}
		xSemaphoreGive(dhtd_mutex);
		}
	return ret;
	}
int get_dht_status()
	{
	size_t n_symbol = 0;
	int ret = ESP_FAIL;
	char buf[100];
	if(xSemaphoreTake(dhtd_mutex, ( TickType_t ) 100 )) // 1 sec wait
		{
		gpio_set_level(DHT_DATA_PIN, 0);
		rmt_rx_memory_reset(DHT_RMT_CHANNEL);
		ret = rmt_rx_start(DHT_RMT_CHANNEL, true);
		if(ret == ESP_OK)
			{
			ret = ESP_FAIL;
			usleep(1000);
			gpio_set_level(DHT_DATA_PIN, 1);
			rmt_item32_t *symbol = (rmt_item32_t *)xRingbufferReceive(ringbuf_handle, &n_symbol, 10);
			rmt_rx_stop(DHT_RMT_CHANNEL);

			if(symbol && n_symbol)
				ret = ESP_OK;
			vRingbufferReturnItem(ringbuf_handle, (void*)symbol);
			}
		xSemaphoreGive(dhtd_mutex);
		}
	sprintf(buf, "DHT\1%d", ret);
	publish_state(buf, 0, 0);

	return ret;
	}
