/*
 * adc_op.c
 *
 *  Created on: Feb 3, 2023
 *      Author: viorel_serbu
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/adc_common.h"
#include "hal/adc_types.h"
#include "esp_netif.h"
#include "esp_adc_cal.h"
#include "driver/timer.h"
#include "esp_wifi.h"
#include "esp_spiffs.h"
#include "common_defines.h"
#include "adc_op.h"
#include "pumpop.h"
#include "waterop.h"

//#define DEBUG_ADC

int stdev_c, stdev_p;

static const char *TAG = "ADC OP";

int sample_count;
int adc_raw[2][NR_SAMPLES];
int adc_mv[2][NR_SAMPLES];

#if ACTIVE_CONTROLLER == PUMP_CONTROLLER
	static adc_channel_t channel[] = {CURRENT_ADC_CHANNEL, SENSOR_ADC_CHANNEL};
#elif ACTIVE_CONTROLLER == WATER_CONTROLLER
	extern dvconfig_t dvconfig[2];
	extern int activeDV;
//	static adc_channel_t channel = dvconfig[0].pin_current - 1;
#endif
esp_adc_cal_characteristics_t adc1_chars;
static bool cali_enable;

void adc_calibration_init(void)
	{
    esp_err_t ret;
    cali_enable = false;
	ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
#if ACTIVE_CONTROLLER == PUMP_CONTROLLER
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11)); // current channel - pump
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11)); // pressure channel - pump
#elif ACTIVE_CONTROLLER == WATER_CONTROLLER
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11)); // current channel - DV2
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11)); // current channel - DV1
#endif
    ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP_FIT);
    if (ret == ESP_ERR_NOT_SUPPORTED)
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    else if (ret == ESP_ERR_INVALID_VERSION)
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    else if (ret == ESP_OK)
    	{
    	ESP_LOGI(TAG, "Calibration scheme supported");
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
        //esp_adc_cal_characterize(ADC_UNIT_2, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc2_chars);
        ESP_LOGI(TAG, "%d / %d / %d / %d / %d / %x / %x / %d / %d", adc1_chars.adc_num, adc1_chars.atten, adc1_chars.bit_width, adc1_chars.coeff_a, adc1_chars.coeff_b, (uint32_t)adc1_chars.high_curve, (uint32_t)adc1_chars.low_curve, adc1_chars.version, adc1_chars.vref);
    	}
    else
    	ESP_LOGI(TAG, "Invalid parameter");
	}

static bool IRAM_ATTR adc_timer_callback(void *args)
	{
    BaseType_t high_task_awoken = pdFALSE;
    //adc_raw[idx][sample_count++] = adc1_get_raw(channel[idx]);
#if ACTIVE_CONTROLLER == PUMP_CONTROLLER
    adc_raw[0][sample_count] = adc1_get_raw(channel[0]);
    adc_raw[1][sample_count++] = adc1_get_raw(channel[1]);
#elif ACTIVE_CONTROLLER == WATER_CONTROLLER
    adc_raw[0][sample_count++] = adc1_get_raw(dvconfig[activeDV].pin_current - 1);
#endif
    if(sample_count >= NR_SAMPLES)
    	timer_pause(ADC_TIMER_GROUP, ADC_TIMER_INDEX);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
	}
void config_adc_timer()
	{
	timer_config_t config = {	.divider = TIMER_DIVIDER,
	        					.counter_dir = TIMER_COUNT_UP,
								.counter_en = TIMER_PAUSE,
								.alarm_en = TIMER_ALARM_EN,
								.auto_reload = true,}; // default clock source is APB
	timer_init(ADC_TIMER_GROUP, ADC_TIMER_INDEX, &config);
	timer_set_counter_value(ADC_TIMER_GROUP, ADC_TIMER_INDEX, 0);
	timer_set_alarm_value(ADC_TIMER_GROUP, ADC_TIMER_INDEX, SAMPLE_PERIOD * TIMER_SCALE_ADC);
	timer_enable_intr(ADC_TIMER_GROUP, ADC_TIMER_INDEX);
	timer_isr_callback_add(ADC_TIMER_GROUP, ADC_TIMER_INDEX, adc_timer_callback, NULL, 0);
	}

int get_pump_adc_values(minmax_t *min, minmax_t *max, int *psensor_mv)
	{
	//minmax_t min[10] = {0}, max[10] = {0};
    uint32_t kd, i, l;
    uint32_t minidx, maxidx;
    uint32_t av[8];
    int *c_val, *d_val;
    int sd = 0, ssd = 0, sqrtd = 0;
    int dd[8], avdd[8], ret = ESP_OK;
    for(l = 0; l < LOOP_COUNT; l++)
    	{
		sample_count = 0;
		timer_set_counter_value(ADC_TIMER_GROUP, ADC_TIMER_INDEX, 0);
		// 100 msec to collect all samples
		timer_start(ADC_TIMER_GROUP, ADC_TIMER_INDEX);
		while(sample_count < NR_SAMPLES)
			vTaskDelay(pdMS_TO_TICKS(2));
		//100 msec end
		d_val = adc_raw[1];
		sd = 0, ssd = 0, sqrtd = 0;
		for(i = 0; i < NR_SAMPLES; i++)
			sd += d_val[i];

		sd /= NR_SAMPLES;
		for(i = 0; i < NR_SAMPLES; i++)
			ssd += (d_val[i] - sd) * (d_val[i] - sd);
		ssd /= NR_SAMPLES;
		for(i = 1; i < ssd; i++)
			{
			if(i * i > ssd)
				break;
			}
		sqrtd = i - 1;
		if(sqrtd <= pump_pres_stdev)
			break;
#ifdef DEBUG_ADC
		//for(i = 0; i < NR_SAMPLES; i++)
		//	ESP_LOGI(TAG, "%d  %5d %5d %5d %5d", i, adc_raw[0][i], adc_mv[0][i], adc_raw[1][i], adc_mv[1][i]);
		ESP_LOGI(TAG, "noisy loop: %d -  av / stdev: %d / %d", l, sd, sqrtd);
#endif
		}
	if(l < LOOP_COUNT)
		{
		//good reading continue processing
		for(i = 0; i < NR_SAMPLES; i++)
			{
			adc_mv[0][i] = esp_adc_cal_raw_to_voltage(adc_raw[0][i], &adc1_chars);
			adc_mv[1][i] = esp_adc_cal_raw_to_voltage(adc_raw[1][i], &adc1_chars);
			}
		minidx = maxidx = 0;
		*psensor_mv = 0;
		c_val = adc_mv[0];
		d_val = adc_mv[1];



		sd = 0, ssd = 0, sqrtd = 0;
		int sc = 0, ssc = 0;
		for(i = 0; i < NR_SAMPLES; i++)
			sd += d_val[i];

		sd /= NR_SAMPLES;
		for(i = 0; i < NR_SAMPLES; i++)
			ssd += (d_val[i] - sd) * (d_val[i] - sd);
		ssd /= NR_SAMPLES;
		for(i = 1; i < ssd; i++)
			{
			if(i * i > ssd)
				break;
			}
		stdev_p = i - 1;

		for(i = 0; i < NR_SAMPLES; i++)
			sc += c_val[i];
		sc /= NR_SAMPLES;

		for(i = 0; i < NR_SAMPLES; i++)
			ssc += (c_val[i] - sc) * (c_val[i] - sc);
		ssc /= NR_SAMPLES;
		for(i = 1; i < ssc / 2; i++)
			{
			if(i * i > ssc)
				break;
			}
		stdev_c = i - 1;
	#ifdef DEBUG_ADC
			ESP_LOGI(TAG, "av / stdev  -  av / stdev %d / %d  -  %d / %d", sc, stdev_c, sd, stdev_p);

			//for(i = 0; i < NR_SAMPLES; i++)
			//	ESP_LOGI(TAG, "%d  %5d %5d %5d %5d", i, adc_raw[0][i], adc_mv[0][i], adc_raw[1][i], adc_mv[1][i]);

	#endif

			kd = 0;
			dd[0] = avdd[0] = avdd[1] = avdd[2] = 0;
			//ignnore first 10 samples
			for(i = 12; i < NR_SAMPLES - 2; i++)
				{
				*psensor_mv += d_val[i];
				av[kd  & 7] = (c_val[i - 2] + c_val[i - 1] + c_val[i] +c_val[i+1] + c_val[i + 2]) / 5;
				if(kd)
					dd[kd & 7] = av[kd & 7] - av[(kd - 1) & 7];
				if(kd >= 5)
					{
					avdd[(kd - 2) & 7] = (dd[(kd - 4) & 7] + dd[(kd - 3) & 7] + dd[(kd - 2) & 7] + dd[(kd - 1) & 7] + dd[kd  & 7]) / 5;
					}
				if(kd >= 8 && minidx < 10 && maxidx < 10) // disregard first 8 samples
					{
					if((avdd[(kd - 2) & 7] >=0) && (avdd[(kd - 3) & 7] < 0)) // check if 0 cross of 1st derivative => local MAX or MIN of the av[i]
						{
						min[minidx].mv = av[(kd - 2)& 7];
						min[minidx].index = kd - 2;
						minidx++;
						}
					else if	((avdd[(kd - 2) & 7] < 0) && (avdd[(kd - 3) & 7] >= 0)) // => local Max
						{
						max[maxidx].mv = av[(kd - 2) & 7];
						max[maxidx].index = kd - 2;
						maxidx++;
						}
					}
				kd++;
				}
#ifdef DEBUG_ADC
			for(i = 0; i < 10; i++)
				ESP_LOGI(TAG, "%5d\t%5d\t%5d\t%5d\5%5d", i, min[i].index, min[i].mv, max[i].index, max[i].mv);
#endif
			*psensor_mv /= (NR_SAMPLES - 14);
			ret =  ESP_OK;
			}
		else
			ret = ESP_FAIL;
		return ret;
        }

