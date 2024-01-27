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
//#include "driver/adc.h"
//#include "driver/adc_common.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_types.h"
#include "esp_netif.h"
//#include "esp_adc_cal.h"
//#include "driver/timer.h"
#include "driver/gptimer.h"
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
static gptimer_handle_t adc_timer;
static adc_cali_handle_t adc1_cal_handle;
static adc_oneshot_unit_handle_t adc1_handle;
static QueueHandle_t adc_evt_queue = NULL;
static SemaphoreHandle_t adcval_mutex;

static bool IRAM_ATTR adc_timer_callback(gptimer_handle_t ad_timer, const gptimer_alarm_event_data_t *edata, void *args)
	{
    BaseType_t high_task_awoken = pdFALSE;
    //adc_raw[idx][sample_count++] = adc1_get_raw(channel[idx]);
#if ACTIVE_CONTROLLER == PUMP_CONTROLLER
    adc_oneshot_read(adc1_handle, CURRENT_ADC_CHANNEL, &adc_raw[0][sample_count]);
    adc_oneshot_read(adc1_handle, SENSOR_ADC_CHANNEL, &adc_raw[1][sample_count++]);
#elif ACTIVE_CONTROLLER == WATER_CONTROLLER
    adc_oneshot_read(adc1_handle, PINSENSE_MOT - 1, &adc_raw[0][sample_count++]);
#endif
    if(sample_count >= NR_SAMPLES)
    	{
    	gptimer_stop(adc_timer);
    	adc_msg_t msg;
   	    msg.source = 1;
   	    msg.val = sample_count;
   		xQueueSendFromISR(adc_evt_queue, &msg, NULL);
    	}
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
	}
void config_adc_timer()
	{
	adc_timer = NULL;
	gptimer_config_t gptconf = 	{
								.clk_src = GPTIMER_CLK_SRC_DEFAULT,
								.direction = GPTIMER_COUNT_UP,
								.resolution_hz = 1000000,					//1 usec resolution

								};
	gptimer_alarm_config_t al_config = 	{
										.reload_count = 0,
										.alarm_count = SAMPLE_PERIOD,
										.flags.auto_reload_on_alarm = true,
										};
	gptimer_event_callbacks_t cbs = {.on_alarm = &adc_timer_callback,}; // register user callback
	ESP_ERROR_CHECK(gptimer_new_timer(&gptconf, &adc_timer));
	ESP_ERROR_CHECK(gptimer_set_alarm_action(adc_timer, &al_config));
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(adc_timer, &cbs, NULL));
	ESP_ERROR_CHECK(gptimer_enable(adc_timer));
	}

int get_pump_adc_values(minmax_t *min, minmax_t *max, int *psensor_mv)
	{
	//minmax_t min[10] = {0}, max[10] = {0};
    uint32_t kd, i, l;
    uint32_t minidx, maxidx;
    uint32_t av[8];
    adc_msg_t msg;
    int *c_val, *d_val;
    int sd = 0, ssd = 0, sqrtd = 0;
    int dd[8], avdd[8], ret = ESP_OK;
    int q_wait = SAMPLE_PERIOD * NR_SAMPLES / 1000 + 50;
    /*
     * take adcval mutex in 1.2 sec wait
     * worst case for 10 loops it takes 1.05 sec
     */
    if(xSemaphoreTake(adcval_mutex, ( TickType_t ) 120 ) == pdTRUE)
    	{
		for(l = 0; l < LOOP_COUNT; l++)
			{
			sample_count = 0;
			// 100 msec to collect all samples

			//ESP_LOGI(TAG, "adc loop %lu", l);
			gptimer_set_raw_count(adc_timer, 0);
			gptimer_start(adc_timer);
			if(xQueueReceive(adc_evt_queue, &msg, q_wait / portTICK_PERIOD_MS)) //wait 200 msec ADC conversion to complete
				{
				//calculate stdev on pressure samples
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
				}
#ifdef DEBUG_ADC
			//for(i = 0; i < NR_SAMPLES; i++)
			//	ESP_LOGI(TAG, "%d  %5d %5d %5d %5d", i, adc_raw[0][i], adc_mv[0][i], adc_raw[1][i], adc_mv[1][i]);
			ESP_LOGI(TAG, "noisy loop: %lu -  av / stdev: %d / %d", l, sd, sqrtd);
#endif
			}
    	}
    else
		{
		ESP_LOGI(TAG, "cannot take adcval_mutex");
		return ESP_FAIL;
		}
    xSemaphoreGive(adcval_mutex);
	if(l < LOOP_COUNT)
		{
		//good reading continue processing
		for(i = 0; i < NR_SAMPLES; i++)
			{
			adc_cali_raw_to_voltage(adc1_cal_handle, adc_raw[0][i], &adc_mv[0][i]);
			adc_cali_raw_to_voltage(adc1_cal_handle, adc_raw[1][i], &adc_mv[1][i]);
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
			ESP_LOGI(TAG, "%5lu\t%5lu\t%5lu\t%5lu\t%5lu", i, min[i].index, min[i].mv, max[i].index, max[i].mv);
#endif
		*psensor_mv /= (NR_SAMPLES - 14);
		ret =  ESP_OK;
		}
	else
		{
		ESP_LOGI(TAG, "ADC conversion too noisy");
		ret = ESP_FAIL;
		}
	return ret;
	}

void get_dv_adc_values(int *dv_mv)
	{
	int mv;
	sample_count = 0;
	*dv_mv = 0;
	gptimer_set_raw_count(adc_timer, 0);
	gptimer_start(adc_timer);
	while(sample_count < NR_SAMPLES)
		vTaskDelay(pdMS_TO_TICKS(2));
	for(int i = 0; i < sample_count; i++)
		{
		adc_cali_raw_to_voltage(adc1_cal_handle, adc_raw[0][i], &mv);
		*dv_mv += mv;
		}
	*dv_mv /= sample_count;
	}

static bool adc_calibration_init5(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
	{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    	{

        adc_cali_curve_fitting_config_t cali_config =
        	{
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
        	};
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        	{
            calibrated = true;
            adc1_cal_handle = handle;
        	}
        ESP_LOGI(TAG, "calibration scheme version is Curve Fitting, %d", calibrated);
        }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    	{
        adc_cali_line_fitting_config_t cali_config =
        	{
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        	};
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        	{
            calibrated = true;
            adc1_cal_handle = handle;
        	}
        ESP_LOGI(TAG, "calibration scheme version is Line Fitting, %d", calibrated);
#endif
	return calibrated;
    }

void adc_init5()
	{
	adc1_cal_handle = NULL;
	adc1_handle = NULL;
	adc_evt_queue = xQueueCreate(5, sizeof(adc_msg_t));
	adcval_mutex = xSemaphoreCreateMutex();
	if(!adcval_mutex)
		{
		ESP_LOGE(TAG, "cannot create adcval_mutex");
		esp_restart();
		}
	adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1,};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
	adc_oneshot_chan_cfg_t config =
		{
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.atten = ADC_ATTEN_DB_11,
	    };
#if ACTIVE_CONTROLLER == PUMP_CONTROLLER
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CURRENT_ADC_CHANNEL, &config)); // current channel - pump
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SENSOR_ADC_CHANNEL, &config)); // pressure channel - pump
#elif ACTIVE_CONTROLLER == WATER_CONTROLLER
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PINSENSE_MOT - 1, &config)); // current channel - DV2
    //ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHANNEL_0, &config)); // current channel - DV1
#endif
	adc_calibration_init5(ADC_UNIT_1, 0, ADC_ATTEN_DB_11, &adc1_cal_handle);
	config_adc_timer();
	}
