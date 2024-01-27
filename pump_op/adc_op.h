/*
 * adc_op.h
 *
 *  Created on: Feb 3, 2023
 *      Author: viorel_serbu
 */

#ifndef MAIN_ADC_OP_H_
#define MAIN_ADC_OP_H_

#define TIMER_SCALE_ADC           	(TIMER_BASE_CLK / TIMER_DIVIDER / 1000000)  // convert counter value to useconds
#define SAMPLE_PERIOD				500
#if ACTIVE_CONTROLLER == WATER_CONTROLLER
	#define NR_SAMPLES					50
#else
	#define NR_SAMPLES					200
#endif
#define ADC_TIMER_GROUP				TIMER_GROUP_1
#define ADC_TIMER_INDEX				TIMER_1

#define LOOP_COUNT					10
#define STDEV_THERSHOLD				30

#define ADC_RESULT_BYTE    			4
#define ADC_CONV_LIMIT_EN   		0

#define GET_UNIT(x)        ((x>>3) & 0x1)


/* ACS712 20A --> 100mv/A
 * min resolution = 15mV --> 150mA
 */

/*
 * val (mV) = (val(raw) * CAL_A) / SCALE_A + CAL_B
 * for ADC raw values below break point to use cal_a1 and cal_b1 coeffs
 * above break point to use cal_a2 and cal_b2
 * coefficients below are for MAC: 68:b6:b3:29:77:18
 */
 /*
#define ADC_LINEAR_BREAK 2900

#define SCALE_A		1000000
#define CAL_A1		828668
#define CAL_B1		36
#define CAL_A2		603010
#define CAL_B2		694
*/

typedef struct
	{
	uint8_t source;
	uint8_t step;
	uint32_t val;
	uint64_t ts;
	} adc_msg_t;

typedef struct
    {
    uint32_t mv;
    uint32_t index;
    } minmax_t;

extern int stdev_c, stdev_p;

void init_adc(void);
//void adc_calibration_init(void);
void adc_init5(void);
void config_adc_timer(void);
int get_pump_adc_values(minmax_t *min, minmax_t *max, int *psensor_mv);
void get_dv_adc_values(int *dv_mv);

#endif /* MAIN_ADC_OP_H_ */
