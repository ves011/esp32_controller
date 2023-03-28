/*
 * pumpop.h
 *
 *  Created on: Jan 28, 2023
 *      Author: viorel_serbu
 */

#ifndef PUMP_OP_PUMPOP_H_
#define PUMP_OP_PUMPOP_H_

#define SENSOR_PIN				(5)	//j5/5
#define	SENSOR_ADC_CHANNEL		(4)
#define CURRENT_PIN				(4)	//j5/6
#define CURRENT_ADC_CHANNEL		(3)
#define PUMP_ONOFF_PIN			(6)	//j5/8

	#define PUMP_ON					(1)
	#define PUMP_OFF				(0)
	#define PUMP_FAULT				(2)
	#define PUMP_ONLINE				(2)
	#define PUMP_OFFLINE			(3)

#define PUMP_ONOFF_LED			(1)	//j5/4
#define PUMP_ONLINE_LED			(8)	//j4/8

#define PUMP_FAULT_LED			(3)	//j4/7
#define PUMP_ONLINE_CMD			(7)	//j5/7

#define ADC_TIMER_GROUP			TIMER_GROUP_1
#define ADC_TIMER_INDEX			TIMER_1

#define CMD_TIMER_GROUP			TIMER_GROUP_1
#define CMD_TIMER_INDEX			TIMER_0
#define PUSH_TIME_MS			(3000)

#define OFFSET_FILE				"psensor_voffset.txt"
#define LIMITS_FILE				"pump_limits.txt"
#define OPERATIONAL_FILE		"pump_status.txt"

struct
	{
    struct arg_str *op;
    struct arg_int *minP;
    struct arg_int *maxP;
    struct arg_int *faultC;
    struct arg_int *stdev;
    struct arg_int *overpt;
    struct arg_end *end;
	} pumpop_args;

typedef struct
	{
	int v_offset;
	} psensor_offset_t;

typedef struct
	{
	uint32_t min_val;
	uint32_t max_val;
	uint32_t faultc;
	uint32_t stdev;
	uint32_t overp_lim;
	uint32_t pump_op_response;
	} pump_limits_t;

extern int pump_pres_stdev;

int do_pumpop(int argc, char **argv);
int get_pump_state(void);
int start_pump(int from);
int stop_pump(int from);
void register_pumpop();
int read_pump_adc(void);
int set_pump_0_offset(void);
int pump_operational(int po);
void pump_mon_task(void *pvParameters);
void process_adc_current(minmax_t *min, minmax_t *max);
void publish_MQTT_client_status(void);


#endif /* PUMP_OP_PUMPOP_H_ */
