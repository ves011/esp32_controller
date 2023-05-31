/*
 * pumpop.h
 *
 *  Created on: Jan 28, 2023
 *      Author: viorel_serbu
 */

/**
 * @file pumpop.h
 * @brief header file definitions for control logic of submersible water pump
 */
#ifndef PUMP_OP_PUMPOP_H_
#define PUMP_OP_PUMPOP_H_

/**
 * @defgroup  ESP32_GPIO pump ctrl GPIO
 *
 * @{
 */
 /** Pressure sensor */
#define SENSOR_PIN				(5)	//j5/5
 /** Pressure sensor ADC channel = SENSOR_PIN -1*/
#define	SENSOR_ADC_CHANNEL		(4)
 /** ACS712 current sensor */
#define CURRENT_PIN				(4)	//j5/6
 /** ACS712 current sensor ADC channel = CURRENT_PIN - 1*/
#define CURRENT_ADC_CHANNEL		(3)
 /** Pump ON/OF control pin */
#define PUMP_ONOFF_PIN			(6)	//j5/8
/** @} */

	#define PUMP_ON					(1)
	#define PUMP_OFF				(0)
	#define PUMP_FAULT				(4)
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
/** How long(msec) the button has to be pressed  to send the command*/
#define PUSH_TIME_MS			(3000)

/** under this value the pump is OFF */
#define PUMP_CURRENT_OFF		300

 /** Name of the file storing pressure sensor output (mV) for 0kPA */
#define OFFSET_FILE				"psensor_voffset.txt"
 /** Name of the file storing operating limits for pump - see pump_limits definition */
#define LIMITS_FILE				"pump_limits.txt"
 /** Name of the file storing pump status: online | offline */
#define OPERATIONAL_FILE		"pump_status.txt"

#define DEFAULT_PRES_MIN_LIMIT		300
#define DEFAULT_PRES_MAX_LIMIT		370
#define DEFAULT_PUMP_CURRENT_LIMIT	5000
#define DEFAULT_STDEV				20
#define DEFAULT_OVERP_TIME_LIMIT	10


/**
 * @brief pump command and parameters
 */
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

/**
 * @brief pump limits structure passed to rw_params() function.
 */
typedef struct
	{
	uint32_t min_val;			/*!< min pressure limit										*/
	uint32_t max_val;			/*!< max pressure limit										*/
	uint32_t faultc;			/*!< max acceptable current when pump running (mA)			*/
	uint32_t stdev;				/*!< max acceptable stdev for pressure measurements
										below this limit the measurement is OK
										above this limit measurement is rejected: too noisy	*/
	uint32_t overp_lim;			/*!< how long the pressure needs to be above max_val before the pump to be stopped (sec) */
	} pump_limits_t;

extern int pump_pres_stdev;

/**
 * @brief dispatch pump commands based on arguments
 * @return 0 on success, 1 on error
 */
int do_pumpop(int argc, char **argv);

/**
 * @brief gets the pump state and publish response on state topic
 * @return ESP_OK or ESP_FAIL
 */
int get_pump_state(void);

/**
 * @brief start pump
 * @param from
 * 		if from == 1 function is called inside pump monitor loop
 * @return 	ESP_OK on success
 * 			ESP_FAIL on error
 */
int start_pump(int from);

/**
 * @brief stop pump
 * @param from
 * 		if from == 1 function is called inside pump monitor loop
 * @return 	ESP_OK on success
 * 			ESP_FAIL on error
 */
int stop_pump(int from);
void register_pumpop();
int set_pump_0_offset(void);
int pump_operational(int po);
void pump_mon_task(void *pvParameters);
void process_adc_current(minmax_t *min, minmax_t *max);


#endif /* PUMP_OP_PUMPOP_H_ */
