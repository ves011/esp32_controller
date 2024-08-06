/*
 * water_op.h
 *
 *  Created on: Jun 8, 2023
 *      Author: viorel_serbu
 */

#ifndef WATER_OP_WATEROP_H_
#define WATER_OP_WATEROP_H_

#define DV0						0
#define DV1						1
#define DVCOUNT					2
/*
	#define PINEN_DV0				(6)
	#define PINEN_DV1				(7)
*/
	/* A1 ON 	/ B1 OFF 	--> open
	 * A1 OFF 	/ B1 ON 	--> close
	 * A1 OFF 	/ B1 OFF 	--> inactive
	 */
/*
	#define PINMOT_A1				(8)
	#define PINMOT_B1				(3)
	//#define PINMOT_A2				(12)
	//#define PINMOT_B2				(11)
	#define PINSENSE_MOT			(4)
*/
	#define CURRENT_OFF_LIM			20
	#define CURRENT_OFF_COUNT		3


#define WATER_OFF				0
#define WATER_ON				1

#define STDEV_MAX				10
#define DVSTATE_OK				1
#define DVSTATE_FAULT			2
#define DVOPEN					1
#define DVCLOSE					0
#define DVOP_INPROGRESS			2
#define MINPRES					100

/** Name of the file storing dv program */
#define PROGRAM_FILE		"dv_program.txt"
#define STATUS_FILE			"program_status.txt"

/** watering program state */
#define NOT_STARTED				0
#define IN_PROGRESS				1
#define COMPLETED				2
#define ABORTED					3
#define START_ERROR				4
#define INVALID					5  // if start time is after stop time

#define RETRY_OP_WATERING		5
#define NO_PUMP_RESPONSE		6
#define PUMP_WRONG_STATE		7
#define DV_ERROR				8
#define PUMP_PRESSURE_LOW		9
#define START_WATERING_ERROR	10
#define STOP_WATERING_ERROR		11

#define RESET_PROGRAM_H			15
#define RESET_PROGRAM_M			22

#define FAULT_PUMP				10
#define DV_FAULT				11
#define PUMP_NO_RESPONSE		12
#define DV_OPEN_FAIL			13
#define DV_CLOSE_FAIL			14
#define GET_STATE_NOT_PERMITED	15
#define GET_STATE_ERROR			16
#define OP_ABORTED				17


#define WATER_PUMP_DESC			"pump01"
#define PUMP_CMD_TOPIC			"pump01/cmd"

#define WATERING_STATE			"wstate"
#define PUMP_STATE				"pstate"
#define PROG_HISTORY			"phist"
#define ERR_START				"err_start"
#define ERR_STOP				"err_stop"

#define STATE_W					"state"
#define STATE_P					"program"



typedef struct
	{
	uint8_t dvno;
	uint8_t pin_current;
	uint8_t pin_led;
	uint8_t state;
	uint8_t status;
	uint16_t off_current;
	} dvconfig_t;

typedef struct
		{
		struct
			{
			int dv;
			int starth;
			int startm;
			int stoph;
			int stopm;
			int cs;
			int fault;
			} p[DVCOUNT];
		} dvprogram_t;


int do_dvop(int argc, char **argv);
void register_waterop(void);
void get_dv_current(int *dv_current, int *stdev);
void parse_devstr(int argc, char **argv);

#endif /* WATER_OP_WATEROP_H_ */
