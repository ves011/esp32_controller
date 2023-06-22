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

#define PINOP_DV1				(5)
#define PINCURRENT_DV1			(1)
#define PINLED_DV1				(12)
#define PINCMD_DV1				(11)
#define INPUTPINS_DV1			((1ULL << PINCURRENT_DV1) | (1ULL << PINCMD_DV1))
#define OUTPUTPINS_DV1			((1ULL << PINLED_DV1) | (1ULL << PINOP_DV1))

#define PINOP_DV2				(7)
#define PINCURRENT_DV2			(4)
#define PINLED_DV2				(8)
#define PINCMD_DV2				(3)
#define INPUTPINS_DV2			((1ULL << PINCURRENT_DV2) | (1ULL << PINCMD_DV2))
#define OUTPUTPINS_DV2			((1ULL << PINLED_DV2) | (1ULL << PINOP_DV2))

#define WATER_OFF				0
#define WATER_ON				1

#define STDEV_MAX				10
#define DVSTATE_OK				1
#define DVSTATE_FAULT			2
#define DVOPEN					1
#define DVCLOSE					0
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


#define WATER_PUMP_DESC			"pump01"
#define PUMP_CMD_TOPIC			"pump01/cmd"

typedef struct
	{
	uint8_t dvno;
	uint8_t pin_op;
	uint8_t pin_current;
	uint8_t pin_cmd;
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

struct {
    struct arg_str *op;
    struct arg_int *dv;
    struct arg_str *start;
    struct arg_str *stop;
    struct arg_end *end;
} waterop_args;

int do_dvop(int argc, char **argv);
void register_waterop(void);
void get_dv_current(int *dv_current, int *stdev);
void parse_devstr(int argc, char **argv);

#endif /* WATER_OP_WATEROP_H_ */
