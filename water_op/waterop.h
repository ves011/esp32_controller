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

/** Name of the file storing dv program */
#define PROGRAM_FILE		"dv_program.txt"
#define STATUS_FILE			"program_status.txt"

/** watering program state */
#define NOT_STARTED				0
#define IN_PROGRESS				1
#define COMPLETED				2
#define ABORTED					3
#define INVALID					4  // if start time is after stop time

#define WATER_PUMP_DESC			"pump01"
#define PUMP_CMD_TOPIC			"pump01/cmd"

typedef struct
	{
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
			} p[2];
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
