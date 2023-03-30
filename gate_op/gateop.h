/*
 * gateop.h
 *
 *  Created on: Sep 28, 2022
 *      Author: viorel_serbu
 */

/**
 * @file gateop.h
 * @brief header file definitions for control logic of sliding gate
 */

#ifndef GATEOP_H_
#define GATEOP_H_

#define GATEOPEN			(1)
#define GATECLOSED			(0)
#ifdef ESP32_DEV_KIT
	#define GATE_PIN_OPEN		(32)
	#define GATE_PIN_CLOSE		(33)
	#define GATE_PIN_SIGNAL		(19)
	#define GATE_PIN_SENSE		(18)
	#define PCNT_PIN_CTRL		(12)
#else
	#define GATE_PIN_OPEN		(7)
	#define GATE_PIN_CLOSE		(4)
	#define GATE_PIN_SIGNAL		(13)
	#define GATE_PIN_SENSE		(5)	//OGI
	#define PCNT_PIN_CTRL		(14)
#endif

#define PW_TIMER_GROUP				TIMER_GROUP_1
#define PW_TIMER_INDEX				TIMER_0
#define STEADY_TIMER_GROUP			TIMER_GROUP_1
#define STEADY_TIMER_INDEX			TIMER_1


//#define OPEN_PW				(800)/* open pulse width in msec */
//#define CLOSE_PW			(400) /* close pulse width in msec */
#define CMD_PW				(300) /* command pulse width in msec */

#define CMD_OPEN			(0)
#define CMD_CLOSE			(1)
#define CMD_COMPLETE		(2)

#define OPEN_GATE_ACK		(1)
#define OPEN_GATE_NACK		(2)
#define CLOSE_GATE_ACK		(3)
#define CLOSE_GATE_NACK		(4)
#define OP_GATE_UNKNOWN		(5)


#define STATE_OPENP			(1)
#define STATE_OPEN			(2)
#define STATE_CLOSED		(3)

#define STEADY_STATE		(1)
#define MOVING_STATE		(2)

#define PULSE_COUNT_OPENF		(26)
#define PULSE_COUNT_OPENP		(5)

#define PULSE_COUNT_CLOSEF		(52)
#define PULSE_COUNT_CLOSEP		(10)
#define CMD_SOURCE				(1)
#define STEADY_TIMER_SOURCE		(2)

#define PIN_LIST { GATE_PIN_OPEN, GATE_PIN_CLOSE, GATE_PIN_SIGNAL, GATE_PIN_SENSE}
#define GPIO_OUTPUT_PIN_SEL		((1ULL << GATE_PIN_OPEN) | (1ULL << GATE_PIN_CLOSE) | (1UL << GATE_PIN_SIGNAL))
#define GPIO_INPUT_PIN_SEL		((1ULL << GATE_PIN_SENSE) | (1ULL << PCNT_PIN_CTRL))

struct {
    struct arg_str *op;
    struct arg_int *t;
    struct arg_end *end;
} gateop_args;

typedef struct
	{
	uint32_t pulse_count;
	uint32_t moving_state;
	uint32_t op_response;
	uint32_t source;
	uint64_t time;
	} gate_evt_t;

int open_gate();
int close_gate();
int sense_state();
int get_gate_state(void);
int do_gateop(int argc, char **argv);
void register_gateop();



#endif /* GATEOP_H_ */
