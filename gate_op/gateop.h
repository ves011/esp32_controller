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
	#define GATE_PIN_SENSE		(18)
#else
	#define GATE_PIN_OPEN		(7)
	#define GATE_PIN_CLOSE		(4)
	#define GATE_PIN_SENSE		(5)	//OGI
#endif

#define CMD_PW				(300000) /* command pulse width in msec */

#define CMD_OPEN			(0)
#define CMD_CLOSE			(1)
#define CMD_COMPLETE		(2)

#define STATE_OPENP			(1)
#define STATE_OPEN			(2)
#define STATE_CLOSED		(3)

#define STEADY_STATE		(1)
#define MOVING_STATE		(2)
#define OPEN_IN_PROGRESS	(3)
#define CLOSE_IN_PROGRESS	(4)

#define PULSE_COUNT_STEADY_OPEN		2100
#define PULSE_COUNT_STEADY_CLOSE	-100
#define PULSE_COUNT_OPEN			1000
#define PULSE_COUNT_CLOSE			400

#define PC_TIMER_SOURCE			(1)

#define GPIO_OUTPUT_PIN_SEL		((1ULL << GATE_PIN_OPEN) | (1ULL << GATE_PIN_CLOSE))
#define GPIO_INPUT_PIN_SEL		(1ULL << GATE_PIN_SENSE)

typedef struct
	{
	int pulse_count;
	uint32_t moving_state;
	uint32_t source;
	uint64_t time;
	} gate_evt_t;

//void open_gate();
//void close_gate();
//void get_gate_state(void);
int do_gateop(int argc, char **argv);
void register_gateop();



#endif /* GATEOP_H_ */
