/*
 * project_specific.h
 *
 *  Created on: Mar 20, 2023
 *      Author: viorel_serbu
 */

/**
 * @file project_specific.h
 * @brief defines the controller type: ACTIVE_CONTROLLER and ID: CTRL_DEV_ID
 */
#ifndef MAIN_PROJECT_SPECIFIC_H_
#define MAIN_PROJECT_SPECIFIC_H_
#include "common_defines.h"
#define TEST_BUILD (1)
#if(TEST_BUILD == 1)
	#define WITH_CONSOLE
	#define TEST1
	#define CTRL_DEV_ID					(99)
	#define LOG_PORT_DEV				8081
	#define LOG_SERVER_DEV				"proxy.gnet"
#else
	#define CTRL_DEV_ID					(1)
#endif
#define WITH_CONSOLE
#define COMM_PROTO	MQTT_PROTO
#define OTA_SUPPORT



#define ACTIVE_CONTROLLER			(WMON_CONTROLLER)

#define WIFI_STA_ON 					(1)
#define MQTT_PUBLISH					(1)

#if ACTIVE_CONTROLLER == (AGATE_CONTROLLER)
	#define DEV_NAME					"Poarta Auto"
	#define PROMPT_STR "AGATE"
#elif ACTIVE_CONTROLLER == WESTA_CONTROLLER
	#define DEV_NAME					"Statia meteo"
	#define PROMPT_STR "WESTA"
	#define USE_I2C
#elif ACTIVE_CONTROLLER == OTA_CONTROLLER
	#define DEV_NAME					"OTA controller"
	#define PROMPT_STR "OTADEV"
#elif ACTIVE_CONTROLLER == WMON_CONTROLLER
	#define DEV_NAME					"WMON controller"
	#define PROMPT_STR "WMON"
#endif
/*
Message definitions for device monitor queue
*/
#define MSG_WIFI			1	// wifi connect (.val = 1)/disconnect (.val = 0) event 
#define MSG_BAT				2	// battery level .val = ADC battery measurement * 1000
#define MSG_LED_FLASH		3	// nw state and remote state flashing
#define NW_STATE_CHANGE		4	// nw connected (.val = 1) / disconnected (.val = 0)
#define REMOTE_STATE_CHANGE	5	// remote connected (.val = 1) / disconnected (.val = 0)
#define INIT_COMPLETE		6	// init completed


#endif /* MAIN_PROJECT_SPECIFIC_H_ */
