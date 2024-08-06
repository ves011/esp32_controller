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

#define TEST_BUILD 1
#if(TEST_BUILD == 1)
	#define WITH_CONSOLE
	#define TEST1
	#define CTRL_DEV_ID					100
#else
	#define CTRL_DEV_ID					1
#endif

#define ACTIVE_CONTROLLER			AGATE_CONTROLLER


#endif /* MAIN_PROJECT_SPECIFIC_H_ */
