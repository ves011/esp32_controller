/*
 * aht20.h
 *
 *  Created on: Nov 9, 2024
 *      Author: viorel_serbu
 */

#ifndef WESTA_OP_AHT20_H_
#define WESTA_OP_AHT20_H_

#define NRETRY						3
#define AHT20_I2C_ADDRESS			0x38
#define SOFT_RESET_CMD				0xba
#define GET_STATUS_WORD				0x71
#ifndef CALIBRATION_CMD
	#define CALIBRATION_CMD				(uint8_t[3]){0xbe, 0x08, 0x00}
#endif
#ifndef GATDATA_CMD
	#define GET_DATA_CMD				(uint8_t[3]){0xac, 0x33, 0x00}
#endif
#define CALIBRATION_BIT				1 << 3
#define COMPLETE_BIT				1 << 7

int aht20_init(i2c_master_dev_handle_t handle);
int get_aht_data(th_data_t *ahtdata);


#endif /* WESTA_OP_AHT20_H_ */
