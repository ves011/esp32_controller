/*
 * westaop.h
 *
 *  Created on: Apr 2, 2023
 *      Author: viorel_serbu
 */

#ifndef WESTA_OP_WESTAOP_H_
#define WESTA_OP_WESTAOP_H_

#define I2C_MASTER_SCL_IO           4      						/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           1      						/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define BMP280_I2C_ADDRESS			0x76

#define PTH_POLL_INT				2000 / portTICK_PERIOD_MS //1800000 / portTICK_PERIOD_MS

#define DEFAULT_PSL					1013.25

struct
	{
    struct arg_str *dst;
    struct arg_str *op;
    struct arg_int *os_mode;
    struct arg_int *filter;
    struct arg_int *odr;
    struct arg_int *power_mode;
    struct arg_end *end;
	} westaop_args;

extern double psl;

void register_westaop(void);
int do_westaop(int argc, char **argv);

#endif /* WESTA_OP_WESTAOP_H_ */
