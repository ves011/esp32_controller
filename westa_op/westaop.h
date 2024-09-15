/*
 * westaop.h
 *
 *  Created on: Apr 2, 2023
 *      Author: viorel_serbu
 */
/**
 * @file westa.h
 * @brief header file definitions for weather station operation
 */

#ifndef WESTA_OP_WESTAOP_H_
#define WESTA_OP_WESTAOP_H_

#define I2C_MASTER_SCL_IO           4      					/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           1      					/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                  /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

 /** address of BMP280 device */
#define BMP280_I2C_ADDRESS			0x76
 /** reading interval of pressure temp and humidity */
//#define PTH_POLL_INT				10000 	/ portTICK_PERIOD_MS 	// 10 sec
//#define PTH_POLL_INT				1800000 / portTICK_PERIOD_MS 	//30 mins
#define PTH_POLL_INT				1800 	//30 mins
//#define PTH_POLL_INT				30 	//30 sec
 /** default normal sea level pressure */
#define DEFAULT_PSL					1013.25
 /** default measuring point elevationA */
#define DEFAULT_ELEVATION			87.5

 /** Name of the file storing normal pressure calculation: <measuring point xx.xx> altitude <sea level pressure xxxx.xxx> */
#define PNORM_FILE		"pnorm.txt"

typedef struct
	{
	double psl;
	double hmp;
	} pnorm_param_t;

extern SemaphoreHandle_t pthfile_mutex;

void register_westaop(void);
/*
 * @brief command line function called by console_repl or MQTT client.
 *    westa bmp read|state|set|pset -> ops addressed to BMP280 sensor
 *    westa dht read -> op addressed to DHT22 sensor
 *    westa range <[start_date]>[end_date]> <average_points>
 *        date format is yyyy-mm-dd/hh:mm:ss
 *        no spaces are allowed within parameters
 *        date to by filled from left to right; missing part will take 0 value
 *        <average_points> provides the average value for given number of points
 *           value is reported for the middle of the interval
 */
int do_westaop(int argc, char **argv);

#endif /* WESTA_OP_WESTAOP_H_ */
