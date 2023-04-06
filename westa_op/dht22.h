/*
 * dht22.h
 *
 *  Created on: Apr 5, 2023
 *      Author: viorel_serbu
 */

#ifndef WESTA_OP_DHT22_H_
#define WESTA_OP_DHT22_H_

#define DHT_DATA_PIN 	5
#define DHT_RMT_CHANNEL RMT_CHANNEL_4

typedef struct
	{
	double humidity;
	double temperature;
	} dht_data_t;

int dht_init(void);
int get_dht_data(dht_data_t * dhtd);

#endif /* WESTA_OP_DHT22_H_ */
