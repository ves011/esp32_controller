/*
 * dht22.h
 *
 *  Created on: Apr 5, 2023
 *      Author: viorel_serbu
 */

#ifndef WESTA_OP_DHT22_H_
#define WESTA_OP_DHT22_H_

//#define DHT_RMT_CHANNEL RMT_CHANNEL_4

int dht_init(void);
int get_dht_data(th_data_t * dhtd);
//int get_dht_status(void);

#endif /* WESTA_OP_DHT22_H_ */
