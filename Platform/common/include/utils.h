/*******************************************************************************
 * @file:   utils.h
 *******************************************************************************/
/*******************************************************************************
Copyright 2020 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/


#ifndef _UTILS_H
#define _UTILS_H

#include <stdint.h>

#include "gnss_data_api.h"

#define GPS_BUFF_SIZE (2000)
#define IMU_BUFF_SIZE (2000)
#define DEBUG_BUFF_SIZE (2000)


typedef struct
 {
	uint8_t* buffer;
	uint16_t in;
	uint16_t out;
	uint16_t size;
} fifo_type;


void fifo_init(fifo_type* fifo, uint8_t* buffer, uint16_t size);
uint16_t fifo_get(fifo_type* fifo, uint8_t* buffer, uint16_t len);
uint16_t fifo_status(fifo_type* fifo);
void fifo_push(fifo_type* fifo, uint8_t* buffer, uint16_t size);

char *i2a(int num, char *str, int radix);
void float2arr(double data, char *a, unsigned char id, unsigned char dd);

int print_nmea_gga(double *ep, double *xyz, int nsat, int type, double dop, 
	double age, char *buff);
int print_pos_gga(gtime_t time, double *pos, int num_of_sat, int fixID,
	double hdop, double age, char *gga);


#endif /* _UTILS_H */


