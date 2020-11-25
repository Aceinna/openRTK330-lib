#ifndef _NTRIP_SERVER_H_
#define _NTRIP_SERVER_H_

#include <stdio.h>
#include "netconn.h"

void base_station_run_update(void);
uint8_t base_station_get_run_status(void);
uint8_t station_position_calc(double lat, double lon, double height, uint8_t mode);
double get_station_pos_lat(void);
double get_station_pos_lon(void);
double get_station_pos_height(void);

int32_t ntrip_server_request(struct netconn *conn, NETCONN_STATE *state, uint8_t *buf, uint16_t size);
void ntrip_server_send_pos(struct netconn *conn, NETCONN_STATE *state);

#endif
