#ifndef _NTRIP_CLIENT_H_
#define _NTRIP_CLIENT_H_

#include <stdio.h>
#include "netconn.h"

int32_t ntrip_client_request(struct netconn *conn, NETCONN_STATE *state, uint8_t *buf, uint16_t size);

#endif
