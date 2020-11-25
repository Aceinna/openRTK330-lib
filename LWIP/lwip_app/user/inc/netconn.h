#ifndef _NETCONN_H_
#define _NETCONN_H_

#include <stdio.h>
#include "lwip/err.h"
#include "lwip/api.h"
#include "lwip/opt.h"
#include "lwip_comm.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "utils.h"

// netconn client state
typedef enum
{
    NETCONN_STATE_OFF               = 0,
    NETCONN_STATE_CONNECT           = 1,
    NETCONN_STATE_REQUEST           = 2,
    NETCONN_STATE_INTERACTIVE       = 3,
    NETCONN_STATE_TIMEOUT           = 4,
    NETCONN_STATE_LINK_DOWN         = 5
} NETCONN_STATE;

err_t netconn_read(struct netconn *conn, uint8_t *buf, uint16_t *len, uint16_t max_length);
err_t netconn_push_rx_data(struct netconn *conn, fifo_type *rx_fifo);

uint8_t netconn_push_tx_data(NETCONN_STATE *state, fifo_type *tx_fifo, uint8_t *buf, uint16_t len);
uint16_t netconn_get_rx_data(NETCONN_STATE *state, fifo_type *rx_fifo, uint8_t *buf, uint16_t len);

void netconn_check_ret(err_t err, NETCONN_STATE *state);
void netconn_link_down(NETCONN_STATE *state);
uint8_t netconn_is_interactive(NETCONN_STATE *state);

#endif /* _TCP_NETCONN_H_ */
