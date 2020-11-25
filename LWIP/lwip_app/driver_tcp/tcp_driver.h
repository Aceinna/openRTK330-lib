#ifndef _TCP_DRIVER_H_
#define _TCP_DRIVER_H_

#include <stdio.h>

#include "lwip/err.h"
#include "lwip/api.h"
#include "lwip/opt.h"
#include "lwip_comm.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "utils.h"

#define driver_server_ip "192.168.1.1"

#define driver_client_port  2203
#define driver_data_client_port  2204

#define DRIVER_TX_BUFSIZE (4*1024)
#define DRIVER_RX_BUFSIZE 500


typedef enum
{
    CLIENT_STATE_OFF                 = 0,
    CLIENT_STATE_CONNECT             = 1,
    CLIENT_STATE_REQUEST             = 2,
    CLIENT_STATE_INTERACTIVE         = 3,
    CLIENT_STATE_TIMEOUT             = 4,
    CLIENT_STATE_LINK_DOWN           = 5
} driver_client_state_enum_t;


typedef struct CLIENT_S_
{
    struct netconn *client;
    uint8_t client_state;
    fifo_type   client_tx_fifo;
    fifo_type   client_rx_fifo;
    osMutexId   tx_fifo_mutex;
} client_s;

void driver_interface(void);
void driver_output_data_interface(void);
err_t client_read_data(client_s* client, uint8_t *rx_buf, uint16_t *rx_len);
void tcp_driver_fifo_init();
void tcp_driver_data_fifo_init();
uint8_t get_tcp_driver_state();
uint8_t get_tcp_data_driver_state();
uint8_t driver_data_push(uint8_t* buf, uint16_t len);
uint8_t driver_push(uint8_t* buf, uint16_t len);
err_t client_write_data(client_s* client, const uint8_t *tx_buf, uint16_t tx_len, uint8_t apiflags);
void set_server_ip(ip_addr_t* value);

#endif
