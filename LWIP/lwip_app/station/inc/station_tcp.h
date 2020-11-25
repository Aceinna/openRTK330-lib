#ifndef _STATION_TCP_H_
#define _STATION_TCP_H_

#include <stdio.h>
#include "utils.h"
#include "netconn.h"
#include "tls_tcp.h"

// #define STATION_TCP_DEBUG

#define STATION_TCP_STREAM_MAX_TIMEOUT  30

typedef enum
{
    STA_WORK_NONE               = 0,
    STA_WORK_NTRIP_CLIENT       = 1,
    STA_WORK_ACEINNA_CLIENT     = 2,
    STA_WORK_NTRIP_SERVER       = 3,
    STA_WORK_MAX                = 4
} STATION_WORK_MODE;

#pragma pack(4)

typedef struct {
    struct netconn *p_netconn;
    NETCONN_STATE state;

    uint32_t timeout;
    fifo_type tx_fifo;
    fifo_type rx_fifo;
    uint8_t run;

    TLS_TCP tls_tcp;
    TLS_TCP_STATE tls_state;
} STATION_TCP;

#pragma pack()

extern int32_t sta_status;

extern STATION_TCP *p_sta_tcp;

#define STA_TCP_TXBUF_SIZE  2048
#define STA_TCP_RXBUF_SIZE  2048
extern uint8_t sta_tcp_txbuf[STA_TCP_TXBUF_SIZE];
extern uint8_t sta_tcp_rxbuf[STA_TCP_RXBUF_SIZE];

#define STA_TCP_SEND_BUFFER_SIZE  1024
extern uint8_t sta_tcp_send_buffer[STA_TCP_SEND_BUFFER_SIZE];

uint8_t station_tcp_is_interactive(void);
uint8_t station_tcp_is_stream_available(void);
void station_tcp_add_stream_timeout(void);
void station_tcp_clear_stream_timeout(void);
void station_tcp_stop(void);
void station_tcp_send_data(uint8_t* buf, uint16_t len);
uint16_t station_tcp_get_data(uint8_t* buf, uint16_t len);

void station_tcp_interface(void);

#endif
