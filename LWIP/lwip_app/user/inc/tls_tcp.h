#ifndef _TLS_TCP_H_
#define _TLS_TCP_H_

#include <stdio.h>
#include "lwip/err.h"
#include "lwip/api.h"
#include "lwip/opt.h"
#include "lwip_comm.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "utils.h"
#include "mbedtls_config.h"
#include "mbedtls/platform.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"
#include "mbedtls/memory_buffer_alloc.h"

typedef enum
{
    TLS_TCP_OFF               = 0,
    TLS_TCP_CONNECT           = 1,
    TLS_TCP_HANDSHAKE         = 2,
    TLS_TCP_REQUEST           = 3,
    TLS_TCP_INTERACTIVE       = 4,
    TLS_TCP_LINK_DOWN         = 5
} TLS_TCP_STATE;

typedef struct {
    mbedtls_net_context server_fd;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_ssl_context ssl;
    mbedtls_ssl_config conf;
} TLS_TCP;


void tls_tcp_link_down(TLS_TCP_STATE *state);
uint8_t tls_tcp_is_interactive(TLS_TCP_STATE *state);

int32_t tls_tcp_write(mbedtls_ssl_context *ssl, const unsigned char *buf, size_t len);
int32_t tls_tcp_push_rx_data(mbedtls_ssl_context *ssl, uint8_t *buf, uint16_t size, fifo_type *rx_fifo);
uint8_t tls_tcp_push_tx_data(TLS_TCP_STATE *state, fifo_type *tx_fifo, uint8_t *buf, uint16_t len);
uint16_t tls_tcp_get_rx_data(TLS_TCP_STATE *state, fifo_type *rx_fifo, uint8_t* buf, uint16_t len);

#endif /* _TLS_TCP_H_ */
