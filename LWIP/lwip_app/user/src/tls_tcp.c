#include "tls_tcp.h"
#include "station_tcp.h"

void tls_tcp_link_down(TLS_TCP_STATE *state)
{
    if (*state >= TLS_TCP_CONNECT && *state <= TLS_TCP_INTERACTIVE) {
        *state = TLS_TCP_LINK_DOWN;
    }
}

uint8_t tls_tcp_is_interactive(TLS_TCP_STATE *state)
{
    return (*state == TLS_TCP_INTERACTIVE);
}


int32_t tls_tcp_write(mbedtls_ssl_context *ssl, const unsigned char *buf, size_t len)
{
    int32_t ret;

    while ((ret = mbedtls_ssl_write(ssl, buf, len)) <= 0)
    {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
        {
            return -1;
        }
    }
    return 0;
}

int32_t tls_tcp_push_rx_data(mbedtls_ssl_context *ssl, uint8_t *buf, uint16_t size, fifo_type *rx_fifo)
{
    int32_t ret;
    uint16_t len = 0;

    ret = mbedtls_ssl_read(ssl, buf, size);

    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
        return ret;
    }

    if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) {
        return ret;
    }

    if (ret < 0) {  // mbedtls_ssl_read failed
        return ret;
    }

    if (ret == 0) { // EOF
        return ret;
    }

    taskENTER_CRITICAL();

    fifo_push(rx_fifo, buf, ret);

    taskEXIT_CRITICAL();

	return ret;
}

uint8_t tls_tcp_push_tx_data(TLS_TCP_STATE *state, fifo_type *tx_fifo, uint8_t *buf, uint16_t len)
{
    if (tls_tcp_is_interactive(state)) {
        taskENTER_CRITICAL();
        fifo_push(tx_fifo, buf, len);
        taskEXIT_CRITICAL();
        return 1;
    }
    return 0;
}

uint16_t tls_tcp_get_rx_data(TLS_TCP_STATE *state, fifo_type *rx_fifo, uint8_t* buf, uint16_t len)
{
    if (tls_tcp_is_interactive(state)) {
        return fifo_get(rx_fifo, buf, len);
    }
    return 0;
}
