#include "station_tcp.h"
#include "stm32f4xx_hal.h"
#include "ntrip_client.h"
#include "ntrip_server.h"
#include "osapi.h"
#include "lwip_comm.h"
#include "user_config.h"
#include "string.h"
#include "lwip/ip.h"
#include "lwip/tcp.h"
#include "station_tcp.h"
#include "aceinna_client_api.h"

STATION_TCP sta_tcp;
STATION_TCP *p_sta_tcp = &sta_tcp;

uint8_t sta_tcp_txbuf[STA_TCP_TXBUF_SIZE];
uint8_t sta_tcp_rxbuf[STA_TCP_RXBUF_SIZE];

uint8_t sta_tcp_send_buffer[STA_TCP_SEND_BUFFER_SIZE];

int32_t sta_status = -1;

uint8_t station_tcp_is_interactive(void)
{
    if (get_station_mode() == MODE_OPENARC_CLIENT && is_aceinna_client_tls()) {
        return tls_tcp_is_interactive(&p_sta_tcp->tls_state);
    } else {
        return netconn_is_interactive(&p_sta_tcp->state);
    }
}

uint8_t station_tcp_is_stream_available(void)
{
    return (p_sta_tcp->timeout < STATION_TCP_STREAM_MAX_TIMEOUT);
}

void station_tcp_add_stream_timeout(void)
{
    p_sta_tcp->timeout++;
}

void station_tcp_clear_stream_timeout(void)
{
    p_sta_tcp->timeout = 0;
}

void station_tcp_stop(void)
{
    p_sta_tcp->run = 0;
}

void station_tcp_send_data(uint8_t *buf, uint16_t len)
{
    if (get_station_mode() == MODE_OPENARC_CLIENT && is_aceinna_client_tls()) {
        tls_tcp_push_tx_data(&p_sta_tcp->tls_state, &p_sta_tcp->tx_fifo, buf, len);
    } else {
        netconn_push_tx_data(&p_sta_tcp->state, &p_sta_tcp->tx_fifo, buf, len);
    }
}

uint16_t station_tcp_get_data(uint8_t *buf, uint16_t len)
{
    if (get_station_mode() == MODE_OPENARC_CLIENT && is_aceinna_client_tls()) {
        return tls_tcp_get_rx_data(&p_sta_tcp->tls_state, &p_sta_tcp->rx_fifo, buf, len);
    }

    return netconn_get_rx_data(&p_sta_tcp->state, &p_sta_tcp->rx_fifo, buf, len);
}

static void station_tcp_thread(STATION_WORK_MODE mode)
{
    const char *host = NULL;
    uint16_t port = 0;
    ip_addr_t ip;
    uint16_t len;
    uint8_t write_fail = 0;
    int32_t res = -1;
    err_t err = ERR_OK;
    uint16_t interactive = 0;
    uint8_t signout_msg = 0;
    uint8_t connect_flag = 0;

    sta_status = -1;

    if (mode >= STA_WORK_MAX) {
        return;
    }

    memset(sta_tcp_send_buffer, 0, STA_TCP_SEND_BUFFER_SIZE);
    memset(sta_tcp_txbuf, 0, STA_TCP_TXBUF_SIZE);
    memset(sta_tcp_rxbuf, 0, STA_TCP_RXBUF_SIZE);
    fifo_init(&p_sta_tcp->tx_fifo, sta_tcp_txbuf, STA_TCP_TXBUF_SIZE);
	fifo_init(&p_sta_tcp->rx_fifo, sta_tcp_rxbuf, STA_TCP_RXBUF_SIZE);
    p_sta_tcp->state        = NETCONN_STATE_OFF;
    p_sta_tcp->timeout      = STATION_TCP_STREAM_MAX_TIMEOUT;
    p_sta_tcp->run          = 1;

    if (mode == STA_WORK_NTRIP_CLIENT) {
        host = get_ntrip_client_ip();
        port = get_ntrip_client_port();
    } else if (mode == STA_WORK_NTRIP_SERVER) {
        host = get_ntrip_server_ip();
        port = get_ntrip_server_port();
    }

    for (;;) {
        if (is_eth_link_down()) {
            netconn_link_down(&p_sta_tcp->state);
        }

        if (p_sta_tcp->run == 0) {
            p_sta_tcp->state = NETCONN_STATE_LINK_DOWN;
        }

        switch (p_sta_tcp->state)
        {
        case NETCONN_STATE_OFF:
            if (!is_eth_link_down()) {
                if (get_eth_mode() == ETHMODE_DHCP) {
                    if (is_dhcp_address_assigned()) {
                        p_sta_tcp->state = NETCONN_STATE_CONNECT;
                    }
                } else {
                    p_sta_tcp->state = NETCONN_STATE_CONNECT;
                }
            }
            sta_status = -1;
            OS_Delay(100);

#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] OFF\r\n", mode);
#endif
            break;

        case NETCONN_STATE_CONNECT:
            connect_flag = 0;
            if (mode == STA_WORK_NTRIP_CLIENT) {
                sta_status = 0;
            } else if (mode == STA_WORK_NTRIP_SERVER) {
                sta_status = 4;
            }

            err = get_ip_by_host((const char*)host, &ip);
            if (!err) {
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] [%s:%d] dns ok\r\n", mode, host, port);
#endif
                signout_msg = 0;

                p_sta_tcp->p_netconn = netconn_new(NETCONN_TCP);

                if (p_sta_tcp->p_netconn != NULL) {
                    
                    p_sta_tcp->p_netconn->pcb.tcp->so_options |= SOF_KEEPALIVE;

                    err = netconn_connect(p_sta_tcp->p_netconn, &ip, port);
                    if (!err) {
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] [%s:%d] connect ok\r\n", mode, host, port);
#endif
                        p_sta_tcp->p_netconn->recv_timeout = 10;
                        p_sta_tcp->p_netconn->send_timeout = 100;

                        p_sta_tcp->timeout      = STATION_TCP_STREAM_MAX_TIMEOUT;
                        p_sta_tcp->rx_fifo.in   = 0;
                        p_sta_tcp->rx_fifo.out  = 0;
                        p_sta_tcp->tx_fifo.in   = 0;
                        p_sta_tcp->tx_fifo.out  = 0;
                        p_sta_tcp->state        = NETCONN_STATE_REQUEST;

                        signout_msg = 1;
                        connect_flag = 1;
                        
                    } else {
                        
                        netconn_close(p_sta_tcp->p_netconn);
                        osDelay(100);
                        netconn_delete(p_sta_tcp->p_netconn);
                        p_sta_tcp->p_netconn = NULL;
                        OS_Delay(1000);
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] [%s:%d] connect fail\r\n", mode, host, port);
#endif
                    }
                } else {
                    OS_Delay(1000);
                } 
            } else {
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] [%s:%d] dns fail\r\n", mode, host, port);
#endif
            }
            if (!connect_flag) {
                if (mode == STA_WORK_NTRIP_CLIENT) {
                    sta_status = 1;
                } else if (mode == STA_WORK_NTRIP_SERVER) {
                    sta_status = 5;
                }
                OS_Delay(3000);
            } else {
                if (mode == STA_WORK_NTRIP_CLIENT) {
                    sta_status = 2;
                } else if (mode == STA_WORK_NTRIP_SERVER) {
                    sta_status = 6;
                }
            }
            break;

        case NETCONN_STATE_REQUEST:
            if (mode == STA_WORK_NTRIP_CLIENT) {
                res = ntrip_client_request(p_sta_tcp->p_netconn, &p_sta_tcp->state, sta_tcp_send_buffer, STA_TCP_SEND_BUFFER_SIZE);

            } else if (mode == STA_WORK_NTRIP_SERVER) {
                res = ntrip_server_request(p_sta_tcp->p_netconn, &p_sta_tcp->state, sta_tcp_send_buffer, STA_TCP_SEND_BUFFER_SIZE);

            }

            if (!res) {
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] request ok\r\n", mode);
#endif
                write_fail = 0;
                p_sta_tcp->state = NETCONN_STATE_INTERACTIVE;
                interactive = 900;
                OS_Delay(10);
            } else {
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] request fail\r\n", mode);
#endif
                OS_Delay(200);
            }
            break;

        case NETCONN_STATE_INTERACTIVE:
            err = netconn_push_rx_data(p_sta_tcp->p_netconn, &p_sta_tcp->rx_fifo);
            if (ERR_IS_FATAL(err)) {
                p_sta_tcp->state = NETCONN_STATE_LINK_DOWN;
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] read fail %d\r\n", mode, err);
#endif
            } else {
                do {
                    len = fifo_get(&p_sta_tcp->tx_fifo, sta_tcp_send_buffer, STA_TCP_SEND_BUFFER_SIZE);
                    if (len) {
                        err = netconn_write(p_sta_tcp->p_netconn, sta_tcp_send_buffer, len, NETCONN_NOFLAG);
                        if (ERR_IS_FATAL(err)) {
                            p_sta_tcp->state = NETCONN_STATE_LINK_DOWN;
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] write data fail %d\r\n", mode, err);
#endif
                        } else if (err) {
                            if (write_fail++ >= 3) {
                                p_sta_tcp->state = NETCONN_STATE_LINK_DOWN;
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] write count fail %d\r\n", mode, err);
#endif
                            }
                        } else {
#ifdef STATION_TCP_DEBUG
    // printf("STA-TCP:[%d] write data ok %d\r\n", mode, len);
#endif
                            write_fail = 0;
                            if (len == STA_TCP_SEND_BUFFER_SIZE) {
                                OS_Delay(10);
                            }
                        }
                    }
                } while (len == STA_TCP_SEND_BUFFER_SIZE && err == ERR_OK);
            }

            if (station_tcp_is_stream_available()) {
                if (mode == STA_WORK_NTRIP_CLIENT) {
                    sta_status = 3;
                } else if (mode == STA_WORK_NTRIP_SERVER) {
                    sta_status = 7;
                }
            } else {
                if (mode == STA_WORK_NTRIP_CLIENT) {
                    sta_status = 2;
                } else if (mode == STA_WORK_NTRIP_SERVER) {
                    sta_status = 6;
                }
            }

            if (p_sta_tcp->state != NETCONN_STATE_LINK_DOWN) {
                if (mode == STA_WORK_NTRIP_SERVER) {
                    ntrip_server_send_pos(p_sta_tcp->p_netconn, &p_sta_tcp->state);
                }
            }

            if (p_sta_tcp->state == NETCONN_STATE_LINK_DOWN) {
                OS_Delay(1000);
            }
            break;
        
        case NETCONN_STATE_TIMEOUT:
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] timeout\r\n", mode);
#endif
            break;

        case NETCONN_STATE_LINK_DOWN:
#ifdef STATION_TCP_DEBUG
    printf("STA-TCP:[%d] link down\r\n", mode);
#endif

            if (p_sta_tcp->p_netconn != NULL) {
                netconn_close(p_sta_tcp->p_netconn);
                osDelay(100);
                netconn_delete(p_sta_tcp->p_netconn);
                p_sta_tcp->p_netconn = NULL;
            }

            p_sta_tcp->state = NETCONN_STATE_OFF;

            if (p_sta_tcp->run == 0) {
                return;
            }
            break;

        default:
            OS_Delay(100);
            break;
        }
    }
}




void station_tcp_interface(void)
{
    uint8_t status;

#ifdef STATION_TCP_DEBUG
    printf("STATION: choose mode [%ld]\r\n", get_station_mode());
#endif

    sta_status = -1;

    switch (get_station_mode())
    {
    case MODE_NTRIP_CLIENT:
        station_tcp_thread(STA_WORK_NTRIP_CLIENT);
        break;

    case MODE_OPENARC_CLIENT:
        sta_status = 8;
        aceinna_client_thread();

        break;

    case MODE_NTRIP_SERVER:
        status = base_station_get_run_status();
        if (status == 1) {
            station_tcp_thread(STA_WORK_NTRIP_SERVER);

        } else if (status == 3) {
            station_tcp_thread(STA_WORK_NTRIP_CLIENT);
            
        } else {
            OS_Delay(3000);
        }
        break;

    default:
        OS_Delay(100); 
        break;
    }

}
