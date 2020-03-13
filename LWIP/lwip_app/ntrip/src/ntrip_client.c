#include <string.h>

#include "ntrip_client.h"
#include "base64.h"
#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "user_config.h"
#include "calibrationAPI.h"
#include "platformAPI.h"
#include "uart.h"


// ntrip
struct netconn *Ntrip_client = NULL;
uint8_t NTRIP_client_state = NTRIP_STATE_OFF;

// ntrip buf
fifo_type ntrip_tx_fifo;
fifo_type ntrip_rx_fifo;
CCMRAM uint8_t ntripTxBuf[NTRIP_TX_BUFSIZE];
CCMRAM uint8_t ntripRxBuf[NTRIP_RX_BUFSIZE];
uint32_t ntripStreamCount = NTRIP_STREAM_CONNECTED_MAX_COUNT;

/** ***************************************************************************
 * @name fill_localrtk_request_payload() 
 * @brief fill Local RTK Request
 * @param *payload point to buffer
 *        *payloadLen point to buffer length
 * @retval N/A
 ******************************************************************************/
void fill_localrtk_request_payload(uint8_t *payload, uint16_t *payloadLen)
{
    uint8_t temp[25];

    strcpy((char *)payload, "GET /");
    strcat((char *)payload, (const char *)gUserConfiguration.mountPoint);
    strcat((char *)payload, " HTTP/1.1\r\n");
    strcat((char *)payload, "User-Agent: NTRIP Aceinna/0.1\r\n");

    strcat((char *)payload, "Ntrip-Sn:");
    sprintf((char *)temp, "%ld", GetUnitSerialNum());
    strcat((char *)payload, (const char *)temp);
    strcat((char *)payload, "\r\n");
    
    strcat((char *)payload, "Ntrip-Pn:");
    strcpy((char *)temp, (const char *)platformBuildInfo());
    for (uint8_t i = 0; i < strlen((const char*)temp); i++)
    {
        if (temp[i] == ' ')
        {
            temp[i] = 0;
            break;
        }
    }
    strcat((char *)payload, (const char *)temp);
    strcat((char *)payload, "\r\n");

    strcat((char *)payload, "Authorization: Basic ");
	
    uint8_t key[100];
    uint8_t base64_buf[128];
    strcpy((char *)key, (const char *)gUserConfiguration.username);
    strcat((char *)key, ":");
    strcat((char *)key, (const char *)gUserConfiguration.password);
    base64_encode(key, base64_buf);

    strcat((char *)payload, (const char *)base64_buf);
    strcat((char *)payload, "\r\n\r\n");
    
    *payloadLen = strlen((const char *)payload);
}

/** ***************************************************************************
 * @name ntrip_read_data() 
 * @brief ntrip client revieve function
 * @param *rxBuf point to ntrip client recieve buffer
 *        *rxLen point to buffer length
 * @retval success(ERR_OK) fail(other)
 ******************************************************************************/
err_t ntrip_read_data(uint8_t *rxBuf, uint16_t *rxLen)
{
	struct netbuf *rxNetbuf;
	uint16_t len = 0;
	struct pbuf *q;
	err_t err;

	err = netconn_recv(Ntrip_client, &rxNetbuf);
	if (err == ERR_OK)
	{
		taskENTER_CRITICAL();
		for (q = rxNetbuf->p; q != NULL; q = q->next)
		{
			if (q->len > (NTRIP_RX_BUFSIZE - len))
				memcpy(rxBuf + len, q->payload, (NTRIP_RX_BUFSIZE - len));
			else
				memcpy(rxBuf + len, q->payload, q->len);
			len += q->len;
			if (len > NTRIP_RX_BUFSIZE)
				break;
		}
		*rxLen = len;
		taskEXIT_CRITICAL();
	}
	netbuf_delete(rxNetbuf);

	if (ERR_IS_FATAL(err))
	{
#ifdef DEVICE_DEBUG
		printf("ntrip_read_data fail %d\r\n", err);
#endif
		NTRIP_client_state = NTRIP_STATE_CONNECT;
	}
	
	return err;
}

/** ***************************************************************************
 * @name ntrip_push_rx_data() 
 * @brief ntrip client revieve data and push to rx fifo
 * @param *fifo point to rx fifo
 * @retval success(ERR_OK) fail(other)
 ******************************************************************************/
err_t ntrip_push_rx_data(fifo_type* fifo)
{
	struct netbuf *rxNetbuf;
	struct pbuf *q;
	err_t err;
    uint16_t len = 0;

	err = netconn_recv(Ntrip_client, &rxNetbuf);
	if (err == ERR_OK)
	{
		taskENTER_CRITICAL();
		for (q = rxNetbuf->p; q != NULL; q = q->next)
		{
            fifo_push(fifo, q->payload, q->len);
            len += q->len;
		}
		taskEXIT_CRITICAL();
        //printf("ntrip_push_rx_data=%d\r\n", len);
	}
	netbuf_delete(rxNetbuf);

	if (ERR_IS_FATAL(err))
	{
#ifdef DEVICE_DEBUG
		printf("ntrip_push_rx_data fail %d\r\n", err);
#endif
		NTRIP_client_state = NTRIP_STATE_CONNECT;
	}
	
	return err;
}

/** ***************************************************************************
 * @name ntrip_write_data
 * @brief ntrip client write function
 * @param *txBuf point to ntrip client send buffer
 *        [in] txLen buffer length
 * 		  [in] apiflags write flag
 * @retval success(ERR_OK) fail(other)
 ******************************************************************************/
err_t ntrip_write_data(uint8_t *txBuf, uint16_t txLen, uint8_t apiflags)
{
	err_t err;

	err = netconn_write(Ntrip_client, txBuf, txLen, apiflags);
	if (ERR_IS_FATAL(err))
	{
#ifdef DEVICE_DEBUG
		printf("ntrip_write_data fail %d\r\n", err);
#endif
		if (NTRIP_client_state == NTRIP_STATE_REQUEST || NTRIP_client_state == NTRIP_STATE_INTERACTIVE)
		{
			NTRIP_client_state = NTRIP_STATE_CONNECT;
		}
	}
    
	return err;
}

/** ***************************************************************************
 * @name ntrip_push_tx_data
 * @brief push data to tx fifo
 * @param *buf point to send data buffer
 *        len send length
 * @retval success(1) fail(0)
 ******************************************************************************/
uint8_t ntrip_push_tx_data(uint8_t* buf, uint16_t len)
{
    if (is_ntrip_interactive())
    {
        fifo_push(&ntrip_tx_fifo, (uint8_t*)buf, len);
        return 1;
    }
    return 0;
}

/** ***************************************************************************
 * @name ntrip_link_down
 * @brief link down ntrip client
 * @param 
 * @retval 
 ******************************************************************************/
void ntrip_link_down(void)
{
    if (NTRIP_client_state >= NTRIP_STATE_CONNECT && NTRIP_client_state <= NTRIP_STATE_INTERACTIVE)
    {
        NTRIP_client_state = NTRIP_STATE_LINK_DOWN;
    }
}

/** ***************************************************************************
 * @name is_ntrip_interactive
 * @brief get ntrip state
 * @param 
 * @retval true: is connected  false: others
 ******************************************************************************/
uint8_t is_ntrip_interactive(void)
{
    return (NTRIP_client_state == NTRIP_STATE_INTERACTIVE);
}

/** ***************************************************************************
 * @name add_ntrip_stream_count
 * @brief add count
 * @param 
 * @retval
 ******************************************************************************/
void add_ntrip_stream_count(void)
{
    ntripStreamCount++;
}

/** ***************************************************************************
 * @name clear_ntrip_stream_count
 * @brief reset count
 * @param 
 * @retval
 ******************************************************************************/
void clear_ntrip_stream_count(void)
{
    ntripStreamCount = 0;
}

/** ***************************************************************************
 * @name is_ntrip_stream_available
 * @brief get ntrip stream state
 * @param 
 * @retval true: available  false: unavailable
 ******************************************************************************/
uint8_t is_ntrip_stream_available(void)
{
    return (ntripStreamCount < NTRIP_STREAM_CONNECTED_MAX_COUNT);
}

/** ***************************************************************************
 * @name NTRIP_interface
 * @brief ntrip client interface
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void NTRIP_interface(void)
{
    static ip_addr_t server_ipaddr;
    static uint8_t txBuf[512];
    static uint8_t write_fail = 0;
    uint16_t txLen = 0;
    err_t err;

    if (is_eth_link_down())
    {
        ntrip_link_down();
    }
    
    switch (NTRIP_client_state)
    {
    case NTRIP_STATE_CONNECT:
#ifdef DEVICE_DEBUG
        printf("ntrip:connect...\r\n");
#endif

#ifdef DEVICE_DEBUG
        printf("ntrip:reset start...\r\n");
#endif
        if (Ntrip_client != NULL)
        {
            netconn_close(Ntrip_client);
            osDelay(100);
            netconn_delete(Ntrip_client);
            Ntrip_client = NULL;
        }
#ifdef DEVICE_DEBUG
        printf("ntrip:reset over!\r\n");
#endif

        OS_Delay(50);
        Ntrip_client = netconn_new(NETCONN_TCP);
        
        err = ipaddr_aton((const char*)gUserConfiguration.ip, &server_ipaddr);
        if (!err)
        {
            for (uint8_t i = 0; i < 5; i++)
            {
                err = dns_get_ip_by_hostname(gUserConfiguration.ip, &server_ipaddr);
                if (err)
                {
                    break;
                }
                else
                {
                    OS_Delay(3000);
                }
            }
        }

        if (err)
        {
#ifdef DEVICE_DEBUG
            printf("ntrip:server ip %s\r\n", ip_ntoa(&server_ipaddr));
#endif
            err = netconn_connect(Ntrip_client, &server_ipaddr, gUserConfiguration.port);
            if (err == ERR_OK)
            {
                Ntrip_client->recv_timeout = 10;
                Ntrip_client->send_timeout = 100;

                NTRIP_client_state = NTRIP_STATE_REQUEST;
#ifdef DEVICE_DEBUG
                printf("ntrip:connect success!\r\n");
#endif
            }
            else
            {
#ifdef DEVICE_DEBUG
                printf("ntrip:connect err {%d}\r\n", err);
#endif
            }
        }
        break;

    case NTRIP_STATE_REQUEST:
#ifdef DEVICE_DEBUG
        printf("ntrip:request...\r\n");
#endif
        OS_Delay(100);
        fill_localrtk_request_payload(txBuf, &txLen);

        err = ntrip_write_data(txBuf, txLen, NETCONN_COPY);
        if (err == ERR_OK)
        {
            OS_Delay(1000);
            uint16_t rxLen = 0;
            err = ntrip_read_data(ntripRxBuf, &rxLen);
            if (err == ERR_OK)
            {
                ntripRxBuf[rxLen] = 0;
                if (rxLen && strstr((char *)ntripRxBuf, "ICY 200 OK") != NULL)
                {
#ifdef DEVICE_DEBUG
                    printf("ntrip:ICY 200 OK\r\n");
#endif
                    NTRIP_client_state = NTRIP_STATE_INTERACTIVE;
                    write_fail = 0;
                    OS_Delay(100);
                }
            }
        }
        break;

    case NTRIP_STATE_INTERACTIVE:
        err = ntrip_push_rx_data(&ntrip_rx_fifo);
        //printf("rx err=%d\r\n",err);
        txLen = fifo_get(&ntrip_tx_fifo, txBuf, 512);
        if (txLen)
        {
            err = ntrip_write_data(txBuf, txLen, NETCONN_NOFLAG);
            if (err != 0) {
#ifdef DEVICE_DEBUG
                printf("ntrip:tx err {%d}\r\n", err);
#endif
                write_fail++;
                if (write_fail >= 3)
                {
                    NTRIP_client_state = NTRIP_STATE_OFF;
                }
            }
        }
        break;

    case NTRIP_STATE_LINK_DOWN:
#ifdef DEVICE_DEBUG
        printf("ntrip:link down start...\r\n");
#endif
        if (Ntrip_client != NULL)
        {
            netconn_close(Ntrip_client);
            osDelay(100);
            netconn_delete(Ntrip_client);
            Ntrip_client = NULL;
        }
#ifdef DEVICE_DEBUG
        printf("ntrip:link down over!\r\n");
#endif
        NTRIP_client_state = NTRIP_STATE_OFF;
        break;

    case NTRIP_STATE_OFF:
        if (!is_eth_link_down())
        {
            if (gUserConfiguration.ethMode == ETHMODE_DHCP)
            {
                if (is_dhcp_address_assigned())
                {
                    NTRIP_client_state = NTRIP_STATE_CONNECT;
                }
            }
            else if (gUserConfiguration.ethMode == ETHMODE_STATIC)
            {
                NTRIP_client_state = NTRIP_STATE_CONNECT;
            }
        }

        OS_Delay(20);
        break;

    default:
        OS_Delay(100);
        break;
    }
}
