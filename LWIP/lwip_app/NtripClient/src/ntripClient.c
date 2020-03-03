#include "ntripClient.h"
#include <string.h>
#include "base64.h"
#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "UserConfiguration.h"
#include "calibrationAPI.h"
#include "platformAPI.h"


// ntrip
struct netconn *Ntrip_client = NULL;
uint8_t NTRIP_client_start = NTRIP_START_ON;
uint8_t NTRIP_client_state = NTRIP_STATE_OFF;
uint8_t NTRIP_base_stream = BSAE_ON;

// ntrip buf
FIFO_Type ntrip_tx_fifo;
FIFO_Type ntrip_rx_fifo;
CCMRAM uint8_t ntripTxBuf[NTRIP_TX_BUFSIZE];
CCMRAM uint8_t ntripRxBuf[NTRIP_RX_BUFSIZE];
uint16_t ntripTxLen = 0;
uint16_t ntripRxLen = 0;
uint32_t ntripStreamCount = NTRIP_STREAM_CONNECTED_MAX_COUNT;

/** ***************************************************************************
 * @name Fill_EthLocalRTKPacketPayload() 
 * @brief fill Local RTK Request
 * @param *payload point to buffer
 *        *payloadLen point to buffer length
 * @retval N/A
 ******************************************************************************/
void Fill_EthLocalRTKPacketPayload(uint8_t *payload, uint16_t *payloadLen)
{
    uint8_t temp[25];

    strcpy((char *)payload, "GET ");
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
    strcpy((char *)key, (const char *)gUserConfiguration.username);
    strcat((char *)key, ":");
    strcat((char *)key, (const char *)gUserConfiguration.password);
    uint8_t *buf = base64_encode(key);

    strcat((char *)payload, (const char *)buf);
    free(buf);

    strcat((char *)payload, "\r\n\r\n");
    
    *payloadLen = strlen((const char *)payload);
}

/** ***************************************************************************
 * @name Fill_EthCloudRTKPacketPayload() 
 * @brief fill Cloud RTK Request
 * @param *payload point to buffer
 *        *payloadLen point to buffer length
 * @retval N/A
 ******************************************************************************/
void Fill_EthCloudRTKPacketPayload(uint8_t *payload, uint16_t *payloadLen)
{
    uint8_t sn[15];

    strcpy((char *)payload, "SOURCE ");
    strcat((char *)payload, (const char *)gUserConfiguration.password);
    strcat((char *)payload, " ");
    strcat((char *)payload, (const char *)gUserConfiguration.mountPoint);
    strcat((char *)payload, "\r\n");
    
    strcat((char *)payload, "Aceinna-Sn:");
    sprintf((char *)sn, "%ld", GetUnitSerialNum());
    strcat((char *)payload, (const char *)sn);
    strcat((char *)payload, "\r\n");
    
    strcat((char *)payload, "Source-Agent: NTRIP Aceinna/0.1\r\n\r\n");

    *payloadLen = strlen((const char *)payload);
}


/** ***************************************************************************
 * @name NTRIP_GET_Data() 
 * @brief ntrip client revieve function
 * @param *rxBuf point to ntrip client recieve buffer
 *        *rxLen point to buffer length
 * @retval success(ERR_OK) fail(other)
 ******************************************************************************/
err_t NTRIP_GET_Data(uint8_t *rxBuf, uint16_t *rxLen)
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
		printf("NTRIP_GET_Data fail %d\r\n", err);
#endif
		NTRIP_client_state = NTRIP_STATE_CONNECT;
	}
	
	return err;
}

err_t NTRIP_Recieve_Data(FIFO_Type* fifo)
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
            FifoPush(fifo, q->payload, q->len);
            len += q->len;
		}
		taskEXIT_CRITICAL();
        //printf("NTRIP_Recieve_Data=%d\r\n", len);
	}
	netbuf_delete(rxNetbuf);

	if (ERR_IS_FATAL(err))
	{
#ifdef DEVICE_DEBUG
		printf("NTRIP_Recieve_Data fail %d\r\n", err);
#endif
		NTRIP_client_state = NTRIP_STATE_CONNECT;
	}
	
	return err;
}

/** ***************************************************************************
 * @name NTRIP_Write_Data
 * @brief ntrip client write function
 * @param *txBuf point to ntrip client send buffer
 *        [in] txLen buffer length
 * 		  [in] apiflags write flag
 * @retval success(ERR_OK) fail(other)
 ******************************************************************************/
err_t NTRIP_Write_Data(uint8_t *txBuf, uint16_t txLen, uint8_t apiflags)
{
	err_t err;

	err = netconn_write(Ntrip_client, txBuf, txLen, apiflags);
	if (ERR_IS_FATAL(err))
	{
#ifdef DEVICE_DEBUG
		printf("NTRIP_Write_Data fail %d\r\n", err);
#endif
		if (NTRIP_client_state == NTRIP_STATE_REQUEST || NTRIP_client_state == NTRIP_STATE_INTERACTIVE)
		{
			NTRIP_client_state = NTRIP_STATE_CONNECT;
		}
	}
    
	return err;
}

void ntrip_link_down(void)
{
    if (NTRIP_client_state >= NTRIP_STATE_CONNECT && NTRIP_client_state <= NTRIP_STATE_INTERACTIVE)
    {
        NTRIP_client_state = NTRIP_STATE_LINK_DOWN;
    }
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
    uint16_t txLen = 0;
    err_t err;

    if (NTRIP_client_start == NTRIP_START_OFF || get_eth_link_state() == ETH_LINK_DOWN)
    {
        ntrip_link_down();
    }
    
    switch (NTRIP_client_state)
    {
    case NTRIP_STATE_CONNECT:
#ifdef DEVICE_DEBUG
        printf("ntrip:connect...\r\n");
#endif
        if (Ntrip_client != NULL)
        {
            netconn_close(Ntrip_client);
            osDelay(100);
            netconn_delete(Ntrip_client);
            Ntrip_client = NULL;
        }
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

                NTRIP_client_state = NTRIP_STATE_REQUEST;
#ifdef DEVICE_DEBUG
                printf("ntrip:connect success!\r\n");
#endif
            }
            else
            {
#ifdef DEVICE_DEBUG
                printf("ntrip:connect err = %d\r\n", err);
#endif
            }
        }
        break;

    case NTRIP_STATE_REQUEST:
#ifdef DEVICE_DEBUG
        printf("ntrip:request...\r\n");
#endif
        OS_Delay(100);
        if (gUserConfiguration.rtkType == LocalRTK)
        {
            Fill_EthLocalRTKPacketPayload(ntripTxBuf, &txLen);
        }
        else if (gUserConfiguration.rtkType == CloudRTK)
        {
            Fill_EthCloudRTKPacketPayload(ntripTxBuf, &txLen);
        }

        err = NTRIP_Write_Data(ntripTxBuf, txLen, NETCONN_COPY);
        if (err == ERR_OK)
        {
            OS_Delay(1000);
            uint16_t rxLen = 0;
            err = NTRIP_GET_Data(ntripRxBuf, &rxLen);
            if (err == ERR_OK)
            {
                ntripRxBuf[rxLen] = 0;
                if (rxLen && strstr((char *)ntripRxBuf, "ICY 200 OK") != NULL)
                {
#ifdef DEVICE_DEBUG
                    printf("ntrip:ICY 200 OK\r\n");
#endif
                    NTRIP_client_state = NTRIP_STATE_INTERACTIVE;
                    OS_Delay(100);
                }
            }
        }
        break;

    case NTRIP_STATE_INTERACTIVE:
        if (gUserConfiguration.rtkType == LocalRTK) // LocalRTK
        {
            NTRIP_Recieve_Data(&ntrip_rx_fifo);

            txLen = FifoGet(&ntrip_tx_fifo, txBuf, 512);
            if (txLen)
            {
                NTRIP_Write_Data(txBuf, txLen, NETCONN_NOFLAG);
            }
        }
        else // CloudRTK
        {
            err = NTRIP_GET_Data(ntripRxBuf, &ntripRxLen);
            if (err == ERR_OK)
            {
                if (ntripRxLen)
                {
                    ntripRxBuf[ntripRxLen] = 0;
#ifdef DEVICE_DEBUG
                    printf("%s", ntripRxBuf); //print #GPGGA
#endif
                    ntripRxLen = 0;
                }
            }
        }
        break;

    case NTRIP_STATE_LINK_DOWN:
#ifdef DEVICE_DEBUG
        printf("ntrip:link down!\r\n");
#endif
        if (Ntrip_client != NULL)
        {
            netconn_close(Ntrip_client);
            osDelay(100);
            netconn_delete(Ntrip_client);
            Ntrip_client = NULL;
        }
        NTRIP_client_state = NTRIP_STATE_OFF;
        break;

    case NTRIP_STATE_OFF:
        if (get_eth_link_state() && NTRIP_client_start == NTRIP_START_ON)
        {
            if (gUserConfiguration.ethMode == ETHMODE_DHCP)
            {
                if (eth_dhcp_state == DHCP_STATE_ADDRESS_ASSIGNED || eth_dhcp_state == DHCP_STATE_TIMEOUT)
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
