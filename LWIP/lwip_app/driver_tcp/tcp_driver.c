#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "user_config.h"
#include "calibrationAPI.h"
#include "platformAPI.h"
#include "uart.h"

#include "serial_port.h"
#include "uart.h"
#include "crc16.h"
#include "ucb_packet.h"
#include "platformAPI.h"
#include "configuration.h"
#include "user_message.h"
#include "lwip/ip.h"
#include "lwip/opt.h"

#include "lwip/udp.h"
#include "lwip/netif.h"
#include "netbios.h"
#include "commAPI.h"
#include "user_config.h"
#include "tcp_driver.h"
#include "cJSON.h"
#include "app_version.h"

client_s driver_client;
client_s driver_data_client;

uint8_t driver_tx_buf[DRIVER_TX_BUFSIZE];
uint8_t driver_rx_buf[DRIVER_RX_BUFSIZE];
uint8_t driver_data_tx_buf[DRIVER_TX_BUFSIZE];
uint8_t driver_data_rx_buf[DRIVER_RX_BUFSIZE];

ip_addr_t server_ip;

void client_link_down(uint8_t* client_state)
{
    if (*client_state >= CLIENT_STATE_CONNECT && *client_state <= CLIENT_STATE_INTERACTIVE)
    {
        *client_state = CLIENT_STATE_LINK_DOWN;
    }    
}

err_t client_write_data(client_s* client, const uint8_t *tx_buf, uint16_t tx_len, uint8_t apiflags)
{
	err_t err;

	err = netconn_write(client->client, tx_buf, tx_len, apiflags);
	if (ERR_IS_FATAL(err))
	{
		if (client->client_state == CLIENT_STATE_REQUEST || client->client_state == CLIENT_STATE_INTERACTIVE)
		{
			client->client_state = CLIENT_STATE_CONNECT;
		}
	}
    
	return err;
}


err_t client_read_data(client_s* client, uint8_t *rx_buf, uint16_t *rx_len)
{
	struct netbuf *rxNetbuf;
	uint16_t len = 0;
	struct pbuf *q;
	err_t err;

	err = netconn_recv(client->client, &rxNetbuf);
	if (err == ERR_OK)
	{
		taskENTER_CRITICAL();
		for (q = rxNetbuf->p; q != NULL; q = q->next)
		{
			if (q->len > (DRIVER_RX_BUFSIZE - len))
				memcpy(rx_buf + len, q->payload, (DRIVER_RX_BUFSIZE - len));
			else
				memcpy(rx_buf + len, q->payload, q->len);
			len += q->len;
			if (len > DRIVER_RX_BUFSIZE)
				break;
		}
		*rx_len = len;
		taskEXIT_CRITICAL();
	}
	netbuf_delete(rxNetbuf);

	if (ERR_IS_FATAL(err))
	{
		client->client_state = CLIENT_STATE_CONNECT;
	}
	
	return err;
}

uint8_t is_client_interactive(client_s* client)
{
    return (client->client_state == CLIENT_STATE_INTERACTIVE);
}

uint8_t client_push_tx_data(client_s* client, uint8_t* buf, uint16_t len)
{
    if (is_client_interactive(client))
    {
        fifo_push(&client->client_tx_fifo, (uint8_t*)buf, len);
        return 1;
    }
    return 0;
}

uint8_t driver_data_push(uint8_t* buf, uint16_t len)
{
	osMutexWait(driver_data_client.tx_fifo_mutex, osWaitForever);
    client_push_tx_data(&driver_data_client,buf,len);
	osMutexRelease(driver_data_client.tx_fifo_mutex);

    return 1;   
}

uint8_t driver_push(uint8_t* buf, uint16_t len)
{
	osMutexWait(driver_client.tx_fifo_mutex, osWaitForever);
    client_push_tx_data(&driver_client,buf,len);
	osMutexRelease(driver_client.tx_fifo_mutex);

    return 1;   
}


void set_server_ip(ip_addr_t* value)
{
    memcpy(&server_ip,value,sizeof(ip_addr_t));
}


extern uint8_t debug_com_log_on;
extern uint32_t debug_p1_log_delay;
void driver_interface(void)
{
    static ip_addr_t server_ipaddr;
    static char tx_buf[512];
    static uint8_t write_fail = 0;
    uint16_t tx_len = 0;
    err_t err;
    if (is_eth_link_down())
    {
        client_link_down(&(driver_client.client_state));
    }
    
    switch (driver_client.client_state)
    {
        case CLIENT_STATE_CONNECT:
            if(server_ip.addr == 0)
            {
                break;
            }
            if (driver_client.client != NULL)
            {
                netconn_close(driver_client.client);
                // osDelay(100);
                netconn_delete(driver_client.client);
                driver_client.client = NULL;
            }

            // OS_Delay(50);
            driver_client.client = netconn_new(NETCONN_TCP);

            err = IP4_ADDR(&server_ipaddr, ip4_addr1(&server_ip.addr), ip4_addr2(&server_ip.addr), \
            ip4_addr3(&server_ip.addr), ip4_addr4(&server_ip.addr));
    
            if (err)
            {
                err = netconn_connect(driver_client.client, &server_ipaddr, driver_client_port);
                if (err == ERR_OK)
                {
                    driver_client.client->recv_timeout = 10;
                    driver_client.client->send_timeout = 100;
                    driver_client.client_state = CLIENT_STATE_REQUEST;
                }
                else
                {
                }
            }
            break;

        case CLIENT_STATE_REQUEST:
            
            sprintf(tx_buf,"hello pc i'm openrtk\r\n");
            tx_len = strlen(tx_buf);
            err = client_write_data(&driver_client, (uint8_t *)tx_buf, tx_len, NETCONN_COPY);
            if (err == ERR_OK)
            {
                OS_Delay(100);
                uint16_t rx_len = 0;
                err = client_read_data(&driver_client, driver_rx_buf, &rx_len);
                if (rx_len && strstr((char *)driver_rx_buf, "i am pc") != NULL)
                {
                    driver_client.client_state = CLIENT_STATE_INTERACTIVE;
                }
            }
            else
            {
                write_fail++;
                if (write_fail >= 5)
                {
                    driver_client.client_state = CLIENT_STATE_OFF;
                }
            }
            break;
        case CLIENT_STATE_INTERACTIVE:
            handle_tcp_commands();
            break;
        case CLIENT_STATE_LINK_DOWN:
            if (driver_client.client != NULL)
            {
                netconn_close(driver_client.client);
                osDelay(100);
                netconn_delete(driver_client.client);
                driver_client.client = NULL;
            }
            driver_client.client_state = CLIENT_STATE_OFF;
            break;

        case CLIENT_STATE_OFF:
            if (!is_eth_link_down())
            {
                if (get_eth_mode() == ETHMODE_DHCP)
                {
                    if (is_dhcp_address_assigned())
                    {
                        driver_client.client_state = CLIENT_STATE_CONNECT;
                    }
                }
                else if (get_eth_mode() == ETHMODE_STATIC)
                {
                    driver_client.client_state = CLIENT_STATE_CONNECT;
                }
            }

            OS_Delay(20);
            break;

        default:
            OS_Delay(100);
            break;
    }
}





void driver_output_data_interface(void)
{
    static ip_addr_t server_ipaddr;
    static char tx_buf[DRIVER_TX_BUFSIZE];
    static uint8_t write_fail = 0;
    uint16_t tx_len = 0;
    err_t err;
    cJSON *root, *fmt;
    char *out;
    if (is_eth_link_down())
    {
        client_link_down(&driver_data_client.client_state);
    }
    
    switch (driver_data_client.client_state)
    {
        case CLIENT_STATE_CONNECT:
            if(server_ip.addr == 0)
            {
                break;
            }
            if (driver_data_client.client != NULL)
            {
                netconn_close(driver_data_client.client);
                // osDelay(100);
                netconn_delete(driver_data_client.client);
                driver_data_client.client = NULL;
            }

            // OS_Delay(50);
            driver_data_client.client = netconn_new(NETCONN_TCP);

            err = IP4_ADDR(&server_ipaddr, ip4_addr1(&server_ip.addr), ip4_addr2(&server_ip.addr), \
            ip4_addr3(&server_ip.addr), ip4_addr4(&server_ip.addr));
    
            if (err)
            {
                err = netconn_connect(driver_data_client.client, &server_ipaddr, driver_data_client_port);
                if (err == ERR_OK)
                {
                    driver_data_client.client->recv_timeout = 10;
                    driver_data_client.client->send_timeout = 100;
                    driver_data_client.client_state = CLIENT_STATE_REQUEST;
                }
                else
                {
                }
            }
            break;

        case CLIENT_STATE_REQUEST:
            // OS_Delay(100);
            sprintf(tx_buf,"hello pc i'm openrtk_data\r\n");
            tx_len = strlen(tx_buf);
            err = client_write_data(&driver_data_client, (uint8_t *)tx_buf, tx_len, NETCONN_COPY);
            if (err == ERR_OK)
            {
                OS_Delay(100);
                uint16_t rx_len = 0;
                err = client_read_data(&driver_data_client, driver_data_rx_buf, &rx_len);
                if(rx_len)
                {
                    if (strstr((const char*)driver_data_rx_buf, "get configuration\r\n") != NULL)
                    {
                        root = cJSON_CreateObject();
                        cJSON_AddItemToObject(root, "openrtk configuration", fmt = cJSON_CreateObject());
                        cJSON_AddItemToObject(fmt, "Product Name", cJSON_CreateString(PRODUCT_NAME_STRING));
                        cJSON_AddItemToObject(fmt, "Product PN", cJSON_CreateString((const char *)platformBuildInfo()));
                        cJSON_AddItemToObject(fmt, "Product SN", cJSON_CreateNumber(GetUnitSerialNum()));
                        cJSON_AddItemToObject(fmt, "Version", cJSON_CreateString(APP_VERSION_STRING));

                        char packet_type_str[5] = "s1";
                        uint16_t user_packet_rate = 100;

                        cJSON_AddItemToObject(fmt, "userPacketType", cJSON_CreateString(packet_type_str));
                        cJSON_AddItemToObject(fmt, "userPacketRate", cJSON_CreateNumber(user_packet_rate));

                        float *ins_para = get_user_ins_para();
                        cJSON_AddItemToObject(fmt, "leverArmBx", cJSON_CreateNumber(*ins_para));
                        cJSON_AddItemToObject(fmt, "leverArmBy", cJSON_CreateNumber(*(ins_para + 1)));
                        cJSON_AddItemToObject(fmt, "leverArmBz", cJSON_CreateNumber(*(ins_para + 2)));
                        cJSON_AddItemToObject(fmt, "pointOfInterestBx", cJSON_CreateNumber(*(ins_para + 6)));
                        cJSON_AddItemToObject(fmt, "pointOfInterestBy", cJSON_CreateNumber(*(ins_para + 7)));
                        cJSON_AddItemToObject(fmt, "pointOfInterestBz", cJSON_CreateNumber(*(ins_para + 8)));
                        cJSON_AddItemToObject(fmt, "rotationRbvx", cJSON_CreateNumber(*(ins_para + 9)));
                        cJSON_AddItemToObject(fmt, "rotationRbvy", cJSON_CreateNumber(*(ins_para + 10)));
                        cJSON_AddItemToObject(fmt, "rotationRbvz", cJSON_CreateNumber(*(ins_para + 11)));

                        out = cJSON_Print(root);
                        cJSON_Delete(root);
                        client_write_data(&driver_data_client, (uint8_t *)out, strlen(out), NETCONN_COPY);
                        
                        free(out);
                        debug_com_log_on = 0;
                    }
                    if (strstr((const char*)driver_data_rx_buf, "log debug on\r\n") != NULL)
                    {
                        debug_com_log_on = 1;
                        debug_p1_log_delay = 100;
                        driver_data_client.client_state = CLIENT_STATE_INTERACTIVE;
                    }
                }
            }
            else
            {
                write_fail++;
                if (write_fail >= 5)
                {
                    driver_data_client.client_state = CLIENT_STATE_OFF;
                }
            }
            break;
        case CLIENT_STATE_INTERACTIVE:
            tx_len = fifo_get(&driver_data_client.client_tx_fifo, (uint8_t *)tx_buf, DRIVER_TX_BUFSIZE);
            if (tx_len)
            {
                err = client_write_data(&driver_data_client, (uint8_t *)tx_buf, tx_len, NETCONN_NOFLAG);
                if (err != 0) {
                    write_fail++;
                    if (write_fail >= 3)
                    {
                        driver_data_client.client_state = CLIENT_STATE_OFF;
                    }
                }
            }
            break;
        case CLIENT_STATE_LINK_DOWN:
            if (driver_data_client.client != NULL)
            {
                netconn_close(driver_data_client.client);
                osDelay(100);
                netconn_delete(driver_data_client.client);
                driver_data_client.client = NULL;
            }
            driver_data_client.client_state = CLIENT_STATE_OFF;
            break;

        case CLIENT_STATE_OFF:
            if (!is_eth_link_down())
            {
                if (get_eth_mode() == ETHMODE_DHCP)
                {
                    if (is_dhcp_address_assigned())
                    {
                        driver_data_client.client_state = CLIENT_STATE_CONNECT;
                    }
                }
                else if (get_eth_mode() == ETHMODE_STATIC)
                {
                    driver_data_client.client_state = CLIENT_STATE_CONNECT;
                }
            }

            OS_Delay(20);
            break;

        default:
            OS_Delay(100);
            break;
    }
}

void tcp_driver_fifo_init()
{
	fifo_init(&driver_client.client_tx_fifo, driver_tx_buf, DRIVER_TX_BUFSIZE);
    osMutexDef(FIFO_MUTEX);
    driver_client.tx_fifo_mutex = osMutexCreate(osMutex(FIFO_MUTEX));
}

void tcp_driver_data_fifo_init()
{
	fifo_init(&driver_data_client.client_tx_fifo, driver_data_tx_buf, DRIVER_TX_BUFSIZE);
    osMutexDef(FIFO_MUTEX);
    driver_data_client.tx_fifo_mutex = osMutexCreate(osMutex(FIFO_MUTEX));
}


uint8_t get_tcp_driver_state()
{
    return driver_client.client_state;
}

uint8_t get_tcp_data_driver_state()
{
    return driver_data_client.client_state;
}
