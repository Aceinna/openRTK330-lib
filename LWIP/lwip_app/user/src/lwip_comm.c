#include "lwip_comm.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/init.h"
#include "lwip/timers.h"
#include "lwip/tcp_impl.h"
#include "lwip/ip_frag.h"
#include "lwip/tcpip.h"
#include "lwip/timers.h"
#include "lwip/dns.h"
#include "ethernetif.h"
#include "stm32f4xx_hal.h"
#include "osapi.h"
#include <stdio.h>
#include <string.h>
#include "httpd.h"
#include "netbios.h"
#include "ntrip_client.h"
#include "user_config.h"
#include "cmsis_os.h"

/* network interface structure */
struct netif gnetif; 
uint8_t eth_link_state = ETH_LINK_DOWN;
uint8_t eth_dhcp_state = DHCP_STATE_OFF;

/** ***************************************************************************
 * @name Netif_Config
 * @brief Initializes the lwIP stack
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void Netif_Config(void)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;

	ip_addr_set_zero(&ipaddr);
    ip_addr_set_zero(&netmask);
    ip_addr_set_zero(&gw);

    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

    netif_set_default(&gnetif);

    if (netif_is_link_up(&gnetif)) {
        netif_set_up(&gnetif);
    } else {
        netif_set_down(&gnetif);
    }

    netif_set_link_callback(&gnetif, ethernetif_update_config);
}

/** ***************************************************************************
 * @name ethernet_init
 * @brief init the lwip stack and the ethernet on the board
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void ethernet_init(void)
{
	tcpip_init(NULL, NULL);

	Netif_Config();

	netbios_init();

    /* Initialize webserver */
	httpd_init(); 
    
    user_notification(&gnetif);

	/* Start DHCPClient */
	osThreadDef(DHCP, dhcp_thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate(osThread(DHCP), &gnetif);
}

/** ***************************************************************************
 * @name dhcp_supplied_address
 * @brief check if DHCP supplied netif->ip_addr
 * @param netif : the netif to check
 * @retval 1 if DHCP supplied netif->ip_addr (states BOUND or RENEWING),
 *         0 otherwise
 ******************************************************************************/
uint8_t dhcp_supplied_address(const struct netif *netif)
{
    if ((netif != NULL) && (netif->dhcp != NULL)) {
        struct dhcp* dhcp = netif->dhcp;
        return (dhcp->state == DHCP_BOUND) || (dhcp->state == DHCP_RENEWING) ||
            (dhcp->state == DHCP_REBINDING);
    }
    return 0;
}

/** ***************************************************************************
 * @name user_notification
 * @brief Notify the User about the nework interface config status.
 * @param [in] netif : the lwip network interface structure for this ethernetif
 * @retval N/A
 ******************************************************************************/
void user_notification(struct netif *netif)
{
    if (netif_is_up(netif))
    {
        set_eth_link_up();

        if (gUserConfiguration.ethMode == ETHMODE_STATIC)
        {
            netif_set_static_ip(netif);
        }
    }
    else
    {
        set_eth_link_down();
    }
}

/** ***************************************************************************
 * @name ethernetif_notify_conn_changed
 * @brief This function notify user about link status changement.
 * @param [in] netif : the lwip network interface structure for this ethernetif
 * @retval N/A
 ******************************************************************************/
void ethernetif_notify_conn_changed(struct netif *netif)
{
    if (netif_is_link_up(netif))
    {
        set_eth_link_up();
        
        if (gUserConfiguration.ethMode == ETHMODE_STATIC)
        {
            netif_set_static_ip(netif);
        }

        netif_set_up(netif);
    }
    else
    {
#ifdef DEVICE_DEBUG
    printf("ETH link down\r\n");
#endif
        set_eth_link_down();

        netif_set_down(netif);
    }
}

/** ***************************************************************************
 * @name set_eth_link_up / set_eth_link_down / is_eth_link_down
 * @brief link state
 * @param N/A
 * @retval 
 ******************************************************************************/
void set_eth_link_up(void)
{
    eth_link_state = ETH_LINK_UP;
}

void set_eth_link_down(void)
{
    eth_link_state = ETH_LINK_DOWN;
}

uint8_t is_eth_link_down(void)
{
    return (eth_link_state ==  ETH_LINK_DOWN);
}

/** ***************************************************************************
 * @name netif_ethernet_config_changed
 * @brief This function can only be called after changing the eth params
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void netif_ethernet_config_changed(void)
{
    if (!is_eth_link_down())
    {
        // close connected ntrip
        ntrip_link_down();

        if (gUserConfiguration.ethMode == ETHMODE_STATIC)
        {
            netif_set_static_ip(&gnetif);
        }
    }
}

/** ***************************************************************************
 * @name netif_ntrip_config_changed
 * @brief This function can only be called after changing the ntrip params
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void netif_ntrip_config_changed(void)
{
    if (!is_eth_link_down())
    {
        // close connected ntrip
        ntrip_link_down();
    }
}

/** ***************************************************************************
 * @name netif_set_static_ip
 * @brief set static ip
 * @param argument: the lwip network interface structure
 * @retval N/A
 ******************************************************************************/
void netif_set_static_ip(struct netif *netif)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;

    IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
    IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

#ifdef DEVICE_DEBUG
    printf("Use static ip:\r\n");
    printf("static ip:%d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    printf("netmask:%d.%d.%d.%d\r\n", NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
    printf("gateway:%d.%d.%d.%d\r\n", GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif

    netif_set_addr(netif, &ipaddr, &netmask, &gw);
}

/** ***************************************************************************
 * @name dhcp_link_down
 * @brief dhcp over
 * @param argument: N/A
 * @retval N/A
 ******************************************************************************/
void dhcp_link_down(void)
{
    if (eth_dhcp_state >= DHCP_STATE_START && eth_dhcp_state <= DHCP_STATE_ADDRESS_ASSIGNED)
    {
        eth_dhcp_state = DHCP_STATE_LINK_DOWN;
    }
}

/** ***************************************************************************
 * @name is_dhcp_address_assigned
 * @brief get dhcp state
 * @param argument: N/A
 * @retval true: dhcp has success  false: other
 ******************************************************************************/
uint8_t is_dhcp_address_assigned(void)
{
    return (eth_dhcp_state == DHCP_STATE_ADDRESS_ASSIGNED);
}

/** ***************************************************************************
 * @name dhcp_thread
 * @brief DHCP Process
 * @param argument: the lwip network interface structure
 * @retval N/A
 ******************************************************************************/
void dhcp_thread(void const *argument)
{
	struct netif *netif = (struct netif *)argument;

    for (;;)
    {
        if (gUserConfiguration.ethMode != ETHMODE_DHCP || is_eth_link_down())
        {
            dhcp_link_down();
        }
        
        switch (eth_dhcp_state)
        {
        case DHCP_STATE_START:
        {
#ifdef DEVICE_DEBUG
    printf("DHCP start...\r\n");
#endif
            ip_addr_set_zero(&netif->ip_addr);
            ip_addr_set_zero(&netif->netmask);
            ip_addr_set_zero(&netif->gw);
            dhcp_start(netif);
            eth_dhcp_state = DHCP_STATE_WAIT_ADDRESS;
        }
        break;

        case DHCP_STATE_WAIT_ADDRESS:
        {
            if (dhcp_supplied_address(netif)) // dhcp ok
            {
                eth_dhcp_state = DHCP_STATE_ADDRESS_ASSIGNED;

#ifdef DEVICE_DEBUG
                printf("DHCP ok!\r\n");
                printf("IP:%s\r\n", ipaddr_ntoa((const ip_addr_t *)&netif->ip_addr));
                printf("netmask:%s\r\n", ipaddr_ntoa((const ip_addr_t *)&netif->netmask));
                printf("gateway:%s\r\n", ipaddr_ntoa((const ip_addr_t *)&netif->gw));
#endif
            }
            else
            {
                /* DHCP timeout */
                if (netif->dhcp->tries > MAX_DHCP_TRIES)
                {
                    eth_dhcp_state = DHCP_STATE_TIMEOUT;

                    /* Stop DHCP */
                    dhcp_stop(netif);

#ifdef DEVICE_DEBUG
    printf("DHCP timeout!\r\n");
#endif
                }
            }
        }
        break;

        case DHCP_STATE_ADDRESS_ASSIGNED:
        {

        }
        break;

        case DHCP_STATE_TIMEOUT:
        {
            // process dhcp time out
            eth_dhcp_state = DHCP_STATE_OFF;
        }
        break;

        case DHCP_STATE_LINK_DOWN:
        {
            /* Stop DHCP */
#ifdef DEVICE_DEBUG
    printf("DHCP link down!\r\n");
#endif
            dhcp_stop(netif);
            eth_dhcp_state = DHCP_STATE_OFF;
        }
        break;

        case DHCP_STATE_OFF:
        {
            if (gUserConfiguration.ethMode == ETHMODE_DHCP && !is_eth_link_down())
            {
                eth_dhcp_state = DHCP_STATE_START;
            }
        }    
        break;

        default:
            break;
        }

        ethernetif_link_state_check(netif);
        osDelay(250);
        add_ntrip_stream_count();
    }
}

/** ***************************************************************************
 * @name dns_get_callback
 * @brief dns found, set the global server ip
 * @param argument: the lwip network interface structure
 * @retval N/A
 ******************************************************************************/
void dns_get_callback(const char *name, ip_addr_t *host_ip, void *callback_arg)
{
#ifdef DEVICE_DEBUG
    printf("ntrip:hostname %s | ip %s\r\n", name, ip_ntoa(host_ip));
#endif
}

/** ***************************************************************************
 * @name dns_get_ip_by_hostname
 * @brief dns found, set the global server ip
 * @param argument: the lwip network interface structure
 * @retval N/A
 ******************************************************************************/
uint8_t dns_get_ip_by_hostname(uint8_t *hostname, ip_addr_t* addr)
{
    if(dns_gethostbyname((char*)hostname, addr, dns_get_callback, NULL) == ERR_OK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

