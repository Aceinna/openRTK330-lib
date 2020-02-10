#include "LwipComm.h"
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
#include "osresources.h"
#include <stdio.h>
#include <string.h>
#include "httpd.h"
#include "netbios.h"
#include "ntripClient.h"
#include "UserConfiguration.h"
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
    
    User_notification(&gnetif);

	/* Start DHCPClient */
	osThreadDef(DHCP, DHCP_thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
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
 * @name User_notification
 * @brief Notify the User about the nework interface config status.
 * @param [in] netif : the lwip network interface structure for this ethernetif
 * @retval N/A
 ******************************************************************************/
void User_notification(struct netif *netif)
{
    if (netif_is_up(netif))
    {
        eth_link_state = ETH_LINK_UP;

        if (gUserConfiguration.ethMode == ETHMODE_STATIC)
        {
            netif_set_static_ip(netif);
        }
    }
    else
    {
        eth_link_state = ETH_LINK_DOWN;
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
        eth_link_state = ETH_LINK_UP;
        
        if (gUserConfiguration.ethMode == ETHMODE_STATIC)
        {
            netif_set_static_ip(netif);
        }

        netif_set_up(netif);
    }
    else
    {
        eth_link_state = ETH_LINK_DOWN;

        netif_set_down(netif);
    }
}

/** ***************************************************************************
 * @name get_eth_link_state
 * @brief 
 * @param N/A
 * @retval link state
 ******************************************************************************/
uint8_t get_eth_link_state(void)
{
    return eth_link_state;
}

/** ***************************************************************************
 * @name netif_ethernet_config_changed
 * @brief This function can only be called after changing the eth params
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void netif_ethernet_config_changed(void)
{
    if (get_eth_link_state())
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
    if (get_eth_link_state())
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

    printf("Use static ip:\r\n");
    printf("static ip:%d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    printf("netmask:%d.%d.%d.%d\r\n", NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
    printf("gateway:%d.%d.%d.%d\r\n", GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

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
 * @name DHCP_thread
 * @brief DHCP Process
 * @param argument: the lwip network interface structure
 * @retval N/A
 ******************************************************************************/
void DHCP_thread(void const *argument)
{
	struct netif *netif = (struct netif *)argument;

    for (;;)
    {
        if (gUserConfiguration.ethMode != ETHMODE_DHCP || eth_link_state == ETH_LINK_DOWN)
        {
            dhcp_link_down();
        }
        
        switch (eth_dhcp_state)
        {
        case DHCP_STATE_START:
        {
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

                printf("DHCP ok!\r\n");
                printf("IP:%s\r\n", ipaddr_ntoa((const ip_addr_t *)&netif->ip_addr));
                printf("netmask:%s\r\n", ipaddr_ntoa((const ip_addr_t *)&netif->netmask));
                printf("gateway:%s\r\n", ipaddr_ntoa((const ip_addr_t *)&netif->gw));
            }
            else
            {
                /* DHCP timeout */
                if (netif->dhcp->tries > MAX_DHCP_TRIES)
                {
                    eth_dhcp_state = DHCP_STATE_TIMEOUT;

                    /* Stop DHCP */
                    dhcp_stop(netif);

                    printf("DHCP timeout!\r\n");
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
            /* use static ip */
            netif_set_static_ip(netif);
            eth_dhcp_state = DHCP_STATE_OFF;
        }
        break;

        case DHCP_STATE_LINK_DOWN:
        {
            /* Stop DHCP */
            dhcp_stop(netif);
            eth_dhcp_state = DHCP_STATE_OFF;
        }
        break;

        case DHCP_STATE_OFF:
        {
            if (gUserConfiguration.ethMode == ETHMODE_DHCP && eth_link_state == ETH_LINK_UP)
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
        ntripStreamCount++;
    }
}

/** ***************************************************************************
 * @name dnsFoundCallback
 * @brief dns found, set the global server ip
 * @param argument: the lwip network interface structure
 * @retval N/A
 ******************************************************************************/
void dnsFoundCallback(const char *name, ip_addr_t *host_ip, void *callback_arg)
{
    printf("ntrip:hostname %s | ip %s\r\n", name, ip_ntoa(host_ip));
}

/** ***************************************************************************
 * @name dns_get_ip_by_hostname
 * @brief dns found, set the global server ip
 * @param argument: the lwip network interface structure
 * @retval N/A
 ******************************************************************************/
uint8_t dns_get_ip_by_hostname(uint8_t *hostname, ip_addr_t* addr)
{
    if(dns_gethostbyname((char*)hostname, addr, dnsFoundCallback, NULL) == ERR_OK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

