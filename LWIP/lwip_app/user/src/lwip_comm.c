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
#include "user_config.h"
#include "cmsis_os.h"
#include "station_tcp.h"

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
	osThreadDef(DHCP, dhcp_thread, osPriorityLow, 0, TASK_DHCP_STACK);
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

        if (get_eth_mode() == ETHMODE_STATIC)
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
        
        if (get_eth_mode() == ETHMODE_STATIC)
        {
            netif_set_static_ip(netif);
        }

        netif_set_up(netif);
    }
    else
    {
#ifdef STATION_TCP_DEBUG
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
        // close connected
        station_tcp_stop();

        if (get_eth_mode() == ETHMODE_STATIC)
        {
            netif_set_static_ip(&gnetif);
        }
    }
}

/** ***************************************************************************
 * @name netif_station_tcp_config_changed
 * @brief This function can only be called after changing the ntrip params
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void netif_station_tcp_config_changed(void)
{
    // close connected
    station_tcp_stop();
}

/** ***************************************************************************
 * @name netif_set_static_ip
 * @brief set static ip
 * @param argument: the lwip network interface structure
 * @retval N/A
 ******************************************************************************/
void netif_set_static_ip(struct netif *netif)
{
    ip_addr_t ip, netmask, gateway;
    uint8_t *ip_ary, *netmask_ary, *gateway_ary;

    ip_ary = get_static_ip();
    netmask_ary = get_static_netmask();
    gateway_ary = get_static_gateway();

    IP4_ADDR(&ip, ip_ary[0], ip_ary[1], ip_ary[2], ip_ary[3]);
    IP4_ADDR(&netmask, netmask_ary[0], netmask_ary[1], netmask_ary[2], netmask_ary[3]);
    IP4_ADDR(&gateway, gateway_ary[0], gateway_ary[1], gateway_ary[2], gateway_ary[3]);

#ifdef STATION_TCP_DEBUG
    printf("Use static ip:\r\n");
    printf("static ip:%d.%d.%d.%d\r\n", ip_ary[0], ip_ary[1], ip_ary[2], ip_ary[3]);
    printf("netmask:%d.%d.%d.%d\r\n", netmask_ary[0], netmask_ary[1], netmask_ary[2], netmask_ary[3]);
    printf("gateway:%d.%d.%d.%d\r\n", gateway_ary[0], gateway_ary[1], gateway_ary[2], gateway_ary[3]);
#endif

    netif_set_addr(netif, &ip, &netmask, &gateway);
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

uint8_t* get_netif_ip(void)
{
    if (get_eth_mode() == ETHMODE_DHCP) {
        return (uint8_t*)&gnetif.ip_addr;
    } else {  // ETHMODE_STATIC
        return get_static_ip();
    }
}

uint8_t* get_netif_netmask(void)
{
    if (get_eth_mode() == ETHMODE_DHCP) {
        return (uint8_t*)&gnetif.netmask;
    } else {  // ETHMODE_STATIC
        return get_static_netmask();
    }
}

uint8_t* get_netif_gateway(void)
{
    if (get_eth_mode() == ETHMODE_DHCP) {
        return (uint8_t*)&gnetif.gw;
    } else {  // ETHMODE_STATIC
        return get_static_gateway();
    }
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
        if (get_eth_mode() != ETHMODE_DHCP || is_eth_link_down())
        {
            dhcp_link_down();
        }
        
        switch (eth_dhcp_state)
        {
        case DHCP_STATE_START:
        {
#ifdef STATION_TCP_DEBUG
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

                // printf("DHCP ok!\r\n");
                // printf("IP:%s\r\n", ipaddr_ntoa((const ip_addr_t *)&netif->ip_addr));
                // printf("netmask:%s\r\n", ipaddr_ntoa((const ip_addr_t *)&netif->netmask));
                // printf("gateway:%s\r\n", ipaddr_ntoa((const ip_addr_t *)&netif->gw));
#ifdef STATION_TCP_DEBUG
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

#ifdef STATION_TCP_DEBUG
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
#ifdef STATION_TCP_DEBUG
    printf("DHCP link down!\r\n");
#endif
            dhcp_stop(netif);
            eth_dhcp_state = DHCP_STATE_OFF;
        }
        break;

        case DHCP_STATE_OFF:
        {
            if (get_eth_mode() == ETHMODE_DHCP && !is_eth_link_down())
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
        station_tcp_add_stream_timeout();
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
#ifdef STATION_TCP_DEBUG
    printf("hostname %s | ip %s\r\n", name, ip_ntoa(host_ip));
#endif
}

/** ***************************************************************************
 * @name get_ip_by_host
 * @brief dns found, set the global server ip
 * @param argument: the lwip network interface structure
 * @retval N/A 
 ******************************************************************************/
err_t get_ip_by_host(const char *host, ip_addr_t *ip)
{
    err_t err = ERR_OK;
    uint8_t i = 0;

    if (!ipaddr_aton(host, ip)) {
        for (i = 0; i < 5; i++) {
            err = dns_gethostbyname(host, ip, dns_get_callback, NULL);
            if (!err)
                break;
            OS_Delay(3000);
        }
    }
    
    return err;
}
