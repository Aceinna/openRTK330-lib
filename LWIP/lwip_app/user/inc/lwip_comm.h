#ifndef _LWIP_COMM_H
#define _LWIP_COMM_H

#include <stdio.h>
#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "user_config.h"

#define ETH_LINK_DOWN 0
#define ETH_LINK_UP 1

#define MAX_DHCP_TRIES 5

typedef enum
{
    DHCP_STATE_OFF                  = 0,
    DHCP_STATE_START                = 1,
    DHCP_STATE_WAIT_ADDRESS         = 2,        
    DHCP_STATE_ADDRESS_ASSIGNED     = 3,
    DHCP_STATE_TIMEOUT              = 4,
    DHCP_STATE_LINK_DOWN            = 5
} dhcp_state_enum_t;

extern struct netif gnetif;
extern uint8_t eth_link_state;
extern uint8_t eth_dhcp_state;

void ethernet_init(void);

uint8_t dhcp_supplied_address(const struct netif *netif);
void user_notification(struct netif *netif);
void ethernetif_notify_conn_changed(struct netif *netif);

void set_eth_link_up(void);
void set_eth_link_down(void);
uint8_t is_eth_link_down(void);

void netif_ethernet_config_changed(void);
void netif_station_tcp_config_changed(void);
void netif_set_static_ip(struct netif *netif);
void dhcp_link_down(void);
uint8_t is_dhcp_address_assigned(void);

uint8_t* get_netif_ip(void);
uint8_t* get_netif_netmask(void);
uint8_t* get_netif_gateway(void);

void dhcp_thread(void const *argument);

err_t get_ip_by_host(const char *host, ip_addr_t *ip);


#endif
