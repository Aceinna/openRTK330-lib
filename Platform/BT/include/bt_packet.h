#if 0
#ifndef _BT_PACKET_H_
#define _BT_PACKET_H_

#include <stdint.h>

#define BT_CMD_NUM  2
#define CMD_MAX_LEN 20


enum BT_CMD_TYPE
{
    RTK_MES_CMD = 0,
    RTK_SN_CMD,
    BT_CMD_MAX
};

int bt_uart_parse(uint8_t* bt_buff);

#endif
#else
/*******************************************************************************
* File Name          : bt_packet.h
* Author             : daich
* Revision           : 1.0
* Date               : 
* Description        : bt_packet.h
*
* HISTORY***********************************************************************
* 24/04/2020  |                                             | Daich
* Description: created
*******************************************************************************/

#ifndef _BT_INTERFACE_H_
#define _BT_INTERFACE_H_
#include "stdint.h" 
#include "constants.h"        
#include "stdio.h"


#define BT_CMD_NUM  2
#define CMD_MAX_LEN 20
enum BT_CMD_TYPE
{
    RTK_MES_CMD = 0,
    RTK_SN_CMD,
    BT_CMD_MAX
};

#define SHELL_ENABLE        false
#define SHELL_TASK          false
#define SHELL_RX_MAX        (256+32)        
#define SHELL_TX_MAX        (512)          

bool is_cmd_right(void * buffer,void * cmd);   
void shell_service(void); 
void debug_uart_handle(void);
void ConsoleTask(void const *argument);
void parse_debug_cmd();
int bt_uart_parse(uint8_t* bt_buff);
#endif
#endif