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
