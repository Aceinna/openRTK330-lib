/*******************************************************************************
* File Name          : can.h
* Author             : ymj
* Revision           : 1.0
* Date               : 3/31/2020
* Description        : can.h
*
* HISTORY***********************************************************************
* 3/31/2020  |                                             | ymj
* Description: create
*******************************************************************************/

#ifndef _CAN_H_
#define _CAN_H_

#include <stdint.h>
#include "constants.h"
#include "stm32f4xx_hal.h"

// supported baud rate
typedef enum {
  _ECU_125K      =    0,                    // 125kbps
  _ECU_250K      =    1,                    // 250kbps
  _ECU_500K      =    2,                    // 500kbps
  _ECU_1000K     =    3                     // 1000kbps
} _ECU_BAUD_RATE;

// MTLT's state machine
typedef enum {
  DESC_IDLE                  =   0,        // ready for being used
  DESC_OCCUPIED              =   1,        // unavailable
  DESC_PENDING               =   2         // in queue
} DESC_STATE;

// MTLT's ODR on CAN
enum {
  CAN_PACKET_RATE_0           =           0,   //quiet
  CAN_PACKET_RATE_2           =           2,   // 2Hz
  CAN_PACKET_RATE_5           =           5,   // 5Hz
  CAN_PACKET_RATE_10          =           10,  // 10Hz
  CAN_PACKET_RATE_20          =           20,  // 20Hz
  CAN_PACKET_RATE_25          =           25,  // 25Hz
  CAN_PACKET_RATE_50          =           50,  // 50Hz
  CAN_PACKET_RATE_100         =           100, // 100Hz
};

typedef struct
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8];
} CanTxMsg;

typedef struct
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];
} CanRxMsg;

#define CAN_ERROR                  -1
#define CAN_NO_ERROR                1

#define USER_CAN_IDE                1
#define USER_CAN_RTR                0

#define CAN_BAUD_RATE_RETRY         4          // retry times for baud rate auto detection
#define CAN_DETECT_TIME             3000       // ms, listening period at each of channels

extern uint32_t canRxIntCounter, canStartDetectRxIntCounter; 
extern uint32_t filterNum;

void can_config(uint8_t mode, int baudRate);
void can_config_filter_mask_message(uint32_t baseID, uint32_t baseMask);
void can_config_filter_list_message(uint32_t ID1, uint32_t ID2);
void can_config_filter_clear(void);
void can_rtx_fun_config(void (*callback1)(void), void(*callback2)(void));
BOOL can_transmit(CanTxMsg* canTxMsg);
BOOL can_detect_baudrate(_ECU_BAUD_RATE *rate);


#endif