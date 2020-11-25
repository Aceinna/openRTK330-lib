/** ***************************************************************************
  * @file ucb_packet.h utility functions for interfacing with Memsic proprietary
  *       UCB (unified code base) packets.  UCB packet structure
  * @Author rhilles
  * @date  2010-08-03 10:20:52 -0700 (Tue, 03 Aug 2010)
  * @rev 15924
  * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
  *
  * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY O ANY
  * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  * PARTICULAR PURPOSE.
  *
  * @brief these are in ucb_packet_types.def on the 440 these were in
  *        xbowProtocol.h
  *****************************************************************************/
#ifndef UCB_PACKET_H
#define UCB_PACKET_H

#include "stdio.h"
#include "constants.h"

#define UCB_SYNC_LENGTH				2
#define UCB_PACKET_TYPE_LENGTH		2
#define UCB_PAYLOAD_LENGTH_LENGTH	1
#define UCB_CRC_LENGTH				2

#define UCB_SYNC_INDEX				0
#define UCB_PACKET_TYPE_INDEX		(UCB_SYNC_INDEX + UCB_SYNC_LENGTH)
#define UCB_PAYLOAD_LENGTH_INDEX    (UCB_PACKET_TYPE_INDEX + UCB_PACKET_TYPE_LENGTH)


#define UCB_MAX_PAYLOAD_LENGTH		255
#define UCB_USER_IN                 200         
#define UCB_USER_OUT                201      
#define UCB_ERROR_INVALID_TYPE      202     


typedef struct {
    uint16_t        packetType;
    uint16_t        packetCode;
}ucb_packet_t;

typedef struct {
    uint16_t        packetType;
    uint8_t         packetCode[2];
}usr_packet_t;


typedef struct {
     uint8_t       packetType;      // 0
     uint8_t       sync_MSB;        // 3
     uint8_t       sync_LSB;        // 4
     uint8_t       code_MSB;        // 5
     uint8_t       code_LSB;        // 6
     uint8_t	   payloadLength;   // 7
     uint8_t       payload[UCB_MAX_PAYLOAD_LENGTH + 3]; // aligned to 4 bytes 
} UcbPacketStruct;

// NEEDS TO BE CHECKED
//  Xbow Packet Code
typedef enum
{
    UCB_PING,               //  0 PK 0x504B input packets
    UCB_SOFTWARE_RESET,     // 11 SR 0x5352
    UCB_WRITE_APP,          // 14 WA 0x5743
    UCB_J2BOOT,             // 15 JB 0x4A42
    UCB_J2IAP,              // 16 JI 0x4A49
    UCB_J2APP,              // 17 JA 0x4A41
    UCB_INPUT_PACKET_MAX,
//**************************************************
    UCB_IDENTIFICATION,     // 18 ID 0x4944 output packets
    UCB_VERSION_DATA,       // 19 VR 0x4652
    UCB_VERSION_ALL_DATA,   // 20 VA 0x5641
//**************************************************
    UCB_PKT_NONE,           // 27   marker after last valid packet 
    UCB_NAK,                // 28
} UcbPacketType;


#define UCB_IDENTIFICATION_LENGTH 69
#define UCB_VERSION_DATA_LENGTH 5
#define UCB_VERSION_ALL_DATA_LENGTH 15


/// UCB packet-specific utility functions ucb_packet.c
extern UcbPacketType     ucbpacket_bytes_to_type    (const uint8_t bytes []);
extern BOOL              ucbpacket_type_to_bytes    (UcbPacketType type, uint8_t bytes []);
extern BOOL UcbPacketIsAnOutputPacket(UcbPacketType type);

// send_packet.c
extern void SendUcbPacket(uint16_t port, UcbPacketStruct *ptrUcbPacket);
// handle packet.c
extern void HandleUcbPacket(UcbPacketStruct *ptrUcbPacket);


#endif
