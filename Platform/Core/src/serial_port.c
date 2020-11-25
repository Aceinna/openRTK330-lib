/** ***************************************************************************
 * @file extern_port.c functions for general external port interface
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief UCB (Unified Code Base) external serial interface
 *****************************************************************************/
/*******************************************************************************
Copyright 2020 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#include <stdint.h>
#include "constants.h"
#include "serial_port.h"
#include "uart.h"
#include "crc16.h"
#include "ucb_packet.h"
#include "platformAPI.h"
#include "configuration.h"
#include "user_message.h"
#include "main.h"
#include "tcp_driver.h"

typedef struct{
    int      type;
    uint16_t code;
}ucbInputSyncTableEntry_t;
// NEEDS TO BE CHECKED
/// List of allowed input packet codes
ucbInputSyncTableEntry_t ucbInputSyncTable[] = {
    {UCB_PING,              0x504B},    //  "PK"
    {UCB_SOFTWARE_RESET,    0x5352},    //  "SR"
    {UCB_WRITE_APP,         0x5741},    //  "WA"
    {UCB_J2BOOT,            0x4A42},    //  "JB"
    {UCB_J2IAP,             0x4A49},    //  "JI"
    {UCB_J2APP,             0x4A41},    //  "JA"
    {UCB_INPUT_PACKET_MAX,  0x00000000},    //  "  "
};

uint8_t dataBuffer[512];

/** ****************************************************************************
 * @name HandleUcbRx
 * @brief handles received ucb packets
 * Trace:
 *	[SDD_UCB_TIMEOUT_01 <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_PACKET_CRC <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_CONVERT_DATA <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_STORE_DATA <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_UNKNOWN_01 <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_CRC_FAIL_01 <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_VALID_PACKET <-- SRC_HANDLE_UCB_RX]
 *
 * @param [in] port - logical port type
 * @param [out] packetPtr - UCB packet to read the packet into
 * @retval TRUE if a full packet has been seen (can fail CRC)
 *         FALSE if needing more to fill in a packet
 ******************************************************************************/
extern client_s driver_client;

BOOL HandleUcbRx (UcbPacketStruct  *ucbPacket)
{
    static int state = 0, crcError = 0, len = 0;
    static uint8_t *ptr;
    static uint16_t crcMsg = 0, code;
	static uint32_t sync = 0;
    unsigned char tmp;
	unsigned int  pos = 0, synced = 0, type;
	uint16_t crcCalc, bytesInBuffer = 0;
    ucbInputSyncTableEntry_t *syncTable;
    
	while(1){
        if(!bytesInBuffer){
            if(get_tcp_driver_state() == CLIENT_STATE_INTERACTIVE)
            {
                client_read_data(&driver_client, dataBuffer, &bytesInBuffer);
            }
            if(bytesInBuffer == 0)
            {
                bytesInBuffer = uart_read_bytes(UART_USER,dataBuffer, sizeof(dataBuffer),0);
                fifo_push(&fifo_user_uart,dataBuffer,bytesInBuffer);
            }
            if(!bytesInBuffer){
                return 0; // nothing to do
            }
            pos = 0; 
        }
        tmp = dataBuffer[pos++];
        bytesInBuffer--;
        sync   = (sync << 8) | tmp;
        synced = 0;
        if((sync & 0xFFFF0000) == 0x55550000){
            code = sync & 0xffff;
            syncTable = ucbInputSyncTable;
            while (syncTable->type != UCB_INPUT_PACKET_MAX)
            {
                if (syncTable->code == code){
                    synced = 1;
                    type   = syncTable->type;
                    break;
                }
                syncTable++;
            }
            if(!synced){
                type = checkUserInPacketType(code);
                if(type != UCB_ERROR_INVALID_TYPE){
                    synced = 1;
                }
            }
        }
        if(synced){
            ucbPacket->packetType    = type;
            ucbPacket->payloadLength = 0;
            ucbPacket->code_MSB      = (sync >> 8) & 0xff;
            ucbPacket->code_LSB      = sync & 0xff;
        	state  = 1;
		    len    = 0;
            synced = 0;
            continue;
        }
        switch(state){
        case 0:
            break;
        case 1:
            ucbPacket->payloadLength = tmp;
            if(tmp == 0){
                state = 3;  // crc next
            }else{
                state = 2;  // data next
                len   = 0;
            }
            ptr   = ucbPacket->payload;
            break;
        case 2:
            if(len++ > UCB_MAX_PAYLOAD_LENGTH){
                state = 0;
                break;
            }
            *ptr++ = tmp;
            if(len == ucbPacket->payloadLength){
                //crc next
                state  = 3;
                crcMsg = 0; 
            }
            break;
        case 3:
            crcMsg = tmp;
            *ptr++ = tmp;   
            state = 4;
            break;
        case 4:
            state   = 0;
            crcMsg  = crcMsg | ((uint16_t)tmp << 8);
            *ptr++  = tmp;   
            crcCalc = CalculateCRC((uint8_t*)&ucbPacket->code_MSB, len + 3);
            if(crcMsg != crcCalc){
                crcError++;
            }else {
                // process message here
               HandleUcbPacket (ucbPacket);
               return 0;   // will come back later
            }
            break;
        default:
            while(1){}; // should not be here
        }
    }

}
/* end HandleUcbRx */

/** ****************************************************************************
 * @name HandleUcbTx
 * @brief builds a UCB packet and then triggers transmission of it. Packet:
 *  Preamble = 0x5555
 *  Packet Type 0x####
 *  Length 0x##
 *  payload (uint8_t)data[Length]
 *  CRC 0x####
 * Trace: [SDD_UCB_PROCESS_OUT <-- SRC_UCB_OUT_PKT]
 * @param [in] port - port type UCB or CRM
 * @param [in] packetPtr -- buffer structure with payload, type and size
 * @retval valid packet in packetPtr TRUE
 ******************************************************************************/
void HandleUcbTx (int port, UcbPacketStruct *ptrUcbPacket)
{
	uint16_t crc;
	uint8_t data[2];
    bool ret;

	/// get byte representation of packet type, index adjust required since sync
    /// isn't placed in data array
    ptrUcbPacket->sync_MSB = 0x55;
	ptrUcbPacket->sync_LSB = 0x55;

	ret = ucbpacket_type_to_bytes(ptrUcbPacket->packetType, data);
    if (ret) {
        ptrUcbPacket->code_MSB = data[0];
	    ptrUcbPacket->code_LSB = data[1];
    } else {
        ptrUcbPacket->payloadLength = 2;
        ptrUcbPacket->payload[0]    =  ptrUcbPacket->code_MSB;
        ptrUcbPacket->payload[1]    =  ptrUcbPacket->code_LSB;
        ptrUcbPacket->code_MSB = 0x15; // NAK
        ptrUcbPacket->code_LSB = 0x15;
    }

    crc = CalculateCRC((uint8_t *)&ptrUcbPacket->code_MSB, ptrUcbPacket->payloadLength + 3);
    ptrUcbPacket->payload[ptrUcbPacket->payloadLength+1]   = (crc >> 8) & 0xff;
    ptrUcbPacket->payload[ptrUcbPacket->payloadLength]     =  crc  & 0xff;
    if(get_tcp_driver_state() == CLIENT_STATE_INTERACTIVE)
    {
        client_write_data(&driver_client, &ptrUcbPacket->sync_MSB, ptrUcbPacket->payloadLength + 7,  0x01);
    }
    uart_write_bytes(port, (const char *)&ptrUcbPacket->sync_MSB, ptrUcbPacket->payloadLength + 7,1);
}
/* end HandleUcbTx */
