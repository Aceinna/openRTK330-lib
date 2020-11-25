/** ***************************************************************************
 * @file handle_packet.c functions for handling serial UCB packets and CRM
 *       packets
 * @brief some of the callbacks for message handling of non-sensor data to be
 *        sent to Nav-View eg "PING".
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/

//********************************
#include <stdint.h>
#include "ucb_packet.h"
#include "serial_port.h"
#include "eepromAPI.h"
#include "crc16.h"
#include "calibrationAPI.h"
#include "configurationAPI.h"
#include "hwAPI.h"
#include "platformAPI.h"
#include "configureGPIO.h"
#include "user_message.h"
#include "bsp.h"
#include "commAPI.h"
#include "uart.h"

static UcbPacketStruct primaryUcbPacket;    /// all other data

BOOL fReset = FALSE;

void Reset(){fReset = TRUE;}

static void _SetNak(uint16_t port, UcbPacketStruct *ptrUcbPacket);

/** ****************************************************************************
 * @name _UcbPing
 * @brief Reply to a PING command
 * Trace: [SDD_UCB_PING <-- SRC_UCB_PING]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbPing (uint16_t port,
                      UcbPacketStruct    *ptrUcbPacket)
{
	ptrUcbPacket->payloadLength = 0; /// return ping acknowledgement
	HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbWriteApp
 * @brief Write data as 16 bit cells into an unlocked EEPROM.
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbWriteApp (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{
    uint32_t startAddress;
    uint8_t  wordsToWrite;
    uint16_t bytesToWrite;
// packet structure 
// header   code  payload len  start addr  numbytes              crc 
// 5555     5747  [x]          [yyyy]     [z]        [payload]   [cc]
    startAddress = (uint32_t)((ptrUcbPacket->payload[0] << 24) |
                               ptrUcbPacket->payload[1] << 16 |
                               ptrUcbPacket->payload[2] << 8 |
                               ptrUcbPacket->payload[3]);
    wordsToWrite = ptrUcbPacket->payload[4];
    bytesToWrite = (uint16_t)wordsToWrite;

//    SetMaxDelay_Watchdog();

    /// verify that the packet length matches packet specification
    if ((ptrUcbPacket->payloadLength == (bytesToWrite + 5))) {

		//if(!EEPROM_WriteApp(startAddress,&ptrUcbPacket->payload[5],bytesToWrite)){           //TODO:
		if(EEPROM_WriteApp(startAddress,&ptrUcbPacket->payload[5],bytesToWrite)){           //TODO:
            ptrUcbPacket->payloadLength = 5;
        } else {
            _SetNak(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
//    RestoreDelay_Watchdog();	
}

/** ****************************************************************************
 * @name _UcbJump2BOOT
 * @brief
 * Trace:
 *	[SDD_UCB_UNLOCK_EEPROM <-- SRC_UCB_UNLOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbJump2BOOT (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{
	HandleUcbTx(port, ptrUcbPacket);
    DelayMs(10);
    if(platformIsInBootMode()){
        // already there
        return;
    }
    HW_EnforceBootMode();
    HW_SystemReset();
} 

/** ****************************************************************************
 * @name _UcbJump2APP
 * @brief
 * Trace:
 *	[SDD_UCB_UNLOCK_EEPROM <-- SRC_UCB_UNLOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbJump2APP (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{
    HandleUcbTx(port, ptrUcbPacket);
    DelayMs(10);
    //return;
//    if(sigValid)  
    {
        HW_EnforceAppMode();
        HW_SystemReset();
    }
} 



/** ****************************************************************************
 * @name _UcbSoftwareReset
 * @brief Force a watchdog reset from the above function.
 *
 * Trace: [SDD_UCB_SW_RESET <-- SRC_UCB_SW_RESET]
 * softwareReset
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbSoftwareReset (uint16_t port,
                                UcbPacketStruct    *ptrUcbPacket)
{
    /// return software reset acknowledgement
	HandleUcbTx(port, ptrUcbPacket);

    DelayMs(10);
   
    HW_SystemReset();
}



/** ****************************************************************************
 * @name _SetNak
 * @brief set up UCB error NAK packet. Return NAK with requested packet type in
 *        data field. HandleUcbTx() needs to be called.
 * Trace:
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
static void _SetNak (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{

	/// return NAK, requested packet type placed in data field by external port
	ptrUcbPacket->packetType 	= UCB_NAK;
	ptrUcbPacket->payloadLength = UCB_PACKET_TYPE_LENGTH;
}

/** ****************************************************************************
 * @name _UcbError
 * @brief UCB error packet
 * Trace: [SDD_UCB_UNKNOWN_02 <-- SRC_UCB_UNKNOWN]
 *        [SDD_UCB_TIMEOUT_02 <-- SRC_UCB_TIMEOUT_REPLY]
 *        [SDD_UCB_CRC_FAIL_02 <-- SRC_UCB_CRCFAIL_REPLY]
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
static void _UcbError (uint16_t port,
                       UcbPacketStruct    *ptrUcbPacket)
{
	/// return NAK, requested packet type placed in data field by external port
	ptrUcbPacket->packetType 	= UCB_NAK;
	ptrUcbPacket->payloadLength = UCB_PACKET_TYPE_LENGTH;
    HandleUcbTx(port, ptrUcbPacket);
}


/** ****************************************************************************
 * @name HandleUcbPacket - API
 * @brief general handler
 * Trace: [SDD_HANDLE_PKT <-- SRC_HANDLE_PACKET]
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
void HandleUcbPacket (UcbPacketStruct *ptrUcbPacket)
{
    int result;

    uint16_t port = UART_USER;
    if (ptrUcbPacket)
    {
		switch (ptrUcbPacket->packetType) {
            case UCB_PING:
                _UcbPing(port, ptrUcbPacket); break;
            case UCB_J2BOOT:
            case UCB_J2IAP:
                _UcbJump2BOOT (port, ptrUcbPacket); break;
            case UCB_J2APP:                                     //TODO:
                _UcbJump2APP (port, ptrUcbPacket); break; 
            case UCB_SOFTWARE_RESET:
                _UcbSoftwareReset(port, ptrUcbPacket); break;
            case UCB_WRITE_APP:
                _UcbWriteApp(port, ptrUcbPacket); break;
                
            case UCB_USER_IN:
                result = HandleUserInputPacket(ptrUcbPacket);
                if(result != USER_PACKET_OK){
  		            _SetNak(port,ptrUcbPacket);
                }
                HandleUcbTx(port, ptrUcbPacket);
                if(fReset){
                    fReset = FALSE;
                    OS_Delay(10);
                    NVIC_SystemReset();
                }
                break;
            default:
                _UcbError(port, ptrUcbPacket); break;
                break; /// default handler - unknown send NAK
		}
    }
}
/* end HandleUcbPacket() */


/** ****************************************************************************
 * @name _ProcessUcbCommands
 *
 * @brief  This routine will test for the a port to be assigned to the UCB
 * function and test that port for a received packet.  If the packet is received
 * it will call a handler.
 *
 * Trace:
 * [SDD_PROCESS_USER_PORTS_01 <-- SRC_PROCESS_UCB_COMMANDS]
 * [SDD_PROCESS_USER_PORTS_02 <-- SRC_PROCESS_UCB_COMMANDS]
 * [SDD_PROCESS_USER_PORTS_03 <-- SRC_PROCESS_UCB_COMMANDS]
 * [SDD_PROCESS_COMMANDS_SEQ <-- SRC_PROCESS_UCB_COMMANDS]
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/

void handle_tcp_commands(void)
{
    HandleUcbRx (&primaryUcbPacket);
}


void ProcessUserCommands (void)
{
    /// check received packets and handle appropriately
    HandleUcbRx (&primaryUcbPacket);

} /* end ProcessUcbCommands() */

