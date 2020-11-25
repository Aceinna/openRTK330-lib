/** ***************************************************************************
 * @file ucb_packet.c utility functions for interfacing with Memsic proprietary
 *       UCB (unified code base) packets
 * @brief UCB Packet Type (char to byte) eg 'P' 'K' -> 0x504b or bytes to chars
 *        and CRC calculation and conversion
*
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY O ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
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


#include <string.h> // strcmp, strstr
#include "ucb_packet.h"
#include "user_message.h"

// NEEDS TO BE CHECKED
/// List of allowed packet codes 
ucb_packet_t ucbPackets[] = {		//       
    {UCB_J2IAP,              0x4A49},   //  "JI"
    {UCB_PING,               0x504B},   //  "PK" 
    {UCB_SOFTWARE_RESET,     0x5352},   //  "SR"
    {UCB_WRITE_APP,          0x5741},   //  "WA"
    {UCB_IDENTIFICATION,     0x4944},   //  "ID" 
    {UCB_VERSION_DATA,       0x5652},   //  "VR" 
    {UCB_VERSION_ALL_DATA,   0x5641},   //  "VA" 
    {UCB_PKT_NONE,           0x0000}   //  "  "     should be last in the table as a end marker 
};



/** ****************************************************************************
 * @name ucbpacket_bytes_to_type
 * @brief Convert the packet bytes into the packet type enum eg "PK" -> 0x504B
 * Trace:
 * [SDD_UCB_UNKNOWN_01 <-- SRC_UCB_PKT_ENUM]
 * [SDD_HANDLE_PKT  <-- SRC_UCB_PKT_ENUM]
 * [SDD_UCB_VALID_PACKET <-- SRC_UCB_PKT_ENUM]
 * @param [in] byte array, containing one byte
 * @Retval packet type enum
 ******************************************************************************/
UcbPacketType ucbpacket_bytes_to_type(const uint8_t bytes[])
{
    UcbPacketType packetType = UCB_ERROR_INVALID_TYPE;
    ucb_packet_t *pkt = ucbPackets;
    uint16_t receivedCode = (uint16_t)(((bytes[0] & 0xff) << 8) | (bytes[1] & 0xff));

    /// search through the packet code table for a matching code - check type
    /// against valid types

    while (pkt->packetType != UCB_PKT_NONE) {
        if (receivedCode == pkt->packetCode) {
            packetType = pkt->packetType;
            break;
        }
        pkt++;
    }

    if (packetType == UCB_ERROR_INVALID_TYPE) {
        packetType = (UcbPacketType)checkUserOutPacketType(receivedCode);
    }

    return packetType;
}
/* end ucbpacket_bytes_to_type */

/******************************************************************************
 * Function name:	UcbPacketTypeToBytes
* @brief Convert the packet type enum into bytes ['P'] ['K'] -> 0x504B
 * Trace:
 * [SDD_UCB_UNKNOWN_01 <-- SRC_UCB_PKT_STR]
 * [SDD_HANDLE_PKT  <-- SRC_UCB_PKT_STR]
 * @param [in] byte array, containing one byte
 * @Retval length
 ******************************************************************************/
BOOL ucbpacket_type_to_bytes(UcbPacketType type, uint8_t bytes[])
{
    ucb_packet_t *ptr = ucbPackets;
    // carefull here, since for sake of speed it's indexing but not lookup
    // through ucbPackets structure

    if (type == UCB_USER_IN) {
        user_inpacket_type_to_bytes(bytes);
        return TRUE;
    }

    while (ptr->packetType != UCB_PKT_NONE) {
        if (ptr->packetType == type) {
            bytes[0] = (uint8_t)((ptr->packetCode >> 8) & 0xff);
            bytes[1] = (uint8_t)(ptr->packetCode & 0xff);
            return TRUE;
        }
        ptr++;
    }

    bytes[0] = 0;
    bytes[1] = 0;

    return FALSE;
}
/* end ucbpacket_type_to_bytes */

/** ****************************************************************************
 * @name UcbPacketIsAnOutputPacket API
 * @brief Returns TRUE if given packet type is an output packet type
 * Trace: [SDD_PORT_CFG_VALID_03 <-- SRC_UCB_PKT_OUTTYPE]
 * @param [in] UCB packet type
 * @retval TRUE if output packet type, FALSE otherwise
 ******************************************************************************/
BOOL UcbPacketIsAnOutputPacket (UcbPacketType type)
{
	BOOL isAnOutputPacket = TRUE;
    // Up to user to decide if packet can be enabled for output 
	switch (type) {
        case UCB_IDENTIFICATION:
        case UCB_VERSION_DATA:
        case UCB_VERSION_ALL_DATA:
            break;
		default:
          isAnOutputPacket = FALSE;
	}

    if((int)type == UCB_USER_OUT){
          isAnOutputPacket = TRUE;
    }


	return isAnOutputPacket;
}
/* end UcbPacketIsAnOutputPacket */

