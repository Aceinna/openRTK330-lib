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
Copyright 2018 ACEINNA, INC

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

//#include "UserConfiguration.h"
#include "ucb_packet.h"


// NEEDS TO BE CHECKED
/// List of allowed packet codes 
ucb_packet_t ucbPackets[] = {		//       
    {UCB_WRITE_APP,          0x5741},   //  "WA" 
    {UCB_JUMP2_BOOT,         0x4A42},   //  "JB" 
    {UCB_J2IAP,              0x4A49},   //  "JI"
    {UCB_JUMP2_APP,          0x4A41},   //  "JA" 
    {UCB_PING,               0x504B},   //  "PK" 
    {UCB_ECHO,               0x4348},   //  "CH" 
    {UCB_GET_PACKET,         0x4750},   //  "GP" 
    {UCB_SET_FIELDS,         0x5346},   //  "SF" 
    {UCB_GET_FIELDS,         0x4746},   //  "GF" 
    {UCB_READ_FIELDS,        0x5246},   //  "RF" 
    {UCB_WRITE_FIELDS,       0x5746},   //  "WF" 
    {UCB_UNLOCK_EEPROM,      0x5545},   //  "UE" 
    {UCB_READ_EEPROM,        0x5245},   //  "RE" 
    {UCB_WRITE_EEPROM,       0x5745},   //  "WE" 
    {UCB_PROGRAM_RESET,      0x5052},   //  "PR" 
    {UCB_SOFTWARE_RESET,     0x5352},   //  "SR" 
    {UCB_WRITE_CAL,          0x5743},   //  "WC" 
    {UCB_IDENTIFICATION,     0x4944},   //  "ID" 
    {UCB_VERSION_DATA,       0x5652},   //  "VR" 
    {UCB_VERSION_ALL_DATA,   0x5641},   //  "VA" 
    {UCB_SCALED_0,           0x5330},   //  "S0" 
    {UCB_SCALED_1,           0x5331},   //  "S1" 
    {UCB_SCALED_M,           0x534D},   //  "SM"
    {UCB_TEST_0,             0x5430},   //  "T0" 
    {UCB_FACTORY_1,          0x4631},   //  "F1" 
    {UCB_FACTORY_2,          0x4632},   //  "F2"
    {UCB_FACTORY_M,          0x464D},   //  "FM"
    {UCB_MAG_CAL_1_COMPLETE, 0x4342},   //  "CB" 
    {UCB_MAG_CAL_3_COMPLETE, 0x4344},   //  "CD" 
    {UCB_USER_OUT,           0x5550},   //  "UP" 
    {UCB_PKT_NONE,           0x0000}   //  "  "     should be last in the table as a end marker 
};



/** ****************************************************************************
 * @name UcbPacketBytesToPacketType
 * @brief Convert the packet bytes into the packet type enum eg "PK" -> 0x504B
 * Trace:
 * [SDD_UCB_UNKNOWN_01 <-- SRC_UCB_PKT_ENUM]
 * [SDD_HANDLE_PKT  <-- SRC_UCB_PKT_ENUM]
 * [SDD_UCB_VALID_PACKET <-- SRC_UCB_PKT_ENUM]
 * @param [in] byte array, containing one byte
 * @Retval packet type enum
 ******************************************************************************/
UcbPacketType UcbPacketBytesToPacketType (const uint8_t bytes [])
{
	UcbPacketType packetType = UCB_ERROR_INVALID_TYPE;
    ucb_packet_t *pkt = ucbPackets; 
	UcbPacketCodeType receivedCode = (UcbPacketCodeType)(((bytes[0] & 0xff) << 8) |
                                                          (bytes[1] & 0xff));

    /// search through the packet code table for a matching code - check type
    /// against valid types

	while (pkt->packetType != UCB_PKT_NONE) {
        if ( receivedCode == pkt->packetCode ) {
            packetType = pkt->packetType;
            break;
        }
        pkt++;
	}
    
#ifndef USER_PACKETS_NOT_SUPPORTED
    if(packetType == UCB_ERROR_INVALID_TYPE){
        packetType = (UcbPacketType)checkUserPacketType(receivedCode);
    }
#endif
	return packetType;
}
/* end UcbPacketBytesToPacketType */

/******************************************************************************
 * Function name:	UcbPacketTypeToBytes
* @brief Convert the packet type enum into bytes ['P'] ['K'] -> 0x504B
 * Trace:
 * [SDD_UCB_UNKNOWN_01 <-- SRC_UCB_PKT_STR]
 * [SDD_HANDLE_PKT  <-- SRC_UCB_PKT_STR]
 * @param [in] byte array, containing one byte
 * @Retval length
 ******************************************************************************/
void UcbPacketPacketTypeToBytes (UcbPacketType type,
                                 uint8_t       bytes [])
{
    ucb_packet_t *ptr = ucbPackets;
// carefull here, since for sake of speed it's indexing but not lookup through ucbPackets structure  

#ifndef USER_PACKETS_NOT_SUPPORTED
    if(type == UCB_USER_OUT){
        userPacketTypeToBytes(bytes);
        return;
    }
#endif
    
    while(ptr->packetType != UCB_PKT_NONE){
        if(ptr->packetType == type){
            bytes[0] = (uint8_t)((ptr->packetCode >> 8) & 0xff);
            bytes[1] = (uint8_t)(ptr->packetCode & 0xff);
            return;
        }
        ptr++;
    }
    
		bytes[0] = 0;
		bytes[1] = 0;
}
/* end UcbPacketPacketTypeToBytes */

/******************************************************************************
 * Function name:	UcbPacketTypeToCode
* @brief Convert the packet type enum into packet code 
         For example UCB_PING ->0x504B ("PK") 
 * Trace:
 * [SDD_UCB_UNKNOWN_01 <-- SRC_UCB_PKT_STR]
 * [SDD_HANDLE_PKT     <-- SRC_UCB_PKT_STR]
 * @param [in] byte array, containing one byte
 * @Retval length
 ******************************************************************************/
void UcbPacketPacketTypeToCode (UcbPacketType type, uint16_t *code)
{
// carefull here, since for sake of speed it's indexing but not lookup through ucbPackets structure  
    if (type < UCB_PKT_NONE) {
        *code = ucbPackets[type].packetCode;
	}else{
        *code = ucbPackets[UCB_PKT_NONE].packetCode;
    }
}
/* end UcbPacketPacketTypeToCode */



/** ****************************************************************************
 * Function name:	UcbPacketBytesToPayloadLength
 * @brief Convert the packet bytes into the packet payload length
 * Trace:
 * [SDD_UCB_STORE_DATA <-- SRC_UCB_PAYLEN]
 * [SDD_UCB_PKT_PAYLEN <-- SRC_UCB_PAYLEN]
 * @param [in] byte array, containing one byte
 * @Retval length
 ******************************************************************************/
uint8_t UcbPacketBytesToPayloadLength (const uint8_t bytes [])
{
	return (uint8_t)(bytes[0] & 0xff);
}
/* end UcbPacketBytesToPayloadLength */

/** ****************************************************************************
 * @name UcbPacketPayloadLengthToBytes
 * @brief	Convert the payload length into bytes
 * Trace:
 * [SDD_UCB_PROCESS_OUT <-- SRC_UCB_PKT_LENBYT]
 * [SDD_UCB_PKT_LENBYT <-- SRC_UCB_PKT_LENBYT]
 * @param [in] type - payload type
 * @param [out] byte array, containing one byte
 * @Retval none
 ******************************************************************************/
void	UcbPacketPayloadLengthToBytes (uint8_t length,
                                       uint8_t bytes [])
{
	bytes[0] = (uint8_t)(length & 0xff);
}
/* end UcbPacketBytesToPayloadLength */

/** ****************************************************************************
 * @name UcbPacketBytesToCrc
 * Description:
 * @param [in] byte array with byte[0] containing input value
 * @retval N/A
 ******************************************************************************/
UcbPacketCrcType UcbPacketBytesToCrc (const uint8_t bytes [])
{
	return ((UcbPacketCrcType)(((bytes[0] & 0xff) << 8) | (bytes[1] & 0xff)));
}
/* end UcbPacketBytesToCrc */

/** ****************************************************************************
 * @name UcbPacketCrcToBytes
 * @broef This function converts a value into a 2 byte string.
 * Trace: [SDD_UCB_PROCESS_OUT <-- SRC_UCB_PKT_CRCSTR]
 * @param [in] crc - 16-bit value (crc value)
 * @param [out] byte array with byte[0] containing the upper byte of input value
 * @retval N/A
 ******************************************************************************/
void UcbPacketCrcToBytes (const UcbPacketCrcType crc,
                          uint8_t                bytes [])
{
	bytes[0] = (uint8_t)((crc >> 8) & 0xff);
	bytes[1] = (uint8_t)(crc & 0xff);
}
/* end UcbPacketCrcToBytes */


/** ****************************************************************************
 * @name UcbPacketIsAnInputPacket
 * @brief Returns TRUE if given packet type is an input packet type
 * Trace:
 * [SDD_UCB_UNKNOWN_01 <-- SRC_UCB_PKT_INTYPE]
 * [SDD_UCB_UNKNOWN_02 <-- SRC_UCB_PKT_INTYPE]
 * [SDD_UCB_VALID_PACKET <-- SRC_UCB_PKT_INTYPE]
 * @parame [in]	UCB packet type enum
 * @retval TRUE if output packet type, FALSE otherwise
 ******************************************************************************/
BOOL UcbPacketIsAnInputPacket (UcbPacketType type)
{
	BOOL isAnInputPacket;

	switch (type) {
        case UCB_PING:
        case UCB_ECHO:
        case UCB_GET_PACKET:
        case UCB_SET_FIELDS:
        case UCB_GET_FIELDS:
        case UCB_READ_FIELDS:
        case UCB_WRITE_FIELDS:
        case UCB_UNLOCK_EEPROM:
        case UCB_READ_EEPROM:
        case UCB_WRITE_EEPROM:
        case UCB_PROGRAM_RESET:
        case UCB_SOFTWARE_RESET:
        case UCB_WRITE_CAL:
            isAnInputPacket = TRUE;
            break;
		default:
          isAnInputPacket = FALSE;
	}

    if((int)type == UCB_USER_IN){
          isAnInputPacket = TRUE;
    }

	return isAnInputPacket;
}
/* end UcbPacketIsAnInputPacket */

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
        case UCB_SCALED_0:
        case UCB_SCALED_1:
        case UCB_SCALED_M:
        case UCB_TEST_0:
        case UCB_FACTORY_1:
        case UCB_FACTORY_2:
        case UCB_FACTORY_M:
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

