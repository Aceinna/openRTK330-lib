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
#include "parameters.h"
#include "eepromAPI.h"
#include "crc16.h"
#include "BITStatus.h"
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
 * @name _UcbEcho
 * @brief Reply to an ECHO command
 * Trace: [SDD_UCB_ECHO <-- SRC_UCB_ECHO]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbEcho (uint16_t port,
                      UcbPacketStruct    *ptrUcbPacket)
{
	HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbGetPacket
 * @brief Reply with the requested packet if it is an output packet type
 * Trace:
 *	[SDD_UCB_GETPACKET <-- SRC_UCB_GETPACKET]
 *	[SDD_RESP_ERROR <-- SRC_UCB_GETPACKET]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbGetPacket (uint16_t port,
                           UcbPacketStruct    *ptrUcbPacket)
{
    UcbPacketType requestedType;
    
	if (ptrUcbPacket->payloadLength == 2) {
		requestedType = UcbPacketBytesToPacketType(ptrUcbPacket->payload);

		if (UcbPacketIsAnOutputPacket(requestedType) == TRUE) {
			ptrUcbPacket->packetType = requestedType; ///< response packet type
		 	SendUcbPacket(port, ptrUcbPacket); ///< generic response packet handler
            return;
		} else {
            _SetNak(port, ptrUcbPacket);
		}
	} else {
        _SetNak(port, ptrUcbPacket);
	}
	HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbSetFields
 * @brief handles a UCB set fields command packet
 * Trace:
 * [SDD_UCB_SETFIELDS <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_ID <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_DATA <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_NAK1 <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_NAK2 <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_PAIR_VALID <-- SRC_UCB_SETFIELDS]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbSetFields (uint16_t port,
                           UcbPacketStruct    *ptrUcbPacket)

{
	uint8_t numFields = ptrUcbPacket->payload[0];
	uint8_t fieldCount;
	uint8_t validFieldCount;
    /** some fields need to be set together, so collect all field ID's and data
        in one set of arrays */
	uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
    /// array sizes are based on maximum number of fields to change
	uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];

	/// verify that the packet length matches packet specification
    if ((numFields > 0) &&
    	(ptrUcbPacket->payloadLength == (1 + numFields * 4)))
    {
	    /// loop through all fields and data specified in set fields request
	    for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
	    	/// read field ID and field data from packet into usable arrays
            fieldId[fieldCount]   = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 1] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 2]);
            fieldData[fieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 3] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 4]);
	    }

	    validFieldCount = CheckRamFieldData(numFields, fieldId, fieldData, fieldId);
		if (validFieldCount > 0) {	/// all or some requested field changes valid?
			/// build and send positive acknowledgement packet
			ptrUcbPacket->payloadLength = (uint8_t)(1 + (validFieldCount * 2));
	    	ptrUcbPacket->payload[0]    = validFieldCount; /// number of valid fields

			/// place valid field ID's in payload
			for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
				ptrUcbPacket->payload[(fieldCount * 2) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
				ptrUcbPacket->payload[(fieldCount * 2) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
			}
	        HandleUcbTx(port, ptrUcbPacket); ///< send acknowledgement
		}

		/// any invalid requested field changes?
		if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
     	    HandleUcbTx(port, ptrUcbPacket);
		}

		if (validFieldCount > 0) { /// apply any changes
			SetFieldData(); // xbowsp_fields.c
		}
	} else {
        _SetNak(port, ptrUcbPacket);
	    HandleUcbTx(port, ptrUcbPacket);
	}
}

/** ****************************************************************************
 * @name _UcbGetFields
 * @brief Handles UCB get fields command packet
 * Trace:
 * [SDD_UCB_GETFIELDS <-- SRC_UCB_GETFIELDS]
 * [SDD_UCB_GETFIELDS_ID <-- SRC_UCB_GETFIELDS]
 * [SDD_UCB_GETFIELDS_NAK1 <-- SRC_UCB_GETFIELDS]
 * [SDD_UCB_GETFIELDS_NAK2 <-- SRC_UCB_GETFIELDS]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbGetFields (uint16_t port,
                           UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t  numFields = ptrUcbPacket->payload[0];
	uint8_t  fieldCount;
	uint8_t  validFieldCount = 0;
	uint16_t fieldId [UCB_MAX_PAYLOAD_LENGTH / 4];

	/// verify that the packet length matches packet specification
    if ((numFields > 0) &&
    	(ptrUcbPacket->payloadLength == (1 + numFields * 2))) {

        /// read all fields specified in get fields request
        for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
            /// read field ID from packet into usable array
            fieldId[validFieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 2) + 1] << 8) |
                                                   ptrUcbPacket->payload[(fieldCount * 2) + 2]);

            /// check get field address bounds
            if (((fieldId[validFieldCount] >= LOWER_CONFIG_ADDR_BOUND) &&
                 (fieldId[validFieldCount] <= UPPER_CONFIG_ADDR_BOUND)) ||
                 (fieldId[validFieldCount] == PRODUCT_CONFIGURATION_FIELD_ID)) {
                ++validFieldCount;
            }
        }

        if (validFieldCount > 0) {	/// all or some requested get field addresses valid?
            /// build and return valid get fields with data packet
            ptrUcbPacket->payloadLength = (uint8_t)(1 + validFieldCount * 4);

            /// number of fields being returned
            ptrUcbPacket->payload[0] = validFieldCount;

            /// retrieve all fields specified in get fields request
            for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
                /** product configuration field is out of normal
                    configuration address range, needs to be fetched from
                    calibration structure */
                if (fieldId[fieldCount] == PRODUCT_CONFIGURATION_FIELD_ID) {
                    uint16_t cfg = GetProductConfiguration();

                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((cfg >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( cfg       & 0xff);
                }
                else {	/// normal field, exists in configuration structure
                    uint16_t param = configGetParam(fieldId[fieldCount]);

                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((param >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( param  & 0xff);
                }
            }
            HandleUcbTx(port, ptrUcbPacket);
        }

        /// any invalid get fields addresses?
        if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
}

/** ****************************************************************************
 * @name _UcbReadFields
 * @brief Handles UCB read fields command
 * Trace:
 * [SDD_UCB_READFIELDS <-- SRC_UCB_READFIELDS]
 * [SDD_UCB_READFIELDS_ID <-- SRC_UCB_READFIELDS]
 * [SDD_UCB_READFIELDS_NAK1 <-- SRC_UCB_READFIELDS]
 * [SDD_UCB_READFIELDS_NAK2 <-- SRC_UCB_READFIELDS]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbReadFields (uint16_t port,
                            UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t  numFields = ptrUcbPacket->payload[0];
	uint8_t  fieldCount;
	uint8_t  validFieldCount = 0;
	uint16_t fieldId [UCB_MAX_PAYLOAD_LENGTH / 4];
	uint16_t fieldData;

//    SetMaxDelay_Watchdog(); // Set the watchdog delay to its maximum value

    /// verify that the packet length matches packet specification
    if ((numFields > 0) &&
    (ptrUcbPacket->payloadLength == (1 + numFields * 2))) {
        /// read all fields specified in get fields request
        for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
            /// read field ID from packet into usable array
            fieldId[validFieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 2) + 1] << 8) |
                                                   ptrUcbPacket->payload[(fieldCount * 2) + 2]);

            /// check read field address bounds
            if (((fieldId[validFieldCount] >= LOWER_CONFIG_ADDR_BOUND) &&
                 (fieldId[validFieldCount] <= UPPER_CONFIG_ADDR_BOUND)) ||
                 (fieldId[validFieldCount] == PRODUCT_CONFIGURATION_FIELD_ID)) {
                ++validFieldCount;
            }
        }

        if (validFieldCount > 0) { /// all or some requested addresses valid?
            /// build and return valid get fields with data packet
            ptrUcbPacket->payloadLength = (uint8_t)(1 + validFieldCount * 4);

            ptrUcbPacket->payload[0] = validFieldCount; ///< # being returned

            /// retrieve all fields specified in get fields request
            for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
                /** product configuration field is out of normal configuration
                    address range, needs to be fetched from calibration
                    structure */
                if (fieldId[fieldCount] == PRODUCT_CONFIGURATION_FIELD_ID) {
                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);

                    EEPROM_ReadProdConfig( &fieldData);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((fieldData >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( fieldData       & 0xff);
                } else {	/// normal field, exists in configuration structure
                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                    /// read field from EEPROM
                    EEPROM_ReadByte(fieldId[fieldCount], sizeof(fieldData), &fieldData);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((fieldData >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( fieldData       & 0xff);
                }
            }
            HandleUcbTx(port, ptrUcbPacket);
        }

        /// invalid get fields addresses?
        if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
//    RestoreDelay_Watchdog();
}

/** ****************************************************************************
 * @name _UcbWriteFields
 * @briefHandle UCB write fields command packet
 * Trace:
 * [SDD_UCB_WRITEFIELDS <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_ID <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_NAK1 <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_NAK2 <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_DATA <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_PAIR_VALID <-- SRC_UCB_WRITEFIELDS]
 *
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbWriteFields (uint16_t port,
                             UcbPacketStruct    *ptrUcbPacket)
{
    uint8_t  numFields = ptrUcbPacket->payload[0];
    uint8_t  fieldCount;
    uint8_t  validFieldCount;
    /** some fields need to be set together, so collect all field ID's and data
       in one set of arrays */
    uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
    /// array sizes are based on maximum number of fields to change
    uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];

//    SetMaxDelay_Watchdog(); // Set the watchdog delay to its maximum value

    /// verify that the packet length matches packet specification
    if( ( numFields > 0 ) &&
        ( ptrUcbPacket->payloadLength == (1 + numFields * 4) ) )
    {
        /// loop through all fields and data specified in set fields request
        for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
            /// read field ID and field data from packet into usable arrays
            fieldId[fieldCount]   = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 1] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 2]);
            fieldData[fieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 3] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 4]);
        }

        /// check if data to set is valid xbowsp_fields.c
        validFieldCount = CheckEepromFieldData(numFields,
                                               fieldId,
                                               fieldData,
                                               fieldId);
// there is no check for corect number of changed fields only that something has changed
        if (validFieldCount > 0) { ///< all or some requested field changes valid?
            /// apply any changes
            if (WriteFieldData() == TRUE) { // xbowsp_fields.c
                /// build and send positive acknowledgement packet
                ptrUcbPacket->payloadLength = (uint8_t)(1 + (validFieldCount * 2));

                /// number of valid fields
                ptrUcbPacket->payload[0] = validFieldCount;

                /// place valid field ID's in payload
                for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
                    ptrUcbPacket->payload[(fieldCount * 2) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 2) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                }
                HandleUcbTx(port, ptrUcbPacket);
            } else {
                _SetNak(port, ptrUcbPacket);
                HandleUcbTx(port, ptrUcbPacket);
            }
        }

        /// any invalid requested field changes?
        if (validFieldCount < numFields) {
            // _SetNak(port, ptrUcbPacket);    
            // HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
//    RestoreDelay_Watchdog(); // Restore the watchdog delay to its original value
}


/** ****************************************************************************
 * @name _UcbUnlockEeprom
 * @brief unlock the EEPROM if the CRC of the unit serial number and payload is 0
 * Trace:
 *	[SDD_UCB_UNLOCK_EEPROM <-- SRC_UCB_UNLOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbUnlockEeprom (uint16_t port,
                              UcbPacketStruct    *ptrUcbPacket)
{
	//CrcCcittType crc = CRC_CCITT_INITIAL_SEED;
//    uint8_t      serialNumberBytes [6];

//    readEEPROMSerialNumber(&gCalibration[0].serialNumber);

	/// low 16-bits of SN first, little endian order
//	serialNumberBytes[0] = (uint8_t)(gCalibration[0].serialNumber & 0xff);
//	serialNumberBytes[1] = (uint8_t)((gCalibration[0].serialNumber >> 8) & 0xff);
	/// high 16-bits of SN next, little endian order
//	serialNumberBytes[2] = (uint8_t)((gCalibration[0].serialNumber >> 16) & 0xff);
//	serialNumberBytes[3] = (uint8_t)((gCalibration[0].serialNumber >> 24) & 0xff);
    // crc
//	serialNumberBytes[4] = ptrUcbPacket->payload[0];
//	serialNumberBytes[5] = ptrUcbPacket->payload[1];
    
    /// CRC serial number
//    crc = CalculateCRC(serialNumberBytes, 4);

// 	if (crc == 0) 
    { ///< correct unlock code?
        gBitStatus.hwStatus.bit.unlockedEEPROM = TRUE;
	    ptrUcbPacket->payloadLength = 0;
    }
    // else {
    //    _SetNak(port, ptrUcbPacket);
	//}
	HandleUcbTx(port, ptrUcbPacket);
} /* end HandleUcbUnlockEeprom() */

/** ****************************************************************************
 * @name _UcbReadEeprom
 * @brief Read 16 bit cells from EEPROM, passed in starting address and number
 * of cells in the packet payload
 *
 * Trace:
 *	[SDD_UCB_READ_EEPROM <-- SRC_UCB_READ_EEPROM]
 *   [SDD_UCB_READ_EEPROM_ERROR <-- SRC_UCB_READ_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbReadEeprom (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToRead;
    uint8_t  bytesToRead;

//    SetMaxDelay_Watchdog(); ///< Set the watchdog delay to its maximum value

    startAddress = (uint16_t)((ptrUcbPacket->payload[0] << 8) | ptrUcbPacket->payload[1]);
    wordsToRead  = ptrUcbPacket->payload[2];
    bytesToRead  = (uint8_t)(wordsToRead * 2);

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == 3) {
        ptrUcbPacket->payloadLength = (uint8_t)(ptrUcbPacket->payloadLength + bytesToRead);
        EEPROM_ReadByte(startAddress, bytesToRead, &(ptrUcbPacket->payload[3]));
	} else {
        _SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
//    RestoreDelay_Watchdog(); /// Restore the watchdog delay to its original value
}

/** ****************************************************************************
 * @name _UcbReadEeprom
 * @brief Read 16 bit cells from EEPROM, passed in starting address and number
 * of cells in the packet payload
 *
 * Trace:
 *	[SDD_UCB_READ_EEPROM <-- SRC_UCB_READ_EEPROM]
 *   [SDD_UCB_READ_EEPROM_ERROR <-- SRC_UCB_READ_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbReadCal (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToRead;
    uint8_t  bytesToRead;

//    SetMaxDelay_Watchdog(); ///< Set the watchdog delay to its maximum value

    startAddress  = (uint16_t)((ptrUcbPacket->payload[0] << 8) | ptrUcbPacket->payload[1]);
    startAddress *= SIZEOF_WORD;
    wordsToRead  = ptrUcbPacket->payload[2];
    bytesToRead  = (uint8_t)(wordsToRead * SIZEOF_WORD);

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == 3) {
        ptrUcbPacket->payloadLength = (uint8_t)(ptrUcbPacket->payloadLength + bytesToRead);
        EEPROM_ReadFromCalPartition(startAddress, bytesToRead, &(ptrUcbPacket->payload[3]));
	} else {
        _SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
//    RestoreDelay_Watchdog(); /// Restore the watchdog delay to its original value
}


/** ****************************************************************************
 * @name _UcbWriteEeprom
 * @brief Write data as 16 bit cells into an unlocked EEPROM.
 * Trace:
 *	[SDD_UCB_WRITE_EEPROM <-- SRC_UCB_WRITE_EEPROM]
 *	[SDD_UCB_WRITE_EEPROM_ERROR <-- SRC_UCB_WRITE_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbWriteEeprom (uint16_t port,
                             UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToWrite;
    uint16_t bytesToWrite;

    startAddress = (uint16_t)((ptrUcbPacket->payload[0] << 8) |
                               ptrUcbPacket->payload[1]);
    wordsToWrite = ptrUcbPacket->payload[2];
    bytesToWrite = (uint16_t)wordsToWrite * 2;

//    SetMaxDelay_Watchdog();

    /// verify that the packet length matches packet specification
    if ((ptrUcbPacket->payloadLength == (bytesToWrite + 3)) &&
        (gBitStatus.hwStatus.bit.unlockedEEPROM == TRUE) ) {
        /// flag current CRC as invalid
        gBitStatus.swDataBIT.bit.calibrationCRCError = TRUE;

        /// 0 means no errors
        if (EEPROM_WriteWords(startAddress,
                             wordsToWrite,
                             &(ptrUcbPacket->payload[3])) == 0) {
            ptrUcbPacket->payloadLength = 3;
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
 * @name _UcbWriteCal
 * @brief Write data as 16 bit cells into an unlocked EEPROM.
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbWriteCal (uint16_t port,
                          UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToWrite;
    uint16_t bytesToWrite;

    startAddress = (uint16_t)((ptrUcbPacket->payload[0] << 8) | ptrUcbPacket->payload[1]);
    startAddress *= SIZEOF_WORD;
    wordsToWrite  = ptrUcbPacket->payload[2];
    bytesToWrite  = (uint16_t)wordsToWrite * SIZEOF_WORD;

//    SetMaxDelay_Watchdog();

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == (bytesToWrite + 3)) {
        /// 0 means no errors
        if (EEPROM_WriteToCalPartition(startAddress, bytesToWrite,
                             &(ptrUcbPacket->payload[3])) != 0) {
            ptrUcbPacket->payloadLength = 3;
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
 * @name 
 * @brief
 * Trace:
 *	[SDD_UCB_UNLOCK_EEPROM <-- SRC_UCB_UNLOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _Ucb_HARDWARE_TEST (uint16_t port, UcbPacketStruct    *ptrUcbPacket)
{
	ResetForEnterBootMode();
    DelayMs(10);
    {
        HW_HDTestMode();
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
            case UCB_ECHO:
                _UcbEcho(port, ptrUcbPacket); break;
            case UCB_GET_PACKET:
                _UcbGetPacket(port, ptrUcbPacket); break;
            case UCB_GET_FIELDS:
                _UcbGetFields(port, ptrUcbPacket); break;
            case UCB_J2BOOT:
            case UCB_J2IAP:
                _UcbJump2BOOT (port, ptrUcbPacket); break;
            case UCB_J2APP:                                     //TODO:
                _UcbJump2APP (port, ptrUcbPacket); break; 
            case UCB_HARDWARE_TEST: 
                _Ucb_HARDWARE_TEST (port, ptrUcbPacket); break;             
            case UCB_READ_CAL:
                _UcbReadCal(port, ptrUcbPacket); break;
            case UCB_SOFTWARE_RESET:
                _UcbSoftwareReset(port, ptrUcbPacket); break;
            case UCB_WRITE_APP:
                _UcbWriteApp(port, ptrUcbPacket); break;
#ifndef BOOT_MODE
            case UCB_SET_FIELDS:
                _UcbSetFields(port, ptrUcbPacket); break;
            case UCB_READ_FIELDS:
                _UcbReadFields(port, ptrUcbPacket); break;
            case UCB_WRITE_FIELDS:
                _UcbWriteFields(port, ptrUcbPacket); break;
            case UCB_UNLOCK_EEPROM:
                _UcbUnlockEeprom(port, ptrUcbPacket); break;
            case UCB_READ_EEPROM:
                _UcbReadEeprom(port, ptrUcbPacket); break;
            case UCB_WRITE_EEPROM:
                _UcbWriteEeprom(port, ptrUcbPacket); break;
            case UCB_WRITE_CAL:
                _UcbWriteCal(port, ptrUcbPacket); break;
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
#endif // BOOT_MODE
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

