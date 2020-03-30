/** ***************************************************************************
 * @file send_packet.c UCB callbacks for assembling then sending serial packets
 * 
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
/*******************************************************************************
 *  Copyright 2020 ACEINNA, INC
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
*******************************************************************************/

//****************************
#include "ucb_packet.h"
#include "platform_version.h"
#include "platformAPI.h"
#include "parameters.h"
#include "hwAPI.h"

#include "crc16.h"
#include "BitStatus.h"
#include "configurationAPI.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "rtcm.h"
#include "configureGPIO.h"
#include "bsp.h"
#include "uart.h"
#include "timer.h"
#include "serial_port.h"

#include "osapi.h"
#include "calibrationAPI.h"
#include "sensorsAPI.h"
#include "user_message.h"
#include "Indices.h"

void _UcbIdentification(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbVersionData(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbVersionAllData(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbScaled0(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbScaled1(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbTest0(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbFactory1(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbFactory2(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbFactory3(uint16_t port, UcbPacketStruct *ptrUcbPacket);

uint8_t divideCount = 200; /// continuous packet rate divider - set initial delay

static  UcbPacketStruct continuousUcbPacket; 

/** ****************************************************************************
 * @name _UcbIdentification send ID packet
 * @brief
 * Trace: [SDD_UCB_TX_ID <-- SRC_UCB_TX_ID]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbIdentification(uint16_t port,
                        UcbPacketStruct *ptrUcbPacket)

{
    uint8_t packetIndex = 0;
    uint8_t stringIndex = 0;
    static uint32_t sernum = 0;
    static uint8_t *versionString;

    sernum = GetUnitSerialNum();
#ifndef BOOT_MODE
    const uint8_t PART_NUMBER_STRING[] = SOFTWARE_PART;
    versionString = GetUnitVersion();
#else
    const uint8_t PART_NUMBER_STRING[] = BOOT_SOFTWARE_PART;
    versionString = (uint8_t *)VERSION_STRING;
#endif

    /// serial number
    packetIndex = uint32ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 sernum);
    /// model string
    while ((stringIndex < N_VERSION_STR) && (versionString[stringIndex] != 0))
    {
        ptrUcbPacket->payload[packetIndex++] = (uint8_t)versionString[stringIndex++];
    }

    /// space between
    ptrUcbPacket->payload[packetIndex++] = ' ';
    stringIndex = 0;

    /// software part number
    while (stringIndex < SOFTWARE_PART_LEN && (uint8_t)PART_NUMBER_STRING[stringIndex] != 0)
    {
        ptrUcbPacket->payload[packetIndex++] = (uint8_t)PART_NUMBER_STRING[stringIndex++];
    }

    ptrUcbPacket->payload[packetIndex++] = 0;  ///< zero delimiter
    ptrUcbPacket->payloadLength = packetIndex; ///< return packet length
    HandleUcbTx(port, ptrUcbPacket);           ///< send identification packet
}

/** ****************************************************************************
 * @name _UcbVersionData send VR packet
 * @brief
 * Trace: [SDD_UCB_TX_VR <-- SRC_UCB_TX_VR]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbVersionData(uint16_t port,
                     UcbPacketStruct *ptrUcbPacket)
{
    /// return packet length
    ptrUcbPacket->payloadLength = UCB_VERSION_DATA_LENGTH;

    /// 525 digital processor DUP version data - append
    ptrUcbPacket->payload[0] = (uint8_t)dupFMversion.major;
    ptrUcbPacket->payload[1] = (uint8_t)dupFMversion.minor;
    ptrUcbPacket->payload[2] = (uint8_t)dupFMversion.patch;
    ptrUcbPacket->payload[3] = (uint8_t)dupFMversion.stage;
    ptrUcbPacket->payload[4] = (uint8_t)dupFMversion.build;
    HandleUcbTx(port, ptrUcbPacket); /// send version data packet
}

/** ****************************************************************************
 * @name _UcbVersionAllData send VA (version all) packet
 * @brief
 * Trace: [SDD_UCB_TX_VA <-- SRC_UCB_TX_VA]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbVersionAllData(uint16_t port,
                        UcbPacketStruct *ptrUcbPacket)
{
    uint8_t packetIndex = 0;

    // DUP IOUP are here to allow compatibility with NavView
    /// 525 digital processor DUP version data
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.major;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.minor;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.patch;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.stage;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.build;

    /// 525 input output processor IOUP version data
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.major;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.minor;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.patch;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.stage;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.build;

    /// boot version data
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.major;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.minor;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.patch;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.stage;
    ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.build;

    ptrUcbPacket->payloadLength = packetIndex; ///< return packet length
    HandleUcbTx(port, ptrUcbPacket);           ///< send version all data packet
}

/** ****************************************************************************
 * @name _UcbScaled0 send S0 packet
 * @brief Scaled sensor 0 message load (SPI / UART) send (UART) scaled and
 *        filtered sensor data
 * Trace: [SDD_UCB_TX_S3 <-- SRC_UCB_TX_S3]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbScaled0(uint16_t port,
                 UcbPacketStruct *ptrUcbPacket)
{
	uint16_t packetIndex = 0;

	/// set packet length
	ptrUcbPacket->payloadLength = UCB_SCALED_0_LENGTH;
    /// X-accelerometer, Y, Z
	packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
	/// X-angular, Y, Z rate
	packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);
	/// X-magnetometer, Y, Z
	packetIndex = appendMagReadings(ptrUcbPacket->payload, packetIndex);
	/// rate and board temperature
	packetIndex = appendTemps(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, // itow???
                                  packetIndex,
                                  GetSensorsSamplingTstamp());

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  gBitStatus.BITStatus.all );

    if( platformGetUnitCommunicationType() == UART_COMM ) {
        HandleUcbTx(port, ptrUcbPacket); /// send Scaled 0 packet
    }
}

/** ****************************************************************************
 * @name _UcbScaled1 send S1 packet
 * @brief Sclaed sensor 1 load (SPI / UART) send (UART) filtered and scaled data
 * Trace: [SDD_UCB_TX_S1 <-- SRC_UCB_TX_S1]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbScaled1(uint16_t port,
                 UcbPacketStruct *ptrUcbPacket)
{
    uint16_t packetIndex = 0;

    ptrUcbPacket->payloadLength = UCB_SCALED_1_LENGTH;
    /// X-accelerometer, Y, Z
    packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
    /// X-angular rate, Y, Z
    packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);
#ifdef RUN_PROFILING

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 (uint16_t)SCALE_BY_2POW16_OVER_200(gEkfElapsedTime));
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 (uint16_t)SCALE_BY_2POW16_OVER_200(gEkfAvgTime));
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 (uint16_t)SCALE_BY_2POW16_OVER_200(gEkfMaxTime));

#else
    /// rate and board temperature
    packetIndex = appendTemps(ptrUcbPacket->payload, packetIndex);
#endif

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// packet counter
                                 packetIndex,
                                 GetSensorsSamplingTstamp());

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                 packetIndex,
                                 gBitStatus.BITStatus.all);

    if (platformGetUnitCommunicationType() == UART_COMM)
    {
        HandleUcbTx(port, ptrUcbPacket); /// send Scaled 1 packet
    }}

/** ****************************************************************************
 * @name _UcbScaledM send SM packet
 * @brief Sclaed sensor 1 load (SPI / UART) send (UART) filtered and scaled data
 * Trace: [SDD_UCB_TX_S1 <-- SRC_UCB_TX_S1]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbScaledM(uint16_t port,
                 UcbPacketStruct *ptrUcbPacket)
{
    static uint16_t sampleIdx = 0;
    static uint16_t sampleSubset = 0;
    uint16_t packetIndex = 0;

    ptrUcbPacket->payloadLength = UCB_SCALED_M_LENGTH;
    for (int i = 0; i < NUM_SENSOR_CHIPS; i++)
    {
        /// X-accelerometer, Y, Z
        packetIndex = appendChipAccels(ptrUcbPacket->payload, packetIndex, i);
        /// X-angular rate, Y, Z
        packetIndex = appendChipRates(ptrUcbPacket->payload, packetIndex, i);
        /// rate temperature
        packetIndex = appendChipTemps(ptrUcbPacket->payload, packetIndex, i);
    }

    packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
    /// X-angular rate, Y, Z
    packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);
    /// rate temperature
    packetIndex = appendTemp(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// sensors subset
                                 packetIndex,
                                 sampleSubset);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// sampleIdx
                                 packetIndex,
                                 sampleIdx++);

    if (platformGetUnitCommunicationType() == UART_COMM)
    {
        HandleUcbTx(port, ptrUcbPacket); /// send Scaled 1 packet
    }
}

/** ****************************************************************************
 * @name _UcbTest0 send T0 packet
 * @brief
 * Trace: [SDD_UCB_TX_T0 <-- SRC_UCB_TX_T0]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbTest0(uint16_t port,
               UcbPacketStruct *ptrUcbPacket)
{
    uint8_t packetIndex = 0;

    /// set packet length
    ptrUcbPacket->payloadLength = UCB_TEST_0_LENGTH;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT
                                 packetIndex,
                                 gBitStatus.BITStatus.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware BIT
                                 packetIndex,
                                 gBitStatus.hwBIT.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware power BIT
                                 packetIndex,
                                 0X0000); // Place holder for NavView
                                          /// hardware environmental BIT
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 gBitStatus.hwEnvBIT.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com BIT
                                 packetIndex,
                                 gBitStatus.comBIT.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com serial A BIT
                                 packetIndex,
                                 gBitStatus.comSABIT.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com serial b BIT
                                 packetIndex,
                                 gBitStatus.comSBBIT.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software BIT
                                 packetIndex,
                                 gBitStatus.swBIT.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software algorithm BIT
                                 packetIndex,
                                 gBitStatus.swAlgBIT.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software data BIT
                                 packetIndex,
                                 gBitStatus.swDataBIT.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware status
                                 packetIndex,
                                 gBitStatus.hwStatus.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com status
                                 packetIndex,
                                 gBitStatus.comStatus.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software status
                                 packetIndex,
                                 gBitStatus.swStatus.all);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// SENSOR status
                                 packetIndex,
                                 gBitStatus.sensorStatus.all);
    HandleUcbTx(port, ptrUcbPacket); /// send Test 0 packet
}

/** ****************************************************************************
 * @name _UcbFactory1 send F1 packet Factory (Raw) sensor data
 * @brief Raw data 1 load (SPI / UART) and send (UART) raw sensor counts
 * Trace: [SDD_UCB_TX_F1 <-- SRC_UCB_TX_F1]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbFactory1(uint16_t port,
                  UcbPacketStruct *ptrUcbPacket)
{
    uint16_t packetIndex = 0;

    ptrUcbPacket->payloadLength = UCB_FACTORY_1_LENGTH;
    packetIndex = appendInertialCounts(ptrUcbPacket->payload, packetIndex);
    packetIndex = appendAllTempCounts(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                 packetIndex,
                                 gBitStatus.BITStatus.all);

    if (platformGetUnitCommunicationType() == UART_COMM)
    {
        HandleUcbTx(port, ptrUcbPacket); /// send Factory 1 packet
    }
}

/** ****************************************************************************
 * @name _UcbFactory2 send F2 packet Factory (Raw) sensor data
 * @brief Raw data 2 load (SPI / UART) and send (UART) raw sensor counts
 * Trace: [SDD_UCB_TX_F2 <-- SRC_UCB_TX_F2]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbFactory2(uint16_t port,
                  UcbPacketStruct *ptrUcbPacket)
{
    uint16_t packetIndex = 0;

    ptrUcbPacket->payloadLength = UCB_FACTORY_2_LENGTH;
    packetIndex = appendInertialCounts(ptrUcbPacket->payload, packetIndex);
    packetIndex = appendMagnetometerCounts(ptrUcbPacket->payload, packetIndex);
    packetIndex = appendAllTempCounts(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                 packetIndex,
                                 gBitStatus.BITStatus.all);
    HandleUcbTx(port, ptrUcbPacket); /// send Factory 2 packet
}

/** ****************************************************************************
 * @name _UcbFactoryM send FM packet Factory (Raw) sensor data
 * @brief Raw data 2 load (SPI / UART) and send (UART) raw sensor counts
 * Trace: [SDD_UCB_TX_F2 <-- SRC_UCB_TX_F2]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbFactoryM(uint16_t port, UcbPacketStruct *ptrUcbPacket)
{
    ptrUcbPacket->packetType    = UCB_FACTORY_M;
	ptrUcbPacket->payloadLength = FillRawSensorsPayload(ptrUcbPacket->payload);
	HandleUcbTx(port, ptrUcbPacket); /// send Factory 2 packet
}

/** ****************************************************************************
 * @name SendUcbPacket API - taskUserCommunication.c
 * @brief top level send packet routine - calls other send routines based on
 *        packet type
 * Trace:
 *	[SDD_OUTPUT_PACKET <-- SRC_DATA_PACKET_TYPES]
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr -- filled in packet from the mapped physical port
 ******************************************************************************/
//extern sensorsSamplingData_t _SensorsData;
void SendUcbPacket(uint16_t port,
                   UcbPacketStruct *ptrUcbPacket)
{
    int result;

    if (ptrUcbPacket)
    {
        // FIXME could be a lookup table using a search
        switch (ptrUcbPacket->packetType)
        {
        case UCB_IDENTIFICATION: // ID 0x4944
            _UcbIdentification(port, ptrUcbPacket);
            break;
        case UCB_VERSION_DATA: // VR 0x5652
            _UcbVersionData(port, ptrUcbPacket);
            break;
        case UCB_VERSION_ALL_DATA: // VA 0x5641
            _UcbVersionAllData(port, ptrUcbPacket);
            break;
#ifndef BOOT_MODE
        case UCB_SCALED_0: // S0 0x5330
            _UcbScaled0(port, ptrUcbPacket);
            break;
        case UCB_SCALED_1: // S1 0x5331
            _UcbScaled1(port, ptrUcbPacket);
            break;
        case UCB_SCALED_M: // SM 0x534D
            _UcbScaledM(port, ptrUcbPacket);
            break;
        case UCB_TEST_0: // T0 0x5430
            _UcbTest0(port, ptrUcbPacket);
            break;
        case UCB_FACTORY_1: // F1 0x4631
            _UcbFactory1(port, ptrUcbPacket);
            break;
        case UCB_FACTORY_2: // F2 0x4632
            _UcbFactory2(port, ptrUcbPacket);
            break;
        case UCB_FACTORY_M: // F2 0x464D
            _UcbFactoryM(port, ptrUcbPacket);
            break;

        case UCB_USER_OUT:
            result = HandleUserOutputPacket(ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
            if (!result)
            {
                ptrUcbPacket->packetType = UCB_ERROR_INVALID_TYPE;
            }
            HandleUcbTx(port, ptrUcbPacket); /// send user packet
            break;
#endif // BOOT_MODE
        default:
            break; /// default handler?
        }
    }
}

void send_gnss_data(void)
{
    uint8_t type [UCB_PACKET_TYPE_LENGTH];
    obs_t* ptr_rover_obs = &g_ptr_gnss_data->rov;
    
    if (g_gnss_sol.gnss_update == 1){
        if (checkUserOutPacketType(gConfiguration.packetCode) == UCB_USER_OUT){
            // pS 0x7053
            type[0] = 0x70;
            type[1] = 0x53;
            continuousUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            SendUcbPacket(UART_USER, &continuousUcbPacket);

            //sK 0x734B
            type[0] = 0x73;
            type[1] = 0x4B;
            continuousUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            // skyview data may more than one packet
            uint8_t snum = ptr_rover_obs->n / 10;
            if (ptr_rover_obs->n % 10 != 0){
                snum++;
            }
            for (uint8_t i = 0; i < snum; i++){
                SendUcbPacket(UART_USER, &continuousUcbPacket);
            }
        }
        g_gnss_sol.gnss_update = 0;
    }
}

/** ****************************************************************************
 * @name SendContinuousPacket
 *
 * @brief This generates the automatic transmission of UCB packets. The specified
 * packet type will be sent at some multiple of the 10 mSec acquisition rate.
 * This allows large packets that require more time to be sent.
 *
 * Trace:
 * [SDD_PROCESS_PRIMARY_01 <-- SRC_PROCESS_PRIMARY]
 * [SDD_PROCESS_PRIMARY_02 <-- SRC_PROCESS_PRIMARY]
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void SendContinuousPacket(void)
{
    uint8_t type [UCB_PACKET_TYPE_LENGTH];
    uint16_t divider = 1;
    //uint16_t divider = configGetPacketRateDivider(gConfiguration.packetRateDivider); 

    if (divider != 0) { ///< check for quiet mode
        if (divideCount == 1) {
            gConfiguration.packetCode = 0x7331; //s1
            /// get enum for requested continuous packet type
            type[0] = (uint8_t)((gConfiguration.packetCode >> 8) & 0xff);
            type[1] = (uint8_t)(gConfiguration.packetCode & 0xff);

            /// set continuous output packet type based on configuration
            continuousUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            SendUcbPacket(UART_USER, &continuousUcbPacket);

            divideCount = divider;
        } else {
            --divideCount;
        }
    }

    // send 'pS' packet and 'sK' packet, will display in the web GUI through python driver
    // need to fill the data in 'Fill_posPacketPayload' and 'Fill_skyviewPacketPayload'
    send_gnss_data();
} 
