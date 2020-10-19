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
#include "app_version.h"
#include "cJSON.h"
#include "user_config.h"
#include "uart.h"
#include "spi.h"
#include <stdlib.h>
#include "tcp_driver.h"


#ifdef INS_APP
#include "ins_interface_API.h"
// #include "insoutmsg.h"
// extern GnssInsSystem mGnssInsSystem;
#endif

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

uint8_t debug_com_log_on = 0;
uint32_t debug_p1_log_delay = 0;

extern void sendP1Packet(uint8_t gps_update);


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
    
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  gBitStatus.BITStatus.all );

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


void debug_com_rx_data_handle(void)
{
    uint8_t dataBuffer[512];
    int bytes_in_buffer = 0;
    cJSON *root, *fmt;
    char *out;

    bytes_in_buffer = uart_read_bytes(UART_DEBUG, dataBuffer, sizeof(dataBuffer), 0);
    if (bytes_in_buffer > 0){
        if (strstr((const char*)dataBuffer, "get configuration\r\n") != NULL)
        {
            root = cJSON_CreateObject();
            cJSON_AddItemToObject(root, "openrtk configuration", fmt = cJSON_CreateObject());
            cJSON_AddItemToObject(fmt, "Product Name", cJSON_CreateString(PRODUCT_NAME_STRING));
            cJSON_AddItemToObject(fmt, "Product PN", cJSON_CreateString((const char *)platformBuildInfo()));
            cJSON_AddItemToObject(fmt, "Product SN", cJSON_CreateNumber(GetUnitSerialNum()));
            cJSON_AddItemToObject(fmt, "Version", cJSON_CreateString(APP_VERSION_STRING));

            uint8_t *user_packet_type = get_user_packet_type();
            char packet_type_str[5] = {0};
            packet_type_str[0] = user_packet_type[0];
            packet_type_str[1] = user_packet_type[1];

            uint16_t user_packet_rate = get_user_packet_rate();

            cJSON_AddItemToObject(fmt, "userPacketType", cJSON_CreateString(packet_type_str));
            cJSON_AddItemToObject(fmt, "userPacketRate", cJSON_CreateNumber(user_packet_rate));

            float *ins_para = get_user_ins_para();
            cJSON_AddItemToObject(fmt, "leverArmBx", cJSON_CreateNumber(*ins_para));
            cJSON_AddItemToObject(fmt, "leverArmBy", cJSON_CreateNumber(*(ins_para + 1)));
            cJSON_AddItemToObject(fmt, "leverArmBz", cJSON_CreateNumber(*(ins_para + 2)));
            cJSON_AddItemToObject(fmt, "pointOfInterestBx", cJSON_CreateNumber(*(ins_para + 3)));
            cJSON_AddItemToObject(fmt, "pointOfInterestBy", cJSON_CreateNumber(*(ins_para + 4)));
            cJSON_AddItemToObject(fmt, "pointOfInterestBz", cJSON_CreateNumber(*(ins_para + 5)));
            cJSON_AddItemToObject(fmt, "rotationRbvx", cJSON_CreateNumber(*(ins_para + 6)));
            cJSON_AddItemToObject(fmt, "rotationRbvy", cJSON_CreateNumber(*(ins_para + 7)));
            cJSON_AddItemToObject(fmt, "rotationRbvz", cJSON_CreateNumber(*(ins_para + 8)));

            out = cJSON_Print(root);
            cJSON_Delete(root);

            uart_write_bytes(UART_DEBUG, out, strlen(out), 1);
            free(out);
            debug_com_log_on = 0;
        }
        if (strstr((const char*)dataBuffer, "log debug on\r\n") != NULL)
        {
            debug_com_log_on = 1;
            debug_p1_log_delay = 100;
        }
    }
}

void send_gnss_data(void)
{
    uint8_t type [UCB_PACKET_TYPE_LENGTH];
    
#ifdef INS_APP
    if (checkUserOutPacketType(gConfiguration.packetCode) == UCB_USER_OUT){
        if (get_mGnssInsSystem_mlc_STATUS() == 4 || g_gnss_sol.gnss_update == 1){
            // pS 0x7053
            type[0] = 0x70;
            type[1] = 0x53;
            continuousUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            SendUcbPacket(UART_USER, &continuousUcbPacket);
        }
        
        if (g_gnss_sol.gnss_update == 1){
            //sK 0x734B
            type[0] = 0x73;
            type[1] = 0x4B;
            continuousUcbPacket.packetType = UcbPacketBytesToPacketType(type);

            uint8_t snum;
            // skyview data may more than one packet
            snum = g_ptr_gnss_sol->rov_n / 10;
            if (g_ptr_gnss_sol->rov_n % 10 != 0){
                snum++;
            }

            for (uint8_t i = 0; i < snum; i++){
                SendUcbPacket(UART_USER, &continuousUcbPacket);
            }
        }
    }
    g_gnss_sol.gnss_update = 0;
#else
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

            uint8_t snum;
            // skyview data may more than one packet
            if (strstr(APP_VERSION_STRING, "RAWDATA")){
                snum = g_ptr_gnss_data->rov.n / 10;
                if (g_ptr_gnss_data->rov.n % 10 != 0){
                    snum++;
                }
            } else{
                snum = g_ptr_gnss_sol->rov_n / 10;
                if (g_ptr_gnss_sol->rov_n % 10 != 0){
                    snum++;
                }
            }

            for (uint8_t i = 0; i < snum; i++){
                SendUcbPacket(UART_USER, &continuousUcbPacket);
            }
        }
        g_gnss_sol.gnss_update = 0;
    }
#endif
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
extern client_s driver_data_client;
static void fill_imu_data()
{
    double   accel_g[3];
    float   rate_dps[3];
    char sum = 0;
    uint8_t imu_data_buf[500] = {0};
    GetAccelData_mPerSecSq(accel_g);
    GetRateData_degPerSec(rate_dps);
    double gga_time = get_gnss_time();
	int data_len = sprintf((char*)imu_data_buf,"$GPIMU,%6.2f,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,",    \
		gga_time,accel_g[0], accel_g[1],accel_g[2], \
		rate_dps[0],rate_dps[1],rate_dps[2]);
    for(int i = 0;i < data_len;i++)
    {
        sum ^= imu_data_buf[i];
    }
    int end_len = sprintf((char*)imu_data_buf + strlen((char*)imu_data_buf),"%02x\r\n",sum);
    if (debug_com_log_on) {
        uart_write_bytes(UART_DEBUG,(const char*)imu_data_buf,data_len + end_len,1);
    }
    if(get_tcp_data_driver_state() == CLIENT_STATE_INTERACTIVE)
    {
        //driver_data_push((const char*)imu_data_buf,data_len + end_len);
        client_write_data(&driver_data_client,(const char*)imu_data_buf,data_len + end_len,0x01);
    }
}


#ifdef INCEPTIO
static  UcbPacketStruct inceptioUcbPacket;
uint8_t rawimuPacketDivide = 0;
uint8_t inspvaPacketDivide = 0;
uint8_t insstdPacketDivide = 0;
uint8_t bestgnssPacketDivide = 0;
uint8_t rawimuPacketRate = 0;
uint8_t inspvaPacketRate = 0;
uint8_t insstdPacketRate = 0;
uint8_t bestgnssPacketRate = 0;

#endif
uint8_t spi_buff[SPI_BUF_SIZE];

void SendContinuousPacket(void)
{
    uint8_t type[UCB_PACKET_TYPE_LENGTH];

#ifdef INCEPTIO
    // come here 100Hz
    static char s1_buff[50] = {0};
    static char iN_buff[50] = {0};
    static char d1_buff[50] = {0};
    static char gN_buff[50] = {0};
    static char sT_buff[20] = {0};
    static char s1_len = 43,iN_len = 45,d1_len = 37,gN_len = 45,sT_len = 19;

    // RAWIMU s1
    if (rawimuPacketRate != PACKET_RATE_QUIET && mGnssInsSystem.mlc_STATUS == INS_FUSING) {
        if (rawimuPacketDivide >= rawimuPacketRate) {
            gConfiguration.packetCode = 0x7331;
            type[0] = 0x73;
            type[1] = 0x31;
            inceptioUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            SendUcbPacket(UART_USER, &inceptioUcbPacket);
            rawimuPacketDivide = 1;

            memset(s1_buff,0,50);
            memcpy((char *)s1_buff,(const char *)&inceptioUcbPacket.sync_MSB, inceptioUcbPacket.payloadLength + 7);
            s1_len = inceptioUcbPacket.payloadLength + 7;
        } else {
            rawimuPacketDivide++;
        }
    }
    
    // INSPVA iN
    if (inspvaPacketRate != PACKET_RATE_QUIET) {
        if (inspvaPacketDivide >= inspvaPacketRate) {
            gConfiguration.packetCode = 0x694E;
            type[0] = 0x69;
            type[1] = 0x4E;
            inceptioUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            SendUcbPacket(UART_USER, &inceptioUcbPacket);
            inspvaPacketDivide = 1;

            memset(iN_buff,0,50);
            memcpy((char *)iN_buff,(const char *)&inceptioUcbPacket.sync_MSB, inceptioUcbPacket.payloadLength + 7);
            iN_len = inceptioUcbPacket.payloadLength + 7;
        } else {
            inspvaPacketDivide++;
        }
    }

    // INSSTD d1
    if (insstdPacketRate != PACKET_RATE_QUIET) {
        if (insstdPacketDivide >= insstdPacketRate) {
            gConfiguration.packetCode = 0x6431;
            type[0] = 0x64;
            type[1] = 0x31;
            inceptioUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            SendUcbPacket(UART_USER, &inceptioUcbPacket);
            insstdPacketDivide = 1;

            memset(d1_buff,0,50);  
            memcpy((char *)d1_buff,(const char *)&inceptioUcbPacket.sync_MSB, inceptioUcbPacket.payloadLength + 7);
            d1_len = inceptioUcbPacket.payloadLength + 7;   
        } else {
            insstdPacketDivide++;
        }
    }

    // BESTGNSS gN
    // if (bestgnssPacketRate != PACKET_RATE_QUIET) {
    // if (bestgnssPacketDivide >= bestgnssPacketRate) {
        // 1Hz 
        if (g_gnss_sol.gnss_update == 1) {
            gConfiguration.packetCode = 0x674E;
            type[0] = 0x67;
            type[1] = 0x4E;
            inceptioUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            SendUcbPacket(UART_USER, &inceptioUcbPacket);
        
            g_gnss_sol.gnss_update = 0;
            memset(gN_buff,0,50);
            memcpy((char *)gN_buff,(const char *)&inceptioUcbPacket.sync_MSB, inceptioUcbPacket.payloadLength + 7);
            gN_len = inceptioUcbPacket.payloadLength + 7;
        }
    //     bestgnssPacketDivide = 1;
    // } else {
    //     bestgnssPacketDivide++;
    // }
    // }
//STATUS sT
    gConfiguration.packetCode = 0x7354;
    type[0] = 0x73;
    type[1] = 0x54;
    inceptioUcbPacket.packetType = UcbPacketBytesToPacketType(type);
    SendUcbPacket(UART_USER, &inceptioUcbPacket);

    memset(sT_buff,0,20);
    memcpy((char *)sT_buff,(const char *)&inceptioUcbPacket.sync_MSB, inceptioUcbPacket.payloadLength + 7);
    sT_len = inceptioUcbPacket.payloadLength + 7;

    memset(spi_buff,0,SPI_BUF_SIZE);
    memcpy((char *)spi_buff,s1_buff,s1_len);
    memcpy((char *)spi_buff+s1_len,iN_buff,iN_len);
    memcpy((char *)spi_buff+s1_len+iN_len,d1_buff,d1_len);
    memcpy((char *)spi_buff+s1_len+iN_len+d1_len,gN_buff,gN_len);
    memcpy((char *)spi_buff+s1_len+iN_len+d1_len+gN_len,sT_buff,sT_len);    
    // uart_write_bytes(UART_DEBUG,(char *)spi_buff,170+19,1);
    spi_ready_flag ++;
    if (spi_ready_flag >= 2)
    {
        spi_ready_flag = 1;
        MX_SPI5_Init();
    }
    
    DRDY_ON();

#else

#ifdef INS_APP
    // 100Hz
    uint16_t divider = configGetPacketRateDivider(gConfiguration.packetRateDivider);
    if (divider != 0 && divider < 2) {
        divider = 2;
    }
    divider = divider / 2;
#else
    uint16_t divider = 1;
#endif

    if (divider != 0) { ///< check for quiet mode
        if (divideCount == 1) {
            // gConfiguration.packetCode = 0x5331; //s1 7331
            /// get enum for requested continuous packet type
            type[0] = (uint8_t)((gConfiguration.packetCode >> 8) & 0xff);
            type[1] = (uint8_t)(gConfiguration.packetCode & 0xff);

            /// set continuous output packet type based on configuration
            continuousUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            SendUcbPacket(UART_USER, &continuousUcbPacket);
#ifdef DEBUG_ALL
            fill_imu_data();
#endif
            divideCount = divider;
        } else {
            --divideCount;
        }
    }
    // send 'pS' packet and 'sK' packet, will display in the web GUI through python driver
    // need to fill the data in 'Fill_posPacketPayload' and 'Fill_skyviewPacketPayload'
    send_gnss_data();

#ifndef RAW_APP
    if(nema_update_flag)
    {
#ifdef INS_APP

#ifdef USER_INS_NMEA
        if (strlen(ggaBuff) == 0) {
            uart_write_bytes(UART_USER, (char *)gga_buff, strlen(gga_buff), 1);
            uart_write_bytes(UART_USER, (char *)rmc_buff, strlen(rmc_buff), 1);
        }
#else
        if (strlen(ggaBuff) == 0) {
            uart_write_bytes(UART_USER, (char *)gga_buff, strlen(gga_buff), 1);
        } else {
            uart_write_bytes(UART_USER, (char *)ggaBuff, strlen(ggaBuff), 1);
        }
        uart_write_bytes(UART_USER, (char *)rmc_buff, strlen(rmc_buff), 1);
#endif

#else
        uart_write_bytes(UART_USER, (char *)gga_buff, strlen(gga_buff), 1);
        uart_write_bytes(UART_USER, (char *)rmc_buff, strlen(rmc_buff), 1);
#endif

        uart_write_bytes(UART_USER, (char *)gsa_buff, strlen(gsa_buff), 1);    
        uart_write_bytes(UART_USER, (char *)zda_buff, strlen(zda_buff), 1); 
        nema_update_flag = 0;
    }
#else
   if(nema_update_flag)
    {
        double ecef[3];
        ecef[0] = g_gnss_sol.pos_ecef[0];
        ecef[1] = g_gnss_sol.pos_ecef[1];
        ecef[2] = g_gnss_sol.pos_ecef[2];
        
        gtime_t time = gpst2time(g_gnss_sol.gps_week, g_gnss_sol.gps_tow*0.001);
        print_rmc(time, ecef,1, rmc_buff);   
        print_gsv((unsigned char *)gsv_buff,1,sky_view_ptr);

        uart_write_bytes(UART_USER, (char *)gga_buff, strlen(gga_buff), 1);
        uart_write_bytes(UART_USER, (char *)rmc_buff, strlen(rmc_buff), 1);
        uart_write_bytes(UART_USER, (char *)gsv_buff, strlen(gsv_buff), 1);    
        nema_update_flag = 0;
    }
#endif

#endif
}

void debug_com_process(void)
{
    if (uart_sem_wait(UART_DEBUG, 0) == RTK_SEM_OK){
        debug_com_rx_data_handle();
    }
#ifdef INS_APP
    if (debug_com_log_on)
    {
        if (!debug_p1_log_delay){
#ifndef DEBUG_ALL
            sendP1Packet(g_ptr_gnss_sol->gnss_update);
#endif
        } else{
            debug_p1_log_delay--;
        }
    }
#endif
}