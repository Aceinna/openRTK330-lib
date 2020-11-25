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
#include <stdlib.h>

#include "ucb_packet.h"
#include "platform_version.h"
#include "platformAPI.h"
#include "hwAPI.h"

#include "crc16.h"
#include "configuration.h"
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

#include "tcp_driver.h"
#include "ins_interface_API.h"

void _UcbIdentification(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbVersionData(uint16_t port, UcbPacketStruct *ptrUcbPacket);
void _UcbVersionAllData(uint16_t port, UcbPacketStruct *ptrUcbPacket);


extern void send_pone_packet(void);


/** ****************************************************************************
 * @name uint32ToBuffer
 * @brief formats the input word for output to a byte buffer.
 * @author Douglas Hiranaka, Jul. 2014
 * @param [in] buffer - points to output buffer being loaded.
 * @param [in] index - response[index] is where data is added.
 * @param [in] inWord - data being added.
 * @retval return the incremented index
 ******************************************************************************/
uint32_t
uint32ToBuffer(uint8_t  *buffer,
               uint16_t index,
               uint32_t inWord)
{
    buffer[index++] = (uint8_t)((inWord >> 24) & 0xff);
    buffer[index++] = (uint8_t)((inWord >> 16) & 0xff);
    buffer[index++] = (uint8_t)((inWord >>  8) & 0xff);
    buffer[index++] = (uint8_t)(inWord & 0xff);

    return index;
}

/** ****************************************************************************
 * @name _UcbIdentification send ID packet
 * @brief
 * Trace: [SDD_UCB_TX_ID <-- SRC_UCB_TX_ID]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbIdentification(uint16_t port, UcbPacketStruct *ptrUcbPacket)

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

        default:
            break; /// default handler?
        }
    }
}

uint8_t debug_com_log_on = 1;
uint32_t debug_p1_log_delay = 0;

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

            float *ins_para = get_user_ins_para();
            cJSON_AddItemToObject(fmt, "pri_lever_arm_bx", cJSON_CreateNumber(*ins_para));
            cJSON_AddItemToObject(fmt, "pri_lever_arm_by", cJSON_CreateNumber(*(ins_para + 1)));
            cJSON_AddItemToObject(fmt, "pri_lever_arm_bz", cJSON_CreateNumber(*(ins_para + 2)));
            cJSON_AddItemToObject(fmt, "vrp_lever_arm_bx", cJSON_CreateNumber(*(ins_para + 3)));
            cJSON_AddItemToObject(fmt, "vrp_lever_arm_by", cJSON_CreateNumber(*(ins_para + 4)));
            cJSON_AddItemToObject(fmt, "vrp_lever_arm_bz", cJSON_CreateNumber(*(ins_para + 5)));
            cJSON_AddItemToObject(fmt, "user_lever_arm_bx", cJSON_CreateNumber(*(ins_para + 6)));
            cJSON_AddItemToObject(fmt, "user_lever_arm_by", cJSON_CreateNumber(*(ins_para + 7)));
            cJSON_AddItemToObject(fmt, "user_lever_arm_bz", cJSON_CreateNumber(*(ins_para + 8)));
            cJSON_AddItemToObject(fmt, "rotation_rbvx", cJSON_CreateNumber(*(ins_para + 9)));
            cJSON_AddItemToObject(fmt, "rotation_rbvy", cJSON_CreateNumber(*(ins_para + 10)));
            cJSON_AddItemToObject(fmt, "rotation_rbvz", cJSON_CreateNumber(*(ins_para + 11)));

            out = cJSON_Print(root);
            cJSON_Delete(root);

            uart_write_bytes(UART_DEBUG, out, strlen(out), 1);
            free(out);
            // debug_com_log_on = 0;
            debug_p1_log_delay = 100;
        }
        if (strstr((const char*)dataBuffer, "log debug on\r\n") != NULL)
        {
            debug_com_log_on = 1;
            debug_p1_log_delay = 100;
        }
    }
}



/** ****************************************************************************
 * @name send_continuous_packet
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
    double accel_g[3];
    float rate_dps[3];
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
        client_write_data(&driver_data_client, (const uint8_t*)imu_data_buf, data_len + end_len, 0x01);
    }
}

void debug_com_process(void)
{
    if (uart_sem_wait(UART_DEBUG, 0) == RTK_SEM_OK){
        debug_com_rx_data_handle();
    }

#ifdef DEBUG_ALL
    fill_imu_data();
#endif

#ifdef INS_APP
    if (debug_com_log_on) {
        if (!debug_p1_log_delay){
#ifndef DEBUG_ALL
            send_pone_packet();
#endif
        } else {
            debug_p1_log_delay--;
        }
    }
#endif
    set_wheel_tick_update_flag(0);
    set_gnss_obs_valid(0);
}


uint8_t spi_buff[SPI_BUF_SIZE];

static  UcbPacketStruct user_tx_packet;

uint8_t divideCount = 200; /// continuous packet rate divider for ucb factory

uint8_t rawimu_packet_divide    = 0;
uint8_t inspvax_packet_divide   = 0;
uint8_t odospeed_packet_divide  = 0;
uint8_t nmea_ins_divide   = 0;
uint8_t nmea_pashr_divide = 0;
uint8_t nmea_vtg_divide   = 0;

uint8_t rawimu_packet_rate     = 0;
uint8_t bestgnss_packet_rate   = 0;
uint8_t inspvax_packet_rate    = 0;
uint8_t odospeed_packet_rate   = 0;
uint8_t satellites_packet_rate = 0;
uint8_t nmea_ins_rate   = 0;
uint8_t nmea_gga_rate   = 0;
uint8_t nmea_rmc_rate   = 0;
uint8_t nmea_pashr_rate = 0;
uint8_t nmea_gsa_rate   = 0;
uint8_t nmea_zda_rate   = 0;
uint8_t nmea_vtg_rate   = 0;

void reset_user_packet_divide(void)
{
    rawimu_packet_divide    = rawimu_packet_rate;
    inspvax_packet_divide   = inspvax_packet_rate;
    odospeed_packet_divide  = odospeed_packet_rate;
    nmea_ins_divide         = nmea_ins_rate;
    nmea_pashr_divide       = nmea_pashr_rate;
    nmea_vtg_divide         = nmea_vtg_rate;
}

static void send_user_packets(void)
{
    uint16_t const port = UART_USER;

    if (rawimu_packet_rate != 0) {
        if (rawimu_packet_divide >= rawimu_packet_rate) {
            send_rawimu_packet(port, &user_tx_packet);
            rawimu_packet_divide = 1;
        } else {
            rawimu_packet_divide++;
        }
    }

    if (g_gnss_sol.gnss_update == 1) {
        if (bestgnss_packet_rate != 0) {
            send_bestgnss_packet(port, &user_tx_packet, g_ptr_gnss_sol);
        }
        if (satellites_packet_rate != 0) {
            send_satellites_packet(port, &user_tx_packet, g_ptr_gnss_sol);
        }
        g_gnss_sol.gnss_update = 0;
    }

    if (get_mGnssInsSystem_mlc_STATUS() == 4) {
        if (inspvax_packet_rate != 0) {
            if (inspvax_packet_divide >= inspvax_packet_rate) {
                send_inspvax_packet(port, &user_tx_packet, g_ptr_ins_sol);
                inspvax_packet_divide = 1;
            } else {
                inspvax_packet_divide++;
            }
        }
    }

    if (get_wheel_tick_update_flag() == 1) {
        send_odospeed_packet(port, &user_tx_packet);
    }
}


static char insBuff[200] = { 0 };
static char pashrBuff[120] = { 0 };
static char vtgBuff[120] = { 0 };

static void send_user_nmea(void) 
{
    uint16_t const port = UART_USER;

    if (get_mGnssInsSystem_mlc_STATUS() == 4) {
        if (nmea_ins_rate != 0) {
            if (nmea_ins_divide >= nmea_ins_rate) {
                double ep[6];
                gtime_t gpstime = gpst2time(get_imu_week(), get_imu_timestamp());
                gtime_t utctime = gpst2utc(gpstime);
                time2epoch(utctime, ep);
                print_nmea_ins(ep, insBuff);
                uart_write_bytes(port, (char *)insBuff, strlen(insBuff), 1);
                nmea_ins_divide = 1;
            } else {
                nmea_ins_divide++;
            }
        }

        if (nmea_pashr_rate != 0) {
            if (nmea_pashr_divide >= nmea_pashr_rate) {
                double ep[6];
                gtime_t gpstime = gpst2time(get_imu_week(), get_imu_timestamp());
                gtime_t utctime = gpst2utc(gpstime);
                time2epoch(utctime, ep);
                print_nmea_pashr(ep, pashrBuff);
                uart_write_bytes(port, (char *)pashrBuff, strlen(pashrBuff), 1);
                nmea_pashr_divide = 1;
            } else {
                nmea_pashr_divide++;
            }
        }

        if (nmea_vtg_rate != 0) {
            if (nmea_vtg_divide >= nmea_vtg_rate) {
                print_nmea_vtg(vtgBuff);
                uart_write_bytes(port, (char *)vtgBuff, strlen(vtgBuff), 1);
                nmea_vtg_divide = 1;
            } else {
                nmea_vtg_divide++;
            }
        }
    }

    if (nema_update_flag) {
        if (nmea_gga_rate != 0) {
            uart_write_bytes(port, (char *)gga_buff, strlen(gga_buff), 1);
        }
        if (nmea_rmc_rate != 0) {
            uart_write_bytes(port, (char *)rmc_buff, strlen(rmc_buff), 1);
        }
        if (nmea_gsa_rate != 0) {
            uart_write_bytes(port, (char *)gsa_buff, strlen(gsa_buff), 1);
        }
        if (nmea_zda_rate != 0) {
            uart_write_bytes(port, (char *)zda_buff, strlen(zda_buff), 1);
        }
        nema_update_flag = 0;
    }
}


void send_continuous_packet(void)
{
#ifndef RAW_APP
    send_user_packets();
    send_user_nmea();
#else
    if (nema_update_flag) {
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
}
