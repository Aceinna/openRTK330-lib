/** ***************************************************************************
 * @file xbowsp_init.c Initialization for UCB's Comm. and Cal.
 * @Author dan
 * @date   2011-02-09 22:02:39 -0800 (Wed, 09 Feb 2011)
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 * @rev 17479
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @version
 * DKH 10.02.14 set sensor range based on EEPROM config
* 06/01/2020  |                                             | Daich
* Description: add configGetPacketCode function to get user packet type
*******************************************************************************/
#include <string.h>
#include <stdint.h>

//*****************************
// #include "eepromAPI.h"
#include "configuration.h"
#include "parameters.h"
#include "filter.h"
#include "crc16.h"
#include "BitStatus.h"
#include "platform_version.h"
#include "crc.h"
#include "stm32f4xx_hal.h"
#include "eepromAPI.h"
// #include "hwAPI.h"

ConfigurationStruct gConfiguration;
extern BITStatusStruct     gBitStatus;
uint16_t            spiFilterType = 0;  // unfiltered
uint16_t            spiPacketRateDividor = 0;  // quiet

BOOL     ValidPortConfiguration(ConfigurationStruct *proposedConfiguration);


uint16_t configGetUsedChips(void)                     { return gConfiguration.usedChips;}
uint16_t configGetActiveChips(void)                   { return gConfiguration.activeChips;}
uint16_t configGetUsedSensors(int chipIdx)            { return gConfiguration.usedSensors[chipIdx];}
void     configSetUsedChips(uint8_t mask)             { gConfiguration.usedChips = mask;}

// placholders for Nav_view compatibility
softwareVersionStruct dupFMversion;  /// 525 digital processor DUP code base
softwareVersionStruct ioupFMversion; /// 525 input output processor IOUP code base
softwareVersionStruct bootFMversion; /// bootloader code base


/** ****************************************************************************
 * @name baudEnumToBaudRate LOCAL convert enumeration to value
 * @param [in] baudRate - enumeration to translate extern_port_config.h
 * @retval actual baud rate value
 ******************************************************************************/
int32_t baudEnumToBaudRate(int baudEnum)
{   // -1 (invalid) can be sent in. Passed through to gGpsDataPtr->GPSConfigureOK
    uint32_t baudRate = baudEnum;

    switch (baudEnum) {
        case BAUD_9600: // 0
            baudRate =    9600;
            break;
        case BAUD_19200: // 1
            baudRate =   19200;
            break;
        case BAUD_38400: // 2
            baudRate =	 38400;
            break;
        case BAUD_4800:  // 4
            baudRate =	  4800;
            break;
        case BAUD_115200: // 5
            baudRate =	115200;
            break;
        case BAUD_230400: // 6
            baudRate =  230400;
            break;
        case BAUD_460800: // 7
            baudRate =	460800;
            break;
        case BAUD_57600:  // 3
        default:
            baudRate =	 57600;
            break;
    }

    return baudRate;
}

/** ****************************************************************************
 * @name: _readConfigIntoMem LOCAL Read configuration from EEPROM into RAM
 * TRACE:
 * [SDD_INIT_CONFIGURATION_ADAHRS <-- SRC_READ_CONFIGURATION_AND_CALIBRATION_INTO_MEMORY]
 * [SDD_EEPROM_INIT_READ <-- SRC_READ_CONFIGURATION_AND_CALIBRATION_INTO_MEMORY]
 * [SDD_INIT_MISALIGN_ADAHRS <-- SRC_READ_CONFIGURATION_AND_CALIBRATION_INTO_MEMORY]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _readConfigIntoMem ()
{
    EEPROM_ReadFactoryConfiguration(&gConfiguration); // s_eeprom.c
}

/** ****************************************************************************
 * @name initConfigureUnit initializes the data structures and configurations
 *     that are read from EEPROM for DUP software to run on ADAHRS config
 * TRACE: * [SDD_INIT_CONFIGURATION_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_EEPROM_INIT_READ <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_MISALIGN_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_EEPROM_CRC_DATA <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_EEPROM_CRC_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_BIT_LIMITS_EEPROM <-- SRC_INIT_CONFIGURE_UNIT]
 *
 * [SDD_CAL_G_DATA_CHECK <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_CONFIGURATION_ORIENT_VALID <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_CFG_PORT_DEF_01 <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_CFG_PORT_DEF_02 <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_CONFIGURATION_DEFAULT_BARO <-- SRC_INIT_CONFIGURE_UNIT]
 *
 * [SDD_INIT_RPY_OFFSETS_EXTEND <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_RPY_OFFSETS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_EXT_MAG_CONFIG <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_DUP_SW_VERSION_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_BOOTLOADER_SW_VERSION <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_WATCHDOG <-- SRC_INIT_CONFIGURE_UNIT]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void ApplyFactoryConfiguration(void)
{

    memset(&gBitStatus, 0, sizeof(gBitStatus));

    _readConfigIntoMem();

    /// check user orientation field for validity and set defaults based on com
    //  type if not valid xbow_fields.c
    // if (CheckOrientation(gConfiguration.orientation.all) == FALSE) {
    //     if( !fSPI ) {
    //         gConfiguration.orientation.all = 0;
    //     } else { // SPI
    //         gConfiguration.orientation.all = 0x6b;
    //     }
    // }

    /// check port configuration fields against rules
	// xbow_fields.c
    if (ValidPortConfiguration(&gConfiguration) == FALSE) {
        DefaultPortConfiguration();
    }

    dupFMversion.major = VERSION_MAJOR;
    dupFMversion.minor = VERSION_MINOR;
    dupFMversion.patch = VERSION_PATCH;
    dupFMversion.stage = VERSION_STAGE;
    dupFMversion.build = VERSION_BUILD;

    ioupFMversion.major = VERSION_MAJOR;
    ioupFMversion.minor = VERSION_MINOR;
    ioupFMversion.patch = VERSION_PATCH;
    ioupFMversion.stage = VERSION_STAGE;
    ioupFMversion.build = VERSION_BUILD;

    if(gConfiguration.usedChips > 7 || gConfiguration.usedChips == 0 ){
        gConfiguration.usedChips = 7;
    }

    if(gConfiguration.activeChips > 7 || gConfiguration.activeChips == 0){
        gConfiguration.activeChips = 7;
    }

    for (uint16_t i = 0; i < 3; i++) {
        if (gConfiguration.usedSensors[i] == 0) {
            gConfiguration.usedSensors[i] = 0xffff;
        }
    }
}


void configSetUsedSensors(int idx, uint8_t mask)
{
    gConfiguration.usedSensors[idx]  = 0xffC0;
    gConfiguration.usedSensors[idx] |= mask;
}

void SetDrdyRate(uint8_t rate)
{
    gConfiguration.packetRateDivider = rate;
}

int configApplyOrientation(uint16_t orientation)
{
    if(CheckOrientation(orientation)){
        gConfiguration.orientation.all = orientation;
        return TRUE;
    }
    return FALSE;
} 

int configGetSysRange()
{
    return _400_DPS_RANGE;
}


BOOL configSetBaudRate(int baudRate, BOOL fApply)
{
    BOOL res;

    switch (baudRate){
        case 9600:	   baudRate = BAUD_9600;  break;
        case 19200:	   baudRate = BAUD_19200;  break;
        case 38400:	   baudRate = BAUD_38400;  break;
        case 57600:	   baudRate = BAUD_57600;  break;
        case 4800:	   baudRate = BAUD_4800;   break;
        case 115200:   baudRate = BAUD_115200; break;
        case 230400:   baudRate = BAUD_230400; break;
        case 460800:   baudRate = BAUD_460800; break;
        default:
            return FALSE;
    }
    uint16_t tmp = gConfiguration.baudRateUser;
    gConfiguration.baudRateUser =  baudRate;
    res = ValidPortConfiguration(&gConfiguration);

    if (res == FALSE || !fApply) {
        gConfiguration.baudRateUser = tmp;
    }

    return res;

}


int configGetPacketRate()
{

    int rate = 0;

    switch(gConfiguration.packetRateDivider){
        case PACKET_RATE_DIV_200HZ:
            rate = 200;
            break;
        case PACKET_RATE_DIV_100HZ:
            rate = 100;
            break;
        case  PACKET_RATE_DIV_50HZ:
            rate = 50;
            break;
        case PACKET_RATE_DIV_25HZ:
            rate = 25;
            break;
        case PACKET_RATE_DIV_20HZ:
             rate = 20;
            break;
        case PACKET_RATE_DIV_10HZ:
            rate = 10;
            break;
        case PACKET_RATE_DIV_5HZ:
            rate = 5;
            break;
        case PACKET_RATE_DIV_2HZ:
            rate = 2;
            break;
        case 0:
        default:
            break;

    }

    return rate;
}

BOOL configSetPacketRate(int rate, BOOL fApply)
{
    BOOL res;
    uint16_t divider;

    switch(rate){
        case 200:
            divider = PACKET_RATE_DIV_200HZ;
            break;
        case 100:
            divider = PACKET_RATE_DIV_100HZ;
            break;
        case  50:
            divider = PACKET_RATE_DIV_50HZ;
            break;
        case 25:
            divider = PACKET_RATE_DIV_25HZ;
            break;
        case 20:
             divider = PACKET_RATE_DIV_20HZ;
            break;
        case 10:
            divider = PACKET_RATE_DIV_10HZ;
            break;
        case 5:
            divider = PACKET_RATE_DIV_5HZ;
            break;
        case 2:
            divider = PACKET_RATE_DIV_2HZ;
            break;
        case 0:
            divider = PACKET_RATE_DIV_QUIET;
            break;
        default:
            return FALSE;

    }
    
    uint16_t tmp = gConfiguration.packetRateDivider;

    gConfiguration.packetRateDivider =  divider;      // validation uses 100Hz based criteria

    res = ValidPortConfiguration(&gConfiguration); 

    if (res == FALSE || !fApply) {
        gConfiguration.packetRateDivider =  tmp;
    }

    return res;
}




uint32_t GetFilterCounts(uint32_t type)
{
    switch(type){
        case IIR_02HZ_LPF:
            return 26785;
        case IIR_05HZ_LPF:
            return 10713;
        case IIR_10HZ_LPF:
            return 5356;
        case IIR_20HZ_LPF:
            return 2678;
        case IIR_25HZ_LPF:
            return 2142;
        case IIR_40HZ_LPF:
            return 1338;
        case UNFILTERED:
            return 0;
        case IIR_50HZ_LPF:
        default:
            return 1070;
    }

}

uint16_t configGetParam(int idx)
{
    return ((uint16_t *)&gConfiguration)[idx];
}


/******************************************************************************
 * @name: tSelect_LP_filter API function for selecting LP filter type
 * @author
 * @param [in]  type   - new value selection for LP filter type
 * @param [in]  sensor - sensor type to apply filter to 1 - accel, 2 - rate
 * @retval TRUE if success FALSE otherwise
 ******************************************************************************/
BOOL configSelectUserLPFilter(int sensor, int cutoffFreq, BOOL fApply)
{
    BOOL res = TRUE;
    int  type;

    switch(cutoffFreq){
        case 0:
            type = UNFILTERED;
            break;
        case 2:
            type = IIR_02HZ_LPF;
            break;
        case 5:
            type = IIR_05HZ_LPF;
            break;
        case 10:
            type = IIR_10HZ_LPF;
            break;
        case 20:
            type = IIR_20HZ_LPF;
            break;
        case 40:
            type = IIR_40HZ_LPF;
            break;
        case 25:
            type = IIR_25HZ_LPF;
            break;
        case 50:
            type = IIR_50HZ_LPF;
            break;
        default:
            res = FALSE;
            break; 
    }
    
    if(!fApply || res == FALSE){
        return res;
    }

    gConfiguration.analogFilterClocks[sensor] = GetFilterCounts(type);
   
    return TRUE;
}


BOOL configSetUserOrientation(uint16_t *input, BOOL fApply)
{
    BOOL res;
    uint32_t  orientation = 0;
    uint16_t  sel;
    volatile  int i,j;

    for(i = FORWARD; i <= DOWN; i++){
        sel = input[i];
        j = i;
        switch(sel){
            case PLUS_X:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_X_PLUS_MASK;
                        break;  
                    case RIGHT:
                        orientation |= RIGHT_X_PLUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_X_PLUS_MASK;
                        break;
                    default:   
                        return FALSE;
                }
                break;
            case PLUS_Y:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Y_PLUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Y_PLUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Y_PLUS_MASK;;
                        break;
                    default:
                        return FALSE;
                }
                break;
            case PLUS_Z:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Z_PLUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Z_PLUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Z_PLUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
            case MINUS_X:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_X_MINUS_MASK;
                        break;  
                    case RIGHT:
                        orientation |= RIGHT_X_MINUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_X_MINUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
            case MINUS_Y:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Y_MINUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Y_MINUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Y_MINUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
           case MINUS_Z:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Z_MINUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Z_MINUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Z_MINUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
            default:
                return FALSE;
        }

    }

    res = CheckOrientation(orientation);

    if (res == FALSE || !fApply) {
        return res;
    }

    gConfiguration.orientation.all =  orientation;
    
    return TRUE;

}
static BOOL _useGpsPps = FALSE;

BOOL configIsGpsPPSUsed(void)
{
    return _useGpsPps;
}

void configEnableGpsPps(BOOL enable)
{
    _useGpsPps = enable;
}

BOOL  configSetOutputPacketCode(uint16_t code, BOOL fApply)
{
    uint16_t tmp =  gConfiguration.packetCode;
    BOOL res;

    gConfiguration.packetCode = code;
    res = ValidPortConfiguration(&gConfiguration);

    if (res == FALSE || !fApply) {
        gConfiguration.packetCode =  tmp;
    }

    return res;
}


char *configBuildInfo()
{
    return SOFTWARE_PART;
}

uint16_t configGetSensorFilterTypeForSPI()
{
    return  spiFilterType;
}

void     configSetSensorFilterTypeForSPI(uint16_t type)
{
    spiFilterType = type;
}

void   configSetPacketRateDividorForSPI(uint16_t dvd)
{
    spiPacketRateDividor = dvd;
}

uint16_t configGetPacketRateDividorForSPI()
{
    return spiPacketRateDividor;
}


int configGetCANBaudRate()
{
    switch(gConfiguration.ecuBaudRate){
        case _ECU_125K:
            return 125000;
        case _ECU_500K:
            return 500000;
        case _ECU_1000K:
            return 1000000;
        case _ECU_250K:
        default:
            return 250000;
    }
} 

uint16_t configGetCANPacketRate()
{
    int odr = gConfiguration.CanOdr;
    
    switch(odr){
        case 0:
        case 2:
        case 5:
        case 10:
        case 20:
        case 25:
        case 50:
        case 100:
            break;
        default:
            odr = 100;
    }
    return odr;
}
uint16_t configGetCANPacketsToTransmit()
{
    return gConfiguration.canPacketType;
}

int      configGetBaudRate(void)
{
    return baudEnumToBaudRate(gConfiguration.baudRateUser);
}

int      configGetPacketRateDivider(int configParam)
{
#ifdef USE_SPI
    if(fSPI){
        return spiPacketRateDividor;
    }
    else
#endif
    {
        if(configParam != PACKET_RATE_DIV_200HZ){
            return configParam*2;
        }
        return 1;
    }
}

uint16_t configGetAccelLfpFreq()
{

    return gConfiguration.analogFilterClocks[1];
}

uint16_t configGetRateLfpFreq()
{

    return gConfiguration.analogFilterClocks[2];
}

uint16_t configGetPrefilterFreq()
{

    return gConfiguration.analogFilterClocks[0];
}


uint16_t configGetOrientation()
{

    return gConfiguration.orientation.all;
}

uint16_t configGetPacketCode()
{
    return gConfiguration.packetCode;
}

/*end void initConfigureUnit(void) */
