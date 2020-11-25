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
#include "filter.h"
#include "crc16.h"
#include "platform_version.h"
#include "crc.h"
#include "stm32f4xx_hal.h"
#include "eepromAPI.h"
// #include "hwAPI.h"

ConfigurationStruct gConfiguration;
uint16_t spiFilterType = 0;  // unfiltered



uint16_t configGetUsedChips(void)   { return gConfiguration.usedChips;}
uint16_t configGetActiveChips(void) { return gConfiguration.activeChips;}
uint16_t configGetUsedSensors(int chipIdx) { return gConfiguration.usedSensors[chipIdx];}
void configSetUsedChips(uint8_t mask) { gConfiguration.usedChips = mask;}

// placholders for Nav_view compatibility
softwareVersionStruct dupFMversion;  /// 525 digital processor DUP code base
softwareVersionStruct ioupFMversion; /// 525 input output processor IOUP code base
softwareVersionStruct bootFMversion; /// bootloader code base



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

    _readConfigIntoMem();

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

char *configBuildInfo()
{
    return SOFTWARE_PART;
}

uint16_t configGetSensorFilterTypeForSPI()
{
    return  spiFilterType;
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

