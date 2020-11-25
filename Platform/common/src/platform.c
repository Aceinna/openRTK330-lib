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
 *****************************************************************************/
#include <string.h>
#include <stdint.h>

//*****************************
// #include "BitStatus.h"
#include "platform_version.h"
#include "constants.h"

int     userSerialChan;
int     gpsSerialChan;
int     debugSerialChan;

BOOL fSPI  = FALSE;
BOOL fUART = FALSE;

// BITStatusStruct     gBitStatus;


BOOL platformHasMag()
{
    return FALSE;
}


BOOL _useExtSync = FALSE;

BOOL platformIsExtSyncUsed(void)
{
    return _useExtSync;
}

void platformEnableExtSync(BOOL enable)
{
    _useExtSync = enable;
}

char *platformBuildInfo()
{
    return SOFTWARE_PART;
}

uint32_t _commType = UART_COMM;

uint32_t platformGetUnitCommunicationType()
{
    return _commType;
}

void platformSetUserCommunicationType(uint32_t type)
{
    if(type != UART_COMM && type != SPI_COMM && type != CAN_BUS){
        return;
    }
    _commType = type;
}

int platformGetSysRange()
{
    return _400_DPS_RANGE;
}


BOOL bootMode = FALSE;

BOOL platformIsInBootMode()
{
    return bootMode;
}

void platformSetMode(BOOL isBoot)
{
    bootMode = isBoot;
}

void platformGetVersionBytes(uint8_t *bytes)
{
    bytes[0] = (uint8_t)VERSION_MAJOR_NUM;
    bytes[1] = (uint8_t)VERSION_MINOR_NUM;
    bytes[2] = (uint8_t)VERSION_PATCH_NUM;
    bytes[3] = (uint8_t)VERSION_STAGE_NUM;
    bytes[4] = (uint8_t)VERSION_BUILD_NUM;
}

/*end void initConfigureUnit(void) */
