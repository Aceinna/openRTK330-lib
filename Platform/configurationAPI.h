/** ******************************************************************************
 * @file configurationAPI.h API functions for Interfacing with unit configurationb parameters
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
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


#ifndef _CONFIG_API_H
#define _GONFIG_API_H
#include <stdint.h>

#include "constants.h"

// IMU related functions
uint16_t configGetSensorFilterTypeForSPI();
int      configGetAccelLfpFreq();
int      configGetRateLfpFreq();
uint16_t configGetPrefilterFreq();
uint16_t configGetUsedChips(void);
uint16_t configGetActiveChips(void);
uint16_t configGetUsedSensors(int chipIdx);
void     configSetUsedSensors(int idx, uint8_t mask);
void     configSetUsedChips(uint8_t mask);

void     ApplyFactoryConfiguration();

#endif
