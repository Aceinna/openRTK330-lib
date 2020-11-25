/** ***************************************************************************
 * @file configureGPIO.h BSP call to set up GPIO pins
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

#ifndef _CONFIGURE_IO_H_
#define _CONFIGURE_IO_H_

#include <stdint.h>

#include "stm32f4xx.h"
#include "boardDefinition.h"

void ResetSTIForNormalMode(void);
void ResetSTIForBootMode(void);


#endif