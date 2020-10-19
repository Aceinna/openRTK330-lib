/** ***************************************************************************
 * @file   taskCANCommunication.c
 * @Author
 * @date   Aug, 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 ******************************************************************************/

#include "main.h"
#include "indices.h"
#include "platformAPI.h"
#include "calibrationAPI.h"
#include "osapi.h"

// for can interface
#include "can.h"
#include "sae_j1939.h"
#include "user_message_can.h"
#include "user_config.h"

#include "car_data.h"
   
#define ADDRESS_CLAIM_RETRY                 5

uint32_t packetRateCounter = 0;
uint32_t canLoopCounter    = 0;
BOOL canStarted            = FALSE;


/** ***************************************************************************
 * @name TaskCANCommunication() CAN communication task
 * @brief  perform a thread of CAN transmission and receiving
 *
 * @param [in] 
 * @retval
 ******************************************************************************/
void TaskCANCommunicationJ1939(void const *argument)
{
    int res;
    _ECU_BAUD_RATE baudRate = (_ECU_BAUD_RATE)gEcuConfig.baudRate;
    int            address  = gEcuConfig.address;
    BOOL  finished;
    
    for (;;) {
        if (gOdoConfigurationStruct.can_mode == 0) {
            car_can_initialize();

            while (1)
            {
                OS_Delay(1000);
                if (gOdoConfigurationStruct.can_mode != 0) {
                    break;
                }
            }
            
        } else if (gOdoConfigurationStruct.can_mode == 1) {
            can_config(1, baudRate);

            // initialize J1939 protocol stack   
            if (!(gEcuConfigPtr->ecu_name.words)) {
                gEcuConfigPtr->ecu_name.bits.function         = ACEINNA_SAE_J1939_FUNCTION;
                gEcuConfigPtr->ecu_name.bits.manufacture_code = ACEINNA_SAE_J1939_MANUFACTURER_CODE;
                gEcuConfigPtr->ecu_name.bits.identity_number = GetUnitSerialNum() & 0x1fffff;
            }
            
            sae_j1939_initialize(baudRate, address);
            
            // deactivate semaphore if active 
            res = osSemaphoreWait(g_sem_can_data, 0);
            // Main loop for the task
            while( 1 )
            {
                // wait for events from DACQ task
                // Semaphore given at 100Hz
                res = osSemaphoreWait(g_sem_can_data, 1000);
                if(res != osOK){
                    continue;
                }
                
                canLoopCounter++;

                // Auto-detection 
                if (gEcuInst.state == _ECU_BAUDRATE_DETECT) {
                    canStartDetectRxIntCounter =  canRxIntCounter;
                    finished = can_detect_baudrate(&baudRate);
                    if(!finished){
                        continue;
                    }
                    gEcuInst.state = _ECU_CHECK_ADDRESS;
                }

                canStarted = TRUE;

                // address claiming state
                if ((gEcuInst.state == _ECU_CHECK_ADDRESS) || (gEcuInst.state == _ECU_WAIT_ADDRESS)|| !(canLoopCounter % 100)) {
                    if (canLoopCounter > ADDRESS_CLAIM_RETRY){
                        gEcuInst.state = _ECU_READY;
                    }
                    send_address_claim(&gEcuInst);
                }
                // process incoming messages
                ecu_process();

                packetRateCounter++;
                
                if(packetRateCounter >= gEcuConfig.packet_rate){
                    packetRateCounter   = 0;
                    // prepare and send outgoing periodic data packets
                    // send periodic packets
                    enqeue_periodic_packets();
                }
                
                ecu_transmit(); 
                
                if (gOdoConfigurationStruct.can_mode != 1) {
                    break;
                }
            }
        }
    }

}
