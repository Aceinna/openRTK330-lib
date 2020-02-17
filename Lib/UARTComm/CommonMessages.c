/** ***************************************************************************
 * @file   UARTMessages.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 ******************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
* HISTORY***********************************************************************
* 15/10/2019  |                                             | Daich
* Description: solve the compile warning by adding calibrationAPI.h
*******************************************************************************/

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "algorithmAPI.h"
#include "CommonMessages.h"
#include "algorithm.h"
#include "configurationAPI.h"
#include "appVersion.h"
#include "platformAPI.h"
#include "timer.h"
#ifndef BAREMETAL_OS
    #include "sensorsAPI.h"
    #include "UserMessaging.h"
    #include "calibrationAPI.h"
#else
    #include "bare_osapi.h"
#endif



IMUDataStruct gIMU;


/******************************************************************************
 * @name  FillPingPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_PingPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    int len; 
    uint8_t *model           = (uint8_t*)GetUnitVersion();
    uint8_t *rev             = (uint8_t*)platformBuildInfo();
    unsigned int serialNum   = GetUnitSerialNum();
    len = snprintf((char*)payload, 250, "%s %s %s SN:%u", PRODUCT_NAME_STRING, model, rev, serialNum );
    *payloadLen = len;
    return TRUE;
}

/******************************************************************************
 * @name FillVersionPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_VersionPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
   int len = snprintf((char*)payload, 250, "%s", APP_VERSION_STRING );
   *payloadLen = len;
    return TRUE;
}

/******************************************************************************
 * @name FillTestPacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_z1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    uint64_t tstamp = 0;
    double accels[NUM_AXIS] = {0};
    double mags[NUM_AXIS] = {0};
    double rates[NUM_AXIS] = {0};
    
    data1_payload_t *pld = (data1_payload_t *)payload;  

//  tstamp = platformGetDacqTimeStamp();          // time stamp of last sensor sample in microseconds from system start
//  tstamp = platformGetCurrTimeStamp();            // current time stamp in microseconds from system start
//  tstamp /= 1000;                                 // convert to miliseconds 
//  timer  = getSystemTime();                       // OS timer value (tick defined in FreeRTOSConfig.h)
    pld->timer = tstamp;
    // GetAccelData_mPerSecSq(accels);
    // GetRateData_degPerSec(rates);
    // GetMagData_G(mags);

    for (int i = 0; i < NUM_AXIS; i++){
        pld->accel_mpss[i] = (float)accels[i];
        pld->rate_dps[i] = (float)rates[i];
        pld->mag_G[i] = (float)mags[i];
    }

    *payloadLen = sizeof(data1_payload_t);
    return TRUE;
}


/******************************************************************************
 * @name FillTestPacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_a1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    angle1_payload_t* pld = (angle1_payload_t*)payload;

    // Variables used to hold the EKF values
    real EulerAngles[NUM_AXIS] = {0};
    real CorrRates_B[NUM_AXIS] = {0};
    double accels[NUM_AXIS] = {0};
    // Diagnostic flags
    uint8_t OperMode, LinAccelSwitch, TurnSwitch;

    EKF_GetAttitude_EA(EulerAngles);
    EKF_GetCorrectedAngRates(CorrRates_B);
    //GetAccelData_mPerSecSq(accels);
    EKF_GetOperationalMode(&OperMode);
    EKF_GetOperationalSwitches(&LinAccelSwitch, &TurnSwitch);

    //pld->itow           = getAlgorithmITOW();
    pld->dblItow        = 1.0e-3 * pld->itow;
    pld->roll           = (float)EulerAngles[ROLL];
    pld->pitch          = (float)EulerAngles[PITCH];
    pld->ekfOpMode      = OperMode;
    pld->accelLinSwitch = LinAccelSwitch;
    pld->turnSwitch     = TurnSwitch;

    for(int i = 0; i < NUM_AXIS; i++){
        pld->corrRates[i] = (float)CorrRates_B[i];
        pld->accels[i]    = (float)accels[i];
    }

    *payloadLen  = sizeof(angle1_payload_t);
    
    return TRUE;
}


/******************************************************************************
 * @name FillTestPacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_a2PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    angle2_payload_t* pld = (angle2_payload_t*)payload;

    // Variables used to hold the EKF values
    real EulerAngles[NUM_AXIS] = {0};
    real CorrRates_B[NUM_AXIS] = {0};
    double accels[NUM_AXIS] = {0};

    EKF_GetAttitude_EA(EulerAngles);
    EKF_GetCorrectedAngRates(CorrRates_B);
    // GetAccelData_mPerSecSq(accels);

    // pld->itow           = getAlgorithmITOW();
    pld->dblItow        = 1.0e-3 * pld->itow;
    pld->roll           = (float)EulerAngles[ROLL];
    pld->pitch          = (float)EulerAngles[PITCH];
    pld->yaw            = (float)EulerAngles[YAW];

    for(int i = 0; i < NUM_AXIS; i++){
        pld->corrRates[i] = (float)CorrRates_B[i];
        pld->accels[i]    = (float)accels[i];
    }

    *payloadLen  = sizeof(angle2_payload_t);
    
    return TRUE;
}

/******************************************************************************
 * @name Fills1PacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_s1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    double accels[NUM_AXIS] = {0};
    double mags[NUM_AXIS] = {0};
    double rates[NUM_AXIS] = {0};
    double temp = 0;
    scaled1_payload_t *pld = (scaled1_payload_t *)payload;  
    *payloadLen = sizeof(scaled1_payload_t);

    // GetAccelData_mPerSecSq(accels);
    // GetRateData_degPerSec(rates);
    // GetMagData_G(mags);
    // GetBoardTempData(&temp);

    // pld->tstmp   = platformGetIMUCounter();
    pld->dbTstmp = (double)pld->tstmp * 1.0e-3; // seconds

    for (int i = 0; i < NUM_AXIS; i++){
        pld->accel_g[i]  = (float)accels[i];
        pld->rate_dps[i] = (float)rates[i];
        pld->mag_G[i]    = (float)mags[i];
    }
    
    pld->temp_C = (float)temp;

    return TRUE;
}

/******************************************************************************
 * @name Fille1PacketPayloadt - API call ro prepare user output packet - EKF data 1
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_e1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    // Variables used to hold the EKF values
    uint8_t opMode, linAccelSw, turnSw;
    float data[NUM_AXIS] = {0};
    real EulerAngles[NUM_AXIS];
    
    ekf1_payload_t *pld = (ekf1_payload_t *)payload;  

    *payloadLen  = sizeof(ekf1_payload_t);
    pld->tstmp   = gIMU.timerCntr;
    pld->dbTstmp = IMU_start_time.time + IMU_start_time.msec*0.0001;// seconds

    EKF_GetAttitude_EA(EulerAngles);
    pld->roll    = (float)EulerAngles[ROLL];
    pld->pitch   = (float)EulerAngles[PITCH];
    pld->yaw     = (float)EulerAngles[YAW];

    GetAccelData_g(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accels_g[i] = (float)data[i];
    }

    GetRateData_degPerSec(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rates_dps[i] = data[i];
    }

    GetMagData_G(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->mags[i] = data[i];
    }

    float rateBias[NUM_AXIS];
    EKF_GetEstimatedAngRateBias(rateBias);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rateBias[i] = (float)rateBias[i];
    }

    EKF_GetOperationalMode(&opMode);
    EKF_GetOperationalSwitches(&linAccelSw, &turnSw);
    pld->opMode         = opMode;
    pld->accelLinSwitch = linAccelSw;
    pld->turnSwitch     = turnSw;
    return TRUE;
}

/******************************************************************************
 * @name Fille2PacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_e2PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    float  fData[3];
    real   rData[3];
    double dData[3];

    ekf2_payload_t *pld = (ekf2_payload_t *)payload;  
    
    *payloadLen  = sizeof(ekf2_payload_t);
    pld->tstmp   = gIMU.timerCntr;
    pld->dbTstmp = IMU_start_time.time + IMU_start_time.msec*0.0001;// seconds


    EKF_GetAttitude_EA(rData);
    pld->roll  = (float)rData[ROLL];
    pld->pitch = (float)rData[PITCH];
    pld->yaw   = (float)rData[YAW];

    GetAccelData_g(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accels_g[i] = (float)fData[i]; 
    }

    EKF_GetEstimatedAccelBias(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accelBias[i] = fData[i]; 
    }

    GetRateData_degPerSec(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rates_dps[i] = (float)fData[i]; 
    }

    EKF_GetEstimatedAngRateBias(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rateBias[i] = fData[i]; 
    }


    EKF_GetEstimatedVelocity(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->velocity[i] = fData[i]; 
    }

    GetMagData_G(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->mags[i] = (float)fData[i]; 
    }

    EKF_GetEstimatedLLA(dData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->pos[i] = (double)dData[i]; 
    }

    // // debug
    // double *algoData_6 = (double *)(algoData_4);
    // *algoData_6++ = (double)gEKFInput.lla[LAT];
    // *algoData_6++ = (double)gEKFInput.lla[LON];
    // *algoData_6++ = (double)gEKFInput.lla[ALT]; //(double)gEKFInput.trueCourse; // DW DEBUG

    uint8_t opMode, linAccelSw, turnSw;
    EKF_GetOperationalMode(&opMode);
    EKF_GetOperationalSwitches(&linAccelSw, &turnSw);

    pld->opMode         = opMode;
    pld->accelLinSwitch = linAccelSw;
    pld->turnSwitch     = turnSw;

    return TRUE;
}


/******************************************************************************
 * @name Fill_k1PacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
#ifdef TEST_RTK_TASK
BOOL Fill_k1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    *payloadLen = USR_OUT_RTK1_PAYLOAD_LEN;
    // Output time as represented by gLeveler.timerCntr (uint32_t
    // incremented at each call of the algorithm)
    uint32_t *algoData_1 = (uint32_t *)(payload);
    *algoData_1++ = gIMU.timerCntr;

    // Set the pointer of the algoData array to the payload
    // uint32_t *algoData_2 = (uint32_t*)(algoData_1);
    *algoData_1++ = gGPS.itow;

    // Set the pointer of the algoData array to the payload
    double *algoData_2 = (double *)(algoData_1);
    // double lla[NUM_AXIS];
    // EKF_GetEstimatedLLA(lla);
    *algoData_2++ = gGPS.latitude;
    *algoData_2++ = gGPS.longitude;
    *algoData_2++ = gGPS.altitude;

    // *algoData_2++ = gGPS.base_lat;
    // *algoData_2++ = gGPS.base_lon;
    // *algoData_2++ = gGPS.base_hgt;

    // Set the pointer of the algoData array to the payload
    float *algoData_3 = (float *)(algoData_2);

    *algoData_3++ = (float)gGPS.vNed[0];
    *algoData_3++ = (float)gGPS.vNed[1];
    *algoData_3++ = (float)gGPS.vNed[2];

    *algoData_3++ = gGPS.HDOP;
    *algoData_3++ = gGPS.GPSHorizAcc;
    *algoData_3++ = gGPS.GPSVertAcc;
    *algoData_3++ = gGPS.sol_age;

    float accels[NUM_AXIS];
    GetAccelData_g(accels);
    *algoData_3++ = (float)accels[X_AXIS];
    *algoData_3++ = (float)accels[Y_AXIS];
    *algoData_3++ = (float)accels[Z_AXIS];

    float rates[NUM_AXIS];
    GetRateData_degPerSec(rates);
    *algoData_3++ = (float)rates[X_AXIS];
    *algoData_3++ = (float)rates[Y_AXIS];
    *algoData_3++ = (float)rates[Z_AXIS];

    uint8_t *algoData_4 = (uint8_t *)(algoData_3);
    *algoData_4++ = gGPS.gpsFixType; //gGPS.gnss_nav_quality;
    *algoData_4++ = gGPS.num_gnss_psr;
    *algoData_4++ = gGPS.num_gnss_adr;
}
#endif



/******************************************************************************
 * @name Fill_posPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_posPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{

    return TRUE;
}

/******************************************************************************
 * @name Fill_skyviewPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_skyviewPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
 
    return true;
}
