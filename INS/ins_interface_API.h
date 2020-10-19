/** ***************************************************************************
 * @file int_interface_API.h API functions for Magnitometer functionality
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
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
*******************************************************************************/
#ifndef INS_INTERFACE_API
#define INS_INTERFACE_API
#include <stdint.h>
#include "gnss_data_api.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WIN32
    #define ARM_MCU
#endif


int8_t SetStartGNSSWeek(int32_t week);

int8_t GetKFStatus();
extern int8_t nhcflag;
extern int8_t GNSSOBSFLAG;
extern double lastgnsstime;
extern int KFStatus;


void ins_gnss_time_update();
void ins_fusion();
int copy_gnss_result(gnss_solution_t *gnss_sol);
int get_mGnssInsSystem_mlc_STATUS();

BOOL Fill_std1PacketPayload_ins(uint8_t *payload, uint8_t *payloadLen);
BOOL ins_Fill_posPacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL ins_Fill_inspvaPacketPayload(uint8_t *payload, uint8_t *payloadLen);

void set_wheel_tick_update_flag(int state);
int get_wheel_tick_update_flag();
void add_wheel_tick_count();
void set_wheel_tick_fwd(uint8_t fwd_flag);


extern char ggaBuff[];
extern char pashrBuff[];


int32_t get_ins_status();
int32_t get_pos_type();
double get_ins_latitude();
double get_ins_longitude();
double get_ins_height();
double get_ins_north_velocity();
double get_ins_east_velocity();
double get_ins_up_velocity();
double get_ins_roll();
double get_ins_pitch();
double get_ins_azimuth();
float get_ins_latitude_std();
float get_ins_longitude_std();
float get_ins_altitude_std();
float get_ins_north_velocity_std();
float get_ins_east_velocity_std();
float get_ins_up_velocity_std();
float get_ins_roll_std();
float get_ins_pitch_std();
float get_ins_azimuth_std();

BOOL user_car_speed_handler(uint8_t *payload, uint8_t *payloadLen);

/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
