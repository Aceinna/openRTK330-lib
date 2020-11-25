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

#pragma pack(1)
typedef struct {
	uint16_t gps_week;          // GPS Week number
    uint32_t gps_millisecs;     // Milliseconds into week
	uint32_t ins_status;        //!< Solution status
	uint32_t pos_type;          //!< Position type
	double latitude;			//!< latitude - WGS84 (deg)
	double longitude;			//!< longitude - WGS84 (deg)
	double height;				//!< Height above mean sea level  - WGS84 (m)
	double north_velocity;		//!< velocity in a northerly direction (m/s)
	double east_velocity;		//!< velocity in an easterly direction (m/s)
	double up_velocity;			//!< velocity in an up direction
	double roll;				//!< right handed rotation around y-axis (degrees)
	double pitch;				//!< right handed rotation aruond x-axis (degrees)
	double azimuth;				//!< right handed rotation around z-axis (degrees)
	float latitude_std;
	float longitude_std;
	float altitude_std;
	float north_velocity_std;
	float east_velocity_std;
	float up_velocity_std;
	float roll_std;
	float pitch_std;
	float azimuth_std;
	uint16_t time_since_update;  //!< Elapsed time since the last ZUPT or positionupdate (seconds)
    uint16_t mlc_status;
} ins_solution_t;
#pragma pack()

extern ins_solution_t g_ins_sol;
extern ins_solution_t *g_ptr_ins_sol;


void set_gnss_start_week(int32_t week);
uint8_t is_gnss_start_week_valid(void);
int32_t get_gnss_start_week(void);

uint16_t get_imu_week(void);
double get_imu_timestamp(void);
uint16_t get_imu_week_next(void);
double get_imu_timestamp_next(void);

int8_t GetKFStatus(void);

uint8_t get_gnss_obs_valid(void);
void set_gnss_obs_valid(uint8_t value);

void set_wheel_tick_update_flag(int32_t state);
int32_t get_wheel_tick_update_flag(void);
void add_wheel_tick_count(void);
void set_wheel_tick_fwd(uint8_t fwd_flag);
int32_t get_mGnssInsSystem_mlc_STATUS(void);

int32_t copy_gnss_result(gnss_solution_t *gnss_sol);


void ins_gnss_time_update(void);
void ins_fusion(void);
void ins_user_odo_handler(double speed);

double* get_imu_acc(void);
double* get_imu_gyro(void);
uint16_t get_odo_gps_week(void);
double get_odo_gps_timestamp(void);
int8_t get_odo_mode(void);
double get_odo_speed(void);
int8_t get_odo_fwd(void);
uint64_t get_odo_wheel_tick(void);

int print_nmea_ins(double *ep, char *buff);
int print_nmea_vtg(char *buff);
int print_nmea_pashr(double *ep, char *buff);

/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
