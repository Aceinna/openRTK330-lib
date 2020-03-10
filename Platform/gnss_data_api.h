/** ***************************************************************************
 * @file gnss_data_api.h GNSS data API for algorithm and I/O layer
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
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

#ifndef _GNSS_DATA_API_H
#define _GNSS_DATA_API_H

#include <stdint.h>

#include "constants.h"
#include "rtcm.h"
#include "timer.h"

typedef struct  {
    uint16_t gps_week;
    uint32_t gps_tow; // gps Time Of Week, miliseconds

    uint8_t  gnss_fix_type; // 1 if data is valid
    uint8_t  gnss_update; // 1 if contains new data
    uint8_t  num_sats; // num of satellites in the solution 

    double latitude; // latitude ,  degrees 
    double longitude; // longitude,  degrees 
    double altitude; // above mean sea level [m]
    float vel_ned[3]; // velocities,  m/s  NED (North East Down) x, y, z
    float heading; // [deg]

    float dops[5];
    float sol_age;

	float std_lat;	//!< latitude standard deviation (m)
	float std_lon;	//!< longitude standard deviation (m)
	float std_hgt;	//!< height standard deviation (m)
    float std_vn;
    float std_ve;
    float std_vd;

} gnss_solution_t;

typedef struct {                        /* rtcm data struct */
    gnss_rtcm_t rtcm; /* store RTCM data struct for RTK and PPP */

    /* for algorithm */
    obs_t rov;
    obs_t ref;
    nav_t nav;
} gnss_raw_data_t;

extern int input_rtcm3_data(rtcm_t *rtcm, unsigned char data, obs_t *obs, nav_t *nav);
extern int input_rtcm3(unsigned char data, unsigned int stnID, gnss_rtcm_t *gnss);

#endif /* _GNSS_DATA_API_H */
