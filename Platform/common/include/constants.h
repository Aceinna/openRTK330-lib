/*******************************************************************************
 * @file:   constants.h
 *******************************************************************************/
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


#ifndef _CONSTANTS_H
#define _CONSTANTS_H

#define BMI_RS 1
#define MAXIM_RS 2

#ifndef NULL
#define NULL 0
#endif

#define APPLY     1
#define REMOVE    0

typedef unsigned char bool;
typedef unsigned char BOOL;

#ifndef true
#define true  1
#define false 0
#endif

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

#define  TWO_POW_16  65536UL

// Constants
#define  RAD_TO_DEG     57.29577951308232
#define  DEG_TO_RAD     0.017453292519943
#define D2R        ( 0.017453292519943 ) ///< ( PI/180.0 ) = 0.017453292519943
#define R2D         57.29577951308232

#define RE_WGS84    6378137.0               /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563)     /* earth flattening (WGS84) */

// #define SIGMA        1.0e-8
#define KNOT2MPSEC   5.144444444e-1
#define SQUARE(x) ((x)*(x))

#define ROLL_INCIDENCE_LIMIT  0x1000
#define PITCH_INCIDENCE_LIMIT 0x1000
#define HARD_IRON_LIMIT       8192 // 0.25 G
#define SOFT_IRON_LIMIT       6554 // 20%
/// hard and soft iron resolution (2^16 / 2)
#define IRON_SCALE            32768


// For fast inverse trigonometric functions
#define  TAN15DEG  0.26794919243F
#define  TAN30DEG  0.57735026919F

// The following is the acceleration due to gravity at the calibration location
#define  GRAVITY            9.80665
#define  ACCEL_DUE_TO_GRAV  9.794259
#define  g_TO_M_SEC_SQ  9.80655

#define  MAX_ITOW  604800000    // Second in a week (assuming exactly 24 hours in a day)

// PI and related values
#define  TWO_PI        6.283185307179586
#ifndef PI
#define  PI            3.1415926535897932
#endif
#define  PI_OVER_TWO   1.570796326794897
#define  PI_OVER_FOUR  0.785398163397448
#define  PI_OVER_SIX   0.523598775598299

#define  ONE_OVER_PI      0.318309886183791
#define  ONE_OVER_TWO_PI  0.159154943091895

// Specify constants used to limit variables in the algorithm
#define ONE_DEGREE_IN_RAD     (0.017453292519943)
#define TWO_DEGREES_IN_RAD    (0.034906585039887)
#define TWO_PT_FIVE_DEGREES_IN_RAD    (0.043633231299858)
#define THREE_DEGREES_IN_RAD  (0.052359877559830)
#define FIVE_DEGREES_IN_RAD   (0.087266462599716)
#define SIX_DEGREES_IN_RAD    (0.104719755119660)
#define TEN_DEGREES_IN_RAD    (0.17453292519943)
#define TWENTY_DEGREES_IN_RAD (0.349065850398866)
#define THREE_HUNDRED_EIGHTY_DEGREES_IN_RAD  (6.632251157578453)

#define  FAST_MATH  1

#define MIN_TO_MILLISECONDS 60000.0

/// Specify the data acquisition task rate of the system in Hz. Due to the way data is collected,
/// this is different than the sampling rate of the sensors.  Note: must be 50 or 100 or 200.
#define  DACQ_50_HZ         50
#define  DACQ_100_HZ        100
#define  DACQ_200_HZ        200
#define  DACQ_RATE_INVALID  0

/// Specify the algorithm execution frequency in Hz.
/// So far only 100 and 200 
#define  FREQ_50_HZ         50
#define  FREQ_100_HZ        100
#define  FREQ_200_HZ        200
#define  FREQ_INVALID       0

// Choices for user communication interface
#define UART_COMM       0
#define SPI_COMM        1
#define CAN_BUS         2
#define UNVALID_COMM    100 

// Choices for sensors range
#define _200_DPS_RANGE   0
#define _400_DPS_RANGE   1
#define _1000_DPS_RANGE  2

//#define USE_DOUBLE
#ifdef USE_DOUBLE
#define  real  double
#else
#define real  float
#endif

// Force Q0 positive by flipping the sign on all quaternion-elements when q0 < 0.  This
//   seems to reduce the errors in the system although (in theory) it shouldn't affect
//   the result.
#define FORCE_Q0_POSITIVE

// some value limits
#define INT16_LIMIT 32765
#define INT12_LIMIT 2045

#define USER_PACKET_OK      0
#define UNKNOWN_USER_PACKET 1
#define USER_PACKET_ERROR   2

#define USER_OK      0x00
#define USER_NAK     0x80
#define USER_INVALID 0x81

#define MAXUINT32 4294967295 	///< max unsigned 32 bit int=> ((2^32)-1)
#define MAXUINT16      65535    ///< max unsigned 16 bit int=> ((2^16)-1)
#define MAXINT16     ( 32767)   ///< max signed 16 bit int=> ((2^15)-1)
#define MININT16     (-32768)   ///< max negative signed 16 bit int=> (-(2^15))

///  pre-computed values
#define SCALE_BY_8(value)				( (value) * 8.0 )
#define SCALE_BY_2POW16_OVER_2PI(value)	( (value) * 10430.37835047045 ) // 65536 / (2.0 * PI)
#define SCALE_BY_2POW16_OVER_7PI(value) ( (value) * 2980.108100134415 ) // 65536 / (7.0 * PI)
#define SCALE_BY_2POW16_OVER_2(value)   ( (value) * 32768.0 ) // 65536 / 2
#define SCALE_BY_2POW16_OVER_16(value)  ( (value) * 4096.0 ) // 5536 / 16
#define SCALE_BY_2POW16_OVER_20(value)  ( (value) * 3276.8 ) // 65536 / 20
#define SCALE_BY_2POW16_OVER_64(value)  ( (value) * 1024.0 ) // 65536 / 64
#define SCALE_BY_2POW16_OVER_128(value) ( (value) * 512.0 ) // 65536 / 128
#define SCALE_BY_2POW16_OVER_200(value) ( (value) * 327.68 ) // 65536 / 200
#define SCALE_BY_2POW16_OVER_512(value) ( (value) * 128.0 ) // 65536 / 512

#define SCALE_BY_2POW32_OVER_2PI(value)    ( (value) * 683565287.23678304) // 4294967296 / 2 * PI
#define SCALE_BY_2POW16_OVER_2POW14(value) ( (value) * 4.0 ) // 65536 / 16384

#define TWO_OVER_TWO_POW16_q30       32768
#define TWENTY_OVER_TWO_POW16_q30    327680
#define TWOPI_OVER_MAXINT16_q30      205894

#define TWO_POW16_OVER_7PI_q19    1562434916   // floor( 2^16/( 7*pi ) * 2^19 )
#define TWO_POW16_OVER_20_q19     1717986918   // floor( 2^16/20 * 2^19 )
#define TWO_POW16_OVER_2_q15      1073741824   // floor( 2^16/2 * 2^15 )
#define TWO_POW16_OVER_200_q22    1374389534   // floor( 2^16/200 * 2^22 )
//#define TWO_POW16_OVER_20_q19    1717986918    // floor( 2^16/20 * 2^19 )
#define TWO_POW16_TIMES_100_OVER_7PI_q12 298011 // floor( 2^16/( 7*pi ) * 2^12 )

#define TWO_POW16_OVER_128_q21 1073741824   // floor( 2^16/200 * 2^22 )
#define TWO_POW16_OVER_512_q23 128

#define TWO_POW16_OVER_2PI_q17 1367130551
#define TWO_POW19_OVER_7PI_q16 23841

#define MAXUINT16_OVER_2PI 10430.21919552736
#define DEGREES_TO_RADS        0.017453292519943
#define RADS_TO_DEGREES       57.29577951308232
#define ITAR_RATE_LIMIT        7.15585 // 410 dps * DEGREES_TO_RADS

#define MAXINT16_OVER_2PI   5215.030020292134 //( MAXINT16 / TWOPI)
#define MAXUINT16_OVER_512   127.9980468750000 // ( MAXUINT16 / 512.0)
#define MAXUINT16_OVER_2   32768.0 //( MAXUINT16 / 2.0)

#define MAXINT32_20BIT_OVER131072M  8 // 2^20/(2^17)

/// INT32_TO_MISALIGN_SCALING = 1/2^( 32 - 5 ) as per the definition of misalign
///    in DMU Serial Interface Spec
#define INT32_TO_MISALIGN_SCALING 7.450580596923828e-09

/// BOARD_TEMP_SCALE_FACTOR = 1/256 = 0.00390625 from the TMP102 datasheet
///  ( shift 4-bits due to buffer and 4-bits for scaling)
#define BOARD_TEMP_SCALE_FACTOR 0.00390625
#define BOARD_TEMP_SCALE_FACTOR_q30  419430

/// GYRO_TEMP_SCALE_FACTOR = 1/256 = 0.00390625 from the Maxim21000 datasheet
#define MAXIM21000_TEMP_SCALE_FACTOR  0.00390625
#define BMI160_TEMP_SCALE_FACTOR  0.001953125
#define BMI160_TEMP_OFFSET        23.0

#endif /* _CONSTANTS_H */

