/*****************************************************************************
 * @file platform_version.h
 *
 * @brief Platfor version definition
 ******************************************************************************/
#ifndef _PLATFORM_VERSION_H
#define _PLATFORM_VERSION_H

// DO NOT CHANGE THESE NUMBERS FROM ZERO!  CAUSES A CONFLICT WITH
//   IMUTest RESULTING IN ACCELEROMETER VALUES THAT ARE FLIPPED (WHAT
//   SHOULD BE POSITIVE BECOMES NEGATIVE).
#define VERSION_MAJOR 0
#define VERSION_MINOR 0
#define VERSION_PATCH 0
#define VERSION_STAGE 0
#define VERSION_BUILD 0

#define VERSION_MAJOR_NUM 19
#define VERSION_MINOR_NUM 1
#define VERSION_PATCH_NUM 1
#define VERSION_STAGE_NUM 0
#define VERSION_BUILD_NUM 3

/// Software-version/part-number
// changing the version number changes the CRC for the EEPROM memory so to avoid
// BIT failure the CRC must be updated if the version number is incremented
// Note: if the part number is changed before calibration the cal calculates
// and sets the CRC in EEPROM
//
//                                    1         2
//                           12345678901234567890

#define  SOFTWARE_PART      "5020-3021-01 23.00"    // openrtk330


#define  SOFTWARE_PART_LEN  50
#define  VERSION_STR        SOFTWARE_PART 
#define  N_VERSION_STR      128

#endif /* _PLATFORM_VERSION_H */
