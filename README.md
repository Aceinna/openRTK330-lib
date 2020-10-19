# Aceinna OpenRTK330L Module Open Source Firmware Library



## Introduction

Aceinna's OpenRTK330L is a GNSS/IMU integrated high-precision positioning module that is targeted for vehicular applications, for example autonomous-driving. The module's base firmware library is open-sourced for user to play with raw IMU data generation, data I/O customization and so on. Note the Aceinna's proprietary GNSS/INS algorithm library is NOT open-sourced but provided in the form of static library. 

## Usage

This library repo is designed to work with the open-sourced IMU platform, i.e. the Github repo "platform_aceinna-imu" under the same organization. Please clone the "platform_aceinna-imu" repo and go to "example" subfolder to build the various "Apps" inside. These Apps automatically clones this base library repo for a complete project.

### Release notes

**v1.0.7**, Oct. 19, 2020

- Updated for "RTK_INS" App v2.0.0
- Added CAN interface for data I/O
  - Speed measurement input from car CAN bus, for GNSS/INS/Odometer integrated positioning
  - Standard J1939 CAN message output
- Added wheel-tick speed interface for GNSS/INS/Odometer integrated positioning