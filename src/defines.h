/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
#include "consts.h"

// Set parameters of IMU and board used
#define IMU IMU_BNO085
#define BOARD BOARD_SLIMEVR
#define IMU_ROTATION -PI / 2.0
#define SECOND_IMU_ROTATION PI / 2.0

#if IMU == IMU_BNO085
  #define IMU_NAME "BNO085"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
  #define BNO_HAS_ARVR_STABILIZATION true
  #define I2C_SPEED 400000
#elif IMU == IMU_BNO080
  #define IMU_NAME "BNO080"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
  #define BNO_HAS_ARVR_STABILIZATION false
  #define I2C_SPEED 400000
#elif IMU == IMU_BNO055
  #define IMU_NAME "BNO055"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
  #define BNO_HAS_ARVR_STABILIZATION false
  #define I2C_SPEED 400000
#elif IMU == IMU_MPU9250
  #define IMU_NAME "MPU9250"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
  #define I2C_SPEED 100000
#elif IMU == IMU_MPU6050
  #define IMU_NAME "MPU6050"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG false
  #define I2C_SPEED 100000
#elif IMU == IMU_MPU6500
  #define IMU_NAME "MPU6500"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG false
  #define I2C_SPEED 100000
#else
    #error Select IMU in defines.h
#endif

#if BOARD == BOARD_SLIMEVR || BOARD == BOARD_SLIMEVR_DEV
  #define PIN_IMU_SDA 4
  #define PIN_IMU_SCL 5
  #define PIN_IMU_INT 10
  #define PIN_IMU_INT_2 13
  #define PIN_BATTERY_LEVEL 17
  #if BOARD == BOARD_SLIMEVR_DEV
    #define BNO_ADDR_1 0x4B
    #define BNO_ADDR_2 0x4A
  #else
    #define BNO_ADDR_1 0x4A
    #define BNO_ADDR_2 0x4B
  #endif
#elif BOARD == BOARD_NODEMCU
  #define PIN_IMU_SDA D2
  #define PIN_IMU_SCL D1
  #define PIN_IMU_INT D5
  #define PIN_IMU_INT_2 D6
  #define BNO_ADDR_1 0x4A
  #define BNO_ADDR_2 0x4B
#elif BOARD == BOARD_CUSTOM
  // Define pins by the examples above
#elif BOARD == BOARD_WROOM32
  #define PIN_IMU_SDA 21
  #define PIN_IMU_SCL 22
  #define PIN_IMU_INT 23
  #define PIN_IMU_INT_2 25
  #define PIN_BATTERY_LEVEL 36
  #define BNO_ADDR_1 0x4A
  #define BNO_ADDR_2 0x4B
#endif

#define LOADING_LED LED_BUILTIN
#define CALIBRATING_LED LED_BUILTIN
#define STATUS_LED LED_BUILTIN

//Debug information
//#define FULL_DEBUG
#define serialDebug false // Set to true to get Serial output for debugging
#define serialBaudRate 115200
#define UPDATE_IMU_UNCONNECTED 1
#define SEND_UPDATES_UNCONNECTED 1
#define LED_INTERVAL_STANDBUY 10000
#define STATUS_PRINT_INTERVAL 15000

// Determines how often we sample and send data
#define samplingRateInMillis 10
#define batterySampleRate 10000

// Setup for the Magnetometer
#define useFullCalibrationMatrix true

#define sensorIdTime 1000
#define sensorIdInterval 100

#define batteryADCMultiplier 1.0 / 1024.0 * 5.0

#define FIRMWARE_BUILD_NUMBER 5
#define FIRMWARE_VERSION "0.0.5"
