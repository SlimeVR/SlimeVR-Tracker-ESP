/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#ifndef SLIMEVR_CONSTS_H_
#define SLIMEVR_CONSTS_H_

// List of constants used in other places
#define IMU_MPU9250 1
#define IMU_MPU6500 2
#define IMU_BNO080 3
#define IMU_BNO085 4
#define IMU_BNO055 5
#define IMU_MPU6050 6
#define IMU_BNO086 7
#define IMU_BMI160 8
#define IMU_ICM20948 9

#define BOARD_SLIMEVR_LEGACY 1
#define BOARD_SLIMEVR_DEV 2
#define BOARD_NODEMCU 3
#define BOARD_CUSTOM 4
#define BOARD_WROOM32 5
#define BOARD_WEMOSD1MINI 6
#define BOARD_TTGO_TBASE 7
#define BOARD_ESP01 8
#define BOARD_SLIMEVR 9
#define BOARD_LOLIN_C3_MINI 10
#define BOARD_BEETLE32C3 11
#define BOARD_ES32C3DEVKITM1 12

#define BAT_EXTERNAL 1
#define BAT_INTERNAL 2
#define BAT_MCP3021 3
#define BAT_INTERNAL_MCP3021 4

#define LED_OFF 255

#define POWER_SAVING_LEGACY 0 // No sleeping, but PS enabled
#define POWER_SAVING_NONE 1 // No sleeping, no PS => for connection issues
#define POWER_SAVING_MINIMUM 2 // Sleeping and PS => default
#define POWER_SAVING_MODERATE 3 // Sleeping and better PS => might miss broadcasts, use at own risk
#define POWER_SAVING_MAXIMUM 4 // Actual CPU sleeping, currently has issues with disconnecting

#define DEG_0 0.f
#define DEG_90 -PI / 2
#define DEG_180 PI
#define DEG_270 PI / 2

#ifdef ESP8266
  #define HARDWARE_MCU 1
#elif defined(ESP32)
  #define HARDWARE_MCU 2
#else
  #define HARDWARE_MCU 0
#endif

#define CURRENT_CONFIGURATION_VERSION 1

#endif // SLIMEVR_CONSTS_H_
