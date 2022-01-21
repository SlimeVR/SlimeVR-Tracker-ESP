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
#ifndef SLIMEVR_DEFINES_H_
#define SLIMEVR_DEFINES_H_

#include "consts.h"
#include "debug.h"

// Set parameters of IMU and board used
#define IMU IMU_ICM20948
#define BOARD BOARD_SLIMEVR
#define IMU_ROTATION DEG_90
#define SECOND_IMU_ROTATION DEG_270
#define BATTERY_SHIELD_130K false

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
  #error IMU_MPU9250 cannot be used yet. Use IMU_MPU6050.
  #define IMU_NAME "MPU9250"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
  #define I2C_SPEED 400000
#elif IMU == IMU_MPU6050
  #define IMU_NAME "MPU6050"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG false
  #define I2C_SPEED 400000
  #define IMU_MPU6050_RUNTIME_CALIBRATION // Comment to revert to startup/traditional-calibration
#elif IMU == IMU_MPU6500
  #define IMU_NAME "MPU6500"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG false
  #define I2C_SPEED 400000
  #define IMU_MPU6050_RUNTIME_CALIBRATION // Comment to revert to startup/traditional-calibration
#elif IMU == IMU_ICM20948
  #define IMU_NAME "ICM20948"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
  #define USE_6_AXIS true // uses 9 (with mag) if false
  #define LOAD_BIAS 1 // Loads the bias values from NVS on start (ESP32 Only)
  #define SAVE_BIAS 1 // Periodically saves bias calibration data to NVS (ESP32 Only)
  #define ENABLE_TAP false // monitor accel for (tripple) tap events and send them. Uses more cpu, disable if problems. Server does nothing with value so disabled atm
  #define ICM_ADDR_1 0x68
  #define ICM_ADDR_2 0x69
  #define I2C_SPEED 400000
  #define SECOND_IMU true
#else
    #error Select IMU in defines.h
#endif

#if BOARD == BOARD_SLIMEVR || BOARD == BOARD_SLIMEVR_DEV
  #define PIN_IMU_SDA 4
  #define PIN_IMU_SCL 5
  #define PIN_IMU_INT 10
  #define PIN_IMU_INT_2 13
  #define PIN_BATTERY_LEVEL 17
#elif BOARD == BOARD_NODEMCU || BOARD == BOARD_WEMOSD1MINI
  #define PIN_IMU_SDA D2
  #define PIN_IMU_SCL D1
  #define PIN_IMU_INT D5
  #define PIN_IMU_INT_2 D6
  #define PIN_BATTERY_LEVEL A0
#elif BOARD == BOARD_TTGO_TBASE
  #define PIN_IMU_SDA 5
  #define PIN_IMU_SCL 4
  #define PIN_IMU_INT 14
  #define PIN_IMU_INT_2 13
  #define PIN_BATTERY_LEVEL A0
#elif BOARD == BOARD_CUSTOM
  // Define pins by the examples above
#elif BOARD == BOARD_WROOM32
  #define PIN_IMU_SDA 21
  #define PIN_IMU_SCL 22
  #define PIN_IMU_INT 23
  #define PIN_IMU_INT_2 25
  #define PIN_BATTERY_LEVEL 36
#endif

#define LOADING_LED LED_BUILTIN
#define CALIBRATING_LED LED_BUILTIN
#define STATUS_LED LED_BUILTIN

#if defined(BATTERY_SHIELD_130K) && BATTERY_SHIELD_130K == true
  // Wemos D1 Mini has an internal Voltage Divider with R1=220K and R2=100K > this means, 3.3V analogRead input voltage results in 1023.0
  // Wemos D1 Mini with Wemos BatteryShiled v1.2.0 or higher: BatteryShield with J2 closed, has an additional 130K resistor. So the resulting Voltage Divider is R1=220K+100K=320K and R2=100K > this means, 4.5V analogRead input voltage results in 1023.0
  #define batteryADCMultiplier 1.0 / 1023.0 * 4.5
#else
  // SlimeVR Board can handle max 5V > so analogRead of 5.0V input will result in 1023.0
  #define batteryADCMultiplier 1.0 / 1023.0 * 5.0
#endif

#endif // SLIMEVR_DEFINES_H_
