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
#ifndef SLIMEVR_DEBUG_H_
#define SLIMEVR_DEBUG_H_
#include "consts.h"
#include "logging/Level.h"

#define IMU_MPU6050_RUNTIME_CALIBRATION // Comment to revert to startup/traditional-calibration
#define BNO_USE_ARVR_STABILIZATION true // Set to false to disable stabilization for BNO085+ IMUs
#define BNO_USE_MAGNETOMETER_CORRECTION false // Set to true to enable magnetometer correction for BNO08x IMUs. Only works with USE_6_AXIS set to true.
#define USE_6_AXIS true // uses 9 DoF (with mag) if false (only for ICM-20948 and BNO0xx currently)
#define LOAD_BIAS true // Loads the bias values from NVS on start
#define SAVE_BIAS true // Periodically saves bias calibration data to NVS
#define BIAS_DEBUG false // Printing BIAS Variables to serial (ICM20948 only)
#define ENABLE_TAP false // monitor accel for (triple) tap events and send them. Uses more cpu, disable if problems. Server does nothing with value so disabled atm
#define SEND_ACCELERATION true // send linear acceleration to the server

//Debug information

#define LOG_LEVEL LOG_LEVEL_DEBUG

#if LOG_LEVEL == LOG_LEVEL_TRACE
    #define DEBUG_SENSOR
    #define DEBUG_NETWORK
    #define DEBUG_CONFIGURATION
#endif

#define serialDebug false // Set to true to get Serial output for debugging
#define serialBaudRate 115200
#define LED_INTERVAL_STANDBY 10000
#define PRINT_STATE_EVERY_MS 60000

// Determines how often we sample and send data
#define samplingRateInMillis 10

// Sleeping options
#define POWERSAVING_MODE POWER_SAVING_LEGACY  // Minimum causes sporadic data pauses
#if POWERSAVING_MODE >= POWER_SAVING_MINIMUM
    #define TARGET_LOOPTIME_MICROS (samplingRateInMillis * 1000)
#endif

// Setup for the Magnetometer
#define useFullCalibrationMatrix true

// Battery configuration
#define batterySampleRate 10000
#define BATTERY_LOW_VOLTAGE_DEEP_SLEEP false
#define BATTERY_LOW_POWER_VOLTAGE 3.3f // Voltage to raise error

// Send updates over network only when changes are substantial
// If "false" updates are sent at the sensor update rate (usually 100 TPS)
// If "true" updates will be less frequent in the time of little motion
// Experimental
#define OPTIMIZE_UPDATES true

#define I2C_SPEED 400000

// Send inspection packets over the network to a profiler
// Not recommended for production
#define ENABLE_INSPECTION false

#define FIRMWARE_BUILD_NUMBER 15
#define FIRMWARE_VERSION "0.3.2"

#endif // SLIMEVR_DEBUG_H_
