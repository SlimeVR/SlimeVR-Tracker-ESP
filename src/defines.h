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

//#define FULL_DEBUG

#define IMU_MPU9250 1
#define IMU_MPU6500 2
#define IMU_BNO080 3
#define IMU_BNO085 4
#define IMU_BNO055 5

#define IMU IMU_BNO085

#if IMU == IMU_BNO085
  #define IMU_NAME "BNO085"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
  #define BNO_HASARVR_STABILIZATION true
#elif IMU == IMU_BNO080
  #define IMU_NAME "BNO080"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
  #define BNO_HASARVR_STABILIZATION false
#elif IMU == IMU_BNO055
  #define IMU_NAME "BNO055"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
  #define BNO_HASARVR_STABILIZATION false
#elif IMU == IMU_MPU9250
  #define IMU_NAME "MPU9250"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG true
#elif IMU == IMU_MPU6500
  #define IMU_NAME "MPU6500"
  #define IMU_HAS_ACCELL true
  #define IMU_HAS_GYRO true
  #define IMU_HAS_MAG false
#else
    #error Select IMU in defines.h
#endif

//Debug information
#define serialDebug false // Set to true to get Serial output for debugging
#define serialBaudRate 115200

// Determines how often we sample and send data
#define samplingRateInMillis 10
#define batterySampleRate 10000

// Setup for the Magnetometer
#define useFullCalibrationMatrix true

#define sensorIdTime 1000
#define sensorIdInterval 100