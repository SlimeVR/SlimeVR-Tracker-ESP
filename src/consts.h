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
// List of constants used in other places
#define IMU_MPU9250 1
#define IMU_MPU6500 2
#define IMU_BNO080 3
#define IMU_BNO085 4
#define IMU_BNO086 4
#define IMU_BNO055 5
#define IMU_MPU6050 6

#define BOARD_SLIMEVR 1
#define BOARD_SLIMEVR_DEV 2
#define BOARD_NODEMCU 3
#define BOARD_CUSTOM 4
#define BOARD_WROOM32 5
#define BOARD_WEMOSD1MINI 6

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#ifdef ESP8266
  #define HARDWARE_MCU 1
#elif defined(ESP32)
  #define HARDWARE_MCU 2
#else
  #define HARDWARE_MCU 0
#endif