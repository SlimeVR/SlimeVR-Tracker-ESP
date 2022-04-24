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
// ================================================
// See docs for configuration options and examples:
// https://docs.slimevr.dev/firmware/configuring-project.html#2-configuring-definesh
// ================================================

// Set parameters of IMU and board used
#define BOARD BOARD_CUSTOM

#define IMU_A1 IMU_MPU9250
#define IMU_A2 IMU_MPU9250
#define IMU_B1 IMU_MPU9250
#define IMU_B2 IMU_MPU9250
#define IMU_C1 IMU_MPU9250
#define IMU_C2 IMU_MPU9250
#define IMU_ROTATION_A1 DEG_90
#define IMU_ROTATION_A2 DEG_90
#define IMU_ROTATION_B1 DEG_90
#define IMU_ROTATION_B2 DEG_90
#define IMU_ROTATION_C1 DEG_90
#define IMU_ROTATION_C2 DEG_90
//#define PIN_IMU_SDA 26
//#define PIN_IMU_SCL_A 25
#define PIN_IMU_SCL_B 15
#define PIN_IMU_SCL_C 2
//#define PIN_IMU_INT_A1 255
//#define PIN_IMU_INT_A2 255
#define PIN_IMU_INT_B1 255
#define PIN_IMU_INT_B2 255
#define PIN_IMU_INT_C1 255
#define PIN_IMU_INT_C2 255

// Usable with ESP32 (Wire1) only
#define IMU_1A1 IMU_MPU9250
#define IMU_1A2 IMU_MPU9250
#define IMU_1B1 IMU_MPU9250
#define IMU_1B2 IMU_MPU9250
#define IMU_1C1 IMU_MPU9250
#define IMU_1C2 IMU_MPU9250
#define IMU_ROTATION_1A1 DEG_90
#define IMU_ROTATION_1A2 DEG_90
#define IMU_ROTATION_1B1 DEG_90
#define IMU_ROTATION_1B2 DEG_90
#define IMU_ROTATION_1C1 DEG_90
#define IMU_ROTATION_1C2 DEG_90
#define PIN_IMU_SDA_1 23
#define PIN_IMU_SCL_1A 0
#define PIN_IMU_SCL_1B 4
#define PIN_IMU_SCL_1C 16
#define PIN_IMU_INT_1A1 255
#define PIN_IMU_INT_1A2 255
#define PIN_IMU_INT_1B1 255
#define PIN_IMU_INT_1B2 255
#define PIN_IMU_INT_1C1 255
#define PIN_IMU_INT_1C2 255

// Battery monitoring options (comment to disable):
//   BAT_EXTERNAL for ADC pin, 
//   BAT_INTERNAL for internal - can detect only low battery, 
//   BAT_MCP3021 for external ADC connected over I2C
#define BATTERY_MONITOR BAT_EXTERNAL

// BAT_EXTERNAL definition
// D1 Mini boards with ESP8266 have internal resistors. For these boards you only have to adjust BATTERY_SHIELD_RESISTANCE.
// For other boards you can now adjust the other resistor values.
// The diagram looks like this:
//   (Battery)--- [BATTERY_SHIELD_RESISTANCE] ---(INPUT_BOARD)---  [BATTERY_SHIELD_R2] ---(ESP32_INPUT)--- [BATTERY_SHIELD_R1] --- (GND)
#define BATTERY_SHIELD_RESISTANCE 0 //130k BatteryShield, 180k SlimeVR or fill in external resistor value in kOhm
#define BATTERY_SHIELD_R1 100 // Board voltage divider resistor Ain to GND in kOhm
#define BATTERY_SHIELD_R2 33 // Board voltage divider resistor Ain to INPUT_BOARD in kOhm

// LED configuration:
// Configuration Priority 1 = Highest:
// 1. LED_PIN
// 2. LED_BUILTIN
//
//   LED_PIN
//     - Number or Symbol (D1,..) of the Output
//     - To turn off the LED, set LED_PIN to LED_OFF
//   LED_INVERTED 
//     - false for output 3.3V on high
//     - true for pull down to GND on high

// Board-specific configurations
#if BOARD == BOARD_SLIMEVR
  #define PIN_IMU_SDA 14
  #define PIN_IMU_SCL_A 12
  #define PIN_IMU_INT_A1 16
  #define PIN_IMU_INT_A2 13
  #define PIN_BATTERY_LEVEL 17
  #define LED_PIN 2
  #define LED_INVERTED true
#elif BOARD == BOARD_SLIMEVR_LEGACY || BOARD == BOARD_SLIMEVR_DEV
  #define PIN_IMU_SDA 4
  #define PIN_IMU_SCL_A 5
  #define PIN_IMU_INT_A1 10
  #define PIN_IMU_INT_A2 13
  #define PIN_BATTERY_LEVEL 17
  #define LED_PIN 2
  #define LED_INVERTED true
#elif BOARD == BOARD_NODEMCU || BOARD == BOARD_WEMOSD1MINI
  #define PIN_IMU_SDA D2
  #define PIN_IMU_SCL_A D1
  #define PIN_IMU_INT_A1 D5
  #define PIN_IMU_INT_A2 D6
  #define PIN_BATTERY_LEVEL A0
//  #define LED_PIN 2
//  #define LED_INVERTED true
#elif BOARD == BOARD_ESP01
  #define PIN_IMU_SDA 2
  #define PIN_IMU_SCL_A 0
  #define PIN_IMU_INT_A1 255
  #define PIN_IMU_INT_A2 255
  #define PIN_BATTERY_LEVEL 255
  #define LED_PIN LED_OFF
  #define LED_INVERTED false
#elif BOARD == BOARD_TTGO_TBASE
  #define PIN_IMU_SDA 5
  #define PIN_IMU_SCL_A 4
  #define PIN_IMU_INT_A1 14
  #define PIN_IMU_INT_A2 13
  #define PIN_BATTERY_LEVEL A0
//  #define LED_PIN 2
//  #define LED_INVERTED false
#elif BOARD == BOARD_CUSTOM
  #define PIN_IMU_SDA 26
  #define PIN_IMU_SCL_A 25
  #define PIN_IMU_INT_A1 18
  #define PIN_IMU_INT_A2 17
  #define PIN_BATTERY_LEVEL 36
  #define LED_PIN 22
  #define LED_INVERTED false
#elif BOARD == BOARD_WROOM32
  #define PIN_IMU_SDA 0
  #define PIN_IMU_SCL_A 2
  #define PIN_IMU_INT_A1 23
  #define PIN_IMU_INT_A2 25
  #define PIN_BATTERY_LEVEL 32
  #define LED_PIN 22
  #define LED_INVERTED false
#endif
