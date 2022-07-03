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
#define IMU IMU_BNO085
#define SECOND_IMU IMU
#define BOARD BOARD_SLIMEVR
#define IMU_ROTATION DEG_90
#define SECOND_IMU_ROTATION DEG_270

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
#define BATTERY_SHIELD_RESISTANCE 180 //130k BatteryShield, 180k SlimeVR or fill in external resistor value in kOhm
// #define BATTERY_SHIELD_R1 100 // Board voltage divider resistor Ain to GND in kOhm
// #define BATTERY_SHIELD_R2 220 // Board voltage divider resistor Ain to INPUT_BOARD in kOhm

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
  #define PIN_IMU_SCL 12
  #define PIN_IMU_INT 16
  #define PIN_IMU_INT_2 13
  #define PIN_BATTERY_LEVEL 17
  #define LED_PIN 2
  #define LED_INVERTED true
#elif BOARD == BOARD_SLIMEVR_LEGACY || BOARD == BOARD_SLIMEVR_DEV
  #define PIN_IMU_SDA 4
  #define PIN_IMU_SCL 5
  #define PIN_IMU_INT 10
  #define PIN_IMU_INT_2 13
  #define PIN_BATTERY_LEVEL 17
  #define LED_PIN 2
  #define LED_INVERTED true
#elif BOARD == BOARD_NODEMCU || BOARD == BOARD_WEMOSD1MINI
  #define PIN_IMU_SDA D2
  #define PIN_IMU_SCL D1
  #define PIN_IMU_INT D5
  #define PIN_IMU_INT_2 D6
  #define PIN_BATTERY_LEVEL A0
//  #define LED_PIN 2
//  #define LED_INVERTED true
#elif BOARD == BOARD_ESP01
  #define PIN_IMU_SDA 2
  #define PIN_IMU_SCL 0
  #define PIN_IMU_INT 255
  #define PIN_IMU_INT_2 255
  #define PIN_BATTERY_LEVEL 255
  #define LED_PIN LED_OFF
  #define LED_INVERTED false
#elif BOARD == BOARD_TTGO_TBASE
  #define PIN_IMU_SDA 5
  #define PIN_IMU_SCL 4
  #define PIN_IMU_INT 14
  #define PIN_IMU_INT_2 13
  #define PIN_BATTERY_LEVEL A0
//  #define LED_PIN 2
//  #define LED_INVERTED false
#elif BOARD == BOARD_CUSTOM
  // Define pins by the examples above
#elif BOARD == BOARD_WROOM32
  #define PIN_IMU_SDA 21
  #define PIN_IMU_SCL 22
  #define PIN_IMU_INT 23
  #define PIN_IMU_INT_2 25
  #define PIN_BATTERY_LEVEL 36
//  #define LED_PIN 2
//  #define LED_INVERTED false
#elif BOARD == BOARD_LOLIN_C3_MINI
  #define PIN_IMU_SDA 10
  #define PIN_IMU_SCL 8
  #define PIN_IMU_INT 6
  #define PIN_IMU_INT_2 7
  #define PIN_BATTERY_LEVEL 3
//  #define LED_PIN 2
//  #define LED_INVERTED false
#endif
