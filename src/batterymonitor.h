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
#ifndef SLIMEVR_BATTERYMONITOR_H_
#define SLIMEVR_BATTERYMONITOR_H_

#include <Arduino.h>
#include "globals.h"
#include "network/network.h"
#include <i2cscan.h>
#include <I2Cdev.h>
#include "logging/Logger.h"

#if ESP8266
    #define ADCResulution 1023.0  // ESP8266 has 12bit ADC
    #define ADCVoltageMax 1.0     // ESP8266 input is 1.0 V = 1023.0
#endif
#if ESP32
    #define ADCResulution 4095.0  // ESP32 has 12bit ADC
    #define ADCVoltageMax 3.3     // ESP32 input is 3.3 V = 4095.0
#endif
#ifndef ADCResulution
    #define ADCResulution 1023.0
#endif
#ifndef ADCVoltageMax
    #define ADCVoltageMax 1.0
#endif

#ifndef BATTERY_SHIELD_R1
    #define BATTERY_SHIELD_R1 100.0
#endif
#ifndef BATTERY_SHIELD_R2
    #define BATTERY_SHIELD_R2 220.0
#endif

#if BATTERY_MONITOR == BAT_EXTERNAL
    #ifndef PIN_BATTERY_LEVEL
        #error Internal ADC enabled without pin! Please select a pin.
    #endif
    // Wemos D1 Mini has an internal Voltage Divider with R1=220K and R2=100K > this means, 3.3V analogRead input voltage results in 1023.0
    // Wemos D1 Mini with Wemos Battery Shield v1.2.0 or higher: Battery Shield with J2 closed, has an additional 130K resistor. So the resulting Voltage Divider is R1=220K+100K=320K and R2=100K > this means, 4.5V analogRead input voltage results in 1023.0
    // ESP32 Boards may have not the internal Voltage Divider. Also ESP32 has a 12bit ADC (0..4095). So R1 and R2 can be changed.
    // Diagramm: 
    //   (Battery)--- [BATTERY_SHIELD_RESISTANCE] ---(INPUT_BOARD)---  [BATTERY_SHIELD_R2] ---(ESP_INPUT)--- [BATTERY_SHIELD_R1] --- (GND)
    // SlimeVR Board can handle max 5V > so analogRead of 5.0V input will result in 1023.0
  #define batteryADCMultiplier ADCVoltageMax / ADCResulution * (BATTERY_SHIELD_R1 + BATTERY_SHIELD_R2 + BATTERY_SHIELD_RESISTANCE) / BATTERY_SHIELD_R1
#elif BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
  // Default recommended resistors are 9.1k and 5.1k
  #define batteryADCMultiplier 3.3 / 1023.0 * 14.2 / 9.1
#endif

class BatteryMonitor
{
public:
    void Setup();
    void Loop();

private:
    unsigned long last_battery_sample = 0;
#if BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
    uint8_t address = 0;
#endif
#if BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
    uint16_t voltage_3_3 = 3000;
#endif
    float voltage = -1;
    float level = -1;

    SlimeVR::Logging::Logger m_Logger = SlimeVR::Logging::Logger("BatteryMonitor");
};

#endif // SLIMEVR_BATTERYMONITOR_H_
