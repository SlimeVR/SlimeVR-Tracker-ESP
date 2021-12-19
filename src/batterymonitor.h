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
#include "defines.h"
#include "udpclient.h"
#include <i2cscan.h>
#include <i2cdev.h>
#include "ledstatus.h"

#if BATTERY_MONITOR == BAT_EXTERNAL
    #ifndef PIN_BATTERY_LEVEL
        #error Internal ADC enabled without pin! Please select a pin.
    #endif
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
};

#endif // SLIMEVR_BATTERYMONITOR_H_
