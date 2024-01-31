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
#include "batterymonitor.h"
#include "GlobalVars.h"

#if ESP8266 && (BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_INTERNAL_MCP3021)
ADC_MODE(ADC_VCC);
#endif

void BatteryMonitor::Setup()
{
#if BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
    for (uint8_t i = 0x48; i < 0x4F; i++)
    {
        if (I2CSCAN::hasDevOnBus(i))
        {
            address = i;
            break;
        }
    }
    if (address == 0)
    {
        m_Logger.error("MCP3021 not found on I2C bus");
    }
#endif
}

void BatteryMonitor::Loop()
{
    #if BATTERY_MONITOR == BAT_EXTERNAL || BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
        auto now_ms = millis();
        if (now_ms - last_battery_sample >= batterySampleRate)
        {
            last_battery_sample = now_ms;
            voltage = -1;
            #if ESP8266 && (BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_INTERNAL_MCP3021)
                // Find out what your max measurement is (voltage_3_3).
                // Take the max measurement and check if it was less than 50mV
                // if yes output 5.0V
                // if no output 3.3V - dropvoltage + 0.1V
                auto ESPmV = ESP.getVcc();
                if (ESPmV > voltage_3_3)
                {
                    voltage_3_3 = ESPmV;
                }
                else
                {
                    //Calculate drop in mV
                    ESPmV = voltage_3_3 - ESPmV;
                    if (ESPmV < 50)
                    {
                        voltage = 5.0F;
                    }
                    else
                    {
                        voltage = 3.3F - ((float)ESPmV / 1000.0F) + 0.1F; //we assume 100mV drop on the linear converter
                    }
                }
            #endif
            #if ESP8266 && BATTERY_MONITOR == BAT_EXTERNAL
                voltage = ((float)analogRead(PIN_BATTERY_LEVEL)) * ADCVoltageMax / ADCResolution * ADCMultiplier;
            #endif
            #if ESP32 && BATTERY_MONITOR == BAT_EXTERNAL
                voltage = ((float)analogReadMilliVolts(PIN_BATTERY_LEVEL)) / 1000 * ADCMultiplier;
            #endif
            #if BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
                if (address > 0)
                {
                    Wire.beginTransmission(address);
                    Wire.requestFrom(address, (uint8_t)2);
                    auto MSB = Wire.read();
                    auto LSB = Wire.read();
                    auto status = Wire.endTransmission();
                    if (status == 0)
                    {
                        float v = (((uint16_t)(MSB & 0x0F) << 6) | (uint16_t)(LSB >> 2));
                        v *= ADCMultiplier;
                        voltage = (voltage > 0) ? min(voltage, v) : v;
                    }
                }
            #endif
            if (voltage > 0) //valid measurement
            {
#if BATTERY_REGULATOR == REG_BUCK
                // Estimate battery level, 3.2V is 0%, 3.87V is 50%, 4.15V is 100% (1.0)
                // Mapped from battery discharge with buck regulating to 2.8V
                if (voltage > 4.075)
                    level = map(voltage, 4.075, 4.15, 0.95, 1);
                else if (voltage > 3.775)
                    level = map(voltage, 3.775, 4.075, 0.3, 0.95);
                else if (voltage > 3.45)
                    level = map(voltage, 3.45, 3.775, 0.05, 0.3);
                else
                    level = map(voltage, 3.2, 3.45, 0, 0.05);
#elif BATTERY_REGULATOR == REG_LDO
                // Estimate battery level, 3.2V is 0%, 3.77V is 50%, 4.15V is 100% (1.0)
                // Mapped from battery discharge with ldo
                if (voltage > 4.025)
                    level = map(voltage, 4.025, 4.15, 0.95, 1);
                else if (voltage > 3.65)
                    level = map(voltage, 3.65, 4.025, 0.3, 0.95);
                else if (voltage > 3.4)
                    level = map(voltage, 3.4, 3.65, 0.05, 0.3);
                else
                    level = map(voltage, 3.2, 3.4, 0, 0.05);
#elif BATTERY_REGULATOR == REG_LEGACY
                // Estimate battery level, 3.2V is 0%, 3.7V is 50%, 4.17V is 100% (1.0)
                // Mapped from unknown data?
                if (voltage > 3.975)
                    level = map(voltage, 3.975, 4.17, 0.84375, 1);
                else if (voltage > 3.677)
                    level = map(voltage, 3.677, 3.975, 0.4709, 0.84375);
                else if (voltage > 3.489)
                    level = map(voltage, 3.489, 3.677, 0.1512, 0.4709);
                else if (voltage > 3.36)
                    level = map(voltage, 3.36, 3.489, 0.048, 0.1512);
                else
                    level = map(voltage, 3.2, 3.36, 0, 0.048);
                level = (level - 0.05f) / 0.95f; // Cut off the last 5% (3.36V)
#endif

                if (level > 1)
                    level = 1;
                else if (level < 0)
                    level = 0;
                networkConnection.sendBatteryLevel(voltage, level);
                #ifdef BATTERY_LOW_POWER_VOLTAGE
                    if (voltage < BATTERY_LOW_POWER_VOLTAGE)
                    {
                        #if defined(BATTERY_LOW_VOLTAGE_DEEP_SLEEP) && BATTERY_LOW_VOLTAGE_DEEP_SLEEP
                            ESP.deepSleep(0);
                        #else
                            statusManager.setStatus(SlimeVR::Status::LOW_BATTERY, true);
                        #endif
                    } else {
                        statusManager.setStatus(SlimeVR::Status::LOW_BATTERY, false);
                    }
                #endif
            }
        }
    #endif
}
