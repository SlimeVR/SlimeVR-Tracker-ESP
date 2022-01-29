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

#if BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
ADC_MODE(ADC_VCC);
#endif

void BatteryMonitor::Setup()
{
#if BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
    for (uint8_t i = 0x48; i < 0x4F; i++)
    {
        if (I2CSCAN::isI2CExist(i))
        {
            address = i;
            break;
        }
    }
    if (address == 0)
    {
        Serial.println(F("[ERR] MCP3021 not found on I2C bus"));
    }
#endif
}

void BatteryMonitor::Loop()
{
    #if BATTERY_MONITOR == BAT_EXTERNAL || BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
        auto now_ms = millis();
        if (now_ms - last_battery_sample >= batterySampleRate)
        {
            voltage = -1;
            #if BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
                last_battery_sample = now_ms;
                auto level = ESP.getVcc();
                if (level > voltage_3_3)
                {
                    voltage_3_3 = level;
                }
                else
                {
                    //Calculate drop in mV
                    level = voltage_3_3 - level;
                    if (level < 50)
                    {
                        voltage = 5.0F;
                    }
                    else
                    {
                        voltage = 3.3F - ((float)level / 1000.0F) + 0.1F; //we assume 100mV drop on the linear converter
                    }
                }
            #endif
            #if BATTERY_MONITOR == BAT_EXTERNAL
                last_battery_sample = now_ms;
                voltage = ((float)analogRead(PIN_BATTERY_LEVEL)) * batteryADCMultiplier;
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
                        v *= batteryADCMultiplier;
                        voltage = (voltage > 0) ? min(voltage, v) : v;
                    }
                }
            #endif
            if (voltage > 0) //valid measurement
            {
                // Estimate battery level, 3.2V is 0%, 4.17V is 100% (1.0)
                if (voltage > 3.975)
                    level = (voltage - 2.920) * 0.8;
                else if (voltage > 3.678)
                    level = (voltage - 3.300) * 1.25;
                else if (voltage > 3.489)
                    level = (voltage - 3.400) * 1.7;
                else if (voltage > 3.360)
                    level = (voltage - 3.300) * 0.8;
                else
                    level = (voltage - 3.200) * 0.3;

                level = (level - 0.05) / 0.95; // Cut off the last 5% (3.36V)

                if (level > 1)
                    level = 1;
                else if (level < 0)
                    level = 0;
                Network::sendBatteryLevel(voltage, level);
                #ifdef BATTERY_LOW_POWER_VOLTAGE
                    if (voltage < (float)BATTERY_LOW_POWER_VOLTAGE)
                    {
                        #if defined(BATTERY_LOW_VOLTAGE_DEEP_SLEEP) && BATTERY_LOW_VOLTAGE_DEEP_SLEEP
                            ESP.deepSleep(0);
                        #else
                            LEDManager::setLedStatus(LED_STATUS_LOW_BATTERY);
                        #endif
                    } else {
                        LEDManager::unsetLedStatus(LED_STATUS_LOW_BATTERY);
                    }
                #endif
            }
        }
    #endif
}