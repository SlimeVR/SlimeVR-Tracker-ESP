#include "batterymonitor.h"

#if BATTERY_MONITOR_INTERNAL
ADC_MODE(ADC_VCC);
#endif

void BatteryMonitor::Setup()
{
#if BATTERY_MONITOR_MCP3021
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
        Serial.print(F("[ERR] MCP3021 not found on I2C bus"));
    }
#endif
}

void BatteryMonitor::Loop()
{
#if defined(BATTERY_MONITOR_EXTERNAL) || BATTERY_MONITOR_INTERNAL || BATTERY_MONITOR_MCP3021
    auto now_ms = millis();
    if (now_ms - last_battery_sample >= batterySampleRate)
    {
        voltage = -1;
#if BATTERY_MONITOR_INTERNAL
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
            voltage = 3.3F - ((float)level / 1000.0F) + 0.1F; //we assume 100mV drop on the linear converter
        }
#endif
#if BATTERY_MONITOR_EXTERNAL
        last_battery_sample = now_ms;
        voltage = ((float)analogRead(PIN_BATTERY_LEVEL)) * batteryADCMultiplier;
#endif
#if BATTERY_MONITOR_MCP3021
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
                voltage = (voltage > 0) ? min(voltage, v) : v;
            }
        }
#endif
        if (voltage > 0) //valid measurement
        {
            level = (125 * voltage) * 0.5 - 162.5; // Not good probably
            send2Floats(voltage, level, PACKET_BATTERY_LEVEL);
#ifdef BATTERY_LOW_POWER_VOLTAGE
            if (voltage < (float)BATTERY_LOW_POWER_VOLTAGE)
            {
                ESP.deepSleep(0);
            }
#endif
        }
    }
#endif
}