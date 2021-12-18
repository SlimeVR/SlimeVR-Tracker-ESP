#ifndef SLIMEVR_BATTERYMONITOR_H_
#define SLIMEVR_BATTERYMONITOR_H_

#include <Arduino.h>
#include "defines.h"
#include "udpclient.h"
#include <i2cscan.h>
#include <i2cdev.h>

#ifdef BATTERY_MONITOR_EXTERNAL
#if BATTERY_MONITOR_INTERNAL
#error Can not enable VCCMonitor and internal ADC at the same time, disable one.
#endif
#ifndef defined(PIN_BATTERY_LEVEL)
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
#if BATTERY_MONITOR_MCP3021
    uint8_t address = 0;
#endif
#if BATTERY_MONITOR_INTERNAL
    uint16_t voltage_3_3 = 3000;
#endif
    float voltage = -1;
    float level = -1;
};

#endif // SLIMEVR_BATTERYMONITOR_H_