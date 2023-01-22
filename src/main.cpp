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

#include "Wire.h"
#include "ota.h"
#include "sensors/SensorManager.h"
#include "configuration/Configuration.h"
#include "network/network.h"
#include "globals.h"
#include "credentials.h"
#include <i2cscan.h>
#include "serial/serialcommands.h"
#include "LEDManager.h"
#include "status/StatusManager.h"
#include "batterymonitor.h"
#include "logging/Logger.h"

SlimeVR::Logging::Logger logger("SlimeVR");
SlimeVR::Sensors::SensorManager sensorManager;
SlimeVR::LEDManager ledManager(LED_PIN);
SlimeVR::Status::StatusManager statusManager;
SlimeVR::Configuration::Configuration configuration;

int sensorToCalibrate = -1;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long loopTime = 0;
unsigned long lastStatePrint = 0;
bool secondImuActive = false;
BatteryMonitor battery;

void setup()
{
    Serial.begin(serialBaudRate);

#ifdef ESP32C3 
    // Wait for the Computer to be able to connect.
    delay(2000);
#endif

    Serial.println();
    Serial.println();
    Serial.println();

    logger.info("SlimeVR v" FIRMWARE_VERSION " starting up...");

    statusManager.setStatus(SlimeVR::Status::LOADING, true);

    ledManager.setup();
    configuration.setup();

    SerialCommands::setUp();

#if IMU == IMU_MPU6500 || IMU == IMU_MPU6050 || IMU == IMU_MPU9250 || IMU == IMU_BNO055 || IMU == IMU_ICM20948
    I2CSCAN::clearBus(PIN_IMU_SDA, PIN_IMU_SCL); // Make sure the bus isn't stuck when resetting ESP without powering it down
    // Fixes I2C issues for certain IMUs. Only has been tested on IMUs above. Testing advised when adding other IMUs.
#endif
    // join I2C bus

#if ESP32
    // For some unknown reason the I2C seem to be open on ESP32-C3 by default. Let's just close it before opening it again. (The ESP32-C3 only has 1 I2C.)
    Wire.end();
#endif

    // using `static_cast` here seems to be better, because there are 2 similar function signatures
    Wire.begin(static_cast<int>(PIN_IMU_SDA), static_cast<int>(PIN_IMU_SCL)); 

#ifdef ESP8266
    Wire.setClockStretchLimit(150000L); // Default stretch limit 150mS
#endif
    Wire.setClock(I2C_SPEED);

    // Wait for IMU to boot
    delay(500);
    
    sensorManager.setup();
    
    Network::setUp();
    OTA::otaSetup(otaPassword);
    battery.Setup();

    statusManager.setStatus(SlimeVR::Status::LOADING, false);

    sensorManager.postSetup();

    loopTime = micros();
}

void loop()
{
    SerialCommands::update();
    OTA::otaUpdate();
    Network::update(sensorManager.getFirst(), sensorManager.getSecond());
    sensorManager.update();
    battery.Loop();
    ledManager.update();
#ifdef TARGET_LOOPTIME_MICROS
    long elapsed = (micros() - loopTime);
    if (elapsed < TARGET_LOOPTIME_MICROS)
    {
        long sleepus = TARGET_LOOPTIME_MICROS - elapsed - 100;//µs to sleep
        long sleepms = sleepus / 1000;//ms to sleep
        if(sleepms > 0) // if >= 1 ms
        {
            delay(sleepms); // sleep ms = save power
            sleepus -= sleepms * 1000;
        }
        if (sleepus > 100)
        {
            delayMicroseconds(sleepus);
        }
    }
    loopTime = micros();
#endif
    #if defined(PRINT_STATE_EVERY_MS) && PRINT_STATE_EVERY_MS > 0
        unsigned long now = millis();
        if(lastStatePrint + PRINT_STATE_EVERY_MS < now) {
            lastStatePrint = now;
            SerialCommands::printState();
        }
    #endif
}
