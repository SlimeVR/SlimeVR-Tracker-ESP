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
bool secondImuActive = false;
BatteryMonitor battery;

void setup()
{
    Serial.begin(serialBaudRate);
    Serial.println();
    Serial.println();
    Serial.println();
    
    pinMode(33, OUTPUT);
    digitalWrite(33, LOW);
    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);
    // add pin power for ext imus too
    
    logger.info("SlimeVR v" FIRMWARE_VERSION " starting up...");

    //wifi_set_sleep_type(NONE_SLEEP_T);

    statusManager.setStatus(SlimeVR::Status::LOADING, true);

    ledManager.setup();
    configuration.setup();

    SerialCommands::setUp();

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
//#ifdef ESP32
//    for (uint8_t i=0; i<12; i+=2) {
//#else
//    for (uint8_t i=0; i<6; i==i+2) {
//#endif
//        Network::update(sensorManager.get(i), sensorManager.get(i+1));
//    }
    sensorManager.update();
    Network::update(sensorManager.get());
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
}
