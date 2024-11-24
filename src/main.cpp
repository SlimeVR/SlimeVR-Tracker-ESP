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
#include "GlobalVars.h"
#include "globals.h"
#include "credentials.h"
#include <i2cscan.h>
#include "serial/serialcommands.h"
#include "LEDManager.h"
#include "ButtonMonitor.h"
#include "batterymonitor.h"
#include "logging/Logger.h"

Timer<> globalTimer;
SlimeVR::Logging::Logger logger("SlimeVR");
SlimeVR::Sensors::SensorManager sensorManager;
#if ESP32 && ENABLE_LEDC
SlimeVR::LEDManager ledManager(LED_PIN, LEDC_FREQ_LED, LEDC_BITS_LED);
#else
SlimeVR::LEDManager ledManager(LED_PIN);
#endif
#ifdef PIN_BUTTON_INPUT
SlimeVR::ButtonMonitor buttonMonitor(PIN_BUTTON_INPUT);
#endif
SlimeVR::Status::StatusManager statusManager;
SlimeVR::Configuration::Configuration configuration;
SlimeVR::Network::Manager networkManager;
SlimeVR::Network::Connection networkConnection;

int sensorToCalibrate = -1;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long loopTime = 0;
unsigned long lastStatePrint = 0;
bool secondImuActive = false;
BatteryMonitor battery;

void setup()
{
    // For Somatic Eros, pull ENABLE_LATCH high first thing, so the button doesn't need to be held down any longer.
#ifdef PIN_ENABLE_LATCH
    pinMode(PIN_ENABLE_LATCH, OUTPUT);
    digitalWrite(PIN_ENABLE_LATCH, HIGH);
#endif
#ifdef PIN_IMU_ENABLE
    pinMode(PIN_IMU_ENABLE, OUTPUT);
    digitalWrite(PIN_IMU_ENABLE, LOW);
    delay(200);
    digitalWrite(PIN_IMU_ENABLE, HIGH);
#endif
#ifdef PIN_BAT_STAT_CHRG
    pinMode(PIN_BAT_STAT_CHRG, INPUT);
#endif
#ifdef PIN_BAT_STAT_CHRG_DONE
    pinMode(PIN_BAT_STAT_CHRG_DONE, INPUT);
#endif

#ifdef PIN_TACT_MOTOR
    pinMode(PIN_TACT_MOTOR, OUTPUT);
    digitalWrite(PIN_TACT_MOTOR, HIGH);
#endif

    Serial.begin(serialBaudRate);
    globalTimer = timer_create_default();

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
    ledManager.on();
    configuration.setup();

    SerialCommands::setUp();

    I2CSCAN::clearBus(PIN_IMU_SDA, PIN_IMU_SCL); // Make sure the bus isn't stuck when resetting ESP without powering it down
    // Fixes I2C issues for certain IMUs. Previously this feature was enabled for selected IMUs, now it's enabled for all.
    // If some IMU turned out to be broken by this, check needs to be re-added.

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
#ifdef ESP32 // Counterpart on ESP32 to ClockStretchLimit
    Wire.setTimeOut(150);
#endif
    Wire.setClock(I2C_SPEED);

    // Wait for IMU to boot
    delay(500);

    sensorManager.setup();

    networkManager.setup();
    OTA::otaSetup(otaPassword);
    battery.Setup();

    statusManager.setStatus(SlimeVR::Status::LOADING, false);

    sensorManager.postSetup();

    loopTime = micros();
}

void loop()
{
    globalTimer.tick();
    SerialCommands::update();
    OTA::otaUpdate();
    networkManager.update();
    sensorManager.update();
    battery.Loop();

#ifdef PIN_BUTTON_INPUT
    buttonMonitor.update();
#endif
    ledManager.update();

#ifdef PIN_ENABLE_LATCH
    if (statusManager.hasStatus(SlimeVR::Status::SHUTDOWN_INITIATED))
    {
        ledManager.pattern(150,150,3);
        statusManager.setStatus(SlimeVR::Status::SHUTDOWN_INITIATED,false);
        statusManager.setStatus(SlimeVR::Status::SHUTDOWN_COMPLETE,true);
    }
    if (statusManager.hasStatus(SlimeVR::Status::SHUTDOWN_COMPLETE) && !buttonMonitor.isPressed())
    {
#ifdef PIN_IMU_ENABLE
        digitalWrite(PIN_IMU_ENABLE, LOW);
#endif
#ifdef PIN_ENABLE_LATCH
        digitalWrite(PIN_ENABLE_LATCH, LOW);
#endif
    }
#endif

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
