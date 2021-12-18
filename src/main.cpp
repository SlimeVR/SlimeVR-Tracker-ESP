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

#include "Wire.h"
#include "ota.h"
#include "sensorfactory.h"
#include "configuration.h"
#include "wifihandler.h"
#include "udpclient.h"
#include "defines.h"
#include "credentials.h"
#include <i2cscan.h>
#include "serialcommands.h"
#include "ledstatus.h"
#include "ledmgr.h"
#include "batterymonitor.h"

SensorFactory sensors {};
bool isCalibrating = false;
bool blinking = false;
unsigned long blinkStart = 0;
bool secondImuActive = false;
BatteryMonitor battery;

void commandReceived(int command, void * const commandData, int commandDataLength)
{
    switch (command)
    {
    case COMMAND_CALLIBRATE:
        isCalibrating = true;
        break;
    case COMMAND_SEND_CONFIG:
        sendConfig(getConfigPtr(), PACKET_CONFIG);
        break;
    case COMMAND_BLINK:
        blinking = true;
        blinkStart = millis();
        break;
    }
}

void setup()
{
    //wifi_set_sleep_type(NONE_SLEEP_T);
    // Glow diode while loading
#if ENABLE_LEDS
    pinMode(LOADING_LED, OUTPUT);
    pinMode(CALIBRATING_LED, OUTPUT);
    digitalWrite(CALIBRATING_LED, HIGH);
    digitalWrite(LOADING_LED, LOW);
#endif

    Serial.begin(serialBaudRate);
    setUpSerialCommands();
#if IMU == IMU_MPU6500 || IMU == IMU_MPU6050 || IMU == IMU_MPU9250
    I2CSCAN::clearBus(PIN_IMU_SDA, PIN_IMU_SCL); // Make sure the bus isn't suck when reseting ESP without powering it down
    // Do it only for MPU, cause reaction of BNO to this is not investigated yet
#endif
    // join I2C bus
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
#ifdef ESP8266
    Wire.setClockStretchLimit(150000L); // Default stretch limit 150mS
#endif
    Wire.setClock(I2C_SPEED);

    getConfigPtr();
    setConfigReceivedCallback(setConfig);
    setCommandReceivedCallback(commandReceived);
    // Wait for IMU to boot
    delay(500);
    
    sensors.create();
    sensors.motionSetup();

    setUpWiFi();
    otaSetup(otaPassword);
    battery.Setup();
    LEDMGR::Off(LOADING_LED);
}

// AHRS loop

void loop()
{
    ledStatusUpdate();
    serialCommandsUpdate();
    wifiUpkeep();
    otaUpdate();
    clientUpdate(sensors.getFirst(), sensors.getSecond());
    if (isCalibrating)
    {
        sensors.startCalibration(0);
        isCalibrating = false;
    }
#ifndef UPDATE_IMU_UNCONNECTED
        if(isConnected()) {
#endif
        sensors.motionLoop();
#ifndef UPDATE_IMU_UNCONNECTED
        }
#endif
    // Send updates
#ifndef SEND_UPDATES_UNCONNECTED
    if(isConnected()) {
#endif
        sensors.sendData();
#ifndef SEND_UPDATES_UNCONNECTED
    }
#endif
    battery.Loop();
}
