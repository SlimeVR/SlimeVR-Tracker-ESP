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
#include "sensor.h"
#include "configuration.h"
#include "udpclient.h"
#include "defines.h"
#include "credentials.h"

#if IMU == IMU_BNO080 || IMU == IMU_BNO085
    BNO080Sensor sensor{};
#elif IMU == IMU_BNO055
    BNO055Sensor sensor{};
#elif IMU == IMU_MPU9250
    MPU9250Sensor sensor{};
#elif IMU == IMU_MPU6500
    MPU6050Sensor sensor{};
#else
    #error Unsupported IMU
#endif
DeviceConfig config{};

bool isCalibrating = false;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long last_battery_sample = 0;

void setConfig(DeviceConfig newConfig)
{
    config = newConfig;
    saveConfig(&config);
}

void commandRecieved(int command, void * const commandData, int commandDataLength)
{
    switch (command)
    {
    case COMMAND_CALLIBRATE:
        isCalibrating = true;
        break;
    case COMMAND_SEND_CONFIG:
        sendConfig(&config, PACKET_CONFIG);
        break;
    case COMMAND_BLINK:
        blinking = true;
        blinkStart = now_ms;
        break;
    }
}

void processBlinking();

void setup()
{
    //wifi_set_sleep_type(NONE_SLEEP_T);
    // Glow diode while loading
    pinMode(LOADING_LED, OUTPUT);
    pinMode(CALIBRATING_LED, OUTPUT);
    digitalWrite(CALIBRATING_LED, HIGH);
    digitalWrite(LOADING_LED, LOW);
    
    // join I2C bus
    Wire.flush();
    Wire.begin(D2, D1);
    Wire.setClockStretchLimit(4000);
    Wire.setClock(100000);
    Serial.begin(serialBaudRate);
    while (!Serial)
        ; // wait for connection

    // Load default calibration values and set up callbacks
    config.calibration = {
        {   49.22,    1.26, -203.52},
        {{  1.01176, -0.00048,  0.00230},
        { -0.00048,  1.00366, -0.00535},
        {  0.00230, -0.00535,  0.99003}},
        {   24.95,   81.77,  -10.36},
        {{  1.44652,  0.02739, -0.02186},
        {  0.02739,  1.48749, -0.00547},
        { -0.02186, -0.00547,  1.42441}},
        {   22.97,    13.56,   -90.65}};
    if (hasConfigStored())
    {
        loadConfig(&config);
    }
    setConfigRecievedCallback(setConfig);
    setCommandRecievedCallback(commandRecieved);

    sensor.motionSetup(&config);

    // Don't start if not connected to MPU
    /*while(!accelgyro.testConnection()) {
        Serial.print("Can't communicate with MPU9250, response ");
        Serial.println(accelgyro.getDeviceID(), HEX);
        delay(500);
    }
    Serial.println("Connected to MPU9250");
    //*/

    setUpWiFi(&config);
    otaSetup(otaPassword);
    digitalWrite(LOADING_LED, HIGH);
}

// AHRS loop

void loop()
{
    wifiUpkeep();
    otaUpdate();
    clientUpdate();
    if (isCalibrating)
    {
        sensor.startCalibration(0);
        isCalibrating = false;
    }
    #ifndef UPDATE_IMU_UNCONNECTED
        if(isConnected()) {
    #endif
    sensor.motionLoop();
    #ifndef UPDATE_IMU_UNCONNECTED
        }
    #endif
    // Send updates
    now_ms = millis();
    if (now_ms - last_ms >= samplingRateInMillis)
    {
        last_ms = now_ms;
        processBlinking();

        #ifndef SEND_UPDATES_UNCONNECTED
            if(isConnected()) {
        #endif
        sensor.sendData();
        #ifndef SEND_UPDATES_UNCONNECTED
            }
        #endif
    }
    if(now_ms - last_battery_sample >= batterySampleRate) {
        last_battery_sample = now_ms;
        float battery = ((float) analogRead(A0)) * batteryADCMultiplier;
        sendFloat(battery, PACKET_BATTERY_LEVEL);
    }
}

void processBlinking() {
    if (blinking)
    {
        if (blinkStart + sensorIdTime < now_ms)
        {
            blinking = false;
            digitalWrite(LOADING_LED, HIGH);
        }
        else
        {
            int t = (now_ms - blinkStart) / sensorIdInterval;
            if(t % 2) {
                digitalWrite(LOADING_LED, LOW);
            } else {
                digitalWrite(LOADING_LED, HIGH);
            }
        }
        
    }
}