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

#include "BNO080.h"
#include "sensor.h"
#include "udpclient.h"
#include "defines.h"
#include <i2cscan.h>

namespace {
    void signalAssert() {
        for(int i = 0; i < 200; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }
    }
    
    void sendResetReason(uint8_t reason) {
        sendByte(reason, PACKET_RESET_REASON);
    }
}

unsigned long lastData = 0;
int8_t lastReset = 0;

void BNO080Sensor::motionSetup(DeviceConfig * config)
{
    delay(500);
    uint8_t addr = 0x4A;
    if(!I2CSCAN::isI2CExist(addr)) {
        addr = 0x4B;
        if(!I2CSCAN::isI2CExist(addr)) {
            Serial.println("Can't find I2C device on addr 0x4A or 0x4B, scanning for all I2C devices and returning");
            I2CSCAN::scani2cports();
            signalAssert();
            return;
        }
    }
#ifdef FULL_DEBUG
        imu.enableDebugging(Serial);
#endif
    if(!imu.begin(addr, Wire, PIN_IMU_INT)) {
        Serial.print("Can't connect to ");
        Serial.println(IMU_NAME);
        signalAssert();
        return;
    }
    Serial.print("Connected to ");
    Serial.println(IMU_NAME);
#if defined(BNO_HAS_ARVR_STABILIZATION) && BNO_HAS_ARVR_STABILIZATION
        imu.enableARVRStabilizedGameRotationVector(13);
#else
        imu.enableGameRotationVector(13);
#endif
    lastReset = imu.resetReason();
    lastData = millis();
}

void BNO080Sensor::motionLoop()
{
    //Look for reports from the IMU
    if(imu.dataAvailable())
    {
        lastReset = -1;
        lastData = millis();
        quaternion.x = imu.getQuatI();
        quaternion.y = imu.getQuatJ();
        quaternion.z = imu.getQuatK();
        quaternion.w = imu.getQuatReal();
        quaternion *= sensorOffset;
        newData = true;
    }
    if(lastData + 1000 < millis()) {
        lastData = millis();
        uint8_t rr = imu.resetReason();
        if(rr != lastReset) {
            lastReset = rr;
            sendResetReason(rr);
            digitalWrite(LOADING_LED, LOW);
        }
    }
}

void BNO080Sensor::sendData() {
    if(newData) {
        newData = false;
        sendQuat(&quaternion, PACKET_ROTATION);
#ifdef FULL_DEBUG
            Serial.print("Quaternion: ");
            Serial.print(quaternion.x);
            Serial.print(",");
            Serial.print(quaternion.y);
            Serial.print(",");
            Serial.print(quaternion.z);
            Serial.print(",");
            Serial.println(quaternion.w);
#endif
    }
}

void BNO080Sensor::startCalibration(int calibrationType) {
    // TODO It only calibrates gyro, it should have multiple calibration modes, and check calibration status in motionLoop()
    for(int i = 0; i < 10; ++i) {
        digitalWrite(CALIBRATING_LED, LOW);
        delay(20);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(20);
    }
    digitalWrite(CALIBRATING_LED, LOW);
    delay(2000);
    digitalWrite(CALIBRATING_LED, HIGH);
    imu.calibrateGyro();
    do {
        digitalWrite(CALIBRATING_LED, LOW);
        imu.requestCalibrationStatus();
        delay(20);
        imu.getReadings();
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(20);
    } while(!imu.calibrationComplete());
    imu.saveCalibration();
}