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
#include "sensor.h"
#include <i2cscan.h>
#include <I2Cdev.h>
#include "udpclient.h"
#include "defines.h"

namespace {
    void signalAssert() {
        for(int i = 0; i < 200; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }
    }
}

void BNO055Sensor::motionSetup() {
    delay(3000);
    if (!imu.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        signalAssert();
        return;
    }

    delay(1000);
    imu.setExtCrystalUse(false);
    imu.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P0);
    imu.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P0);
    Serial.println("Connected to BNO055");
}

void BNO055Sensor::motionLoop() {
    // TODO Optimize a bit with setting rawQuat directly
    Quat quat = imu.getQuat();
    quaternion.set(quat.x, quat.y, quat.z, quat.w);
    quaternion *= sensorOffset;
    newData = true;
}

void BNO055Sensor::sendData() {
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
            Serial.print(quaternion.w);
        #endif
    }
}

void BNO055Sensor::startCalibration(int calibrationType) {

}