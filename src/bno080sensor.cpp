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

void BNO080Sensor::motionSetup(DeviceConfig * config)
{
    delay(500);
    if(!imu.begin(BNO080_DEFAULT_ADDRESS, Wire)) {
        Serial.println("Can't connect to BNO08X");
        for(int i = 0; i < 500; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }
    }
    Serial.println("Connected to BNO08X");
    Wire.setClock(400000);
    if(BNO_HASARVR_STABILIZATION)
        imu.enableARVRStabilizedGameRotationVector(10);
    else
        imu.enableGameRotationVector(10);
}

void BNO080Sensor::motionLoop()
{
    //Look for reports from the IMU
    if (imu.dataAvailable() == true)
    {
        quaternion.x = imu.getQuatI();
        quaternion.y = imu.getQuatJ();
        quaternion.z = imu.getQuatK();
        quaternion.w = imu.getQuatReal();
        quaternion *= sensorOffset;
        newData = true;
    }
}

void BNO080Sensor::sendData() {
    if(newData) {
        newData = false;
        sendQuat(&quaternion, PACKET_ROTATION);
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