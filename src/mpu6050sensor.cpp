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

#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h" // not necessary if using MotionApps include file

#include "sensor.h"
#include "udpclient.h"
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
}

bool hasNewData = false;

void MPU6050Sensor::motionSetup(DeviceConfig *config) {
    uint8_t addr = 0x68;

    if(!I2CSCAN::isI2CExist(addr)) {
        addr = 0x69;
        if(!I2CSCAN::isI2CExist(addr)) {
            Serial.println("Can't find I2C device on addr 0x4A or 0x4B, scanning for all I2C devices and returning");
            I2CSCAN::scani2cports();
            signalAssert();
            return;
        }
    }

    imu.initialize();
    if(!imu.testConnection()) {
        Serial.printf("Can't Communicate with MPU6050, response ");
        Serial.println(imu.getDeviceID(), HEX);
    }

    devStatus = imu.dmpInitialize();

    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        imu.setDMPEnabled(true);

        // Do a quick and dirty calibration. As the imu warms up the offsets will change a bit, but this will be good-enough
        imu.CalibrateAccel(6);
        imu.CalibrateGyro(6);
        imu.PrintActiveOffsets();

        imu.setDMPEnabled(true);

        // TODO: Add interupt support
        // mpuIntStatus = imu.getIntStatus();
        
        dmpReady = true; // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));


        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void MPU6050Sensor::motionLoop() {
    if(!dmpReady)
        return;

    if(imu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        imu.dmpGetQuaternion(&rawQuat, fifoBuffer);

        q[0] = rawQuat.x;
        q[1] = rawQuat.y;
        q[2] = rawQuat.z;
        q[3] = rawQuat.w;
        quaternion.set(-q[1], q[0], q[2], q[3]);

        hasNewData = true;
    }
}

void MPU6050Sensor::sendData() {
    if(hasNewData) {
        sendQuat(&quaternion, PACKET_ROTATION);
        hasNewData = false;
    }
}

void MPU6050Sensor::startCalibration(int calibrationType) {
    digitalWrite(CALIBRATING_LED, LOW);
    Serial.println("Calibrating IMU");

    Serial.println("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);

    imu.setDMPEnabled(false);
    imu.CalibrateAccel(6);
    imu.CalibrateGyro(6);
    imu.setDMPEnabled(true);

    Serial.println("Calibrated!");
    digitalWrite(CALIBRATING_LED, HIGH);
}