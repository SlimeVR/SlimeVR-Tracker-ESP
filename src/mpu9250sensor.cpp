/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain, S.J. Remington

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

#include "MPU9250.h"
#include "sensor.h"
#include "udpclient.h"
#include "defines.h"
//#include <i2cscan.h>
#include "calibration.h"


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

void MPU9250Sensor::motionSetup() {
#if serialDebug
    imu.verbose(true);
#endif
    uint8_t addr = 0x68;
    if (!imu.setup(addr)) {
        addr = 0x69;
        if (!imu.setup(addr)) {
            Serial.println("[ERR] Can't find I2C device on addr 0x68 or 0x69, returning");
            signalAssert();
            return;
        }
    }

    Serial.print("[OK] Connected to MPU, ID 0x");
    Serial.println(addr, HEX);

    imu.selectFilter(QuatFilterSel::MADGWICK);
    Serial.println("[NOTICE] Load Calibration Data");
    eeprom.setupEEPROM(&imu);
    // Serial.println("Calibration Start!");
    // imu.verbose(true);
    // imu.calibrateAccelGyro();
    // imu.calibrateMag();
    // imu.verbose(true);
    // Serial.println("Calibration Finished");
    // eeprom.saveCalibration();
    Serial.println("[NOTICE] Finished Load Calibration Data");
}

void MPU9250Sensor::motionLoop() {
    // Update quaternion
    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;
    getMPUScaled();
    //MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[1], Mxyz[0], -Mxyz[2], deltat);
    quaternion.set(-imu.getQuaternionX(), -imu.getQuaternionY(), -imu.getQuaternionW(), imu.getQuaternionZ());
    quaternion *= sensorOffset;
    if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
    }
}

void MPU9250Sensor::sendData() {
<<<<<<< HEAD
    sendQuat(&quaternion, PACKET_ROTATION);
    sendVector(Gxyz, PACKET_GYRO);
    sendVector(Axyz, PACKET_ACCEL);
    sendVector(Mxyz, PACKET_MAG);
    //sendVector(rawMag, PACKET_RAW_MAGENTOMETER);
=======
    if(newData) {
        sendQuat(&quaternion, PACKET_ROTATION);
    }
>>>>>>> main
}

void MPU9250Sensor::getMPUScaled()
{
    if (imu.update()) {
        for (int i = 0; i < 3; i++) {
            Axyz[i] = imu.getAcc(i);
            Gxyz[i] = imu.getGyro(i);
            Mxyz[i] = imu.getMag(i);
            rawMag[i] = Mxyz[i];
        }
    }
    
    //Serial.printf("Gxyz: %f, %f, %f\nAxyz: %f, %f, %f\nMxyz: %f, %f, %f\n", Gxyz[0], Gxyz[1], Gxyz[2], Axyz[0], Axyz[1], Axyz[2], Mxyz[0], Mxyz[1], Mxyz[2]);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// reference vectors are Up (Acc) and West (Acc cross Mag)
// sjr 12/2020
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MPU9250Sensor::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    // include imu.update() function
}

void MPU9250Sensor::startCalibration(int calibrationType) {
    digitalWrite(CALIBRATING_LED, LOW);
    Serial.println("[NOTICE] Gathering raw data for device calibration...");
    Serial.println("[NOTICE] Put down the device and wait for baseline gyro reading calibration");
    // imu.verbose(true);
    // imu.calibrateAccelGyro();
    // if (imu.isConnectedAK8963()) {
    //     imu.calibrateMag();
    // }
    // imu.verbose(false);
    // eeprom.saveCalibration();
    // Wait for sensor to calm down before calibration
    for (int i = 0; i < 3; i++) {
        Gxyz[i] = imu.getGyroBias(i);
    }
    Serial.printf("[NOTICE] Gyro calibration results: %f %f %f\n", Gxyz[0], Gxyz[1], Gxyz[2]);
    sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0, PACKET_RAW_CALIBRATION_DATA);

    // Blink calibrating led before user should rotate the sensor
    Serial.println("[NOTICE] Gently rotate the device while it's gathering accelerometer and magnetometer data");    
    digitalWrite(CALIBRATING_LED, LOW);
    int calibrationDataAcc[3];
    int calibrationDataMag[3];
    for (int i = 0; i < 3; i++)
    {
        calibrationDataAcc[i] = imu.getAccBias(i);
        calibrationDataMag[i] = imu.getMagBias(i);
    }
    sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0, PACKET_RAW_CALIBRATION_DATA);
    sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0, PACKET_RAW_CALIBRATION_DATA);
    digitalWrite(CALIBRATING_LED, HIGH);

    Serial.println("[NOTICE] Calibration data gathered and sent");
    digitalWrite(CALIBRATING_LED, HIGH);
    sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0, PACKET_RAW_CALIBRATION_DATA);
}
