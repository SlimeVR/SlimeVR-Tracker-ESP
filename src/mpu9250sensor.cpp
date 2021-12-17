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
#include "helper_3dmath.h"
#include <i2cscan.h>
#include "calibration.h"
#include "mahony.h"

#define gscale (250. / 32768.0) * (PI / 180.0) //gyro default 250 LSB per d/s -> rad/s

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
    DeviceConfig * const config = getConfigPtr();
    calibration = &config->calibration;
    uint8_t addr = 0x68;
    if(!I2CSCAN::isI2CExist(addr)) {
        addr = 0x69;
        if(!I2CSCAN::isI2CExist(addr)) {
            Serial.println("[ERR] Can't find I2C device on addr 0x68 or 0x69, returning");
            signalAssert();
            return;
        }
    }
    // initialize device
    imu.initialize(addr);
    if(!imu.testConnection()) {
        Serial.print("[ERR] Can't communicate with MPU, response 0x");
        Serial.println(imu.getDeviceID(), HEX);
    } else {
        Serial.print("[OK] Connected to MPU, ID 0x");
        Serial.println(imu.getDeviceID(), HEX);
    }
}

void MPU9250Sensor::motionLoop() {
    // Update quaternion
    now = micros();
    deltat = now - last; //seconds since last update
    if ((deltat * 1.0e-3) >= samplingRateInMillis) {
        last = now;
        getMPUScaled();
        // Orientations of axes are set in accordance with the datasheet
        // See Section 9.1 Orientation of Axes
        // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
        mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[1], Mxyz[0], -Mxyz[2], deltat * 1.0e-6);
        quaternion.set(-q[1], -q[2], -q[0], q[3]);
        quaternion *= sensorOffset;
        if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
            newData = true;
            lastQuatSent = quaternion;
        }
    }
}

void MPU9250Sensor::sendData() {
    if(newData) {
        sendQuat(&quaternion, PACKET_ROTATION);
        newData = false;
    }
}

void MPU9250Sensor::getMPUScaled()
{
    float temp[3];
    int i;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = ((float)gx - calibration->G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - calibration->G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - calibration->G_off[2]) * gscale;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;
    //apply offsets (bias) and scale factors from Magneto
    if(useFullCalibrationMatrix) {
        for (i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - calibration->A_B[i]);
        Axyz[0] = calibration->A_Ainv[0][0] * temp[0] + calibration->A_Ainv[0][1] * temp[1] + calibration->A_Ainv[0][2] * temp[2];
        Axyz[1] = calibration->A_Ainv[1][0] * temp[0] + calibration->A_Ainv[1][1] * temp[1] + calibration->A_Ainv[1][2] * temp[2];
        Axyz[2] = calibration->A_Ainv[2][0] * temp[0] + calibration->A_Ainv[2][1] * temp[1] + calibration->A_Ainv[2][2] * temp[2];
    } else {
        for (i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - calibration->A_B[i]);
    }
    vector_normalize(Axyz);

    Mxyz[0] = (float)mx;
    Mxyz[1] = (float)my;
    Mxyz[2] = (float)mz;
    //apply offsets and scale factors from Magneto
    if(useFullCalibrationMatrix) {
        for (i = 0; i < 3; i++)
            temp[i] = (Mxyz[i] - calibration->M_B[i]);
        Mxyz[0] = calibration->M_Ainv[0][0] * temp[0] + calibration->M_Ainv[0][1] * temp[1] + calibration->M_Ainv[0][2] * temp[2];
        Mxyz[1] = calibration->M_Ainv[1][0] * temp[0] + calibration->M_Ainv[1][1] * temp[1] + calibration->M_Ainv[1][2] * temp[2];
        Mxyz[2] = calibration->M_Ainv[2][0] * temp[0] + calibration->M_Ainv[2][1] * temp[1] + calibration->M_Ainv[2][2] * temp[2];
    } else {
        for (i = 0; i < 3; i++)
            Mxyz[i] = (Mxyz[i] - calibration->M_B[i]);
    }
    rawMag[0] = Mxyz[0];
    rawMag[1] = Mxyz[1];
    rawMag[2] = Mxyz[2];
    vector_normalize(Mxyz);
}

void MPU9250Sensor::startCalibration(int calibrationType) {
    digitalWrite(CALIBRATING_LED, LOW);
    Serial.println("[NOTICE] Gathering raw data for device calibration...");
    int calibrationSamples = 300;
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration
    Serial.println("[NOTICE] Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;
    Serial.printf("[NOTICE] Gyro calibration results: %f %f %f\n", Gxyz[0], Gxyz[1], Gxyz[2]);
    sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0, PACKET_RAW_CALIBRATION_DATA);

    // Blink calibrating led before user should rotate the sensor
    Serial.println("[NOTICE] Gently rotate the device while it's gathering accelerometer and magnetometer data");
    for (int i = 0; i < 3000 / 310; ++i)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        delay(15);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(300);
    }
    int calibrationDataAcc[3];
    int calibrationDataMag[3];
    for (int i = 0; i < calibrationSamples; i++)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        calibrationDataAcc[0] = ax;
        calibrationDataAcc[1] = ay;
        calibrationDataAcc[2] = az;
        calibrationDataMag[0] = mx;
        calibrationDataMag[1] = my;
        calibrationDataMag[2] = mz;
        sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0, PACKET_RAW_CALIBRATION_DATA);
        sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0, PACKET_RAW_CALIBRATION_DATA);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(250);
    }
    Serial.println("[NOTICE] Calibration data gathered and sent");
    digitalWrite(CALIBRATING_LED, HIGH);
    sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0, PACKET_RAW_CALIBRATION_DATA);
}
