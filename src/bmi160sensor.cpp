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

#include "sensor.h"
#include "udpclient.h"
#include "defines.h"
#include "helper_3dmath.h"
#include <i2cscan.h>
#include "calibration.h"
#include "mahony.h"
#include "magneto1.4.h"

constexpr float gscale = (250. / 32768.0) * (PI / 180.0); //gyro default 250 LSB per d/s -> rad/s

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

void BMI160Sensor::motionSetup() {
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
        Serial.print("[ERR] Can't communicate with BMI160, response 0x");
        Serial.println(imu.getDeviceID(), HEX);
    } else {
        Serial.print("[OK] Connected to BMI160, ID 0x");
        Serial.println(imu.getDeviceID(), HEX);
    }

    DeviceConfig * const config = getConfigPtr();
    calibration = &config->calibration[isSecond ? 1 : 0];
    if (!hasConfigStored()) {
        startCalibration(0);
    }
    if (isSecond) {
        working = true;
    }
}

void BMI160Sensor::motionLoop() {
    // Update quaternion
    now = micros();
    deltat = now - last; //seconds since last update
    if ((deltat * 1.0e-3) >= samplingRateInMillis) {
        last = now;
        getScaledValues();
        mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat * 1.0e-6);
        quaternion.set(-q[1], -q[2], -q[0], q[3]);
        quaternion *= sensorOffset;
        if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
            newData = true;
            lastQuatSent = quaternion;
        }
    }
}

void BMI160Sensor::sendData() {
    if(newData) {
        sendQuat(&quaternion, isSecond ? PACKET_ROTATION_2 : PACKET_ROTATION);
        newData = false;
    }
}

void BMI160Sensor::setSecond() {
    isSecond = true;
    sensorOffset = {Quat(Vector3(0, 0, 1), SECOND_IMU_ROTATION)};
}

void BMI160Sensor::getScaledValues()
{
    float temp[3];
    int i;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Gxyz[0] = ((float)gx - calibration->G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - calibration->G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - calibration->G_off[2]) * gscale;
    //Serial.printf("Gx = %f Gy = %f Gz = %f\n", Gxyz[0], Gxyz[1], Gxyz[2]);

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;
    //apply offsets (bias) and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - calibration->A_B[i]);
        Axyz[0] = calibration->A_Ainv[0][0] * temp[0] + calibration->A_Ainv[0][1] * temp[1] + calibration->A_Ainv[0][2] * temp[2];
        Axyz[1] = calibration->A_Ainv[1][0] * temp[0] + calibration->A_Ainv[1][1] * temp[1] + calibration->A_Ainv[1][2] * temp[2];
        Axyz[2] = calibration->A_Ainv[2][0] * temp[0] + calibration->A_Ainv[2][1] * temp[1] + calibration->A_Ainv[2][2] * temp[2];
    #else
        for (i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - calibration->A_B[i]);
    #endif
    vector_normalize(Axyz);
    //Serial.printf("Ax = %f Ay = %f Az = %f\n", Axyz[0], Axyz[1], Axyz[2]);
}

void BMI160Sensor::startCalibration(int calibrationType) {
    digitalWrite(CALIBRATING_LED, LOW);
    Serial.println("[NOTICE] Gathering raw data for device calibration...");
    int calibrationSamples = 300;
    DeviceConfig config{};
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration
    Serial.println("[NOTICE] Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;
    Serial.printf("[NOTICE] Gyro calibration results: %f %f %f\n", Gxyz[0], Gxyz[1], Gxyz[2]);
    config.calibration[isSecond ? 1 : 0].G_off[0] = Gxyz[0];
    config.calibration[isSecond ? 1 : 0].G_off[1] = Gxyz[1];
    config.calibration[isSecond ? 1 : 0].G_off[2] = Gxyz[2];

    // Blink calibrating led before user should rotate the sensor
    Serial.println("[NOTICE] After 3seconds, Gently rotate the device while it's gathering accelerometer data");
    digitalWrite(CALIBRATING_LED, LOW);
    delay(1500);
    digitalWrite(CALIBRATING_LED, HIGH);
    delay(1500);
    Serial.println("[NOTICE] Gathering accelerometer data start!");

    float *calibrationDataAcc = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(100);
    }
    Serial.println("[NOTICE] Calibration data gathered");
    digitalWrite(CALIBRATING_LED, HIGH);
    Serial.println("[NOTICE] Now Calculate Calibration data");

    float A_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, calibrationSamples, A_BAinv);
    Serial.println("[NOTICE] Finished Calculate Calibration data");
    Serial.println("[NOTICE] Now Saving EEPROM");
    for (int i = 0; i < 3; i++)
    {
        config.calibration[isSecond ? 1 : 0].A_B[i] = A_BAinv[0][i];
        config.calibration[isSecond ? 1 : 0].A_Ainv[0][i] = A_BAinv[1][i];
        config.calibration[isSecond ? 1 : 0].A_Ainv[1][i] = A_BAinv[2][i];
        config.calibration[isSecond ? 1 : 0].A_Ainv[2][i] = A_BAinv[3][i];
    }

    setConfig(config);
    Serial.println("[NOTICE] Finished Saving EEPROM");
    delay(4000);
}
