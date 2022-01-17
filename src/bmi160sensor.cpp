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

// Typical sensitivity at 25C
// See p. 9 of https://www.mouser.com/datasheet/2/783/BST-BMI160-DS000-1509569.pdf
// 65.6 LSB/deg/s = 500 deg/s
#define TYPICAL_SENSITIVITY_LSB 65.6

// LSB per temperature step.
// These values were calculated for 500 deg/s sensitivity
#define LSB_COMP_PER_TEMP_STEP_X -0.08055235903f
#define LSB_COMP_PER_TEMP_STEP_Y  0.24165707710f
#define LSB_COMP_PER_TEMP_STEP_Z  0.06904487917f

// Temperature per step from -41 + 1/2^9 degrees C (0x8001) to 87 - 1/2^9 degrees C (0x7FFF)
constexpr float TEMP_STEP = 128. / 65535;
// Middle value is 23 degrees C (0x0000)
#define TEMP_ZERO 23

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float gscale = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);

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

    if (isSecond) {
        addr = 0x69;
        if (!I2CSCAN::isI2CExist(addr)) {
            Serial.println("[ERR] Can't find I2C device on addr 0x69, returning");
            signalAssert();
            return;
        } else {
            Serial.println("[INFO] Second I2C device on addr 0x69");
        }
    }

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
    float temperature = ((imu.getTemperature() * TEMP_STEP) + TEMP_ZERO) - calibration->temperature;
    // TODO: Sensitivity over temp compensation?
    // TODO: Cross-axis sensitivity compensation?
    // TODO: Compensation over temperature change may be not linear but this should work just fine for now
    Gxyz[0] = ((float)gx - (calibration->G_off[0] + (temperature * LSB_COMP_PER_TEMP_STEP_X))) * gscale;
    Gxyz[1] = ((float)gy - (calibration->G_off[1] + (temperature * LSB_COMP_PER_TEMP_STEP_Y))) * gscale;
    Gxyz[2] = ((float)gz - (calibration->G_off[2] + (temperature * LSB_COMP_PER_TEMP_STEP_Z))) * gscale;

    //Serial.printf("{\"X\": \"%.2f\", \"Y\": \"%.2f\", \"Z\": \"%.2f\", \"T\": \"%.2f\"}\n", ((float)gx - (calibration->G_off[0] + (temperature * LSB_COMP_PER_TEMP_STEP_X))), ((float)gy - (calibration->G_off[1] + (temperature * LSB_COMP_PER_TEMP_STEP_Y))), ((float)gz - (calibration->G_off[2] + (temperature * LSB_COMP_PER_TEMP_STEP_Z))), temperature);

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
}

void BMI160Sensor::startCalibration(int calibrationType) {
    digitalWrite(CALIBRATING_LED, LOW);
    Serial.println("[NOTICE] Gathering raw data for device calibration...");
    uint16_t gyroCalibrationSamples = 3000;
    uint8_t accelCalibrationSamples = 300;
    DeviceConfig config{};
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration
    Serial.println("[NOTICE] Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    float temperature = (imu.getTemperature() * TEMP_STEP) + TEMP_ZERO;
    config.calibration[isSecond ? 1 : 0].temperature = temperature;
    // [NOTICE] Calibration temperature: 33.547035
    // [NOTICE] Gyro calibration results: -18.232334 -18.812000 6.854333
    // [NOTICE] Calibration temperature: 50.924255
    // [NOTICE] Gyro calibration results: -19.632668 -14.028666 8.021667
    // Temp diff = 17.4
    // X = -1.4 LSB/deg/s / 17.4 deg C = -0.08055235903
    // Y =  4.3 LSB/deg/s / 17.4 deg C =  0.24165707710
    // Z =  1.2 LSB/deg/s / 17.4 deg C =  0.06904487917
    Serial.printf("[NOTICE] Calibration temperature: %f\n", temperature);
    for (int i = 0; i < gyroCalibrationSamples; i++)
    {
        imu.getRotation(&gx, &gy, &gz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
    }
    Gxyz[0] /= gyroCalibrationSamples;
    Gxyz[1] /= gyroCalibrationSamples;
    Gxyz[2] /= gyroCalibrationSamples;
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

    float *calibrationDataAcc = (float*)malloc(accelCalibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < accelCalibrationSamples; i++)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        imu.getAcceleration(&ax, &ay, &az);
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
    CalculateCalibration(calibrationDataAcc, accelCalibrationSamples, A_BAinv);
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
