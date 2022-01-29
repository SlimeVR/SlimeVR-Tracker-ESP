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

#include "mpu9250sensor.h"
#include "network/network.h"
#include "globals.h"
#include "helper_3dmath.h"
#include <i2cscan.h>
#include "calibration.h"
#include "ledmgr.h"

#define gscale (250. / 32768.0) * (PI / 180.0) //gyro default 250 LSB per d/s -> rad/s
// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
#define Kp 10.0
#define Ki 0.0

CalibrationConfig * calibration;

void get_MPU_scaled();
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

void MPU9250Sensor::motionSetup() {
    DeviceConfig * const config = getConfigPtr();
    calibration = &config->calibration;
    // initialize device
    imu.initialize(addr);
    if(!imu.testConnection()) {
        Serial.print("[ERR] MPU9250: Can't communicate with MPU, response 0x");
        Serial.println(imu.getDeviceID(), HEX);
    } else {
        Serial.print("[OK] Connected to MPU, ID 0x");
        Serial.println(imu.getDeviceID(), HEX);
    }
}

void MPU9250Sensor::motionLoop() {
    // Update quaternion
    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;
    getMPUScaled();
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[1], Mxyz[0], -Mxyz[2], deltat);
    quaternion.set(-q[1], -q[2], -q[0], q[3]);
    quaternion *= sensorOffset;
    if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
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

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// reference vectors are Up (Acc) and West (Acc cross Mag)
// sjr 12/2020
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MPU9250Sensor::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    // Vector to hold integral error for Mahony method
    static float eInt[3] = {0.0, 0.0, 0.0};

    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, hz;  //observed West vector W = AxM
    float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Measured horizon vector = a x m (in body frame)
    hx = ay * mz - az * my;
    hy = az * mx - ax * mz;
    hz = ax * my - ay * mx;
    // Normalise horizon vector
    norm = sqrt(hx * hx + hy * hy + hz * hz);
    if (norm == 0.0f) return; // Handle div by zero

    norm = 1.0f / norm;
    hx *= norm;
    hy *= norm;
    hz *= norm;

    // Estimated direction of Up reference vector
    ux = 2.0f * (q2q4 - q1q3);
    uy = 2.0f * (q1q2 + q3q4);
    uz = q1q1 - q2q2 - q3q3 + q4q4;

    // estimated direction of horizon (West) reference vector
    wx = 2.0f * (q2q3 + q1q4);
    wy = q1q1 - q2q2 + q3q3 - q4q4;
    wz = 2.0f * (q3q4 - q1q2);

    // Error is cross product between estimated direction and measured direction of the reference vectors
    ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
    ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
    ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

    if (Ki > 0.0f)
    {
        eInt[0] += ex; // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
        // Apply I feedback
        gx += Ki * eInt[0];
        gy += Ki * eInt[1];
        gz += Ki * eInt[2];
    }

    // Apply P feedback
    gx = gx + Kp * ex;
    gy = gy + Kp * ey;
    gz = gz + Kp * ez;

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void MPU9250Sensor::startCalibration(int calibrationType) {
    LEDManager::on(CALIBRATING_LED);
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
    Network::sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);

    // Blink calibrating led before user should rotate the sensor
    Serial.println("[NOTICE] Gently rotate the device while it's gathering accelerometer and magnetometer data");
    LEDManager::pattern(CALIBRATING_LED, 15, 300, 3000/310);
    int calibrationDataAcc[3];
    int calibrationDataMag[3];
    for (int i = 0; i < calibrationSamples; i++)
    {
        LEDManager::on(CALIBRATING_LED);
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        calibrationDataAcc[0] = ax;
        calibrationDataAcc[1] = ay;
        calibrationDataAcc[2] = az;
        calibrationDataMag[0] = mx;
        calibrationDataMag[1] = my;
        calibrationDataMag[2] = mz;
        Network::sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
        Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        LEDManager::off(CALIBRATING_LED);
        delay(250);
    }
    Serial.println("[NOTICE] Calibration data gathered and sent");
    LEDManager::off(CALIBRATING_LED);
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
}
