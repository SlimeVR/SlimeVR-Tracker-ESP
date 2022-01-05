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
#include "magneto1.4.h"
// #include "mahony.h"
#ifndef _MAHONY_H_
#include "dmpmag.h"
#endif

constexpr float gscale = (250. / 32768.0) * (PI / 180.0); //gyro default 250 LSB per d/s -> rad/s

#define SKIP_CALC_MAG_INTERVAL 10
#define MAG_CORR_RATIO 0.2

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
    calibration = &config->calibration[isSecond?1:0];
    uint8_t addr = isSecond?0x69:0x68;
    if(!I2CSCAN::isI2CExist(addr)) {
        addr = isSecond?0x68:0x69;
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
    int16_t ax,ay,az,dumb;

    // turn on while flip back to calibrate. then, flip again after 5 seconds.    
    // TODO: Move calibration invoke after calibrate button on slimeVR server available 
    imu.getMotion6(&ax, &ay, &az, &dumb, &dumb, &dumb);
    if(az<0 && 10.0*(ax*ax+ay*ay)<az*az) {
        digitalWrite(CALIBRATING_LED, HIGH);
        Serial.println("Calling Calibration... Flip front to confirm start calibration.");
        delay(5000);
        digitalWrite(CALIBRATING_LED, LOW);
        imu.getMotion6(&ax, &ay, &az, &dumb, &dumb, &dumb);
        if(az>0 && 10.0*(ax*ax+ay*ay)<az*az) 
            startCalibration(0);
    }
    imu.getMagnetometerAdjustments(adjustments);
#ifndef _MAHONY_H_
    devStatus = imu.dmpInitialize();
    if(devStatus == 0){
        for(int i = 0; i < 5; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }

        // turn on the DMP, now that it's ready
        Serial.println(F("[NOTICE] Enabling DMP..."));
        imu.setDMPEnabled(true);

        // TODO: Add interrupt support
        // mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("[NOTICE] DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();

        if (isSecond) {
            working = true;
        }
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("[ERR] DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
#endif
}


void MPU9250Sensor::motionLoop() {
    unsigned long now = micros();
    unsigned long deltat = now - last; //seconds since last update
    last = now;
    if(deltat<samplingRateInMillis*1000) {
        delayMicroseconds(samplingRateInMillis*1000-deltat);
        deltat = samplingRateInMillis*1000;
    }
#ifndef _MAHONY_H_
    // Update quaternion
    if(!dmpReady)
        return;
    Quaternion rawQuat{};
    if(!imu.GetCurrentFIFOPacket(fifoBuffer,imu.dmpPacketSize)) return;
    imu.dmpGetQuaternion(&rawQuat, fifoBuffer);
    Quat quat(-rawQuat.y,rawQuat.x,rawQuat.z,rawQuat.w);
    if(!skipCalcMag){
        getMPUScaled();
        if(Mxyz[0]==0.0f && Mxyz[1]==0.0f && Mxyz[2]==0.0f) return;
        skipCalcMag=SKIP_CALC_MAG_INTERVAL;
        if(correction.length_squared()==0.0f) {
            correction=getCorrection(Axyz,Mxyz,quat);
            if(isSecond) skipCalcMag=SKIP_CALC_MAG_INTERVAL/2;
        }
        else correction = correction.slerp(getCorrection(Axyz,Mxyz,quat),MAG_CORR_RATIO);
    }else skipCalcMag--;
    quaternion=correction*quat;
#else
    getMPUScaled();
    // Orientations of axes are set in accordance with the datasheet
    // See Section 9.1 Orientation of Axes
    // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[1], Mxyz[0], -Mxyz[2], deltat * 1.0e-6);
    quaternion.set(-q[1], -q[2], -q[0], q[3]);

#endif
    quaternion *= sensorOffset;
    if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
    }
}

void MPU9250Sensor::sendData() {
    if(newData) {
        sendQuat(&quaternion, isSecond ? PACKET_ROTATION_2 : PACKET_ROTATION);
        newData = false;
    }
}

void MPU9250Sensor::setSecond() {
    isSecond = true;
    sensorOffset = {Quat(Vector3(0, 0, 1), SECOND_IMU_ROTATION)};
}

void MPU9250Sensor::getMPUScaled()
{
    float temp[3];
    int i;
    int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = ((float)gx - calibration->G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - calibration->G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - calibration->G_off[2]) * gscale;

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

    // Apply correction for 16-bit mode and factory sensitivity adjustments
    Mxyz[0] = (float)mx * 0.15f * adjustments[0];
    Mxyz[1] = (float)my * 0.15f * adjustments[1];
    Mxyz[2] = (float)mz * 0.15f * adjustments[2];
    //apply offsets and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (i = 0; i < 3; i++)
            temp[i] = (Mxyz[i] - calibration->M_B[i]);
        Mxyz[0] = calibration->M_Ainv[0][0] * temp[0] + calibration->M_Ainv[0][1] * temp[1] + calibration->M_Ainv[0][2] * temp[2];
        Mxyz[1] = calibration->M_Ainv[1][0] * temp[0] + calibration->M_Ainv[1][1] * temp[1] + calibration->M_Ainv[1][2] * temp[2];
        Mxyz[2] = calibration->M_Ainv[2][0] * temp[0] + calibration->M_Ainv[2][1] * temp[1] + calibration->M_Ainv[2][2] * temp[2];
    #else
        for (i = 0; i < 3; i++)
            Mxyz[i] = (Mxyz[i] - calibration->M_B[i]);
    #endif
    vector_normalize(Mxyz);
}

void MPU9250Sensor::startCalibration(int calibrationType) {
    digitalWrite(CALIBRATING_LED, LOW);
    Serial.println("[NOTICE] Gathering raw data for device calibration...");
    constexpr int calibrationSamples = 300;
    DeviceConfig *config = getConfigPtr();
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration
    Serial.println("[NOTICE] Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
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
    config->calibration[isSecond?1:0].G_off[0] = Gxyz[0];
    config->calibration[isSecond?1:0].G_off[1] = Gxyz[1];
    config->calibration[isSecond?1:0].G_off[2] = Gxyz[2];

    // Blink calibrating led before user should rotate the sensor
    Serial.println("[NOTICE] After 3seconds, Gently rotate the device while it's gathering accelerometer and magnetometer data");
    digitalWrite(CALIBRATING_LED, LOW);
    delay(1500);
    digitalWrite(CALIBRATING_LED, HIGH);
    delay(1500);
    Serial.println("[NOTICE] Gathering accelerometer and magnetometer data start!!");

    float *calibrationDataAcc = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    float *calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++)
    {
        int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
        digitalWrite(CALIBRATING_LED, LOW);
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
        calibrationDataMag[i * 3 + 0] = mx;
        calibrationDataMag[i * 3 + 1] = my;
        calibrationDataMag[i * 3 + 2] = mz;
        sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0, PACKET_RAW_CALIBRATION_DATA);
        sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0, PACKET_RAW_CALIBRATION_DATA);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(100);
    }
    Serial.println("[NOTICE] Calibration data gathered");
    digitalWrite(CALIBRATING_LED, HIGH);
    delay(250);
    Serial.println("[NOTICE] Now Calculate Calibration data");

    float A_BAinv[4][3];
    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, calibrationSamples, A_BAinv);
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataAcc);
    free(calibrationDataMag);
    Serial.println("[NOTICE] Finished Calculate Calibration data");
    Serial.println("[NOTICE] Now Saving EEPROM");
    for (int i = 0; i < 3; i++)
    {
        config->calibration[isSecond?1:0].A_B[i] = A_BAinv[0][i];
        config->calibration[isSecond?1:0].A_Ainv[0][i] = A_BAinv[1][i];
        config->calibration[isSecond?1:0].A_Ainv[1][i] = A_BAinv[2][i];
        config->calibration[isSecond?1:0].A_Ainv[2][i] = A_BAinv[3][i];

        config->calibration[isSecond?1:0].M_B[i] = M_BAinv[0][i];
        config->calibration[isSecond?1:0].M_Ainv[0][i] = M_BAinv[1][i];
        config->calibration[isSecond?1:0].M_Ainv[1][i] = M_BAinv[2][i];
        config->calibration[isSecond?1:0].M_Ainv[2][i] = M_BAinv[3][i];
    }

    setConfig(*config);
    sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0, PACKET_RAW_CALIBRATION_DATA);
    Serial.println("[NOTICE] Finished Saving EEPROM");
    delay(4000);
}