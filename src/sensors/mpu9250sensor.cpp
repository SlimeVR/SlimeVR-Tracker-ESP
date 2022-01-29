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

#define _DMP_RAW_           // using DMP's acc, gyro and raw mag
// #define _DMP_QUAT_          // using DMP's quat and raw mag
// #define _RAW_FILTER_        // using raw acc, gyro, mag

#define _INTERRUPT_

#include "mpu9250sensor.h"
#include "network/network.h"
#include "globals.h"
#include "helper_3dmath.h"
#include <i2cscan.h>
#include "calibration.h"
#include "ledmgr.h"
#include "magneto1.4.h"

#if defined(_DMP_RAW_) || defined(_RAW_FILTER_)
    //#include "mahony.h"
    #include "madgwick.h"

    #if defined(_MADGWICK_H_)
        #define QuaternionUpdate madgwickQuaternionUpdate
    #elif defined(_MAHONY_H_)
        #define QuaternionUpdate mahonyQuaternionUpdate
    #endif

    constexpr float ascale = (2.f / 32768.f);
    constexpr float mscale = 0.15; // (4912.f / 8190.f) = 0.6 | (4912.f / 32760.f) = 0.15

    #ifdef _DMP_RAW_
        constexpr float gscale = (2000.f / 32768.f) * (PI / 180);
    #else   // _RAW_FILTER_
        constexpr float gscale = (250. / 32768.0) * (PI / 180.0); //gyro default 250 LSB per d/s -> rad/s
    #endif
#else   // _DMP_QUAT_
    #include "dmpmag.h"
    constexpr float ascale = (2.f / 32768.f);
    constexpr float gscale = (250. / 32768.0) * (PI / 180.0); //gyro default 250 LSB per d/s -> rad/s
    constexpr float mscale = 0.15; // (4912.f / 8190.f) = 0.6 | (4912.f / 32760.f) = 0.15
#endif

#define SKIP_CALC_MAG_INTERVAL  10
#define MAG_CORR_RATIO          0.02

#ifdef _INTERRUPT_
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
#endif

void MPU9250Sensor::motionSetup() {
    DeviceConfig * const config = getConfigPtr();
    calibration = &config->calibration[sensorId];
    // initialize device
    imu.initialize(addr);
    if(!imu.testConnection()) {
        Serial.print("[ERR] MPU9250: Can't communicate with MPU, response 0x");
        Serial.println(imu.getDeviceID(), HEX);
        return;
    }
    imu.getMagnetometerAdjustments(magAdjustments);


#if defined(_RAW_FILTER_)
    int16_t ax,ay,az;
    // turn on while flip back to calibrate. then, flip again after 5 seconds.    
    // TODO: Move calibration invoke after calibrate button on slimeVR server available 
    imu.getAcceleration(&ax, &ay, &az);
    if(az<0 && 10.0*(ax*ax+ay*ay)<az*az) {
        digitalWrite(CALIBRATING_LED, HIGH);
        Serial.println("Calling Calibration... Flip front to confirm start calibration.");
        delay(5000);
        digitalWrite(CALIBRATING_LED, LOW);
        imu.getAcceleration(&ax, &ay, &az);
        if(az>0 && 10.0*(ax*ax+ay*ay)<az*az) 
            startCalibration(0);
    }
#elif defined(_DMP_RAW_) || defined(_DMP_QUAT_)
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
        
        
    #ifdef _INTERRUPT_
        // enable interrupt detection
        Serial.println(F("[NOTICE] Enabling interrupt detection..."));
        pinMode(PIN_IMU_INT_2, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT_2), dmpDataReady, RISING);
        mpuIntStatus = imu.getIntStatus();
    #endif

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("[NOTICE] DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();

        working = true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("[ERR] DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // turn on while flip back to calibrate. then, flip again after 5 seconds.    
    // TODO: Move calibration invoke after calibrate button on slimeVR server available 
    while(!getDMPScaled());

    if(rawAcc[2]<0 && 10.0*(rawAcc[0]*rawAcc[0]+rawAcc[1]*rawAcc[1])<rawAcc[2]*rawAcc[2]) {
        digitalWrite(CALIBRATING_LED, HIGH);
        Serial.println("Calling Calibration... Flip front to confirm start calibration.");
        delay(5000);
        digitalWrite(CALIBRATING_LED, LOW);

#ifndef _INTERRUPT_
        imu.resetFIFO();
#endif
        while(!getDMPScaled());

        if(rawAcc[2]>0 && 10.0*(rawAcc[0]*rawAcc[0]+rawAcc[1]*rawAcc[1])<rawAcc[2]*rawAcc[2]) {
            startCalibration(0);
        }
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

#if defined(_DMP_RAW_)
    // Update quaternion
    if (getDMPScaled()) {
        if(skipCalcMag == 0){
            skipCalcMag = SKIP_CALC_MAG_INTERVAL;
            QuaternionUpdate(q,
                                Axyz[0], Axyz[1], Axyz[2],
                                Gxyz[0], Gxyz[1], Gxyz[2],
                                Mxyz[0], Mxyz[1], Mxyz[2],
                                deltat * 1.0e-6
            );
        } else {
            skipCalcMag--;
            QuaternionUpdate(q,
                                Axyz[0], Axyz[1], Axyz[2],
                                Gxyz[0], Gxyz[1], Gxyz[2],
                                deltat * 1.0e-6
            );
        }

        quaternion.set(-q[2], q[1], q[3], q[0]);
        quaternion *= sensorOffset;
    }

#elif defined(_DMP_QUAT_)
    Quaternion rawQuat{};
    if(!imu.GetCurrentFIFOPacket(fifoBuffer, imu.dmpGetFIFOPacketSize())) return;
    imu.dmpGetQuaternion(&rawQuat, fifoBuffer);
    Quat quat(-rawQuat.y,rawQuat.x,rawQuat.z,rawQuat.w);
    if(!skipCalcMag){
        int16_t mx, my, mz;
        imu.getMagnetometer(&mx, &my, &mz);
        // Apply correction for 16-bit mode and factory sensitivity adjustments
        Mxyz[0] = (float)my * mscale * magAdjustments[1];
        Mxyz[1] = (float)mx * mscale * magAdjustments[0];
        Mxyz[2] = -(float)mz * mscale * magAdjustments[2];

        float temp[3];
        //apply offsets and scale factors from Magneto
        #if useFullCalibrationMatrix == true
            for (int i = 0; i < 3; i++)
                temp[i] = (Mxyz[i] - calibration->M_B[i]);
            Mxyz[0] = calibration->M_Ainv[0][0] * temp[0] + calibration->M_Ainv[0][1] * temp[1] + calibration->M_Ainv[0][2] * temp[2];
            Mxyz[1] = calibration->M_Ainv[1][0] * temp[0] + calibration->M_Ainv[1][1] * temp[1] + calibration->M_Ainv[1][2] * temp[2];
            Mxyz[2] = calibration->M_Ainv[2][0] * temp[0] + calibration->M_Ainv[2][1] * temp[1] + calibration->M_Ainv[2][2] * temp[2];
        #else
            for (i = 0; i < 3; i++)
                Mxyz[i] = (Mxyz[i] - calibration->M_B[i]);
        #endif


        if(Mxyz[0]==0.0f && Mxyz[1]==0.0f && Mxyz[2]==0.0f) return;
        skipCalcMag=SKIP_CALC_MAG_INTERVAL;
        if(correction.length_squared()==0.0f) {
            correction=getCorrection(Axyz,Mxyz,quat);
        }
        else correction = correction.slerp(getCorrection(Axyz,Mxyz,quat),MAG_CORR_RATIO);
    }else skipCalcMag--;
    quaternion=correction*quat;
    quaternion *= sensorOffset;

#elif defined(_RAW_FILTER_)
    getMPUScaled();
    // Orientations of axes are set in accordance with the datasheet
    // See Section 9.1 Orientation of Axes
    // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    QuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    quaternion.set(-q[2], q[1], q[3], q[0]);
    quaternion *= sensorOffset;

#endif
    if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
    }
}

void MPU9250Sensor::getMPUScaled()
{
    float temp[3];
    int i;
    int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Axyz[0] = (float)ax * ascale;
    Axyz[1] = (float)ay * ascale;
    Axyz[2] = (float)az * ascale;
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

    Gxyz[0] = (float)gx * gscale;
    Gxyz[1] = (float)gy * gscale;
    Gxyz[2] = (float)gz * gscale;
    for (i = 0; i < 3; i++)
        Gxyz[i] = (Gxyz[i] - calibration[sensorId].G_off[i]);

    // Apply correction for 16-bit mode and factory sensitivity adjustments
    Mxyz[0] = (float)my * mscale * magAdjustments[1];
    Mxyz[1] = (float)mx * mscale * magAdjustments[0];
    Mxyz[2] = -(float)mz * mscale * magAdjustments[2];
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
}

bool MPU9250Sensor::getDMPScaled() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return false;
    
#ifdef _INTERRUPT_
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = imu.getIntStatus();

    // get current FIFO count
    fifoCount = imu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        imu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return false;

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = imu.getFIFOCount();

        // read a packet from FIFO
        imu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
#else
/* Returns 1) when data was successfully read
 *         2) when recovering from overflow, only the earliest packet is read
 *         0) when no valid data is available
 */
    if(imu.GetCurrentFIFOPacket(fifoBuffer, imu.dmpPacketSize) != 0) {
#endif
        imu.dmpGetAccel(rawAcc, fifoBuffer);
        imu.dmpGetGyro(rawGyro, fifoBuffer);

        Axyz[0] = (float)rawAcc[0] * ascale;
        Axyz[1] = (float)rawAcc[1] * ascale;
        Axyz[2] = (float)rawAcc[2] * ascale;

        Gxyz[0] = (float)rawGyro[0] * gscale;
        Gxyz[1] = (float)rawGyro[1] * gscale;
        Gxyz[2] = (float)rawGyro[2] * gscale;

        // See MPU-9250 Product Specification - 9.1 Orientation of Axes
        // XYZ to YX-Z 
        imu.getMagnetometer(&rawMag[0], &rawMag[1], &rawMag[2]);
        Mxyz[0] = (float)rawMag[1] * mscale * magAdjustments[1];
        Mxyz[1] = (float)rawMag[0] * mscale * magAdjustments[0];
        Mxyz[2] = -(float)rawMag[2] * mscale * magAdjustments[2];

        //apply offsets and scale factors from Magneto
        #if useFullCalibrationMatrix == true
            float temp[3];
            for (int i = 0; i < 3; i++)
                temp[i] = (Mxyz[i] - calibration->M_B[i]);
            Mxyz[0] = calibration->M_Ainv[0][0] * temp[0] + calibration->M_Ainv[0][1] * temp[1] + calibration->M_Ainv[0][2] * temp[2];
            Mxyz[1] = calibration->M_Ainv[1][0] * temp[0] + calibration->M_Ainv[1][1] * temp[1] + calibration->M_Ainv[1][2] * temp[2];
            Mxyz[2] = calibration->M_Ainv[2][0] * temp[0] + calibration->M_Ainv[2][1] * temp[1] + calibration->M_Ainv[2][2] * temp[2];
        #else
            for (i = 0; i < 3; i++)
                Mxyz[i] = (Mxyz[i] - calibration->M_B[i]);
        #endif

        return true;
    }

    return false;
}

void MPU9250Sensor::startCalibration(int calibrationType) {
    LEDManager::on(CALIBRATING_LED);
    constexpr int calibrationSamples = 300;
    DeviceConfig *config = getConfigPtr();
#ifdef _DMP_RAW_ || _DMP_QUAT_
    Serial.println("[NOTICE] Gyro and Acc data is calibrated by DMP");
    Serial.println("[NOTICE] Gathering mag data for device calibration...");

    // Blink calibrating led before user should rotate the sensor
    Serial.println("[NOTICE] After 3seconds, Gently rotate the device while it's gathering and magnetometer data");
    digitalWrite(CALIBRATING_LED, LOW);
    delay(1500);
    digitalWrite(CALIBRATING_LED, HIGH);
    delay(1500);
    Serial.println("[NOTICE] Gathering magnetometer data start!!");

    float *calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        while(!getDMPScaled());
        calibrationDataMag[i * 3 + 0] = (float)rawMag[1] * mscale * magAdjustments[1];
        calibrationDataMag[i * 3 + 1] = (float)rawMag[0] * mscale * magAdjustments[0];
        calibrationDataMag[i * 3 + 2] = -(float)rawMag[2] * mscale * magAdjustments[2];
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(80);
    }
    Serial.println("[NOTICE] Calibration data gathered");
    digitalWrite(CALIBRATING_LED, HIGH);
    Serial.println("[NOTICE] Now Calculate Calibration data");

    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataMag);
    Serial.println("[NOTICE] Finished Calculate Calibration data");
    Serial.println("[NOTICE] Now Saving EEPROM");

    for (int i = 0; i < 4; i++)
    {
        Serial.print("M_BAinv[");Serial.print(i);Serial.print("]\t");
        Serial.print(M_BAinv[i][0]);
        Serial.print("\t");
        Serial.print(M_BAinv[i][1]);
        Serial.print("\t");
        Serial.println(M_BAinv[i][2]);
    }

    for (int i = 0; i < 3; i++)
    {
        config->calibration[sensorId].M_B[i] = M_BAinv[0][i];
        config->calibration[sensorId].M_Ainv[0][i] = M_BAinv[1][i];
        config->calibration[sensorId].M_Ainv[1][i] = M_BAinv[2][i];
        config->calibration[sensorId].M_Ainv[2][i] = M_BAinv[3][i];
    }
    
#else
    Serial.println("[NOTICE] Gathering raw data for device calibration...");
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
    Network::sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);
    config->calibration[sensorId].G_off[0] = Gxyz[0];
    config->calibration[sensorId].G_off[1] = Gxyz[1];
    config->calibration[sensorId].G_off[2] = Gxyz[2];

    // Blink calibrating led before user should rotate the sensor
    Serial.println("[NOTICE] Gently rotate the device while it's gathering accelerometer and magnetometer data");
    LEDManager::pattern(CALIBRATING_LED, 15, 300, 3000/310);

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
        LEDManager::on(CALIBRATING_LED);
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        calibrationDataAcc[i * 3 + 0] = (float)ax * ascale;
        calibrationDataAcc[i * 3 + 1] = (float)ay * ascale;
        calibrationDataAcc[i * 3 + 2] = (float)az * ascale;
        calibrationDataMag[i * 3 + 0] = (float)my * mscale * magAdjustments[1];
        calibrationDataMag[i * 3 + 1] = (float)mx * mscale * magAdjustments[0];
        calibrationDataMag[i * 3 + 2] = -(float)mz * mscale * magAdjustments[2];
        Network::sendRawCalibrationData(calibrationDataAcc + (i * 3), CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
        Network::sendRawCalibrationData(calibrationDataMag + (i * 3), CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        LEDManager::off(CALIBRATING_LED);
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
        config->calibration[sensorId].A_B[i] = A_BAinv[0][i];
        config->calibration[sensorId].A_Ainv[0][i] = A_BAinv[1][i];
        config->calibration[sensorId].A_Ainv[1][i] = A_BAinv[2][i];
        config->calibration[sensorId].A_Ainv[2][i] = A_BAinv[3][i];

        config->calibration[sensorId].M_B[i] = M_BAinv[0][i];
        config->calibration[sensorId].M_Ainv[0][i] = M_BAinv[1][i];
        config->calibration[sensorId].M_Ainv[1][i] = M_BAinv[2][i];
        config->calibration[sensorId].M_Ainv[2][i] = M_BAinv[3][i];
    }

    Serial.println("[NOTICE] Calibration data gathered and sent");
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    
#endif
    LEDManager::off(CALIBRATING_LED);
    Serial.println("[NOTICE] Finished Saving EEPROM");
    setConfig(*config);
    delay(4000);
}
