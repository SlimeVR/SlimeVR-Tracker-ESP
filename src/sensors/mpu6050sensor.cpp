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

#include "globals.h"

#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#else
#include "MPU6050_6Axis_MotionApps20.h"
#endif

#include "mpu6050sensor.h"
#include "network/network.h"
#include <i2cscan.h>
#include "calibration.h"
#include "configuration.h"
#include "ledmgr.h"

void MPU6050Sensor::motionSetup()
{
    //DeviceConfig * const config = getConfigPtr();

    imu.initialize(addr);
    if (!imu.testConnection())
    {
        Serial.print("[ERR] MPU6050: Can't communicate with MPU, response ");
        Serial.println(imu.getDeviceID(), HEX);
        return;
    }

    devStatus = imu.dmpInitialize();

    if (devStatus == 0)
    {
#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
        // We don't have to manually calibrate if we are using the dmp's automatic calibration
#else  // IMU_MPU6050_RUNTIME_CALIBRATION

        Serial.println(F("[NOTICE] Performing startup calibration of accel and gyro..."));
        // Do a quick and dirty calibration. As the imu warms up the offsets will change a bit, but this will be good-enough
        delay(1000); // A small sleep to give the users a chance to stop it from moving

        imu.CalibrateGyro(6);
        imu.CalibrateAccel(6);
        imu.PrintActiveOffsets();
#endif // IMU_MPU6050_RUNTIME_CALIBRATION

        LEDManager::pattern(LOADING_LED, 50, 50, 5);

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

        working = true;
        configured = true;
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("[ERR] DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void MPU6050Sensor::motionLoop()
{
    if (!dmpReady)
        return;

    if (imu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        imu.dmpGetQuaternion(&rawQuat, fifoBuffer);
        quaternion.set(-rawQuat.y, rawQuat.x, rawQuat.z, rawQuat.w);
        quaternion *= sensorOffset;
        if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
        {
            newData = true;
            lastQuatSent = quaternion;
        }
    }
}

void MPU6050Sensor::startCalibration(int calibrationType) {
    LEDManager::on(CALIBRATING_LED);
#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
    Serial.println("MPU is using automatic runtime calibration. Place down the device and it should automatically calibrate after a few seconds");

    // Lie to the server and say we've calibrated
    switch (calibrationType)
    {
    case CALIBRATION_TYPE_INTERNAL_ACCEL:
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0);
        break;
    case CALIBRATION_TYPE_INTERNAL_GYRO:
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0);
        break;
    }
    LEDManager::off(CALIBRATING_LED);

#else //!IMU_MPU6050_RUNTIME_CALIBRATION
    Serial.println("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);

    imu.setDMPEnabled(false);
    imu.CalibrateGyro(6);
    imu.CalibrateAccel(6);
    imu.setDMPEnabled(true);

    Serial.println("Calibrated!");
    Serial.println("[NOTICE] Starting offset finder");
    DeviceConfig *const config = getConfigPtr();

    switch (calibrationType)
    {
    case CALIBRATION_TYPE_INTERNAL_ACCEL:
        imu.CalibrateAccel(10);
        sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0, PACKET_RAW_CALIBRATION_DATA);
        config->calibration.A_B[0] = imu.getXAccelOffset();
        config->calibration.A_B[1] = imu.getYAccelOffset();
        config->calibration.A_B[2] = imu.getZAccelOffset();
        saveConfig();
        break;
    case CALIBRATION_TYPE_INTERNAL_GYRO:
        imu.CalibrateGyro(10);
        sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0, PACKET_RAW_CALIBRATION_DATA);
        config->calibration.G_off[0] = imu.getXGyroOffset();
        config->calibration.G_off[1] = imu.getYGyroOffset();
        config->calibration.G_off[2] = imu.getZGyroOffset();
        saveConfig();
        break;
    }

    Serial.println("[NOTICE] Process is over");
    LEDMGR::off(CALIBRATING_LED);

#endif // !IMU_MPU6050_RUNTIME_CALIBRATION
}
