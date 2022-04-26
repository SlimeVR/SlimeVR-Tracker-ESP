/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

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

#include "SensorManager.h"
#include <i2cscan.h>
#include "network/network.h"
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "mpu9250sensor.h"
#include "mpu6050sensor.h"
#include "bmi160sensor.h"
#include "icm20948sensor.h"
#include "ErroneousSensor.h"

namespace SlimeVR
{
    namespace Sensors
    {
        uint8_t imu[16] = {IMU_A1, IMU_A2, IMU_B1, IMU_B2, IMU_C1, IMU_C2, IMU_D1, IMU_D2, IMU_1A1, IMU_1A2, IMU_1B1, IMU_1B2, IMU_1C1, IMU_1C2, IMU_1D1, IMU_1D2};
        double imuRotation[16] = {IMU_ROTATION_A1, IMU_ROTATION_A2, IMU_ROTATION_B1, IMU_ROTATION_B2, IMU_ROTATION_C1, IMU_ROTATION_C2, IMU_ROTATION_D1, IMU_ROTATION_D2, IMU_ROTATION_1A1, IMU_ROTATION_1A2, IMU_ROTATION_1B1, IMU_ROTATION_1B2, IMU_ROTATION_1C1, IMU_ROTATION_1C2, IMU_ROTATION_1D1, IMU_ROTATION_1D2};
        uint8_t imuIntPin[16] = {PIN_IMU_INT_A1, PIN_IMU_INT_A2, PIN_IMU_INT_B1, PIN_IMU_INT_B2, PIN_IMU_INT_C1, PIN_IMU_INT_C2, PIN_IMU_INT_D1, PIN_IMU_INT_D2, PIN_IMU_INT_1A1, PIN_IMU_INT_1A2, PIN_IMU_INT_1B1, PIN_IMU_INT_1B2, PIN_IMU_INT_1C1, PIN_IMU_INT_1C2, PIN_IMU_INT_1D1, PIN_IMU_INT_1D2};
        uint8_t imuSDAPin[2] = {PIN_IMU_SDA, PIN_IMU_SDA_1};
        uint8_t imuSCLPin[8] = {PIN_IMU_SCL_A, PIN_IMU_SCL_B, PIN_IMU_SCL_C, PIN_IMU_SCL_D, PIN_IMU_SCL_1A, PIN_IMU_SCL_1B, PIN_IMU_SCL_1C, PIN_IMU_SCL_1D};

        void SensorManager::setup()
        {
            bool foundIMU = false;
            
#ifdef ESP32
            for (uint8_t i=0; i<16; i+=2) {
#else
            for (uint8_t i=0; i<8; i+=2) {
#endif
                uint8_t firstIMUAddress = 0;
                uint8_t secondIMUAddress = 0;
                Wire.begin(imuSDAPin[i/8], imuSCLPin[i/2]);

                {
                    if (imu[i] == IMU_BNO080 || imu[i] == IMU_BNO085 || imu[i] == IMU_BNO086)
                        firstIMUAddress = I2CSCAN::pickDevice(0x4A, 0x4B, true);
                    else if (imu[i] == IMU_BNO055)
                        firstIMUAddress = I2CSCAN::pickDevice(0x29, 0x28, true);
                    else if (imu[i] == IMU_MPU9250 || imu[i] == IMU_BMI160 || imu[i] == IMU_MPU6500 || imu[i] == IMU_MPU6050 || imu[i] == IMU_ICM20948)
                        firstIMUAddress = I2CSCAN::pickDevice(0x68, 0x69, true);

                    if (firstIMUAddress == 0)
                    {
                        m_Logger.debug("IMU %d not found", i);
                    }
                    else
                    {
                        foundIMU = true;
                        m_Logger.info("IMU %d found at pins %d,%d and address 0x%02X", i, imuSDAPin[i/8], imuSCLPin[i/2], firstIMUAddress);

                        if (imu[i] == IMU_BNO080 || imu[i] == IMU_BNO085 || imu[i] == IMU_BNO086)
                            m_Sensor[i] = new BNO080Sensor(i, imu[i], firstIMUAddress, imuRotation[i], imuIntPin[i]);
                        else if (imu[i] == IMU_BNO055)
                            m_Sensor[i] = new BNO055Sensor(i, firstIMUAddress, imuRotation[i]);
                        else if (imu[i] == IMU_MPU9250)
                            m_Sensor[i] = new MPU9250Sensor(i, firstIMUAddress, imuRotation[i]);
                        else if (imu[i] == IMU_BMI160)
                            m_Sensor[i] = new BMI160Sensor(i, firstIMUAddress, imuRotation[i]);
                        else if (imu[i] == IMU_MPU6500 || imu[i] == IMU_MPU6050)
                            m_Sensor[i] = new MPU6050Sensor(i, imu[i], firstIMUAddress, imuRotation[i]);
                        else if (imu[i] == IMU_ICM20948)
                            m_Sensor[i] = new ICM20948Sensor(i, firstIMUAddress, imuRotation[i]);
                    }

                    if (imu[i+1] == IMU_BNO080 || imu[i+1] == IMU_BNO085 || imu[i+1] == IMU_BNO086)
                        secondIMUAddress = I2CSCAN::pickDevice(0x4B, 0x4A, true);
                    else if (imu[i+1] == IMU_BNO055)
                        secondIMUAddress = I2CSCAN::pickDevice(0x28, 0x29, true);
                    else if (imu[i+1] == IMU_MPU9250 || imu[i+1] == IMU_BMI160 || imu[i+1] == IMU_MPU6500 || imu[i+1] == IMU_MPU6050 || imu[i+1] == IMU_ICM20948)
                        secondIMUAddress = I2CSCAN::pickDevice(0x69, 0x68, true);

                    if (secondIMUAddress == 0 || secondIMUAddress == firstIMUAddress)
                    {
                        m_Logger.debug("IMU %d not found", i+1);
                    }
                    else
                    {
                        foundIMU = true;
                        m_Logger.info("IMU %d found at pins %d,%d and address 0x%02X", i+1, imuSDAPin[i/8], imuSCLPin[i/2], secondIMUAddress);

                        if (imu[i+1] == IMU_BNO080 || imu[i+1] == IMU_BNO085 || imu[i+1] == IMU_BNO086)
                            m_Sensor[i+1] = new BNO080Sensor(i+1, imu[i+1], secondIMUAddress, imuRotation[i+1], imuIntPin[i+1]);
                        else if (imu[i+1] == IMU_BNO055)
                            m_Sensor[i+1] = new BNO055Sensor(i+1, secondIMUAddress, imuRotation[i+1]);
                        else if (imu[i+1] == IMU_MPU9250)
                            m_Sensor[i+1] = new MPU9250Sensor(i+1, secondIMUAddress, imuRotation[i+1]);
                        else if (imu[i+1] == IMU_BMI160)
                            m_Sensor[i+1] = new BMI160Sensor(i+1, secondIMUAddress, imuRotation[i+1]);
                        else if (imu[i+1] == IMU_MPU6500 || imu[i+1] == IMU_MPU6050)
                            m_Sensor[i+1] = new MPU6050Sensor(i+1, imu[i+1], secondIMUAddress, imuRotation[i+1]);
                        else if (imu[i+1] == IMU_ICM20948)
                            m_Sensor[i+1] = new ICM20948Sensor(i+1, secondIMUAddress, imuRotation[i+1]);
                    }
                }
            }
            if (foundIMU) {
#ifdef ESP32
                for (uint8_t i=0; i<16; i++) {
#else
                for (uint8_t i=0; i<8; i++) {
#endif
                    Wire.begin(imuSDAPin[i/8], imuSCLPin[i/2]);
                    m_Sensor[i]->motionSetup();
                }
            } else {
                Serial.println("[ERR] I2C: Can't find I2C device on provided addresses, scanning for all I2C devices and returning");
                I2CSCAN::scani2cports();
            }
        }

        void SensorManager::postSetup()
        {
#ifdef ESP32
            for (uint8_t i=0; i<16; i++) {
#else
            for (uint8_t i=0; i<8; i++) {
#endif
                Wire.begin(imuSDAPin[i/8], imuSCLPin[i/2]);
                m_Sensor[i]->postSetup();
            }
        }

        void SensorManager::update()
        {
            // Gather IMU data
#ifdef ESP32
            for (uint8_t i=0; i<16; i++) {
#else
            for (uint8_t i=0; i<8; i++) {
#endif
                Wire.begin(imuSDAPin[i/8], imuSCLPin[i/2]);
                m_Sensor[i]->motionLoop();
            }

            if (ServerConnection::isConnected()) {
                // Send updates
#ifdef ESP32
                for (uint8_t i=0; i<16; i++) {
#else
                for (uint8_t i=0; i<8; i++) {
#endif
                    m_Sensor[i]->sendData();
                }
            }
        }
    }
}
