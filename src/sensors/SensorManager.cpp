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
        uint8_t imu[16] = {IMUS};
        double imuRotation[16] = {IMU_ROTATION};
        uint8_t imuIntPin[16] = {IMU_INT};
        uint8_t imuSCLPin[8] = {IMU_SCL};

        void SensorManager::setup()
        {
            bool foundIMU = false;

            for (uint8_t i=0; i<IMU_COUNT; i+=2) {
                uint8_t firstIMUAddress = 0;
                uint8_t secondIMUAddress = 0;
                Wire.begin(PIN_IMU_SDA, imuSCLPin[i/2]);

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
                        m_Logger.info("IMU %d found at pins %d,%d and address 0x%02X", i, PIN_IMU_SDA, imuSCLPin[i/2], firstIMUAddress);

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
                        m_Logger.info("IMU %d found at pins %d,%d and address 0x%02X", i+1, PIN_IMU_SDA, imuSCLPin[i/2], secondIMUAddress);

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
                for (uint8_t i=0; i<IMU_COUNT; i++) {
                    swap(i);
                    m_Sensor[i]->motionSetup();
                }
            } else {
                Serial.println("[ERR] I2C: Can't find I2C device on provided addresses, scanning for all I2C devices and returning");
                I2CSCAN::scani2cports();
            }
        }

        void SensorManager::postSetup()
        {
            for (uint8_t i=0; i<IMU_COUNT; i++) {
                swap(i);
                m_Sensor[i]->postSetup();
            }
        }

        void SensorManager::update()
        {
            // Gather IMU data
            for (uint8_t i=0; i<IMU_COUNT; i++) {
                swap(i);
                m_Sensor[i]->motionLoop();
                if (ServerConnection::isConnected()) {
                    m_Sensor[i]->sendData();
                }
            }
        }

        void SensorManager::swap(int id) {
            if (INTERNAL_MUX) {
#ifdef ESP32
                Wire.end();
#endif
                Wire.begin(PIN_IMU_SDA, imuSCLPin[id/2]);
            } else {
                //todo
            }
        }
    }
}
