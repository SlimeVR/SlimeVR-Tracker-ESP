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
        void SensorManager::setup()
        {
            uint8_t firstIMUAddress = 0;
            uint8_t secondIMUAddress = 0;

            {
#if IMU == IMU_BNO080 || IMU == IMU_BNO085 || IMU == IMU_BNO086
                firstIMUAddress = I2CSCAN::pickDevice(0x4A, 0x4B, true);
#elif IMU == IMU_BNO055
                firstIMUAddress = I2CSCAN::pickDevice(0x29, 0x28, true);
#elif IMU == IMU_MPU9250 || IMU == IMU_BMI160 || IMU == IMU_MPU6500 || IMU == IMU_MPU6050 || IMU == IMU_ICM20948
                firstIMUAddress = I2CSCAN::pickDevice(0x68, 0x69, true);
#else
#error Unsupported primary IMU
#endif

                if (firstIMUAddress == 0)
                {
                    m_Sensor1 = new ErroneousSensor(0, IMU);
                }
                else
                {
                    m_Logger.trace("Primary IMU found at address 0x%02X", firstIMUAddress);

#if IMU == IMU_BNO080 || IMU == IMU_BNO085 || IMU == IMU_BNO086
                    m_Sensor1 = new BNO080Sensor(0, IMU, firstIMUAddress, IMU_ROTATION, PIN_IMU_INT);
#elif IMU == IMU_BNO055
                    m_Sensor1 = new BNO055Sensor(0, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_MPU9250
                    m_Sensor1 = new MPU9250Sensor(0, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_BMI160
                    m_Sensor1 = new BMI160Sensor(0, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_MPU6500 || IMU == IMU_MPU6050
                    m_Sensor1 = new MPU6050Sensor(0, IMU, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_ICM20948
                    m_Sensor1 = new ICM20948Sensor(0, firstIMUAddress, IMU_ROTATION);
#endif
                }

                m_Sensor1->motionSetup();
            }

            {
#if SECOND_IMU == IMU_BNO080 || SECOND_IMU == IMU_BNO085 || SECOND_IMU == IMU_BNO086
                secondIMUAddress = I2CSCAN::pickDevice(0x4B, 0x4A, false);
#elif SECOND_IMU == IMU_BNO055
                secondIMUAddress = I2CSCAN::pickDevice(0x28, 0x29, false);
#elif SECOND_IMU == IMU_MPU9250 || SECOND_IMU == IMU_BMI160 || SECOND_IMU == IMU_MPU6500 || SECOND_IMU == IMU_MPU6050 || SECOND_IMU == IMU_ICM20948
                secondIMUAddress = I2CSCAN::pickDevice(0x69, 0x68, false);
#else
#error Unsupported secondary IMU
#endif

                if (secondIMUAddress == firstIMUAddress)
                {
                    m_Logger.debug("No secondary IMU connected");
                }
                else if (secondIMUAddress == 0)
                {
                    m_Sensor2 = new ErroneousSensor(1, SECOND_IMU);
                }
                else
                {
                    m_Logger.trace("Secondary IMU found at address 0x%02X", secondIMUAddress);

#if SECOND_IMU == IMU_BNO080 || SECOND_IMU == IMU_BNO085 || SECOND_IMU == IMU_BNO086
                    m_Sensor2 = new BNO080Sensor(1, SECOND_IMU, secondIMUAddress, SECOND_IMU_ROTATION, PIN_IMU_INT_2);
#elif SECOND_IMU == IMU_BNO055
                    m_Sensor2 = new BNO055Sensor(1, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_MPU9250
                    m_Sensor2 = new MPU9250Sensor(1, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_BMI160
                    m_Sensor2 = new BMI160Sensor(1, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_MPU6500 || SECOND_IMU == IMU_MPU6050
                    m_Sensor2 = new MPU6050Sensor(1, SECOND_IMU, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_ICM20948
                    m_Sensor2 = new ICM20948Sensor(1, secondIMUAddress, SECOND_IMU_ROTATION);
#endif
                }

                m_Sensor2->motionSetup();
            }
        }

        void SensorManager::postSetup()
        {
            m_Sensor1->postSetup();
            m_Sensor2->postSetup();
        }

        void SensorManager::update()
        {
            // Gather IMU data
            m_Sensor1->motionLoop();
            m_Sensor2->motionLoop();

            if (!ServerConnection::isConnected())
            {
                return;
            }

            // Send updates
            m_Sensor1->sendData();
            m_Sensor2->sendData();
        }
    }
}
