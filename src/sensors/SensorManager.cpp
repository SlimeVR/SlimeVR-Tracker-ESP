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
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "mpu9250sensor.h"
#include "mpu6050sensor.h"
#include "bmi160sensor.h"
#include "icm20948sensor.h"
#include "ErroneousSensor.h"
#include "sensoraddresses.h"
#include "GlobalVars.h"

namespace SlimeVR
{
    namespace Sensors
    {
        void SensorManager::setup()
        {
            uint8_t firstIMUAddress = 0;
            uint8_t secondIMUAddress = 0;

            bool sharedIMUAddresses = (PRIMARY_IMU_ADDRESS_ONE == SECONDARY_IMU_ADDRESS_ONE && PRIMARY_IMU_ADDRESS_TWO == SECONDARY_IMU_ADDRESS_TWO);
            {
                firstIMUAddress = I2CSCAN::pickDevice(PRIMARY_IMU_ADDRESS_ONE, PRIMARY_IMU_ADDRESS_TWO, true);
                uint8_t sensorID = 0;
                if(sharedIMUAddresses && firstIMUAddress != PRIMARY_IMU_ADDRESS_ONE)
                {
                    sensorID = 1;
                }

                if (firstIMUAddress == 0)
                {
                    m_Sensor1 = new ErroneousSensor(sensorID, IMU);
                }
                else
                {
                    m_Logger.trace("Primary IMU found at address 0x%02X", firstIMUAddress);

#if IMU == IMU_BNO080 || IMU == IMU_BNO085 || IMU == IMU_BNO086
                    m_Sensor1 = new BNO080Sensor(sensorID, IMU, firstIMUAddress, IMU_ROTATION, PIN_IMU_INT);
#elif IMU == IMU_BNO055
                    m_Sensor1 = new BNO055Sensor(sensorID, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_MPU9250
                    m_Sensor1 = new MPU9250Sensor(sensorID, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_BMI160
                    m_Sensor1 = new BMI160Sensor(sensorID, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_MPU6500 || IMU == IMU_MPU6050
                    m_Sensor1 = new MPU6050Sensor(sensorID, IMU, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_ICM20948
                    m_Sensor1 = new ICM20948Sensor(sensorID, firstIMUAddress, IMU_ROTATION);
#endif
                }

                m_Sensor1->motionSetup();
            }

            {
                secondIMUAddress = I2CSCAN::pickDevice(SECONDARY_IMU_ADDRESS_TWO, SECONDARY_IMU_ADDRESS_ONE, false);
                uint8_t sensorID = 1;
                if(sharedIMUAddresses && secondIMUAddress != SECONDARY_IMU_ADDRESS_TWO)
                {
                    sensorID = 0;
                }

                if (secondIMUAddress == firstIMUAddress)
                {
                    m_Logger.debug("No secondary IMU connected");
                }
                else if (secondIMUAddress == 0)
                {
                    m_Sensor2 = new ErroneousSensor(sensorID, SECOND_IMU);
                }
                else
                {
                    m_Logger.trace("Secondary IMU found at address 0x%02X", secondIMUAddress);

#if SECOND_IMU == IMU_BNO080 || SECOND_IMU == IMU_BNO085 || SECOND_IMU == IMU_BNO086
                    m_Sensor2 = new BNO080Sensor(sensorID, SECOND_IMU, secondIMUAddress, SECOND_IMU_ROTATION, PIN_IMU_INT_2);
#elif SECOND_IMU == IMU_BNO055
                    m_Sensor2 = new BNO055Sensor(sensorID, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_MPU9250
                    m_Sensor2 = new MPU9250Sensor(sensorID, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_BMI160
                    m_Sensor2 = new BMI160Sensor(sensorID, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_MPU6500 || SECOND_IMU == IMU_MPU6050
                    m_Sensor2 = new MPU6050Sensor(sensorID, SECOND_IMU, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_ICM20948
                    m_Sensor2 = new ICM20948Sensor(sensorID, secondIMUAddress, SECOND_IMU_ROTATION);
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
            Sensor* sensors[2] = { m_Sensor1, m_Sensor2 };

            for (auto sensor : sensors) {
                sensor->motionLoop();
            }

            if (!networkConnection.isConnected()) {
                return;
            }

            uint32_t now = micros();
            bool shouldSend = false;

            #ifndef PACKET_BUNDLING
                static_assert(false, "PACKET_BUNDLING not set");
            #endif
            #if PACKET_BUNDLING == PACKET_BUNDLING_LOWLATENCY
                for (auto sensor : sensors) {
                    if (sensor->hasNewDataToSend()) {
                        shouldSend = true;
                        break;
                    }
                }
            #elif PACKET_BUNDLING == PACKET_BUNDLING_BUFFERED
                bool allSensorsReady = true;
                for (auto sensor : sensors) {
                    if (!sensor->isWorking()) continue;
                    if (sensor->hasNewDataToSend()) shouldSend = true;
                    allSensorsReady &= sensor->hasNewDataToSend();
                }

                if (now - m_LastBundleSentAtMicros < PACKET_BUNDLING_BUFFER_SIZE_MICROS) {
                    shouldSend &= allSensorsReady;
                }
            #else
                shouldSend = true;
            #endif
            
            if (!shouldSend) {
                return;
            }

            m_LastBundleSentAtMicros = now;
            
            #if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
                networkConnection.beginBundle();
            #endif

            for (auto sensor : sensors) {
                sensor->sendData();
            }

            #if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
                networkConnection.endBundle();
            #endif
        }
    }
}
