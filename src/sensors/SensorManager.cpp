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
#include <StreamString.h>
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "mpu9250sensor.h"
#include "mpu6050sensor.h"
#include "bmi160sensor.h"
#include "icm20948sensor.h"
#include "ErroneousSensor.h"
#include "sensoraddresses.h"
#include "GlobalVars.h"

#if ESP32
    #include "driver/i2c.h"
#endif

namespace SlimeVR
{
    namespace Sensors
    {
        Sensor* SensorManager::buildSensor(uint8_t sensorID, uint8_t imuType, uint8_t address, float rotation, uint8_t sclPin, uint8_t sdaPin, uint8_t intPin)
        {
            m_Logger.trace("Building IMU with: id=%d,\n\
                            imuType=0x%02X, address=%d, rotation=%f,\n\
                            sclPin=%d, sdaPin=%d, intPin=%d",
                            sensorID,
                            imuType, address, rotation,
                            sclPin, sdaPin, intPin);

            // Convert degrees to angle
            rotation *= PI / 180.0f;

            // Now start detecting and building the IMU
            Sensor* sensor = NULL;

            // Clear and reset I2C bus for each sensor upon startup
            I2CSCAN::clearBus(sdaPin, sclPin);
            swapI2C(sclPin, sdaPin);

            if (I2CSCAN::isI2CExist(address)) {
                m_Logger.trace("IMU %d found at address 0x%02X", sensorID, address);
            } else {
                sensor = new ErroneousSensor(sensorID, imuType);
                return sensor;
            }

            switch (imuType) {
            case IMU_BNO080: case IMU_BNO085: case IMU_BNO086:
                sensor = new BNO080Sensor(sensorID, imuType, address, rotation, sclPin, sdaPin, intPin);
                break;
            case IMU_BNO055:
                sensor = new BNO055Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            case IMU_MPU9250:
                sensor = new MPU9250Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            case IMU_BMI160:
                sensor = new BMI160Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            case IMU_MPU6500: case IMU_MPU6050:
                sensor = new MPU6050Sensor(sensorID, imuType, address, rotation, sclPin, sdaPin);
                break;
            case IMU_ICM20948:
                sensor = new ICM20948Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            default:
                sensor = new ErroneousSensor(sensorID, imuType);
                break;
            }

            sensor->motionSetup();
            return sensor;
        }

        void SensorManager::swapI2C(uint8_t sclPin, uint8_t sdaPin)
        {
            if (sclPin != activeSCL || sdaPin != activeSDA || !running) {
                Wire.flush();
                #if ESP32
                    if (running) {}
                    else {
                        // Reset HWI2C to avoid being affected by I2CBUS reset
                        Wire.end();
                    }
                    // Disconnect pins from HWI2C
                    pinMode(activeSCL, INPUT);
                    pinMode(activeSDA, INPUT);

                    if (running) {
                        i2c_set_pin(I2C_NUM_0, sdaPin, sclPin, false, false, I2C_MODE_MASTER);
                    } else {
                        Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin), I2C_SPEED);
                        Wire.setTimeOut(150);
                    }
                #else
                    Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin));
                #endif

                activeSCL = sclPin;
                activeSDA = sdaPin;
            }
        }

        void SensorManager::setup()
        {
            running = false;
            activeSCL = PIN_IMU_SCL;
            activeSDA = PIN_IMU_SDA;

            uint8_t sensorID = 0;
#define IMU_DESC_ENTRY(...)                                          \
            {                                                        \
                Sensor* sensor = buildSensor(sensorID, __VA_ARGS__); \
                m_Sensors[sensorID] = sensor;                        \
                sensorID++;                                          \
            }
            IMU_DESC_LIST;
#undef IMU_DESC_ENTRY
        }

        void SensorManager::postSetup()
        {
            running = true;
            for (auto sensor : m_Sensors) {
                if (sensor->isWorking()) {
                    swapI2C(sensor->sclPin, sensor->sdaPin);
                    sensor->postSetup();
                }
            }
        }

        void SensorManager::update()
        {
            // Gather IMU data
            for (auto sensor : m_Sensors) {
                if (sensor->isWorking()) {
                    swapI2C(sensor->sclPin, sensor->sdaPin);
                    sensor->motionLoop();
                }
            }

            if (!networkConnection.isConnected())
            {
                return;
            }

            // Send updates
            for (auto sensor : m_Sensors) {
                if (sensor->isWorking()) {
                    sensor->sendData();
                }
            }
        }

    }
}
