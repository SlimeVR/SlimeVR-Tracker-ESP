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

#if ESP32
    #include "driver/i2c.h"
#endif

namespace SlimeVR
{
    namespace Sensors
    {
        Sensor* SensorManager::buildSensor(uint8_t sensorID, uint8_t imuType, uint8_t address, float rotation, uint8_t sclPin, uint8_t sdaPin, int extraParam)
        {
            m_Logger.trace("Building IMU with: id=%d,\n\
                            imuType=0x%02X, address=0x%02X, rotation=%f,\n\
                            sclPin=%d, sdaPin=%d, extraParam=%d",
                            sensorID,
                            imuType, address, rotation,
                            sclPin, sdaPin, extraParam);

            // Now start detecting and building the IMU
            Sensor* sensor = nullptr;

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
                // Extra param used as interrupt pin
                {
                uint8_t intPin = extraParam;
                sensor = new BNO080Sensor(sensorID, imuType, address, rotation, sclPin, sdaPin, intPin);
                }
                break;
            case IMU_BNO055:
                sensor = new BNO055Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            case IMU_MPU9250:
                sensor = new MPU9250Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            case IMU_BMI160:
                // Extra param used as axis remap descriptor
                {
                int axisRemap = extraParam;
                // Valid remap will use all axes, so there will be non-zero term in upper 9 mag bits
                // Used to avoid default INT_PIN misinterpreted as axis mapping
                if (axisRemap < 256) {
                    sensor = new BMI160Sensor(sensorID, address, rotation, sclPin, sdaPin);
                } else {
                    sensor = new BMI160Sensor(sensorID, address, rotation, sclPin, sdaPin, axisRemap);
                }
                }
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

        // TODO Make it more generic in the future and move another place (abstract sensor interface)
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
            uint8_t activeSensorCount = 0;
#define IMU_DESC_ENTRY(...)                                          \
            {                                                        \
                Sensor* sensor = buildSensor(sensorID, __VA_ARGS__); \
                m_Sensors[sensorID] = sensor;                        \
                sensorID++;                                          \
                if (sensor->isWorking()) {                           \
                    m_Logger.info("Sensor %d configured", sensorID); \
                    activeSensorCount++;                             \
                }                                                    \
            }
            // Apply descriptor list and expand to entrys
            IMU_DESC_LIST;

#undef IMU_DESC_ENTRY
            m_Logger.info("%d sensor(s) configured", activeSensorCount);
            // Check and scan i2c if no sensors active
            if (activeSensorCount == 0) {
                m_Logger.error("Can't find I2C device on provided addresses, scanning for all I2C devices and returning");
                I2CSCAN::scani2cports();
            }
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

            if (!networkConnection.isConnected()) {
                return;
            }

            #ifndef PACKET_BUNDLING
                static_assert(false, "PACKET_BUNDLING not set");
            #endif
            #if PACKET_BUNDLING == PACKET_BUNDLING_BUFFERED
                uint32_t now = micros();
                bool shouldSend = false;
                bool allSensorsReady = true;
                for (auto sensor : m_Sensors) {
                    if (!sensor->isWorking()) continue;
                    if (sensor->hasNewDataToSend()) shouldSend = true;
                    allSensorsReady &= sensor->hasNewDataToSend();
                }

                if (now - m_LastBundleSentAtMicros < PACKET_BUNDLING_BUFFER_SIZE_MICROS) {
                    shouldSend &= allSensorsReady;
                }

                if (!shouldSend) {
                    return;
                }

                m_LastBundleSentAtMicros = now;
            #endif
            
            #if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
                networkConnection.beginBundle();
            #endif

            for (auto sensor : m_Sensors) {
                if (sensor->isWorking()) {
                    sensor->sendData();
                }
            }

            #if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
                networkConnection.endBundle();
            #endif
        }

    }
}
