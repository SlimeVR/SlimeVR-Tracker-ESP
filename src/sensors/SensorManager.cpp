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

namespace SlimeVR
{
    namespace Sensors
    {
        #ifndef IMU_DESC_STR
        #define IMU_DESC_STR IMU(PRIMARY_IMU_ADDRESS_ONE,IMU_ROTATION,PIN_IMU_SCL,PIN_IMU_SDA,PIN_IMU_INT);\
                             SECOND_IMU(SECONDARY_IMU_ADDRESS_TWO,IMU_ROTATION,PIN_IMU_SCL,PIN_IMU_SDA,PIN_IMU_INT);
        #endif

        #define STR_WRAP(STR) #STR
        #define STR_WRAP2(STR) STR_WRAP(STR)
        #define IMU_DESC_STR_VAL STR_WRAP2(IMU_DESC_STR)

        // Sensor descriptor string format:
        // imuType(address,rotation,sclpin,sdapin,intpin);

        Sensor* SensorManager::buildSensor(String &desc, uint8_t sensorID)
        {
            uint8_t imuType = 0;
            uint8_t address = 0;
            float rotation = 0.0f;
            uint8_t sclpin = 0;
            uint8_t sdapin = 0;
            uint8_t intpin = 0;

            int nparam = sscanf(desc.c_str(), "%hhu(0x%hhx,%f,%hhu,%hhu,%hhu)",
                                            &imuType, &address, &rotation, &sclpin, &sdapin, &intpin);

            // Convert degrees to angle
            rotation *= PI / 180.0f;

            Sensor* sensor = buildSensor(sensorID, address, imuType, rotation, sclpin, sdapin, intpin);
            return sensor;
        }

        Sensor* SensorManager::buildSensor(uint8_t sensorID, uint8_t address, uint8_t imuType, float rotation, uint8_t sclPin, uint8_t sdaPin, uint8_t intPin)
        {
            Sensor* sensor = NULL;

            // Check IMU address to match previous behavior
            // where if the first IMU addr is absent, the second IMU address will be parsed
            if (I2CSCAN::isI2CExist(address))
            {
                m_Logger.trace("IMU %d found at address 0x%02X", sensorID, address);
            }
            else
            {
                sensor = new ErroneousSensor(sensorID, imuType);
                return sensor;
            }

            switch (imuType)
            {
            case IMU_BNO080: case IMU_BNO085: case IMU_BNO086:
                sensor = new BNO080Sensor(sensorID, imuType, address, rotation, intPin);
                break;
            case IMU_BNO055:
                sensor = new BNO055Sensor(sensorID, address, rotation);
                break;
            case IMU_MPU9250:
                sensor = new MPU9250Sensor(sensorID, address, rotation);
                break;
            case IMU_BMI160:
                sensor = new BMI160Sensor(sensorID, address, rotation);
                break;
            case IMU_MPU6500: case IMU_MPU6050:
                sensor = new MPU6050Sensor(sensorID, imuType, address, rotation);
                break;
            case IMU_ICM20948:
                sensor = new ICM20948Sensor(sensorID, address, rotation);
                break;
            default:
                sensor = new ErroneousSensor(sensorID, imuType);
                break;
            }

            sensor->motionSetup();
            return sensor;
        }

        void SensorManager::setup()
        {
            // Divide sensor from descriptor string by semicolon
            StreamString mstr;
            mstr.print(IMU_DESC_STR_VAL);

            String desc;
            uint8_t sensorID = 0;
            while (mstr.available() && sensorID < MAX_IMU_COUNT)
            {
                desc = mstr.readStringUntil(';');
                if (desc.endsWith(")")) // Verify the end of descriptor
                {
                    Sensor* sensor = buildSensor(desc, sensorID);
                    m_Sensors[sensorID] = sensor;
                    sensorID++;
                }
                else
                {
                    Serial.printf("Bad sensor descriptor %s\n", desc.c_str());
                }
            }
        }

        void SensorManager::postSetup()
        {
            for (auto sensor : m_Sensors)
            {
                if (sensor != NULL)
                {
                    sensor->postSetup();
                }
            }
        }

        void SensorManager::update()
        {
            // Gather IMU data
            for (auto sensor : m_Sensors)
            {
                if (sensor != NULL)
                {
                    sensor->motionLoop();
                }
            }

            if (!networkConnection.isConnected())
            {
                return;
            }

            // Send updates
            for (auto sensor : m_Sensors)
            {
                if (sensor != NULL)
                {
                    sensor->sendData();
                }
            }
        }

    }
}
