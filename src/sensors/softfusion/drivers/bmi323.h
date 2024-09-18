/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2024 Tailsy13 & SlimeVR Contributors

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

#pragma once

#include <cstdint>
#include <array>
#include <algorithm>
#include <limits>
#include "bmi323_lib.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

template <typename I2CImpl>
struct BMI323
{
    // Sensor Fusion required variables
    static constexpr uint8_t Address = 0x68; // i2c address
    static constexpr auto Name = "BMI323";
    static constexpr auto Type = ImuID::BMI323;

    static constexpr float GyrTs = 1.0/200.0; // Delay between each gyro sample
    static constexpr float AccTs = 1.0/200.0; // Dealy between each accel sample
    static constexpr float MagTs = 1.0/25.0; // Delay between each mag sample

    // BMI323 specific constants
    static constexpr float GyroSensitivity = 32.768f;
    static constexpr float AccelSensitivity = 4096.0f;
    static constexpr float GScale = ((32768. / GyroSensitivity) / 32768.) * (PI / 180.0);
    static constexpr float AScale = CONST_EARTH_GRAVITY / AccelSensitivity;

    // Calculate frame length for BMI323 FIFO
    static constexpr int8_t frameLength = BMI323_LIB::LENGTH_FIFO_ACCEL + BMI323_LIB::LENGTH_FIFO_GYRO + BMI323_LIB::LENGTH_TEMPERATURE;

    I2CImpl i2c;
    SlimeVR::Logging::Logger &logger;
    int8_t zxFactor;
    BMI323_LIB bmi323Lib;
    // BMI323(I2CImpl i2c, SlimeVR::Logging::Logger &logger): i2c(i2c), logger(logger), zxFactor(0) {}
    BMI323(I2CImpl i2c, SlimeVR::Logging::Logger &logger): i2c(i2c), logger(logger), zxFactor(0), bmi323Lib(&i2cRead, &i2cWrite, &delayUs, &i2c) {}


    struct Regs {
        struct WhoAmI {
            static constexpr uint8_t reg = 0x00;
            static constexpr uint8_t value = 0x00; // 0x43
        };
    };

    static void delayUs(uint32_t period, void *)
    {
        delay(period / 1000);
    }

    static int8_t i2cRead(uint8_t registerAddress, uint8_t *registerData, uint32_t length, void *interfacePointer)
    {
        I2CImpl i2c = *static_cast<I2CImpl*>(interfacePointer);
        i2c.readBytes(registerAddress, length, registerData);
        return 0;
    }

    static int8_t i2cWrite(uint8_t registerAddress, const uint8_t *registerData, uint32_t length, void *interfacePointer)
    {
        I2CImpl i2c = *static_cast<I2CImpl*>(interfacePointer);
        i2c.writeBytes(registerAddress, length, const_cast<uint8_t*>(registerData));
        return 0;
    }

    /**
     * @brief Extracts the next frame from a FIFO frame
     * @note The indexes (accelIndex, gyroIndex, tempIndex, timeIndex) needs to
     * be reset before the first call
    */
    static uint8_t extractFrame(uint8_t *data, uint8_t index, float *accelData, float *gyroData, float *tempData) {
        uint8_t dataValidity = ACCEL_VALID | GYRO_VALID | TEMP_VALID;
        
        // Unpack accelerometer
        uint8_t accelIndex = index * frameLength + BMI323_LIB::DUMMY_BYTE;
        int16_t isValid = (int16_t)((data[accelIndex + 1] << 8) | data[accelIndex]);
        if (isValid == BMI323_LIB::FIFO_ACCEL_DUMMY_FRAME) {
            dataValidity &= ~ACCEL_VALID;
        } else {
            accelData[0] = lsbToMps2(isValid);
            accelData[1] = lsbToMps2((int16_t)((data[accelIndex + 3] << 8) | data[accelIndex + 2]));
            accelData[2] = lsbToMps2((int16_t)((data[accelIndex + 5] << 8) | data[accelIndex + 4]));
            // accelData.sensor_time = (int16_t)((data[accelIndex + 13] << 8) | data[accelIndex + 12]);
        }

        // Unpack gyroscope data
        uint8_t gyroIndex = accelIndex + BMI323_LIB::LENGTH_FIFO_ACCEL;
        isValid = (int16_t)((data[gyroIndex + 1] << 8) | data[gyroIndex]);
        if (isValid == BMI323_LIB::FIFO_GYRO_DUMMY_FRAME) {
            dataValidity &= ~GYRO_VALID;
        } else {
            #if BMI323_USE_TEMP_CAL
                gyroData[0] = isValid;
                gyroData[1] = (int16_t)((data[gyroIndex + 3] << 8) | data[gyroIndex + 2]);
                gyroData[2] = (int16_t)((data[gyroIndex + 5] << 8) | data[gyroIndex + 4]);
            #else
                gyroData[0] = (isValid) * gScaleX;
                gyroData[1] = ((int16_t)((data[gyroIndex + 3] << 8) | data[gyroIndex + 2])) * gScaleY;
                gyroData[2] = ((int16_t)((data[gyroIndex + 5] << 8) | data[gyroIndex + 4])) * gScaleZ;
            #endif
            // gyroData.sensor_time = (int16_t)((data[gyroIndex + 7] << 8) | data[gyroIndex + 6]);
        }

        // Unpack temperature data
        uint8_t tempIndex = gyroIndex + BMI323_LIB::LENGTH_FIFO_GYRO;
        uint16_t isTempValid = (uint16_t)((data[tempIndex + 1] << 8) | data[tempIndex]);
        if (isTempValid == BMI323_LIB::FIFO_TEMP_DUMMY_FRAME) {
            dataValidity &= ~TEMP_VALID;
        } else {
            tempData[0] = ((int16_t)((data[tempIndex + 1] << 8) | data[tempIndex]) / 512.0f) + 23.0f;
        }

        return dataValidity;
    }

    bool initialize()
    {
        logger.info("jojos38 BMI323 firmware V1.4");
        
        int8_t result;

        // Initialize the sensor
        result = bmi323Lib.initI2C();
        if (result == BMI323_LIB::SUCCESS) {
            logger.info("BMI323 Initialized on address 0x%x", Address);
        } else {
            logger.info("BMI323 Initialization failed");
            return false;
        }

        logger.info("Chip ID 0x%x", bmi323Lib.chipId);

        // Apply the calibration data
        /*if (m_calibration.G_G[0] != 0 || m_calibration.G_G[1] != 0 || m_calibration.G_G[2] != 0 || m_calibration.G_O[0] != 0 || m_calibration.G_O[1] != 0 || m_calibration.G_O[2] != 0) {
            m_Logger.info("Calibration data found");
            printCalibrationData();
            result = bmi323.setGyroOffsetGain(m_calibration.G_O, m_calibration.G_G);
            if (result == BMI323::SUCCESS) {
                m_Logger.info("BMI323 Calibration data applied");
            } else {
                m_Logger.error("BMI323 Calibration data apply failed");
                error = true;
            }
        } else {
            m_Logger.warn("No calibration data found, please calibrate the sensor it only takes a few seconds");
            calibrate = true;
        }*/

        // Set gyroscope configuration
        result = bmi323Lib.setGyroConfig(
            BMI323_LIB::GYRO_ODR_200HZ,
            BMI323_LIB::GYRO_BANDWIDTH_ODR_HALF,
            BMI323_LIB::GYRO_MODE_HIGH_PERF,
            BMI323_LIB::GYRO_RANGE_1000DPS,
            BMI323_LIB::GYRO_AVG_2
        );
        if (result == BMI323_LIB::SUCCESS) {
            logger.info("BMI323 Gyroscope configured");
        } else {
            logger.error("BMI323 Gyroscope configuration failed");
            return false;
        }

        // Set accelerometer configuration
        result = bmi323Lib.setAccelConfig(
            BMI323_LIB::ACCEL_ODR_200HZ,
            BMI323_LIB::ACCEL_BANDWIDTH_ODR_HALF,
            BMI323_LIB::ACCEL_MODE_HIGH_PERF,
            BMI323_LIB::ACCEL_RANGE_8G,
            BMI323_LIB::ACCEL_AVG_2
        );
        if (result == BMI323_LIB::SUCCESS) {
            logger.info("BMI323 Accelerometer configured");
        } else {
            logger.error("BMI323 Accelerometer configuration failed");
            return false;
        }

        // Set FIFO configuration
        bmi323Lib.setFifoConfig(BMI323_LIB::FIFO_ACCEL_EN | BMI323_LIB::FIFO_GYRO_EN | BMI323_LIB::FIFO_TEMP_EN, BMI323_LIB::ENABLE);
        if (result == BMI323_LIB::SUCCESS) {
            logger.info("BMI323 FIFO enabled");
        } else {
            logger.error("BMI323 FIFO enable failed");
            return false;
        }

        #if BMI323_USE_BMM350
            // BMM350
            bmm350Address = (this->address == BMI323::ADDRESS_I2C_PRIMARY) ? BMM350::ADDRESS_I2C_PRIMARY : BMM350::ADDRESS_I2C_SECONDARY;
            result = bmm350.initI2C();
            if (result == BMM350::SUCCESS) {
                m_Logger.info(String("BMM350 initialized on address 0x" + String(bmm350Address, HEX)).c_str());
            } else {
                m_Logger.info("BMM350 initialization failed");
                error = true;
            }

            // Configure the sensor
            result = bmm350.seOdrAvg(BMM350::ODR_25HZ, BMM350::AVG_4);
            if (result == BMM350::SUCCESS) {
                m_Logger.info("BMM350 configured");
            } else {
                m_Logger.error("BMM350 configuration failed");
                error = true;
            }

            // Disable interrupt
            result = bmm350.setInterruptEnabled(BMM350::DISABLE);
            if (result == BMM350::SUCCESS) {
                m_Logger.info("BMM350 interrupt disabled");
            } else {
                m_Logger.error("BMM350 interrupt disable failed");
                error = true;
            }

            // Set power mode
            result = bmm350.setPowermode(BMM350::NORMAL_MODE);
            if (result == BMM350::SUCCESS) {
                m_Logger.info("BMM350 power mode set");
            } else {
                m_Logger.error("BMM350 power mode set failed");
                error = true;
            }
        #endif

        #if BMI323_USE_TEMP_CAL
            // allocate temperature memory after calibration because OOM
            gyroTempCalibrator = new GyroTemperatureCalibrator(
                SlimeVR::Configuration::CalibrationConfigType::BMI323,
                sensorId,
                GyroSensitivity,
                (uint32_t)(0.2f / (1.0f / 200.0f))
            );

            gyroTempCalibrator->loadConfig(GyroSensitivity);
            if (gyroTempCalibrator->config.hasCoeffs) {
                gyroTempCalibrator->approximateOffset(m_calibration.temperature, GOxyzStaticTempCompensated);
            }
        #endif

        #if BMI323_USE_SENSCAL
            String localDevice = WiFi.macAddress();
            for (auto const& offsets : sensitivityOffsets) {
                if (!localDevice.equals(offsets.mac)) continue;
                if (offsets.sensorId != sensorId) continue;
                #define BMI323_CALCULATE_SENSITIVTY_MUL(degrees) (1.0 / (1.0 - ((degrees)/(360.0 * offsets.spins))))
                gScaleX = GScale * BMI323_CALCULATE_SENSITIVTY_MUL(offsets.x);
                gScaleY = GScale * BMI323_CALCULATE_SENSITIVTY_MUL(offsets.y);
                gScaleZ = GScale * BMI323_CALCULATE_SENSITIVTY_MUL(offsets.z);
                m_Logger.debug("Custom sensitivity offset enabled: %s %s",
                    offsets.mac,
                    offsets.sensorId == SENSORID_PRIMARY ? "primary" : "aux"
                );
            }
        #endif

        logger.info("BMI323 initialized");

        return true;
    }

    void motionlessCalibration()
    {

    }

    template <typename AccelCall, typename GyroCall>
    void bulkRead(AccelCall &&processAccelSample, GyroCall &&processGyroSample) {
        
    }

    float getDirectTemp() const
    {
        const float temp = 1.0f;

        return temp;
    }

};

} // namespace