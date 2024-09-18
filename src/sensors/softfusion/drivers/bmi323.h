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

    static constexpr float GyrTs = 1.0/50.0; // Delay between each gyro sample
    static constexpr float AccTs = 1.0/50.0; // Dealy between each accel sample
    static constexpr float MagTs = 1.0/25.0; // Delay between each mag sample

    // BMI323 specific constants
    static constexpr float GyroSensitivity = 32.768f;
    static constexpr float AccelSensitivity = 4096.0f;
    static constexpr float GScale = ((32768. / GyroSensitivity) / 32768.) * (PI / 180.0);
    static constexpr float AScale = CONST_EARTH_GRAVITY / AccelSensitivity;

    // FIFO variables
    static constexpr int8_t FifoFrameLength = BMI323_LIB::LENGTH_FIFO_ACCEL + BMI323_LIB::LENGTH_FIFO_GYRO + BMI323_LIB::LENGTH_TEMPERATURE;
    uint8_t fifoData[I2C_BUFFER_LENGTH] = { 0 }; // 2048 is the maximum size of the FIFO
    int16_t accelData[3] = { 0 };
    int16_t gyroData[3] = { 0 };
    float temperatureData = 0;
    int16_t magData[3] = { 0 };

    // FIFO Data validity bytes
    static const uint8_t ACCEL_VALID = 0x01;   // Bit 0 represents accelerometer data validity
    static const uint8_t GYRO_VALID = 0x02;    // Bit 1 represents gyroscope data validity
    static const uint8_t TEMP_VALID = 0x04;    // Bit 2 represents temperature data validity

    I2CImpl i2c;
    SlimeVR::Logging::Logger &logger;
    BMI323_LIB bmi323Lib;
    // BMI323(I2CImpl i2c, SlimeVR::Logging::Logger &logger): i2c(i2c), logger(logger), zxFactor(0) {}
    BMI323(I2CImpl i2c, SlimeVR::Logging::Logger &logger): i2c(i2c), logger(logger), bmi323Lib(&i2cRead, &i2cWrite, &delayUs, &i2c) {}


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
        uint8_t address = 0x68;

        // Read data from the sensor
        Wire.beginTransmission(address);
        Wire.write(registerAddress);
        Wire.endTransmission();
        Wire.requestFrom(address, length);
        for (auto i = 0u; i < length; i++) {
            registerData[i] = Wire.read();
        }

        return 0;

        /*I2CImpl i2c = *static_cast<I2CImpl*>(interfacePointer);
        i2c.readBytes(registerAddress, length, registerData);
        return 0;*/
    }

    static int8_t i2cWrite(uint8_t registerAddress, const uint8_t *registerData, uint32_t length, void *interfacePointer)
    {
        uint8_t dev_addr = 0x68;

        // Write data to the sensor
        Wire.beginTransmission(dev_addr);
        Wire.write(registerAddress);
        for (auto i = 0u; i < length; i++) {
            Wire.write(registerData[i]);
        }
        Wire.endTransmission();
        
        return 0;

        /*I2CImpl i2c = *static_cast<I2CImpl*>(interfacePointer);
        i2c.writeBytes(registerAddress, length, const_cast<uint8_t*>(registerData));
        return 0;*/
    }

    /**
     * @brief Extracts the next frame from a FIFO frame
     * @note The indexes (accelIndex, gyroIndex, tempIndex, timeIndex) needs to
     * be reset before the first call
    */
    static uint8_t extractFrame(uint8_t *data, uint8_t index, int16_t *accelData, int16_t *gyroData, float *tempData) {
        uint8_t dataValidity = ACCEL_VALID | GYRO_VALID | TEMP_VALID;
        
        // Unpack accelerometer
        uint8_t accelIndex = index * FifoFrameLength + BMI323_LIB::DUMMY_BYTE;
        int16_t isValid = (int16_t)((data[accelIndex + 1] << 8) | data[accelIndex]);
        if (isValid == BMI323_LIB::FIFO_ACCEL_DUMMY_FRAME) {
            dataValidity &= ~ACCEL_VALID;
        } else {
            accelData[0] = isValid;
            accelData[1] = (int16_t)((data[accelIndex + 3] << 8) | data[accelIndex + 2]);
            accelData[2] = (int16_t)((data[accelIndex + 5] << 8) | data[accelIndex + 4]);
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
                gyroData[0] = (isValid);
                gyroData[1] = ((int16_t)((data[gyroIndex + 3] << 8) | data[gyroIndex + 2]));
                gyroData[2] = ((int16_t)((data[gyroIndex + 5] << 8) | data[gyroIndex + 4]));
            #endif
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
            BMI323_LIB::GYRO_ODR_50HZ,
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
            BMI323_LIB::ACCEL_ODR_50HZ,
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
        // Get available data length in the FIFO
        uint16_t availableFifoLength;
        bmi323Lib.getFifoLength(&availableFifoLength);

        // Check for FIFO overflow
        if (availableFifoLength >= 127) {
            logger.error("FIFO OVERFLOW");
        }

        // If there is enough data in the FIFO to get at least a single frame
        if (availableFifoLength >= FifoFrameLength) {        
            // Make sure the length that we read is a multiple of the frame length
            uint16_t fifoReadLength = availableFifoLength = std::min(availableFifoLength, static_cast<uint16_t>(I2C_BUFFER_LENGTH - BMI323_LIB::DUMMY_BYTE)) / FifoFrameLength * FifoFrameLength + BMI323_LIB::DUMMY_BYTE;
            int8_t result = bmi323Lib.readFifoData(fifoData, fifoReadLength);
            if (result == BMI323_LIB::SUCCESS) {
                // Feed sensor fusion
                const uint8_t frameCount = fifoReadLength / FifoFrameLength;
                for (int i = 0; i < frameCount; i++) {
                    float temperature = 0;
                    uint8_t dataValidity = extractFrame(fifoData, i, accelData, gyroData, &temperature);

                    // If the accel data is valid
                    if (dataValidity & ACCEL_VALID) {
                        processAccelSample(accelData, AccTs);
                    }

                    // If the gyro data is valid
                    if (dataValidity & GYRO_VALID) {
                        processGyroSample(gyroData, GyrTs);
                    }

                    // If the temp data is valid
                    if (dataValidity & TEMP_VALID) {
                        temperatureData = temperature;
                    }
                }
            } else {
                logger.error("FIFO data read failed");
            }
        }
    }

    float getDirectTemp() const
    {
        return temperatureData;
    }

};

} // namespace