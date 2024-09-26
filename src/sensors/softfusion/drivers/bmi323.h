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
    /**
     * Sensor Fusion required variables
     */
    static constexpr uint8_t Address = 0x68; // IMU's i2c address used to communicate
    static constexpr auto Name = "BMI323";
    static constexpr auto Type = ImuID::BMI323;
    
    static constexpr float GyrTs = 1.0/200.0; // Delay between each gyro sample
    static constexpr float AccTs = 1.0/200.0; // Dealy between each accel sample
    static constexpr float MagTs = 1.0/25.0; // Delay between each mag sample

    static constexpr float GyroSensitivity = 32.768f;
    static constexpr float AccelSensitivity = 4096.0f;
    static constexpr float GScale = ((32768. / GyroSensitivity) / 32768.) * (PI / 180.0);
    static constexpr float AScale = CONST_EARTH_GRAVITY / AccelSensitivity;

    // FIFO variables
    float latestTemperature = 0;
    static constexpr int8_t FifoFrameLength = BMI323_LIB::LENGTH_FIFO_ACCEL + BMI323_LIB::LENGTH_FIFO_GYRO + BMI323_LIB::LENGTH_TEMPERATURE;
    static const uint8_t ACCEL_VALID = 0x01;   // Bit 0 represents accelerometer data validity
    static const uint8_t GYRO_VALID = 0x02;    // Bit 1 represents gyroscope data validity
    static const uint8_t TEMP_VALID = 0x04;    // Bit 2 represents temperature data validity

    // Intialization and variables
    I2CImpl i2c;
    SlimeVR::Logging::Logger &logger;
    BMI323_LIB bmi323Lib;
    BMI323(I2CImpl i2c, SlimeVR::Logging::Logger &logger): i2c(i2c), logger(logger), bmi323Lib(&i2cRead, &i2cWrite, &delayUs, &i2c) {}

    /**
     * Calibration data for the basic static calibration
     * Detected by sfusion and saved when values are put in
     * Anything can be put in this struct
     */
    struct MotionlessCalibrationData
    {
        bool valid;

        // accelerometer offsets and gain
        uint16_t G_O[3];
        uint8_t G_G[3];
    };

    struct Regs {
        /**
         * The first value (reg) defines which register address the chip ID is located at
         * The second value (value) defines the expected value
         * This is used to make sure we have the correct IMU
         * BUT... It doesn't work on this IMU so I bypassed it
         */
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
        Wire.beginTransmission(Address);
        Wire.write(registerAddress);
        Wire.endTransmission();
        Wire.requestFrom(Address, length);
        for (auto i = 0u; i < length; i++) {
            registerData[i] = Wire.read();
        }
        return 0;
        /*I2CImpl* i2c = static_cast<I2CImpl*>(interfacePointer);;
        i2c->readBytes(registerAddress, length + 2, registerData);
        return 0;*/
    }

    static int8_t i2cWrite(uint8_t registerAddress, const uint8_t *registerData, uint32_t length, void *interfacePointer)
    {
        Wire.beginTransmission(Address);
        Wire.write(registerAddress);
        for (auto i = 0u; i < length; i++) {
            Wire.write(registerData[i]);
        }
        Wire.endTransmission();
        return 0;
        /*I2CImpl* i2c = static_cast<I2CImpl*>(interfacePointer);
        i2c->writeBytes(registerAddress, length + 2, const_cast<uint8_t*>(registerData));
        return 0;*/
    }

    /**
     * @brief Print calibration data
     */
    void printCalibrationData(MotionlessCalibrationData &calibrationData) {
        if (!calibrationData.valid) {
            logger.debug("No calibration data");
        } else {
            logger.debug("Gyro offset: %u, %u, %u", calibrationData.G_O[0], calibrationData.G_O[1], calibrationData.G_O[2]);
            logger.debug("Gyro gain: %u, %u, %u", calibrationData.G_G[0], calibrationData.G_G[1], calibrationData.G_G[2]);
        }
   }

    /**
     * @brief Extracts the next frame from a FIFO frame
     * @note The indexes (accelIndex, gyroIndex, tempIndex, timeIndex) needs to
     * be reset before the first call
    */
    static uint8_t extractFrame(uint8_t *data, uint8_t index, int16_t *accelData, int16_t *gyroData, float *tempData) {
        // Set dataValidity to all valid
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
            gyroData[0] = (isValid);
            gyroData[1] = ((int16_t)((data[gyroIndex + 3] << 8) | data[gyroIndex + 2]));
            gyroData[2] = ((int16_t)((data[gyroIndex + 5] << 8) | data[gyroIndex + 4]));
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

    bool initialize(MotionlessCalibrationData &calibrationData)
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
        if (calibrationData.valid) {
            logger.info("Calibration data found");
            printCalibrationData(calibrationData);
            result = bmi323Lib.setGyroOffsetGain(calibrationData.G_O, calibrationData.G_G);
            if (result == BMI323_LIB::SUCCESS) {
                logger.info("BMI323 Calibration data applied");
            } else {
                logger.error("BMI323 Calibration data apply failed");
                return false;
            }
        } else {
            logger.warn("No calibration data found, please calibrate the sensor");
        }

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

        logger.info("BMI323 initialized");

        return true;
    }

    void motionlessCalibration(MotionlessCalibrationData &calibrationData)
    {
        // Calibrate sensitivity (ALWAYS FIRST)
        logger.info("Calibrating gyroscope sensitivity in 5 seconds... Please do not move the device");
        delay(5000);
        ledManager.on();
        struct BMI323_LIB::SelfCalibResult calibSensitivityResult;
        uint8_t result = bmi323Lib.performGyroCalibration(BMI323_LIB::CALIBRATION_SENSITIVITY, BMI323_LIB::CALIBRATION_APPLY_TRUE, &calibSensitivityResult);
        if (result == BMI323_LIB::SUCCESS) {
            logger.info("Gyroscope sensitivity calibration succeed");
        } else {
            logger.error("Gyroscope sensitivity calibration failed");
            return;
        }

        // Calibrate offset
        logger.info("Calibrating gyroscope offset... Please do not move the device");
        struct BMI323_LIB::SelfCalibResult calibOffsetResult;
        result = bmi323Lib.performGyroCalibration(BMI323_LIB::CALIBRATION_OFFSET, BMI323_LIB::CALIBRATION_APPLY_TRUE, &calibOffsetResult);
        if (result == BMI323_LIB::SUCCESS) {
            logger.info("Gyroscope offset calibration succeed");
        } else {
            logger.error("Gyroscope offset calibration failed");
            return;
        }

        // Save results
        uint16_t offset[3];
        uint8_t gain[3];
        bmi323Lib.getGyroOffsetGain(offset, gain);

        // Save the calibration data
        logger.info("Saving calibration data");
        calibrationData.G_O[0] = offset[0];
        calibrationData.G_O[1] = offset[1];
        calibrationData.G_O[2] = offset[2];

        calibrationData.G_G[0] = gain[0];
        calibrationData.G_G[1] = gain[1];
        calibrationData.G_G[2] = gain[2];

        calibrationData.valid = true;

        printCalibrationData(calibrationData);

        logger.info("Motionless calibration done");
        ledManager.off();
    }

    template <typename AccelCall, typename GyroCall>
    void bulkRead(AccelCall &&processAccelSample, GyroCall &&processGyroSample) {
        // Get available data length in the FIFO
        uint16_t availableFifoLength;
        bmi323Lib.getFifoLength(&availableFifoLength);

        // Check for FIFO overflow
        /*if (availableFifoLength >= 127) {
            logger.error("FIFO OVERFLOW");
        }*/

        // If there is enough data in the FIFO to get at least a single frame
        if (availableFifoLength >= FifoFrameLength) {  

            // Variable to store FIFO data
            uint8_t fifoData[I2C_BUFFER_LENGTH] = { 0 };

            // Make sure the length that we read is a multiple of the frame length
            uint16_t fifoReadLength = availableFifoLength = std::min(availableFifoLength, static_cast<uint16_t>(I2C_BUFFER_LENGTH - BMI323_LIB::DUMMY_BYTE)) / FifoFrameLength * FifoFrameLength + BMI323_LIB::DUMMY_BYTE;
            
            // Read the FIFO data
            int8_t result = bmi323Lib.readFifoData(fifoData, fifoReadLength);

            if (result == BMI323_LIB::SUCCESS) {
                // Count the number of frames in the FIFO
                const uint8_t frameCount = fifoReadLength / FifoFrameLength;
                // Unpack each frame one by one
                for (int i = 0; i < frameCount; i++) {
                    float temperature = 0;
                    int16_t accelData[3] = { 0 };
                    int16_t gyroData[3] = { 0 };
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
                        latestTemperature = temperature;
                    }
                }
            } else {
                logger.error("FIFO data read failed");
            }
        }
    }

    float getDirectTemp() const
    {
        return latestTemperature;
    }

};

} // namespace