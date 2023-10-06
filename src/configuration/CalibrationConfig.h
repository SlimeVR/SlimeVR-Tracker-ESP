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

#ifndef SLIMEVR_CONFIGURATION_CALIBRATIONCONFIG_H
#define SLIMEVR_CONFIGURATION_CALIBRATIONCONFIG_H

#include <stdint.h>

namespace SlimeVR {
    namespace Configuration {
        //Only changed when the config setup is changed, not when a sensor config is updated
        constexpr uint32_t CURRENT_CONFIGURATION_VERSION = 3; 
        struct NoneCalibrationConfig {
            uint8_t file_size[190]; //200 bytes
        };
        

        constexpr uint32_t BMI160CalibrationLatestVersion = 2;
        struct BMI160CalibrationConfig {
            // accelerometer offsets and correction matrix
            float A_B[3];
            float A_Ainv[3][3];

            // magnetometer offsets and correction matrix
            float M_B[3];
            float M_Ainv[3][3];

            // raw offsets, determined from gyro at rest
            float G_off[3];
            float G_Sens[3];

            // calibration temperature for dynamic compensation
            float temperature;
        };

        constexpr uint32_t MPU6050CalibrationLatestVersion = 1;
        struct MPU6050CalibrationConfig {
            // accelerometer offsets and correction matrix
            float A_B[3];

            // raw offsets, determined from gyro at rest
            float G_off[3];
        };

        constexpr uint32_t MPU9250CalibrationLatestVersion = 1;
        struct MPU9250CalibrationConfig {
            // accelerometer offsets and correction matrix
            float A_B[3];
            float A_Ainv[3][3];

            // magnetometer offsets and correction matrix
            float M_B[3];
            float M_Ainv[3][3];

            // raw offsets, determined from gyro at rest
            float G_off[3];
        };

        constexpr uint32_t ICM20948CalibrationLatestVersion = 1;
        struct ICM20948CalibrationConfig {
            // gyroscope bias
            int32_t G[3];

            // accelerometer bias
            int32_t A[3];

            // compass bias
            int32_t C[3];
        };

        constexpr uint32_t ICM42688CalibrationLatestVersion = 1;
        struct ICM42688CalibrationConfig {
            // accelerometer offsets and correction matrix
            float A_B[3];
            float A_Ainv[3][3];

            // magnetometer offsets and correction matrix
            float M_B[3];
            float M_Ainv[3][3];

            // raw offsets, determined from gyro at rest
            float G_off[3];
        };

        enum CalibrationConfigType { NONE, BMI160, MPU6050, MPU9250, ICM20948, ICM42688 };

        const char* calibrationConfigTypeToString(CalibrationConfigType type);

        struct CalibrationConfig {
            CalibrationConfigType type;
            uint32_t version;

            union {
                NoneCalibrationConfig none;
                BMI160CalibrationConfig bmi160;
                MPU6050CalibrationConfig mpu6050;
                MPU9250CalibrationConfig mpu9250;
                ICM20948CalibrationConfig icm20948;
                ICM42688CalibrationConfig icm42688;
            } data;
        };

        static_assert(sizeof(NoneCalibrationConfig) >= sizeof(BMI160CalibrationConfig), "The calibration file_size needs to be increased");
        static_assert(sizeof(NoneCalibrationConfig) >= sizeof(MPU6050CalibrationConfig), "The calibration file_size needs to be increased");
        static_assert(sizeof(NoneCalibrationConfig) >= sizeof(MPU9250CalibrationConfig), "The calibration file_size needs to be increased");
        static_assert(sizeof(NoneCalibrationConfig) >= sizeof(ICM20948CalibrationConfig), "The calibration file_size needs to be increased");
        static_assert(sizeof(NoneCalibrationConfig) >= sizeof(ICM42688CalibrationConfig), "The calibration file_size needs to be increased");
    }
}

#endif
