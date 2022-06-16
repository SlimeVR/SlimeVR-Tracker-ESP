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
        struct BMI160CalibrationConfig {
            // accelerometer offsets and correction matrix
            float A_B[3];
            float A_Ainv[3][3];

            // raw offsets, determined from gyro at rest
            float G_off[3];

            // calibration temperature for dynamic compensation
            float temperature;
        };

        struct MPU6050CalibrationConfig {
            // accelerometer offsets and correction matrix
            float A_B[3];

            // raw offsets, determined from gyro at rest
            float G_off[3];
        };

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

        struct ICM20948CalibrationConfig {
            // gyroscope bias
            int32_t G[3];

            // accelerometer bias
            int32_t A[3];

            // compass bias
            int32_t C[3];
        };

        enum CalibrationConfigType { NONE, BMI160, MPU6050, MPU9250, ICM20948 };

        const char* calibrationConfigTypeToString(CalibrationConfigType type);

        struct CalibrationConfig {
            CalibrationConfigType type;

            union {
                BMI160CalibrationConfig bmi160;
                MPU6050CalibrationConfig mpu6050;
                MPU9250CalibrationConfig mpu9250;
                ICM20948CalibrationConfig icm20948;
            } data;
        };
    }
}

#endif
