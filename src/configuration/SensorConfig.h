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

#ifndef SLIMEVR_CONFIGURATION_SENSORCONFIG_H
#define SLIMEVR_CONFIGURATION_SENSORCONFIG_H

#include <stdint.h>

#include "consts.h"

namespace SlimeVR::Configuration {
struct BMI160SensorConfig {
	// accelerometer offsets and correction matrix
	float A_B[3];
	float A_Ainv[3][3];

	// magnetometer offsets and correction matrix
	float M_B[3];
	float M_Ainv[3][3];

	// raw offsets, determined from gyro at rest
	float G_off[3];

	// calibration temperature for dynamic compensation
	float temperature;
};

struct SoftFusionSensorConfig {
	SensorTypeID ImuType;
	uint16_t MotionlessDataLen;

	// accelerometer offsets and correction matrix
	float A_B[3];
	float A_Ainv[3][3];

	// magnetometer offsets and correction matrix
	float M_B[3];
	float M_Ainv[3][3];

	// raw offsets, determined from gyro at rest
	float G_off[3];

	// calibration temperature for dynamic compensation
	float temperature;

	// real measured sensor sampling rate
	float A_Ts;
	float G_Ts;
	float M_Ts;

	// gyro sensitivity multiplier
	float G_Sens[3];

	uint8_t MotionlessData[60];

	// temperature sampling rate (placed at the end to not break existing configs)
	float T_Ts;
};

struct RuntimeCalibrationSensorConfig {
	SensorTypeID ImuType;
	uint16_t MotionlessDataLen;

	bool sensorTimestepsCalibrated;
	float A_Ts;
	float G_Ts;
	float M_Ts;
	float T_Ts;

	bool motionlessCalibrated;
	uint8_t MotionlessData[60];

	uint8_t gyroPointsCalibrated;
	float gyroMeasurementTemperature1;
	float G_off1[3];
	float gyroMeasurementTemperature2;
	float G_off2[3];

	bool accelCalibrated[3];
	float A_off[3];
};

struct MPU6050SensorConfig {
	// accelerometer offsets and correction matrix
	float A_B[3];

	// raw offsets, determined from gyro at rest
	float G_off[3];
};

struct MPU9250SensorConfig {
	// accelerometer offsets and correction matrix
	float A_B[3];
	float A_Ainv[3][3];

	// magnetometer offsets and correction matrix
	float M_B[3];
	float M_Ainv[3][3];

	// raw offsets, determined from gyro at rest
	float G_off[3];
};

struct ICM20948SensorConfig {
	// gyroscope bias
	int32_t G[3];

	// accelerometer bias
	int32_t A[3];

	// compass bias
	int32_t C[3];
};

struct ICM42688SensorConfig {
	// accelerometer offsets and correction matrix
	float A_B[3];
	float A_Ainv[3][3];

	// magnetometer offsets and correction matrix
	float M_B[3];
	float M_Ainv[3][3];

	// raw offsets, determined from gyro at rest
	float G_off[3];
};

struct BNO0XXSensorConfig {
	bool magEnabled;
};

enum class SensorConfigType {
	NONE,
	BMI160,
	MPU6050,
	MPU9250,
	ICM20948,
	SFUSION,
	BNO0XX,
	RUNTIME_CALIBRATION,
};

const char* calibrationConfigTypeToString(SensorConfigType type);

struct SensorConfig {
	SensorConfigType type;

	union {
		BMI160SensorConfig bmi160;
		SoftFusionSensorConfig sfusion;
		MPU6050SensorConfig mpu6050;
		MPU9250SensorConfig mpu9250;
		ICM20948SensorConfig icm20948;
		BNO0XXSensorConfig bno0XX;
		RuntimeCalibrationSensorConfig runtimeCalibration;
	} data;
};

struct SensorConfigBits {
	bool magEnabled : 1;
	bool magSupported : 1;
	bool calibrationEnabled : 1;
	bool calibrationSupported : 1;
	bool tempGradientCalibrationEnabled : 1;
	bool tempGradientCalibrationSupported : 1;

	// Remove if the above fields exceed a byte, necessary to make the struct 16
	// bit
	uint8_t padding;

	bool operator==(const SensorConfigBits& rhs) const;
	bool operator!=(const SensorConfigBits& rhs) const;
};

// If this fails, you forgot to do the above
static_assert(sizeof(SensorConfigBits) == 2);

}  // namespace SlimeVR::Configuration

#endif
