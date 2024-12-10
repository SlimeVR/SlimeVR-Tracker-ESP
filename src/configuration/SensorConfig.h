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

namespace SlimeVR {
namespace Configuration {
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
	ImuID ImuType;
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
	BNO0XX
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
	} data;
};

uint16_t configDataToNumber(SensorConfig* sensorConfig, bool magSupported);
}  // namespace Configuration
}  // namespace SlimeVR

#endif
