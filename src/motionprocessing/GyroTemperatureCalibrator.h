/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 SlimeVR Contributors

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

#ifndef GYRO_TEMPERATURE_CALIBRATOR_H
#define GYRO_TEMPERATURE_CALIBRATOR_H

#include <Arduino.h>
#include <stdint.h>

#include "../configuration/SensorConfig.h"
#include "../logging/Logger.h"
#include "OnlinePolyfit.h"
#include "debug.h"

// Degrees C
// default: 15.0f
#define TEMP_CALIBRATION_MIN 15.0f

// Degrees C
// default: 45.0f
#define TEMP_CALIBRATION_MAX 45.0f

// Snap calibration to every 1/2 of degree: 20.00, 20.50, 21.00, etc
// default: 0.5f
#define TEMP_CALIBRATION_STEP 0.5f

// Record debug samples if current temperature is off by no more than this value;
// if snapping point is 20.00 - samples will be recorded in range of 19.80 - 20.20
// default: 0.2f
#define TEMP_CALIBRATION_MAX_DEVIATION_FROM_STEP 0.2f

// How long to average gyro samples for before saving a data point
// default: 0.2f
#define TEMP_CALIBRATION_SECONDS_PER_STEP 0.2f

#if IMU == IMU_ICM20948
// 16 bit 333 lsb/K, ~0.00508 degrees per bit
// already calibrated by DMP?
#elif IMU == IMU_MPU6500 || IMU == IMU_MPU6050
// 16 bit 340 lsb/K, ~0.00518 degrees per bit
#elif IMU == IMU_MPU9250
// 16 bit 333 lsb/K, ~0.00508 degrees per bit
#elif IMU == IMU_BMI160
// 16 bit 128 lsb/K, ~0.00195 degrees per bit
#endif

constexpr uint16_t TEMP_CALIBRATION_BUFFER_SIZE
	= (uint16_t)((TEMP_CALIBRATION_MAX - TEMP_CALIBRATION_MIN)
				 * (1 / TEMP_CALIBRATION_STEP));

#define TEMP_CALIBRATION_TEMP_TO_IDX(temperature)                                  \
	(uint16_t)(                                                                    \
		(temperature + TEMP_CALIBRATION_STEP / 2.0f) * (1 / TEMP_CALIBRATION_STEP) \
		- TEMP_CALIBRATION_MIN * (1 / TEMP_CALIBRATION_STEP)                       \
	)

#define TEMP_CALIBRATION_IDX_TO_TEMP(idx) \
	(float)(((float)idx / (1.0f / TEMP_CALIBRATION_STEP)) + TEMP_CALIBRATION_MIN)

struct GyroTemperatureCalibrationState {
	uint16_t temperatureCurrentIdx;
	uint32_t numSamples;
	float tSum;
	int32_t xSum;
	int32_t ySum;
	int32_t zSum;

	GyroTemperatureCalibrationState()
		: temperatureCurrentIdx(-1)
		, numSamples(0)
		, tSum(0.0f)
		, xSum(0)
		, ySum(0)
		, zSum(0){};
};

struct GyroTemperatureOffsetSample {
	float t;
	float x;
	float y;
	float z;

	GyroTemperatureOffsetSample()
		: t(0.0f)
		, x(0)
		, y(0)
		, z(0) {}
};

struct GyroTemperatureCalibrationConfig {
	SlimeVR::Configuration::SensorConfigType type;

	float sensitivityLSB;
	float minTemperatureRange;
	float maxTemperatureRange;
	uint16_t minCalibratedIdx = 0;
	uint16_t maxCalibratedIdx = 0;
	GyroTemperatureOffsetSample samples[TEMP_CALIBRATION_BUFFER_SIZE];
	uint16_t samplesTotal = 0;
	float cx[4] = {0.0};
	float cy[4] = {0.0};
	float cz[4] = {0.0};
	bool hasCoeffs = false;

	GyroTemperatureCalibrationConfig(
		SlimeVR::Configuration::SensorConfigType _type,
		float _sensitivityLSB
	)
		: type(_type)
		, sensitivityLSB(_sensitivityLSB)
		, minTemperatureRange(1000)
		, maxTemperatureRange(-1000) {}

	bool hasData() { return minTemperatureRange != 1000; }

	bool fullyCalibrated() {
		return samplesTotal >= TEMP_CALIBRATION_BUFFER_SIZE && hasCoeffs;
	}

	float getCalibrationDonePercent() {
		return (float)samplesTotal / TEMP_CALIBRATION_BUFFER_SIZE * 100.0f;
	}

	void rescaleSamples(float newSensitivityLSB) {
		if (sensitivityLSB == newSensitivityLSB) {
			return;
		}
		float mul = newSensitivityLSB / sensitivityLSB;
		for (int i = 0; i < TEMP_CALIBRATION_BUFFER_SIZE; i++) {
			if (samples[i].t == 0) {
				continue;
			}
			samples[i].x *= mul;
			samples[i].y *= mul;
			samples[i].z *= mul;
		}
		sensitivityLSB = newSensitivityLSB;
	}

	void reset() {
		minTemperatureRange = 1000;
		maxTemperatureRange = -1000;
		samplesTotal = 0;
		for (int i = 0; i < TEMP_CALIBRATION_BUFFER_SIZE; i++) {
			samples[i].t = 0;
			samples[i].x = 0;
			samples[i].y = 0;
			samples[i].z = 0;
		}
		hasCoeffs = false;
	}
};

class GyroTemperatureCalibrator {
public:
	uint8_t sensorId;
	GyroTemperatureCalibrationConfig config;

	// set when config is fully calibrated is saved OR on startup when loaded config is
	// fully calibrated; left unset when sending saving command over serial so it can
	// continue calibration and autosave later
	bool configSaved = false;
	bool configSaveFailed = false;

	GyroTemperatureCalibrator(
		SlimeVR::Configuration::SensorConfigType _configType,
		uint8_t _sensorId,
		float sensitivity,
		uint32_t _samplesPerStep
	)
		: sensorId(_sensorId)
		, config(_configType, sensitivity)
		, samplesPerStep(_samplesPerStep)
		, m_Logger(SlimeVR::Logging::Logger("GyroTemperatureCalibrator")) {
		char buf[4];
		sprintf(buf, "%u", _sensorId);
		m_Logger.setTag(buf);
	}

	void updateGyroTemperatureCalibration(
		const float temperature,
		const bool restDetected,
		int16_t x,
		int16_t y,
		int16_t z
	);
	bool approximateOffset(const float temperature, float GOxyz[3]);
	bool loadConfig(float newSensitivity);
	bool saveConfig();

	void reset() {
		config.reset();
		configSaved = false;
		configSaveFailed = false;
	}

	bool isCalibrating() { return calibrationRunning; }

private:
	GyroTemperatureCalibrationState state;
	uint32_t samplesPerStep;
	SlimeVR::Logging::Logger m_Logger;

	float lastApproximatedTemperature = 0.0f;
	float lastApproximatedOffsets[3];

	bool calibrationRunning = false;
	OnlineVectorPolyfit<3, 3, (uint64_t)1e9> poly;
	float bst = 0.0f;
	int32_t bsx = 0;
	int32_t bsy = 0;
	int32_t bsz = 0;
	int32_t bn = 0;
	float lastTemp = 0;

	void resetCurrentTemperatureState();
};

#endif
