/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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

#include <vector3.h>

#include <cstdint>
#include <functional>
#include <optional>

#include "../../GlobalVars.h"
#include "../../configuration/Configuration.h"
#include "../../consts.h"
#include "../SensorFusionRestDetect.h"

namespace SlimeVR::Sensors {

template <typename IMU>
class NonBlockingCalibrator {
public:
	NonBlockingCalibrator(
		SensorFusionRestDetect& fusion,
		float accelScale,
		IMU& imu,
		uint8_t sensorId
	)
		: m_fusion{fusion}
		, accelScale{accelScale}
		, imu{imu}
		, sensorId{sensorId} {}

	void setup(Configuration::NonBlockingCalibrationConfig& baseCalibrationConfig) {
		calibrationConfig = baseCalibrationConfig;
		startupMillis = millis();

		printCalibration();

		if (!calibrationConfig.sensorTimestepsCalibrated) {
			nextCalibrationStep = CalibrationStep::SAMPLING_RATE;
		} else if (!calibrationConfig.motionlessCalibrated && HasMotionlessCalib) {
			nextCalibrationStep = CalibrationStep::MOTIONLESS;
		} else if (!calibrationConfig.gyroCalibrated) {
			nextCalibrationStep = CalibrationStep::GYRO_BIAS;
		} else if (!calibrationConfig.accelCalibrated) {
			nextCalibrationStep = CalibrationStep::ACCEL_BIAS;
		} else {
			nextCalibrationStep = CalibrationStep::NONE;
		}
	}

	void tick() {
		if (nextCalibrationStep == CalibrationStep::NONE) {
			return;
		}

		if (millis() - startupMillis < initialStartupDelaySeconds * 1e3) {
			return;
		}

		if (!m_fusion.getRestDetected() && requiresRestForNextStep()) {
			if (isCalibrating) {
				cancelCalibration();
			}

			return;
		}

		switch (nextCalibrationStep) {
			case CalibrationStep::NONE:
				break;
			case CalibrationStep::SAMPLING_RATE:
				tickSamplingRateCalibration();
				break;
			case CalibrationStep::MOTIONLESS:
				tickMotionlessCalibration();
				break;
			case CalibrationStep::GYRO_BIAS:
				tickGyroBiasCalibration();
				break;
			case CalibrationStep::ACCEL_BIAS:
				tickAccelBiasCalibration();
				break;
		}
	}

	void provideAccelSample(const int16_t accelSample[3]) {
		if (accelSampleHandler.has_value()) {
			accelSampleHandler.value()(accelSample);
		}
	}

	void provideGyroSample(const int16_t gyroSample[3]) {
		if (gyroSampleHandler.has_value()) {
			gyroSampleHandler.value()(gyroSample);
		}
	}

	void provideTempSample(float tempSample) {
		if (tempSampleHandler.has_value()) {
			tempSampleHandler.value()(tempSample);
		}
	}

private:
	enum class CalibrationStep {
		NONE,
		SAMPLING_RATE,
		MOTIONLESS,
		GYRO_BIAS,
		ACCEL_BIAS,
	};

	void cancelCalibration() {
		switch (nextCalibrationStep) {
			case CalibrationStep::NONE:
				break;
			case CalibrationStep::SAMPLING_RATE:
				samplingRateCalibrationData.reset();
				break;
			case CalibrationStep::MOTIONLESS:
				motionlessCalibrationData.reset();
				break;
			case CalibrationStep::GYRO_BIAS:
				gyroBiasCalibrationData.reset();
				break;
			case CalibrationStep::ACCEL_BIAS:
				accelBiasCalibrationData.reset();
				break;
		}

		accelSampleHandler.reset();
		gyroSampleHandler.reset();
		tempSampleHandler.reset();

		isCalibrating = false;
	}

	bool requiresRestForNextStep() {
		switch (nextCalibrationStep) {
			case CalibrationStep::NONE:
				return false;
			case CalibrationStep::SAMPLING_RATE:
				return false;
			case CalibrationStep::MOTIONLESS:
				return true;
			case CalibrationStep::GYRO_BIAS:
				return true;
			case CalibrationStep::ACCEL_BIAS:
				return true;
		}

		return false;
	}

	void stepCalibrationForward(bool save = true) {
		switch (nextCalibrationStep) {
			case CalibrationStep::NONE:
				return;
			case CalibrationStep::SAMPLING_RATE:
				samplingRateCalibrationData.reset();

				nextCalibrationStep = CalibrationStep::MOTIONLESS;
				break;
			case CalibrationStep::MOTIONLESS:
				motionlessCalibrationData.reset();

				nextCalibrationStep = CalibrationStep::GYRO_BIAS;
				break;
			case CalibrationStep::GYRO_BIAS:
				gyroBiasCalibrationData.reset();

				nextCalibrationStep = CalibrationStep::ACCEL_BIAS;
				break;
			case CalibrationStep::ACCEL_BIAS:
				accelBiasCalibrationData.reset();

				nextCalibrationStep = CalibrationStep::NONE;
				break;
		}

		accelSampleHandler.reset();
		gyroSampleHandler.reset();
		tempSampleHandler.reset();

		isCalibrating = false;

		if (save) {
			saveCalibration();
		}
	}

	void saveCalibration() {
		SlimeVR::Configuration::CalibrationConfig calibration;
		calibration.type = SlimeVR::Configuration::CalibrationConfigType::NONBLOCKING;
		calibration.data.nonblocking = calibrationConfig;
		configuration.setCalibration(sensorId, calibration);
		configuration.save();

		printf("New calibration saved!\n");
		printCalibration();
	}

	void printCalibration() {
		printf("Current calibration:\n");
		if (calibrationConfig.sensorTimestepsCalibrated) {
			printf(
				"\tCalibrated timesteps: Accel %f, Gyro %f, Temperature %f\n",
				calibrationConfig.A_Ts,
				calibrationConfig.G_Ts,
				calibrationConfig.T_Ts
			);
		} else {
			printf("\tSensor timesteps not calibrated\n");
		}

		if constexpr (HasMotionlessCalib) {
			if (calibrationConfig.motionlessCalibrated) {
				printf("\tMotionless calibration done\n");
			} else {
				printf("\tMotionless calibration not done\n");
			}
		}

		if (calibrationConfig.gyroCalibrated) {
			printf(
				"\tCalibrated gyro bias: %f %f %f\n",
				calibrationConfig.G_off[0],
				calibrationConfig.G_off[1],
				calibrationConfig.G_off[2]
			);
		} else {
			printf("\tGyro bias not calibrated\n");
		}

		if (calibrationConfig.accelCalibrated) {
			printf(
				"\tCalibrated accel bias: %f %f %f\n",
				calibrationConfig.A_off[0],
				calibrationConfig.A_off[1],
				calibrationConfig.A_off[2]
			);
		} else {
			printf("\tAccel bias not calibrated\n");
		}
	}

	std::optional<std::function<void(const int16_t[3])>> accelSampleHandler;
	std::optional<std::function<void(const int16_t[3])>> gyroSampleHandler;
	std::optional<std::function<void(float)>> tempSampleHandler;

	bool isCalibrating = false;

	SensorFusionRestDetect& m_fusion;

	CalibrationStep nextCalibrationStep = CalibrationStep::MOTIONLESS;

	static constexpr float initialStartupDelaySeconds = 5;
	uint64_t startupMillis;

	float accelScale;

	IMU& imu;
	uint8_t sensorId;

	SlimeVR::Configuration::NonBlockingCalibrationConfig calibrationConfig;

	// Sampling rate calibration

	void tickSamplingRateCalibration() {
		if (!samplingRateCalibrationData) {
			samplingRateCalibrationData
				= SamplingRateCalibrationData{millis(), 0, 0, 0};

			accelSampleHandler = [&](const int16_t accelSample[3]) {
				samplingRateCalibrationData.value().accelSamples++;
			};
			gyroSampleHandler = [&](const int16_t gyroSample[3]) {
				samplingRateCalibrationData.value().gyroSamples++;
			};
			tempSampleHandler = [&](float tempSample) {
				samplingRateCalibrationData.value().tempSamples++;
			};

			isCalibrating = true;
		}

		float elapsedTime
			= (millis() - samplingRateCalibrationData.value().startMillis) / 1e3f;

		if (elapsedTime < samplingRateCalibrationSeconds) {
			return;
		}

		float accelTimestep
			= elapsedTime / samplingRateCalibrationData.value().accelSamples;
		float gyroTimestep
			= elapsedTime / samplingRateCalibrationData.value().gyroSamples;
		float tempTimestep
			= elapsedTime / samplingRateCalibrationData.value().tempSamples;

		calibrationConfig.A_Ts = accelTimestep;
		calibrationConfig.G_Ts = gyroTimestep;
		calibrationConfig.T_Ts = tempTimestep;
		calibrationConfig.sensorTimestepsCalibrated = true;

		stepCalibrationForward();
	}

	struct SamplingRateCalibrationData {
		uint64_t startMillis = 0;
		uint64_t accelSamples = 0;
		uint64_t gyroSamples = 0;
		uint64_t tempSamples = 0;
	};

	static constexpr float samplingRateCalibrationSeconds = 5;
	std::optional<SamplingRateCalibrationData> samplingRateCalibrationData;

	// Motionless Calibration

	void tickMotionlessCalibration() {
		if constexpr (HasMotionlessCalib) {
			if (!motionlessCalibrationData) {
				motionlessCalibrationData = {millis()};
				isCalibrating = true;
			}

			if (millis() - motionlessCalibrationData.value().startMillis
				< motionlessCalibrationDelay * 1e3) {
				return;
			}

			typename IMU::MotionlessCalibrationData calibrationData;
			if (imu.motionlessCalibration(calibrationData)) {
				std::memcpy(
					calibrationConfig.MotionlessData,
					&calibrationData,
					sizeof(calibrationData)
				);
				calibrationConfig.motionlessCalibrated = true;

				stepCalibrationForward();
			}
		} else {
			stepCalibrationForward(false);
		}
	}

	struct MotionlessCalibrationData {
		uint64_t startMillis = 0;
	};
	std::optional<MotionlessCalibrationData> motionlessCalibrationData;

	static constexpr float motionlessCalibrationDelay = 5;

	static constexpr bool HasMotionlessCalib
		= requires(IMU& i) { typename IMU::MotionlessCalibrationData; };
	static constexpr size_t MotionlessCalibDataSize() {
		if constexpr (HasMotionlessCalib) {
			return sizeof(typename IMU::MotionlessCalibrationData);
		} else {
			return 0;
		}
	}

	// Gyro Bias Calibration

	void tickGyroBiasCalibration() {
		if (!gyroBiasCalibrationData) {
			gyroBiasCalibrationData = GyroBiasCalibrationData{millis()};

			gyroSampleHandler = [&](const int16_t gyroSample[3]) {
				gyroBiasCalibrationData.value().gyroSums[0] += gyroSample[0];
				gyroBiasCalibrationData.value().gyroSums[1] += gyroSample[1];
				gyroBiasCalibrationData.value().gyroSums[2] += gyroSample[2];
				gyroBiasCalibrationData.value().sampleCount++;
			};
			tempSampleHandler = [&](float tempSample) {
				gyroBiasCalibrationData.value().temperature = tempSample;
			};

			isCalibrating = true;
		}

		if (millis() - gyroBiasCalibrationData.value().startMillis
			< gyroBiasCalibrationSeconds * 1e3) {
			return;
		}

		float gyroOffsetX
			= gyroBiasCalibrationData.value().gyroSums[0]
			/ static_cast<float>(gyroBiasCalibrationData.value().sampleCount);
		float gyroOffsetY
			= gyroBiasCalibrationData.value().gyroSums[1]
			/ static_cast<float>(gyroBiasCalibrationData.value().sampleCount);
		float gyroOffsetZ
			= gyroBiasCalibrationData.value().gyroSums[2]
			/ static_cast<float>(gyroBiasCalibrationData.value().sampleCount);

		calibrationConfig.G_off[0] = gyroOffsetX;
		calibrationConfig.G_off[1] = gyroOffsetY;
		calibrationConfig.G_off[2] = gyroOffsetZ;
		calibrationConfig.gyroCalibrated = true;

		stepCalibrationForward();
	}

	struct GyroBiasCalibrationData {
		uint64_t startMillis = 0;
		float temperature = 0;
		int32_t gyroSums[3]{0, 0, 0};
		size_t sampleCount = 0;
	};

	static constexpr float gyroBiasCalibrationSeconds = 5;
	std::optional<GyroBiasCalibrationData> gyroBiasCalibrationData;

	// Accel Bias Calibration

	void tickAccelBiasCalibration() {
		if (!accelBiasCalibrationData) {
			accelBiasCalibrationData = AccelBiasCalibrationData{};

			accelSampleHandler = [&](const int16_t accelSample[3]) {
				if (accelBiasCalibrationData.value().axisDetermined) {
					accelBiasCalibrationData.value().accelSum
						+= accelSample[accelBiasCalibrationData.value().largestAxis];

					accelBiasCalibrationData.value().sampleCount++;
					return;
				}

				size_t largestAxis;
				float absAxes[3]{
					std::abs(static_cast<float>(accelSample[0])),
					std::abs(static_cast<float>(accelSample[1])),
					std::abs(static_cast<float>(accelSample[2])),
				};

				if (absAxes[0] > absAxes[1] && absAxes[0] > absAxes[2]) {
					largestAxis = 0;
				} else if (absAxes[1] > absAxes[2]) {
					largestAxis = 1;
				} else {
					largestAxis = 2;
				}

				float smallAxisPercentage1
					= absAxes[(largestAxis + 1) % 3] / absAxes[largestAxis];
				float smallAxisPercentage2
					= absAxes[(largestAxis + 2) % 3] / absAxes[largestAxis];

				if (smallAxisPercentage1 > allowableVerticalAxisPercentage
					|| smallAxisPercentage2 > allowableVerticalAxisPercentage) {
					return;
				}

				accelBiasCalibrationData.value().axisDetermined = true;
				accelBiasCalibrationData.value().largestAxis = largestAxis;

				accelBiasCalibrationData.value().currentAxis[0] = accelSample[0];
				accelBiasCalibrationData.value().currentAxis[1] = accelSample[1];
				accelBiasCalibrationData.value().currentAxis[2] = accelSample[2];
			};

			isCalibrating = true;
		}

		if (!accelBiasCalibrationData.value().axisDetermined) {
			return;
		}

		if (accelBiasCalibrationData.value().sampleCount
			< accelBiasCalibrationSampleCount) {
			return;
		}

		float accelAverage
			= accelBiasCalibrationData.value().accelSum
			/ static_cast<float>(accelBiasCalibrationData.value().sampleCount);

		float expected = accelAverage > 0 ? CONST_EARTH_GRAVITY : -CONST_EARTH_GRAVITY;

		float accelOffset = accelAverage * accelScale - expected;

		calibrationConfig.A_off[accelBiasCalibrationData.value().largestAxis]
			= accelOffset;
		calibrationConfig.accelCalibrated = true;

		stepCalibrationForward();
	}

	struct AccelBiasCalibrationData {
		bool axisDetermined = false;
		int16_t currentAxis[3]{0, 0, 0};
		size_t largestAxis = -1;
		int32_t accelSum = 0;
		size_t sampleCount = 0;
	};

	static constexpr size_t accelBiasCalibrationSampleCount = 96;
	static constexpr float allowableVerticalAxisPercentage = 0.05;
	std::optional<AccelBiasCalibrationData> accelBiasCalibrationData;
};

}  // namespace SlimeVR::Sensors
