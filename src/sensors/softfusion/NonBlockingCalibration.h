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
#include "../../utils.h"
#include "../SensorFusionRestDetect.h"

namespace SlimeVR::Sensors {

template <typename IMU>
class NonBlockingCalibrator {
public:
	mutable SlimeVR::Logging::Logger logger{"DynamicCalibration"};

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

		computeNextCalibrationStep();

		printCalibration();
	}

	void tick() {
		if (skippedAStep && !lastTickRest && m_fusion.getRestDetected()) {
			computeNextCalibrationStep();
			skippedAStep = false;
		}

		if (nextCalibrationStep == CalibrationStep::NONE) {
			lastTickRest = m_fusion.getRestDetected();
			return;
		}

		if (millis() - startupMillis < initialStartupDelaySeconds * 1e3) {
			return;
		}

		if (!m_fusion.getRestDetected() && requiresRestForNextStep()) {
			if (isCalibrating) {
				cancelCalibration();
			}

			lastTickRest = m_fusion.getRestDetected();
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
			case CalibrationStep::GYRO_BIAS1:
				tickGyroBiasCalibration();
				break;
			case CalibrationStep::ACCEL_BIAS:
				tickAccelBiasCalibration();
				break;
			case CalibrationStep::GYRO_BIAS2:
				tickGyroBiasCalibration();
				break;
		}

		lastTickRest = m_fusion.getRestDetected();
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
		GYRO_BIAS1,
		ACCEL_BIAS,
		GYRO_BIAS2,
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
			case CalibrationStep::GYRO_BIAS1:
				gyroBiasCalibrationData.reset();
				break;
			case CalibrationStep::ACCEL_BIAS:
				accelBiasCalibrationData.reset();
				break;
			case CalibrationStep::GYRO_BIAS2:
				gyroBiasCalibrationData.reset();
				break;
		}

		accelSampleHandler.reset();
		gyroSampleHandler.reset();
		tempSampleHandler.reset();

		isCalibrating = false;
	}

	void computeNextCalibrationStep() {
		if (!calibrationConfig.sensorTimestepsCalibrated) {
			nextCalibrationStep = CalibrationStep::SAMPLING_RATE;
		} else if (!calibrationConfig.motionlessCalibrated && HasMotionlessCalib) {
			nextCalibrationStep = CalibrationStep::MOTIONLESS;
		} else if (calibrationConfig.gyroPointsCalibrated == 0) {
			nextCalibrationStep = CalibrationStep::GYRO_BIAS1;
		} else if (!allAccelAxesCalibrated()) {
			nextCalibrationStep = CalibrationStep::ACCEL_BIAS;
		} else if (calibrationConfig.gyroPointsCalibrated == 1) {
			nextCalibrationStep = CalibrationStep::GYRO_BIAS2;
		} else {
			nextCalibrationStep = CalibrationStep::NONE;
		}
	}

	bool requiresRestForNextStep() {
		switch (nextCalibrationStep) {
			case CalibrationStep::NONE:
				return false;
			case CalibrationStep::SAMPLING_RATE:
				return false;
			case CalibrationStep::MOTIONLESS:
				return true;
			case CalibrationStep::GYRO_BIAS1:
				return true;
			case CalibrationStep::ACCEL_BIAS:
				return true;
			case CalibrationStep::GYRO_BIAS2:
				return true;
		}

		return false;
	}

	bool allAccelAxesCalibrated() {
		return calibrationConfig.accelCalibrated[0]
			&& calibrationConfig.accelCalibrated[1]
			&& calibrationConfig.accelCalibrated[2];
	}

	bool anyAccelAxesCalibrated() {
		return calibrationConfig.accelCalibrated[0]
			|| calibrationConfig.accelCalibrated[1]
			|| calibrationConfig.accelCalibrated[2];
	}

	void stepCalibrationForward(bool save = true) {
		switch (nextCalibrationStep) {
			case CalibrationStep::NONE:
				return;
			case CalibrationStep::SAMPLING_RATE:
				samplingRateCalibrationData.reset();

				nextCalibrationStep = CalibrationStep::MOTIONLESS;
				if (save) {
					printCalibration(CalibrationPrintFlags::TIMESTEPS);
				}
				break;
			case CalibrationStep::MOTIONLESS:
				motionlessCalibrationData.reset();

				nextCalibrationStep = CalibrationStep::GYRO_BIAS1;
				if (save) {
					printCalibration(CalibrationPrintFlags::MOTIONLESS);
				}
				break;
			case CalibrationStep::GYRO_BIAS1:
				gyroBiasCalibrationData.reset();

				nextCalibrationStep = CalibrationStep::ACCEL_BIAS;
				if (save) {
					printCalibration(CalibrationPrintFlags::GYRO_BIAS);
				}
				break;
			case CalibrationStep::ACCEL_BIAS:
				accelBiasCalibrationData.reset();

				nextCalibrationStep = CalibrationStep::GYRO_BIAS2;
				if (save) {
					printCalibration(CalibrationPrintFlags::ACCEL_BIAS);
				}

				if (!allAccelAxesCalibrated()) {
					skippedAStep = true;
				}
				break;
			case CalibrationStep::GYRO_BIAS2:
				gyroBiasCalibrationData.reset();

				nextCalibrationStep = CalibrationStep::NONE;
				if (save) {
					printCalibration(CalibrationPrintFlags::GYRO_BIAS);
				}
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

		ledManager.blink(100);
	}

	enum class CalibrationPrintFlags {
		TIMESTEPS = 1,
		MOTIONLESS = 2,
		GYRO_BIAS = 4,
		ACCEL_BIAS = 8,
	};

	static constexpr CalibrationPrintFlags PrintAllCalibration
		= CalibrationPrintFlags::TIMESTEPS | CalibrationPrintFlags::MOTIONLESS
		| CalibrationPrintFlags::GYRO_BIAS | CalibrationPrintFlags::ACCEL_BIAS;

	void printCalibration(CalibrationPrintFlags toPrint = PrintAllCalibration) {
		
		if (any(toPrint & CalibrationPrintFlags::TIMESTEPS)) {
			if (calibrationConfig.sensorTimestepsCalibrated) {
				logger.info(
					"Calibrated timesteps: Accel %f, Gyro %f, Temperature %f",
					calibrationConfig.A_Ts,
					calibrationConfig.G_Ts,
					calibrationConfig.T_Ts
				);
			} else {
				logger.info("Sensor timesteps not calibrated");
			}
		}

		if (HasMotionlessCalib && any(toPrint & CalibrationPrintFlags::MOTIONLESS)) {
			if (calibrationConfig.motionlessCalibrated) {
				logger.info("Motionless calibration done");
			} else {
				logger.info("Motionless calibration not done");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::GYRO_BIAS)) {
			if (calibrationConfig.gyroPointsCalibrated != 0) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibrationConfig.gyroMeasurementTemperature1,
					calibrationConfig.G_off1[0],
					calibrationConfig.G_off1[1],
					calibrationConfig.G_off1[2]
				);
			} else {
				logger.info("Gyro bias not calibrated");
			}

			if (calibrationConfig.gyroPointsCalibrated == 2) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibrationConfig.gyroMeasurementTemperature2,
					calibrationConfig.G_off2[0],
					calibrationConfig.G_off2[1],
					calibrationConfig.G_off2[2]
				);
			}
		}

		if (any(toPrint & CalibrationPrintFlags::ACCEL_BIAS)) {
			if (allAccelAxesCalibrated()) {
				logger.info(
					"Calibrated accel bias: %f %f %f",
					calibrationConfig.A_off[0],
					calibrationConfig.A_off[1],
					calibrationConfig.A_off[2]
				);
			} else if (anyAccelAxesCalibrated()) {
				logger.info(
					"Partially calibrated accel bias: %f %f %f",
					calibrationConfig.A_off[0],
					calibrationConfig.A_off[1],
					calibrationConfig.A_off[2]
				);
			} else {
				logger.info("Accel bias not calibrated");
			}
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

	bool skippedAStep = false;
	bool lastTickRest = false;

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

				if (calibrationConfig.gyroPointsCalibrated == 0) {
					return;
				}

				float tempDiff = std::abs(
					calibrationConfig.gyroMeasurementTemperature1 - tempSample
				);
				if (tempDiff < gyroBiasTemperatureDifference) {
					gyroBiasCalibrationData.value().gyroSums[0] = 0;
					gyroBiasCalibrationData.value().gyroSums[1] = 0;
					gyroBiasCalibrationData.value().gyroSums[2] = 0;
					gyroBiasCalibrationData.value().sampleCount = 0;
					gyroBiasCalibrationData.value().startMillis = millis();
				}
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

		if (calibrationConfig.gyroPointsCalibrated == 0) {
			calibrationConfig.G_off1[0] = gyroOffsetX;
			calibrationConfig.G_off1[1] = gyroOffsetY;
			calibrationConfig.G_off1[2] = gyroOffsetZ;
			calibrationConfig.gyroPointsCalibrated = 1;
			calibrationConfig.gyroMeasurementTemperature1
				= gyroBiasCalibrationData.value().temperature;
		} else {
			calibrationConfig.G_off2[0] = gyroOffsetX;
			calibrationConfig.G_off2[1] = gyroOffsetY;
			calibrationConfig.G_off2[2] = gyroOffsetZ;
			calibrationConfig.gyroPointsCalibrated = 2;
			calibrationConfig.gyroMeasurementTemperature2
				= gyroBiasCalibrationData.value().temperature;
		}

		stepCalibrationForward();
	}

	struct GyroBiasCalibrationData {
		uint64_t startMillis = 0;
		float temperature = 0;
		int32_t gyroSums[3]{0, 0, 0};
		size_t sampleCount = 0;
	};

	static constexpr float gyroBiasCalibrationSeconds = 5;
	static constexpr float gyroBiasTemperatureDifference = 5;
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

				float absAxes[3]{
					std::abs(static_cast<float>(accelSample[0])),
					std::abs(static_cast<float>(accelSample[1])),
					std::abs(static_cast<float>(accelSample[2])),
				};

				size_t largestAxis;
				if (absAxes[0] > absAxes[1] && absAxes[0] > absAxes[2]) {
					largestAxis = 0;
				} else if (absAxes[1] > absAxes[2]) {
					largestAxis = 1;
				} else {
					largestAxis = 2;
				}

				if (calibrationConfig.accelCalibrated[largestAxis]) {
					stepCalibrationForward(false);
					return;
				}

				float smallAxisPercentage1
					= absAxes[(largestAxis + 1) % 3] / absAxes[largestAxis];
				float smallAxisPercentage2
					= absAxes[(largestAxis + 2) % 3] / absAxes[largestAxis];

				if (smallAxisPercentage1 > allowableVerticalAxisPercentage
					|| smallAxisPercentage2 > allowableVerticalAxisPercentage) {
					stepCalibrationForward(false);
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
		calibrationConfig.accelCalibrated[accelBiasCalibrationData.value().largestAxis]
			= true;

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
