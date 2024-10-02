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

#include <cmath>
#include <optional>

#include "CalibrationStep.h"

namespace SlimeVR::Sensors::NonBlockingCalibration {

template <typename SensorRawT>
class GyroBiasCalibrationStep : public CalibrationStep<SensorRawT> {
	using CalibrationStep<SensorRawT>::calibrationConfig;
	using typename CalibrationStep<SensorRawT>::TickResult;

public:
	GyroBiasCalibrationStep(
		SlimeVR::Configuration::NonBlockingCalibrationConfig& calibrationConfig
	)
		: CalibrationStep<SensorRawT>{calibrationConfig} {}

	void start() override final {
		CalibrationStep<SensorRawT>::start();
		calibrationData = {millis()};
	}

	TickResult tick() override final {
		if (millis() - calibrationData.value().startMillis
			< gyroBiasCalibrationSeconds * 1e3) {
			return TickResult::CONTINUE;
		}

		float gyroOffsetX = calibrationData.value().gyroSums[0]
						  / static_cast<float>(calibrationData.value().sampleCount);
		float gyroOffsetY = calibrationData.value().gyroSums[1]
						  / static_cast<float>(calibrationData.value().sampleCount);
		float gyroOffsetZ = calibrationData.value().gyroSums[2]
						  / static_cast<float>(calibrationData.value().sampleCount);

		if (calibrationConfig.gyroPointsCalibrated == 0) {
			calibrationConfig.G_off1[0] = gyroOffsetX;
			calibrationConfig.G_off1[1] = gyroOffsetY;
			calibrationConfig.G_off1[2] = gyroOffsetZ;
			calibrationConfig.gyroPointsCalibrated = 1;
			calibrationConfig.gyroMeasurementTemperature1
				= calibrationData.value().temperature;

			return TickResult::DONE;
		}

		if (calibrationConfig.gyroPointsCalibrated == 1) {
			if (calibrationData.value().temperature
				> calibrationConfig.gyroMeasurementTemperature1) {
				calibrationConfig.G_off2[0] = gyroOffsetX;
				calibrationConfig.G_off2[1] = gyroOffsetY;
				calibrationConfig.G_off2[2] = gyroOffsetZ;
				calibrationConfig.gyroMeasurementTemperature2
					= calibrationData.value().temperature;
			} else {
				calibrationConfig.G_off2[0] = calibrationConfig.G_off1[0];
				calibrationConfig.G_off2[1] = calibrationConfig.G_off1[1];
				calibrationConfig.G_off2[2] = calibrationConfig.G_off1[2];
				calibrationConfig.gyroMeasurementTemperature2
					= calibrationConfig.gyroMeasurementTemperature1;

				calibrationConfig.G_off1[0] = gyroOffsetX;
				calibrationConfig.G_off1[1] = gyroOffsetY;
				calibrationConfig.G_off1[2] = gyroOffsetZ;
				calibrationConfig.gyroMeasurementTemperature1
					= calibrationData.value().temperature;
			}

			calibrationConfig.gyroPointsCalibrated = 2;

			return TickResult::DONE;
		}

		if (calibrationData.value().temperature
			< calibrationConfig.gyroMeasurementTemperature1) {
			calibrationConfig.G_off1[0] = gyroOffsetX;
			calibrationConfig.G_off1[1] = gyroOffsetY;
			calibrationConfig.G_off1[2] = gyroOffsetZ;
			calibrationConfig.gyroMeasurementTemperature1
				= calibrationData.value().temperature;
		} else {
			calibrationConfig.G_off2[0] = gyroOffsetX;
			calibrationConfig.G_off2[1] = gyroOffsetY;
			calibrationConfig.G_off2[2] = gyroOffsetZ;
			calibrationConfig.gyroMeasurementTemperature2
				= calibrationData.value().temperature;
		}

		return TickResult::DONE;
	}
	void cancel() override final { calibrationData.reset(); }

	void processGyroSample(const SensorRawT gyroSample[3]) override final {
		calibrationData.value().gyroSums[0] += gyroSample[0];
		calibrationData.value().gyroSums[1] += gyroSample[1];
		calibrationData.value().gyroSums[2] += gyroSample[2];
		calibrationData.value().sampleCount++;
	}

	void processTempSample(float tempSample) override final {
		calibrationData.value().temperature = tempSample;

		if (calibrationConfig.gyroPointsCalibrated == 0) {
			return;
		}

		if (calibrationConfig.gyroPointsCalibrated == 1) {
			float tempDiff
				= std::abs(calibrationConfig.gyroMeasurementTemperature1 - tempSample);

			if (tempDiff < gyroBiasTemperatureDifference) {
				calibrationData.value().gyroSums[0] = 0;
				calibrationData.value().gyroSums[1] = 0;
				calibrationData.value().gyroSums[2] = 0;
				calibrationData.value().sampleCount = 0;
				calibrationData.value().startMillis = millis();
			}

			return;
		}

		if (tempSample >= calibrationConfig.gyroMeasurementTemperature1
			&& tempSample <= calibrationConfig.gyroMeasurementTemperature2) {
			calibrationData.value().gyroSums[0] = 0;
			calibrationData.value().gyroSums[1] = 0;
			calibrationData.value().gyroSums[2] = 0;
			calibrationData.value().sampleCount = 0;
			calibrationData.value().startMillis = millis();
		}
	}

	void swapCalibrationIfNecessary() {
		if (calibrationConfig.gyroPointsCalibrated == 2
			&& calibrationConfig.gyroMeasurementTemperature1
				   > calibrationConfig.gyroMeasurementTemperature2) {
			float tempG_off[3]{
				calibrationConfig.G_off1[0],
				calibrationConfig.G_off1[1],
				calibrationConfig.G_off1[2],
			};
			float tempGTemperature = calibrationConfig.gyroMeasurementTemperature1;

			calibrationConfig.G_off1[0] = calibrationConfig.G_off2[0];
			calibrationConfig.G_off1[1] = calibrationConfig.G_off2[1];
			calibrationConfig.G_off1[2] = calibrationConfig.G_off2[2];
			calibrationConfig.gyroMeasurementTemperature1
				= calibrationConfig.gyroMeasurementTemperature2;

			calibrationConfig.G_off2[0] = tempG_off[0];
			calibrationConfig.G_off2[1] = tempG_off[1];
			calibrationConfig.G_off2[2] = tempG_off[2];
			calibrationConfig.gyroMeasurementTemperature2 = tempGTemperature;
		}
	}

private:
	static constexpr float gyroBiasCalibrationSeconds = 5;
	static constexpr float gyroBiasTemperatureDifference = 5;

	struct CalibrationData {
		uint64_t startMillis = 0;
		float temperature = 0;
		int32_t gyroSums[3]{0, 0, 0};
		size_t sampleCount = 0;
	};

	std::optional<CalibrationData> calibrationData;
};

}  // namespace SlimeVR::Sensors::NonBlockingCalibration
