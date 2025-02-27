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

#include "../../../consts.h"
#include "CalibrationStep.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename SensorRawT>
class AccelBiasCalibrationStep : public CalibrationStep<SensorRawT> {
	using CalibrationStep<SensorRawT>::sensorConfig;
	using typename CalibrationStep<SensorRawT>::TickResult;

public:
	AccelBiasCalibrationStep(
		SlimeVR::Configuration::RuntimeCalibrationSensorConfig& sensorConfig,
		float accelScale
	)
		: CalibrationStep<SensorRawT>{sensorConfig}
		, accelScale{accelScale} {}

	void start() override final {
		CalibrationStep<SensorRawT>::start();
		calibrationData = CalibrationData{};
	}

	TickResult tick() override final {
		if (!calibrationData.value().axisDetermined) {
			return TickResult::CONTINUE;
		}

		if (calibrationData.value().largestAxis == -1) {
			return TickResult::SKIP;
		}

		if (calibrationData.value().sampleCount < accelBiasCalibrationSampleCount) {
			return TickResult::CONTINUE;
		}

		float accelAverage = calibrationData.value().accelSum
						   / static_cast<float>(calibrationData.value().sampleCount);

		float expected = accelAverage > 0 ? CONST_EARTH_GRAVITY : -CONST_EARTH_GRAVITY;

		float accelOffset = accelAverage * accelScale - expected;

		sensorConfig.A_off[calibrationData.value().largestAxis] = accelOffset;
		sensorConfig.accelCalibrated[calibrationData.value().largestAxis] = true;

		return TickResult::DONE;
	}

	void cancel() override final { calibrationData.reset(); }
	void processAccelSample(const SensorRawT accelSample[3]) override final {
		if (calibrationData.value().axisDetermined) {
			calibrationData.value().accelSum
				+= accelSample[calibrationData.value().largestAxis];

			calibrationData.value().sampleCount++;
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

		if (sensorConfig.accelCalibrated[largestAxis]) {
			calibrationData.value().axisDetermined = true;
			calibrationData.value().largestAxis = -1;
			return;
		}

		float smallAxisPercentage1
			= absAxes[(largestAxis + 1) % 3] / absAxes[largestAxis];
		float smallAxisPercentage2
			= absAxes[(largestAxis + 2) % 3] / absAxes[largestAxis];

		if (smallAxisPercentage1 > allowableVerticalAxisPercentage
			|| smallAxisPercentage2 > allowableVerticalAxisPercentage) {
			calibrationData.value().axisDetermined = true;
			calibrationData.value().largestAxis = -1;
			return;
		}

		calibrationData.value().axisDetermined = true;
		calibrationData.value().largestAxis = largestAxis;

		calibrationData.value().currentAxis[0] = accelSample[0];
		calibrationData.value().currentAxis[1] = accelSample[1];
		calibrationData.value().currentAxis[2] = accelSample[2];
	}

	bool allAxesCalibrated() {
		return sensorConfig.accelCalibrated[0] && sensorConfig.accelCalibrated[1]
			&& sensorConfig.accelCalibrated[2];
	}
	bool anyAxesCalibrated() {
		return sensorConfig.accelCalibrated[0] || sensorConfig.accelCalibrated[1]
			|| sensorConfig.accelCalibrated[2];
	}

private:
	static constexpr size_t accelBiasCalibrationSampleCount = 96;
	static constexpr float allowableVerticalAxisPercentage = 0.05;

	struct CalibrationData {
		bool axisDetermined = false;
		int16_t currentAxis[3]{0, 0, 0};
		int32_t largestAxis = -1;
		int32_t accelSum = 0;
		size_t sampleCount = 0;
	};

	std::optional<CalibrationData> calibrationData;
	float accelScale;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
