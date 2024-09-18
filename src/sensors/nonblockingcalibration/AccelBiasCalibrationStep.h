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

#include <optional>

#include "CalibrationStep.h"

namespace SlimeVR::Sensors::NonBlockingCalibration {

class AccelBiasCalibrationStep : public CalibrationStep {
public:
	AccelBiasCalibrationStep(
		SlimeVR::Configuration::NonBlockingCalibrationConfig& calibrationConfig,
		float accelScale
	);

	void start() override final;
	TickResult tick() override final;
	void cancel() override final;

	void processAccelSample(const int16_t accelSample[3]) override final;

	bool allAxesCalibrated();
	bool anyAxesCalibrated();

private:
	static constexpr size_t accelBiasCalibrationSampleCount = 96;
	static constexpr float allowableVerticalAxisPercentage = 0.05;

	struct CalibrationData {
		bool axisDetermined = false;
		int16_t currentAxis[3]{0, 0, 0};
		size_t largestAxis = -1;
		int32_t accelSum = 0;
		size_t sampleCount = 0;
	};

	std::optional<CalibrationData> calibrationData;
	float accelScale;
};

}  // namespace SlimeVR::Sensors::NonBlockingCalibration
