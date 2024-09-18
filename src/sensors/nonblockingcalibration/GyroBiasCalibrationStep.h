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

class GyroBiasCalibrationStep : public CalibrationStep {
public:
	GyroBiasCalibrationStep(
		SlimeVR::Configuration::NonBlockingCalibrationConfig& calibrationConfig
	);
	void start() override final;
	TickResult tick() override final;
	void cancel() override final;

	void processGyroSample(const int16_t gyroSample[3]) override final;
	void processTempSample(float tempSample) override final;

	void swapCalibrationIfNecessary();

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
