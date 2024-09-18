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

#include "SampleRateCalibrationStep.h"

namespace SlimeVR::Sensors::NonBlockingCalibration {

SampleRateCalibrationStep::SampleRateCalibrationStep(
	SlimeVR::Configuration::NonBlockingCalibrationConfig& calibrationConfig
)
	: CalibrationStep{calibrationConfig} {}

void SampleRateCalibrationStep::start() { calibrationData = {millis()}; }

CalibrationStep::TickResult SampleRateCalibrationStep::tick() {
	float elapsedTime = (millis() - calibrationData.value().startMillis) / 1e3f;

	if (elapsedTime < samplingRateCalibrationSeconds) {
		return TickResult::CONTINUE;
	}

	float accelTimestep = elapsedTime / calibrationData.value().accelSamples;
	float gyroTimestep = elapsedTime / calibrationData.value().gyroSamples;
	float tempTimestep = elapsedTime / calibrationData.value().tempSamples;

	calibrationConfig.A_Ts = accelTimestep;
	calibrationConfig.G_Ts = gyroTimestep;
	calibrationConfig.T_Ts = tempTimestep;
	calibrationConfig.sensorTimestepsCalibrated = true;

	return TickResult::DONE;
}

void SampleRateCalibrationStep::cancel() { calibrationData.reset(); }

bool SampleRateCalibrationStep::requiresRest() { return false; }

void SampleRateCalibrationStep::processAccelSample(const int16_t[3]) {
	calibrationData.value().accelSamples++;
}

void SampleRateCalibrationStep::processGyroSample(const int16_t[3]) {
	calibrationData.value().gyroSamples++;
}

void SampleRateCalibrationStep::processTempSample(float) {
	calibrationData.value().tempSamples++;
}

};  // namespace SlimeVR::Sensors::NonBlockingCalibration
