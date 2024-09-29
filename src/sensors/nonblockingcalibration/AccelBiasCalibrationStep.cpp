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

#include "AccelBiasCalibrationStep.h"

#include <cmath>
#include <cstdio>

#include "../../consts.h"

namespace SlimeVR::Sensors::NonBlockingCalibration {

AccelBiasCalibrationStep::AccelBiasCalibrationStep(
	SlimeVR::Configuration::NonBlockingCalibrationConfig& calibrationConfig,
	float accelScale
)
	: CalibrationStep{calibrationConfig}
	, accelScale{accelScale} {}

void AccelBiasCalibrationStep::start() {
	CalibrationStep::start();
	calibrationData = CalibrationData{};
}

CalibrationStep::TickResult AccelBiasCalibrationStep::tick() {
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

	calibrationConfig.A_off[calibrationData.value().largestAxis] = accelOffset;
	calibrationConfig.accelCalibrated[calibrationData.value().largestAxis] = true;

	return TickResult::DONE;
}

void AccelBiasCalibrationStep::cancel() { calibrationData.reset(); }

void AccelBiasCalibrationStep::processAccelSample(const int16_t accelSample[3]) {
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

	if (calibrationConfig.accelCalibrated[largestAxis]) {
		calibrationData.value().axisDetermined = true;
		calibrationData.value().largestAxis = -1;
		return;
	}

	float smallAxisPercentage1 = absAxes[(largestAxis + 1) % 3] / absAxes[largestAxis];
	float smallAxisPercentage2 = absAxes[(largestAxis + 2) % 3] / absAxes[largestAxis];

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

bool AccelBiasCalibrationStep::allAxesCalibrated() {
	return calibrationConfig.accelCalibrated[0] && calibrationConfig.accelCalibrated[1]
		&& calibrationConfig.accelCalibrated[2];
}

bool AccelBiasCalibrationStep::anyAxesCalibrated() {
	return calibrationConfig.accelCalibrated[0] || calibrationConfig.accelCalibrated[1]
		|| calibrationConfig.accelCalibrated[2];
}

}  // namespace SlimeVR::Sensors::NonBlockingCalibration
