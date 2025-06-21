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

#include <cstring>
#include <optional>

#include "CalibrationStep.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename IMU, typename SensorRawT>
class MotionlessCalibrationStep : public CalibrationStep<SensorRawT> {
	using CalibrationStep<SensorRawT>::sensorConfig;
	using typename CalibrationStep<SensorRawT>::TickResult;

public:
	MotionlessCalibrationStep(
		SlimeVR::Configuration::RuntimeCalibrationSensorConfig& sensorConfig,
		IMU& imu
	)
		: CalibrationStep<SensorRawT>{sensorConfig}
		, imu{imu} {}

	void start() override final {
		CalibrationStep<SensorRawT>::start();
		calibrationData = {millis()};
	}

	TickResult tick() override final {
		if constexpr (HasMotionlessCalib) {
			if (millis() - calibrationData.value().startMillis
				< motionlessCalibrationDelay * 1e3) {
				return TickResult::CONTINUE;
			}

			typename IMU::MotionlessCalibrationData motionlessCalibrationData;
			if (!imu.motionlessCalibration(motionlessCalibrationData)) {
				return TickResult::CONTINUE;
			}

			std::memcpy(
				sensorConfig.MotionlessData,
				&motionlessCalibrationData,
				sizeof(motionlessCalibrationData)
			);
			sensorConfig.motionlessCalibrated = true;

			return TickResult::DONE;
		} else {
			return TickResult::DONE;
		}
	}

	void cancel() override final { calibrationData.reset(); }

private:
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

	struct CalibrationData {
		uint64_t startMillis = 0;
	};

	std::optional<CalibrationData> calibrationData;
	IMU& imu;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
