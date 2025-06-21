/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors

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

#include <cstddef>
#include <cstdint>
#include <functional>

#include "configuration/SensorConfig.h"
#include "imuconsts.h"
#include "motionprocessing/types.h"
#include "sensors/SensorFusion.h"

namespace SlimeVR::Sensors {

template <typename IMU>
class CalibrationBase {
public:
	CalibrationBase(
		SlimeVR::Sensors::SensorFusion& fusion,
		IMU& sensor,
		uint8_t sensorId,
		SlimeVR::Logging::Logger& logger,
		SensorToggleState& toggles
	)
		: fusion{fusion}
		, sensor{sensor}
		, sensorId{sensorId}
		, logger{logger}
		, toggles{toggles} {}

	using Consts = IMUConsts<IMU>;
	using RawSensorT = typename Consts::RawSensorT;

	static constexpr bool HasMotionlessCalib
		= requires(IMU& i) { typename IMU::MotionlessCalibrationData; };
	static constexpr size_t MotionlessCalibDataSize() {
		if constexpr (HasMotionlessCalib) {
			return sizeof(typename IMU::MotionlessCalibrationData);
		} else {
			return 0;
		}
	}

	virtual void checkStartupCalibration() {}
	virtual void startCalibration(int calibrationType){};

	virtual bool calibrationMatches(
		const SlimeVR::Configuration::SensorConfig& sensorCalibration
	) = 0;

	virtual void assignCalibration(const Configuration::SensorConfig& sensorCalibration)
		= 0;

	virtual void begin() {}

	virtual void tick() {}

	virtual void scaleAccelSample(sensor_real_t accelSample[3]) = 0;
	virtual float getAccelTimestep() = 0;

	virtual void scaleGyroSample(sensor_real_t gyroSample[3]) = 0;
	virtual float getGyroTimestep() = 0;

	virtual float getTempTimestep() = 0;

	virtual const uint8_t* getMotionlessCalibrationData() = 0;

	virtual void provideAccelSample(const RawSensorT accelSample[3]) {}
	virtual void provideGyroSample(const RawSensorT gyroSample[3]) {}
	virtual void provideTempSample(float tempSample) {}

	virtual float getZROChange() { return IMU::TemperatureZROChange; };

protected:
	void recalcFusion() {
		fusion = Sensors::SensorFusion(
			IMU::SensorVQFParams,
			getGyroTimestep(),
			getAccelTimestep(),
			getTempTimestep()
		);
	}

	Sensors::SensorFusion& fusion;
	IMU& sensor;
	uint8_t sensorId;
	SlimeVR::Logging::Logger& logger;
	SensorToggleState& toggles;
};

}  // namespace SlimeVR::Sensors
