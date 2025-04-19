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
#include "motionprocessing/types.h"
#include "sensors/SensorFusionRestDetect.h"

namespace SlimeVR::Sensor {

template <typename IMU, typename RawSensorT, typename RawVectorT>
class CalibrationBase {
public:
	CalibrationBase(
		SlimeVR::Sensors::SensorFusionRestDetect& fusion,
		IMU& sensor,
		uint8_t sensorId,
		SlimeVR::Logging::Logger& logger,
		float TempTs,
		float AScale,
		float GScale,
		SensorToggleState& toggles
	)
		: fusion{fusion}
		, sensor{sensor}
		, sensorId{sensorId}
		, logger{logger}
		, TempTs{TempTs}
		, AScale{AScale}
		, GScale{GScale}
		, toggles{toggles} {}

	static constexpr bool HasMotionlessCalib
		= requires(IMU& i) { typename IMU::MotionlessCalibrationData; };
	static constexpr size_t MotionlessCalibDataSize() {
		if constexpr (HasMotionlessCalib) {
			return sizeof(typename IMU::MotionlessCalibrationData);
		} else {
			return 0;
		}
	}

	using EatSamplesFn = std::function<void(const uint32_t)>;
	using ReturnLastFn
		= std::function<std::tuple<RawVectorT, RawVectorT, int16_t>(const uint32_t)>;

	virtual void startCalibration(
		int calibrationType,
		const EatSamplesFn& eatSamplesForSeconds,
		const ReturnLastFn& eatSamplesReturnLast
	){};

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
		fusion = Sensors::SensorFusionRestDetect(
			IMU::SensorVQFParams,
			getGyroTimestep(),
			getAccelTimestep(),
			getTempTimestep()
		);
	}

	Sensors::SensorFusionRestDetect& fusion;
	IMU& sensor;
	uint8_t sensorId;
	SlimeVR::Logging::Logger& logger;
	float TempTs;
	float AScale;
	float GScale;
	SensorToggleState& toggles;
};

}  // namespace SlimeVR::Sensor
