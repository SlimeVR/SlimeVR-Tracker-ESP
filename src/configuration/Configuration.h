/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 TheDevMinerTV

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

#ifndef SLIMEVR_CONFIGURATION_CONFIGURATION_H
#define SLIMEVR_CONFIGURATION_CONFIGURATION_H

#include <vector>

#include "../motionprocessing/GyroTemperatureCalibrator.h"
#include "../sensors/SensorToggles.h"
#include "DeviceConfig.h"
#include "logging/Logger.h"

namespace SlimeVR::Configuration {
class Configuration {
public:
	void setup();

	void save();
	void reset();

	void print();

	int32_t getVersion() const;

	size_t getSensorCount() const;
	SensorConfig getSensor(size_t sensorID) const;
	void setSensor(size_t sensorID, const SensorConfig& config);
	SensorToggleState getSensorToggles(size_t sensorId) const;
	void setSensorToggles(size_t sensorId, SensorToggleState state);
	void eraseSensors();

	bool loadTemperatureCalibration(
		uint8_t sensorId,
		GyroTemperatureCalibrationConfig& config
	);
	bool saveTemperatureCalibration(
		uint8_t sensorId,
		const GyroTemperatureCalibrationConfig& config
	);

private:
	void loadSensors();
	bool runMigrations(int32_t version);

	bool m_Loaded = false;

	DeviceConfig m_Config{};
	std::vector<SensorConfig> m_Sensors;
	std::vector<SensorToggleState> m_SensorToggles;

	Logging::Logger m_Logger = Logging::Logger("Configuration");
};
}  // namespace SlimeVR::Configuration

#endif
