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
#pragma once

#include <i2cscan.h>

#include <memory>
#include <optional>

#include "EmptySensor.h"
#include "ErroneousSensor.h"
#include "globals.h"
#include "logging/Logger.h"
#include "sensorinterface/DirectPinInterface.h"
#include "sensorinterface/I2CPCAInterface.h"
#include "sensorinterface/I2CWireSensorInterface.h"
#include "sensorinterface/MCP23X17PinInterface.h"
#include "sensorinterface/RegisterInterface.h"
#include "sensorinterface/i2cimpl.h"

namespace SlimeVR::Sensors {

class SensorManager {
public:
	SensorManager()
		: m_Logger(SlimeVR::Logging::Logger("SensorManager")) {}
	void setup();
	void postSetup();

	void update();

	std::vector<std::unique_ptr<::Sensor>>& getSensors() { return m_Sensors; };
	SensorTypeID getSensorType(size_t id) {
		if (id < m_Sensors.size()) {
			return m_Sensors[id]->getSensorType();
		}
		return SensorTypeID::Unknown;
	}

private:
	SlimeVR::Logging::Logger m_Logger;

	std::vector<std::unique_ptr<::Sensor>> m_Sensors;
	Adafruit_MCP23X17 m_MCP;

	uint32_t m_LastBundleSentAtMicros = micros();

	friend class SensorBuilder;
};
}  // namespace SlimeVR::Sensors
