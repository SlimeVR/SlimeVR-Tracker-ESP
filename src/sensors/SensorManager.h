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
#ifndef SLIMEVR_SENSORMANAGER
#define SLIMEVR_SENSORMANAGER

#ifndef PRIMARY_IMU_ADDRESS_ONE
#define PRIMARY_IMU_ADDRESS_ONE std::nullopt
#endif

#ifndef SECONDARY_IMU_ADDRESS_TWO
#define SECONDARY_IMU_ADDRESS_TWO std::nullopt
#endif

#include <i2cscan.h>

#include <memory>
#include <optional>

#include "EmptySensor.h"
#include "ErroneousSensor.h"
#include "globals.h"
#include "logging/Logger.h"
#include "sensor.h"
#include "sensorinterface/DirectPinInterface.h"
#include "sensorinterface/I2CPCAInterface.h"
#include "sensorinterface/I2CWireSensorInterface.h"
#include "sensorinterface/MCP23X17PinInterface.h"

namespace SlimeVR {
namespace Sensors {
class SensorManager {
public:
	SensorManager()
		: m_Logger(SlimeVR::Logging::Logger("SensorManager")) {}
	void setup();
	void postSetup();

	void update();

	std::vector<std::unique_ptr<Sensor>>& getSensors() { return m_Sensors; };
	SensorTypeID getSensorType(size_t id) {
		if (id < m_Sensors.size()) {
			return m_Sensors[id]->getSensorType();
		}
		return SensorTypeID::Unknown;
	}

private:
	SlimeVR::Logging::Logger m_Logger;

	std::vector<std::unique_ptr<Sensor>> m_Sensors;
	Adafruit_MCP23X17 m_MCP;

	template <typename ImuType>
	std::unique_ptr<Sensor> buildSensor(
		uint8_t sensorID,
		std::optional<uint8_t> imuAddress,
		float rotation,
		SensorInterface* sensorInterface,
		bool optional = false,
		PinInterface* intPin = nullptr,
		int extraParam = 0
	) {
		uint8_t i2cAddress = imuAddress.value_or(ImuType::Address + sensorID);
		m_Logger.trace(
			"Building IMU with: id=%d,\n\
						address=0x%02X, rotation=%f,\n\
						interface=%s, int=%s, extraParam=%d, optional=%d",
			sensorID,
			i2cAddress,
			rotation,
			sensorInterface,
			intPin,
			extraParam,
			optional
		);

		// Now start detecting and building the IMU
		std::unique_ptr<Sensor> sensor;

		// Init I2C bus for each sensor upon startup
		sensorInterface->init();
		sensorInterface->swapIn();

		if (I2CSCAN::hasDevOnBus(i2cAddress)) {
			m_Logger
				.trace("Sensor %d found at address 0x%02X", sensorID + 1, i2cAddress);
		} else {
			if (!optional) {
				m_Logger.error(
					"Mandatory sensor %d not found at address 0x%02X",
					sensorID + 1,
					i2cAddress
				);
				sensor = std::make_unique<ErroneousSensor>(sensorID, ImuType::TypeID);
			} else {
				m_Logger.debug(
					"Optional sensor %d not found at address 0x%02X",
					sensorID + 1,
					i2cAddress
				);
				sensor = std::make_unique<EmptySensor>(sensorID);
			}
			return sensor;
		}

		sensor = std::make_unique<ImuType>(
			sensorID,
			i2cAddress,
			rotation,
			sensorInterface,
			intPin,
			extraParam
		);

		sensor->motionSetup();
		return sensor;
	}

	uint32_t m_LastBundleSentAtMicros = micros();
};
}  // namespace Sensors
}  // namespace SlimeVR

#endif  // SLIMEVR_SENSORFACTORY_H_
