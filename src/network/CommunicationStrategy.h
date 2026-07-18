/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 Gorbit99 & SlimeVR Contributors

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

#include <quat.h>
#include <vector3.h>

#include <array>
#include <cstdint>
#include <string>

#include "../sensors/SensorErrorCodes.h"

namespace SlimeVR::Network {

class CommunicationStrategy {
public:
	virtual void init() = 0;
	virtual void tick() = 0;
	[[nodiscard]] virtual bool isConnected() const = 0;

	virtual bool beginComms() = 0;
	virtual bool endComms() = 0;

	virtual void sendBatteryLevel(float batteryVoltage, float batteryPercentage) = 0;
	virtual void sendFeatureFlags() = 0;
	virtual void sendSignalStrength(uint8_t signalStrength) = 0;

	virtual void sendAcceleration(uint8_t sensorId, Vector3 accel) = 0;
	virtual void
	sendRotation(uint8_t sensorId, const Quat& quaternion, uint8_t accuracyInfo)
		= 0;
	virtual void sendSensorTap(uint8_t sensorId, uint8_t value) = 0;
	virtual void sendSensorError(uint8_t sensorId, SensorErrorCode error) = 0;
	virtual void sendTemperature(uint8_t sensorId, float temperature) = 0;
	virtual void sendFlexData(uint8_t sensorId, float flexLevel) = 0;
	virtual void sendRawIMUData(
		uint8_t sensorId,
		std::array<int16_t, 3>& gyro,
		std::array<int16_t, 3>& accel,
		std::array<int16_t, 3>& mag
	) = 0;
	virtual void sendRawIMUData(
		uint8_t sensorId,
		std::array<float, 3>& gyro,
		std::array<float, 3>& accel,
		std::array<float, 3>& mag
	) = 0;

	[[nodiscard]] virtual std::string getAddressRepresentation() const = 0;
	[[nodiscard]] virtual int getState() const = 0;
};

}  // namespace SlimeVR::Network
