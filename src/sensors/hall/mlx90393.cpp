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

#include "mlx90393.h"

#include <cmath>
#include <cstdint>

#include "GlobalVars.h"
#include "Wire.h"

namespace SlimeVR::Sensors {

MLX90393::MLX90393(
	uint8_t id,
	SensorInterface* interface,
	PinInterface* interrupt,
	uint8_t address
)
	: Sensor{"MLX90393", SensorTypeID::MLX90393, id, EmptyRegisterInterface::instance, 0, interface}
	, interrupt{interrupt} {
	if (address < 4) {
		this->address = BaseAddress + address;
	} else {
		this->address = address;
	}
}

void MLX90393::motionSetup() {
	m_hwInterface->swapIn();

	if (!I2CSCAN::hasDevOnBus(address)) {
		working = false;
		return;
	}

	sendCommand(Command::ExitMode, {0});
	delayMicroseconds(1000);
	sendCommand(Command::Reset, {0});

	writeRegister(0x00, 0x007c);  // Highest gain, default hallconf
	writeRegister(
		0x01,
		0x07c0
	);  // Trigger in int mode, SPI and I2C, enable drift compensation, all axes for
		// burst, data rate highest
	writeRegister(0x02, 0x0012);  // oversample 2x, digital filter at 4

	readRegister(0x24, tempRef);  // Read temperature reference point
	tempRef = __builtin_bswap16(tempRef);

	auto status
		= sendCommand(Command::StartBurstMode, {0xf});  // Start burst mode on all axes

	delayMicroseconds(10);

	if (status.error || !status.burstMode) {
		working = false;
		return;
	}

	lastTempSendMillis = millis();

	working = true;
	interrupt->pinMode(INPUT);
}

SensorDataType MLX90393::getDataType() {
	return SensorDataType::SENSOR_DATATYPE_FLEX_RESISTANCE;
}

void MLX90393::sendData() {
	if (newSample) {
		float strength = sqrtf(
			lastSample[0] * lastSample[0] + lastSample[1] * lastSample[1]
			+ lastSample[2] * lastSample[2]
		);

		networkConnection.sendFlexData(sensorId, strength);
		newSample = false;
	}

	auto elapsedMillis = millis() - lastTempSendMillis;
	if (static_cast<float>(elapsedMillis) >= 1000 / TemperatureSendRateHz) {
		networkConnection.sendTemperature(sensorId, temp);
		lastTempSendMillis = millis();
	}
}

void MLX90393::motionLoop() {
	static uint64_t readCount = 0;
	static uint64_t startMillis = millis();
	static uint64_t lastPrintMillis = millis();

	if (!interrupt->digitalRead()) {
		return;
	}

	readCount++;
	if (millis() - lastPrintMillis >= 1000) {
		lastPrintMillis += 1000;
	}

	uint16_t rawSample[4];
	sendCommand(
		Command::ReadMeasurement,
		{0xf},
		reinterpret_cast<uint8_t*>(rawSample),
		8
	);  // Read all

	rawSample[0] = __builtin_bswap16(rawSample[0]);
	rawSample[1] = __builtin_bswap16(rawSample[1]);
	rawSample[2] = __builtin_bswap16(rawSample[2]);
	rawSample[3] = __builtin_bswap16(rawSample[3]);

	lastSample[0] = static_cast<float>(rawSample[1] - (0x8000)) * SensitivityuTPerLSB;
	lastSample[1] = static_cast<float>(rawSample[2] - (0x8000)) * SensitivityuTPerLSB;
	lastSample[2] = static_cast<float>(rawSample[3] - (0x8000)) * SensitivityuTPerLSB;

	temp = 35 + static_cast<float>(rawSample[0] - tempRef) / 45.2f;

	newSample = true;
}

MLX90393::StatusReply MLX90393::writeRegister(uint8_t address, uint16_t value) {
	assert(address <= 0x3f);

	return sendCommand(
		Command::WriteRegister,
		{0x00,
		 static_cast<uint8_t>(value >> 8),
		 static_cast<uint8_t>(value & 0xff),
		 static_cast<uint8_t>(address << 2)}
	);
}

MLX90393::StatusReply MLX90393::readRegister(uint8_t address, uint16_t& out) {
	assert(address <= 0x3f);

	auto result = sendCommand(
		Command::ReadRegister,
		{0, static_cast<uint8_t>(address << 2)},
		reinterpret_cast<uint8_t*>(&out),
		2
	);

	return result;
}

bool MLX90393::hasNewDataToSend() { return newSample; }

}  // namespace SlimeVR::Sensors
