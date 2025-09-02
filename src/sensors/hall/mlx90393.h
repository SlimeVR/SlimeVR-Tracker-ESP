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

#include "PinInterface.h"
#include "Wire.h"
#include "sensorinterface/SensorInterface.h"
#include "sensors/sensor.h"

namespace SlimeVR::Sensors {

class MLX90393 : public Sensor {
public:
	MLX90393(
		uint8_t id,
		SensorInterface* interface,
		PinInterface* interrupt,
		uint8_t address
	);

	void motionSetup() final;
	void motionLoop() final;
	SensorDataType getDataType() final;
	void sendData() final;
	bool hasNewDataToSend() final;

private:
	enum class Command : uint8_t {
		StartBurstMode = 0x1,
		StartWOCMode = 0x2,
		StartSingleMeasurementMode = 0x3,
		ReadMeasurement = 0x4,
		ReadRegister = 0x5,
		WriteRegister = 0x6,
		ExitMode = 0x8,
		MemoryRecall = 0xd,
		MemoryStore = 0xe,
		Reset = 0xf,
	};

	struct StatusReply {
		uint8_t data : 2;
		uint8_t reset : 1;
		uint8_t singleErrorDetection : 1;
		uint8_t error : 1;
		uint8_t singleMeasurementMode : 1;
		uint8_t wocMode : 1;
		uint8_t burstMode : 1;
	};

	static constexpr uint8_t SendRateHz = 30;
	static constexpr uint8_t BaseAddress = 0x0c;
	static constexpr float SensitivityuTPerLSB = 0.161;
	static constexpr float TemperatureSendRateHz = 10;

	template <size_t N>
	StatusReply sendCommand(
		Command command,
		const uint8_t (&params)[N],
		uint8_t* extraData = nullptr,
		size_t extraRequest = 0
	) {
		Wire.beginTransmission(address);

		if constexpr (N == 0) {
			Wire.write(static_cast<uint8_t>(command) << 4);
		} else {
			uint8_t paramsCopy[N];
			memcpy(paramsCopy, params, sizeof(paramsCopy));

			paramsCopy[0] |= static_cast<uint8_t>(command) << 4;
			Wire.write(paramsCopy, N);
		}

		if (command == Command::Reset) {
			Wire.endTransmission();
			delay(5);
			while (!I2CSCAN::hasDevOnBus(address))
				;
			Wire.beginTransmission(address);
			Wire.write(0x00);  // NOP
		}
		Wire.endTransmission(false);

		Wire.requestFrom(address, 1 + extraRequest);
		uint8_t readByte = Wire.read();

		if (extraRequest > 0) {
			Wire.readBytes(extraData, extraRequest);
		}

		return *reinterpret_cast<StatusReply*>(&readByte);
	}

	StatusReply writeRegister(uint8_t address, uint16_t value);
	StatusReply readRegister(uint8_t address, uint16_t& out);

	uint8_t address;
	PinInterface* interrupt;

	bool newSample = false;
	float lastSample[3] = {0};
	uint16_t tempRef = 0;
	float temp = 0;
	uint64_t lastTempSendMillis = 0;
};

}  // namespace SlimeVR::Sensors
