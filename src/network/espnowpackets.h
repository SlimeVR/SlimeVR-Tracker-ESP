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

#include <cstdint>

namespace SlimeVR::Network::ESPNow {

enum class ESPNowPacketId : uint8_t {
	DeviceInfo = 0x00,
	FullSizeFusion = 0x01,
};

struct ESPNowPacketDeviceInfo {
	ESPNowPacketId packetId = ESPNowPacketId::DeviceInfo;
	uint8_t sensorId;
	uint8_t battPercentage;
	uint8_t batteryVoltage;
	uint8_t temperature;
	uint8_t boardId;
	uint8_t protocolVersion;
	uint8_t reserved0;
	uint8_t imuId;
	uint8_t magId;
	uint16_t firmwareDate;
	uint8_t firmwareMajor;
	uint8_t firmwareMinor;
	uint8_t firmwarePatch;
	uint8_t rssi;
};

struct ESPNowPacketFullSizeFusion {
	ESPNowPacketId packetId = ESPNowPacketId::FullSizeFusion;
	uint8_t sensorId;
	int16_t quat[4];
	int16_t accel[3];
};

union ESPNowPacket {
	ESPNowPacketDeviceInfo deviceInfo;
	ESPNowPacketFullSizeFusion fullSizeFusion;
};

static_assert(sizeof(ESPNowPacket) == 16);

}
