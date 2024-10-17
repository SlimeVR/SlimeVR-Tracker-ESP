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

enum class ESPNowMessageHeader : uint8_t {
	Pairing = 0x00,
	PairingAck = 0x01,
	Connection = 0x02,
	Packet = 0x03,
};

struct ESPNowMessageBase {
	ESPNowMessageHeader header;
};

struct ESPNowPairingMessage {
	ESPNowMessageHeader header = ESPNowMessageHeader::Pairing;
};

struct ESPNowPairingAckMessage {
	ESPNowMessageHeader header = ESPNowMessageHeader::PairingAck;
	uint8_t trackerId;
};

struct ESPNowConnectionMessage {
	ESPNowMessageHeader header = ESPNowMessageHeader::Connection;
};

#pragma pack(push, 1)
struct ESPNowPacketMessage {
	ESPNowMessageHeader header = ESPNowMessageHeader::Packet;
	uint8_t packetId;
	uint8_t sensorId;
	uint8_t rssi;
	uint8_t battPercentage;
	uint16_t battVoltage;
	int16_t quat[4];
	int16_t accel[3];
};
#pragma pack(pop)

static_assert(sizeof(ESPNowPacketMessage) == 21);

union ESPNowMessage {
	ESPNowMessageBase base;
	ESPNowPairingMessage pairing;
	ESPNowPairingAckMessage pairingAck;
	ESPNowConnectionMessage connection;
	ESPNowPacketMessage packet;
};

}  // namespace SlimeVR::Network
