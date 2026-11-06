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

#include <cstdint>

namespace SlimeVR::Network::ProvisioningPackets {

enum class ProvisioningPacketId : uint8_t {
	ProvisioningAvailable,
	ProvisioningRequest,
	ProvisioningStart,
	ProvisioningStatus,
	ProvisioningStatusAck,
	ProvisioningFailed,
	ProvisioningFailedAck,
};

constexpr static uint8_t ESPNOWPacketId = 0xde;

#pragma pack(push, 1)

template <ProvisioningPacketId PacketId>
struct ProvisioningPacket {
	uint8_t espnowPacketId = ESPNOWPacketId;
	ProvisioningPacketId packetId = PacketId;
};

enum class ConnectionStatus : uint8_t {
	Connecting,
	Connected,
	ServerFound,
};

const char* statusToCstr(ConnectionStatus status);

enum class ConnectionError : uint8_t {
	ConnectionFailed,
	ServerNotFound,
};

const char* errorToCstr(ConnectionError error);

struct ProvisioningAvailable
	: ProvisioningPacket<ProvisioningPacketId::ProvisioningAvailable> {};

struct ProvisioningRequest
	: ProvisioningPacket<ProvisioningPacketId::ProvisioningRequest> {
	// Null terminated
	char provisioningPassword[33] = "";
};

struct ProvisioningStart : ProvisioningPacket<ProvisioningPacketId::ProvisioningStart> {
	// Standard credentials length limits, null terminated
	char wifiName[33] = "";
	char wifiPassword[64] = "";
};

struct ProvisioningStatus
	: ProvisioningPacket<ProvisioningPacketId::ProvisioningStatus> {
	ConnectionStatus status = ConnectionStatus::Connecting;
};

struct ProvisioningStatusAck
	: ProvisioningPacket<ProvisioningPacketId::ProvisioningStatusAck> {};

struct ProvisioningFailed
	: ProvisioningPacket<ProvisioningPacketId::ProvisioningFailed> {
	ConnectionError error = ConnectionError::ConnectionFailed;
};

struct ProvisioningFailedAck
	: ProvisioningPacket<ProvisioningPacketId::ProvisioningFailedAck> {};

#pragma pack(pop)

}  // namespace SlimeVR::Network::ProvisioningPackets
