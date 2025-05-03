#pragma once

#include <cstdint>

namespace SlimeVR::Network::ProvisioningPackets {

enum class ProvisioningPacketId : uint8_t {
	ProvisioningAvailable = 0,
	ProvisioningRequest = 1,
	ProvisioningStart = 2,
	ProvisioningStarted = 3,
	ProvisioningStatus = 4,
	ProvisioningFailed = 5,
};

#pragma pack(push, 1)

template <ProvisioningPacketId PacketId>
struct ProvisioningPacket {
	ProvisioningPacketId packetId = PacketId;
};

enum class ConnectionStatus : uint8_t {
	Connecting,
	Connected,
	ServerFound,
};

enum class ConnectionError : uint8_t {
	ConnectionFailed,
	ServerNotFound,
};

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

struct ProvisioningStarted
	: ProvisioningPacket<ProvisioningPacketId::ProvisioningStarted> {};

struct ProvisioningStatus
	: ProvisioningPacket<ProvisioningPacketId::ProvisioningStatus> {
	ConnectionStatus status = ConnectionStatus::Connecting;
};

struct ProvisioningFailed
	: ProvisioningPacket<ProvisioningPacketId::ProvisioningFailed> {
	ConnectionError error = ConnectionError::ConnectionFailed;
};

#pragma pack(pop)

}  // namespace SlimeVR::Network::ProvisioningPackets
