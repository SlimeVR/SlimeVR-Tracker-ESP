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
