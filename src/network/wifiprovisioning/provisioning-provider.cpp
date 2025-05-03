#include <cstdint>
#include <cstring>

#include "credentials.h"
#include "logging/Logger.h"
#include "network/wifiprovisioning/provisioning-packets.h"
#include "network/wifiprovisioning/provisioning-party.h"
#include "provisioning-provider.h"

#if ESP8266
#include <ESP8266WiFi.h>
#elif ESP32
#include <WiFi.h>
#endif

namespace SlimeVR::Network {

ProvisioningProvider::ProvisioningProvider(SlimeVR::Logging::Logger& logger) noexcept
	: ProvisioningParty{logger} {}

void ProvisioningProvider::init() {
	provisioningTimeout.reset();
	messageTimeout.reset();
	addPeer(BroadcastMacAddress);
}

bool ProvisioningProvider::tick() {
	if (provisioningTimeout.elapsed()) {
		logger.info("Provisioning timed out");
		return false;
	}

	if (messageTimeout.elapsed()) {
		ProvisioningPackets::ProvisioningAvailable packet{};
		sendMessage(
			BroadcastMacAddress,
			reinterpret_cast<uint8_t*>(&packet),
			sizeof(packet)
		);
		messageTimeout.reset();
	}

	return true;
}

void ProvisioningProvider::handleMessage(
	uint8_t macAddress[6],
	const uint8_t* data,
	uint8_t length
) {
	auto packetId = static_cast<ProvisioningPackets::ProvisioningPacketId>(data[0]);
	switch (packetId) {
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningRequest: {
			auto packet
				= *reinterpret_cast<const ProvisioningPackets::ProvisioningRequest*>(
					data
				);
			// Ensure it's null terminated for security
			packet.provisioningPassword[sizeof(packet.provisioningPassword) - 1] = '\0';
			handleProvisioningRequest(macAddress, packet.provisioningPassword);
			break;
		}
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningStarted: {
			break;
		}
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningStatus: {
			break;
		}
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningFailed: {
			break;
		}
	}
}

void ProvisioningProvider::stop() {}

void ProvisioningProvider::handleProvisioningRequest(
	uint8_t macAddress[6],
	const char* password
) {
	if (strcmp(password, provisioningPassword) != 0) {
		return;
	}
	ProvisioningPackets::ProvisioningStart packet{};
	// TODO: swap with the getSSID/getPassword function once that gets merged in
	strcpy(packet.wifiName, WiFi.SSID().c_str());
	strcpy(packet.wifiPassword, WiFi.psk().c_str());
	addPeer(macAddress);
	sendMessage(macAddress, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
	removePeer(macAddress);
}

}  // namespace SlimeVR::Network
