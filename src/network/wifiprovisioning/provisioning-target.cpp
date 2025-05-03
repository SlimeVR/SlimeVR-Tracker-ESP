#include <Arduino.h>
#include <core_esp8266_features.h>

#include <cstdint>
#include <cstring>

#include "credentials.h"
#include "logging/Logger.h"
#include "network/wifiprovisioning/provisioning-packets.h"
#include "network/wifiprovisioning/provisioning-party.h"
#include "provisioning-target.h"
#include "user_interface.h"

namespace SlimeVR::Network {

ProvisioningTarget::ProvisioningTarget(SlimeVR::Logging::Logger& logger) noexcept
	: ProvisioningParty{logger} {}

void ProvisioningTarget::init() {
	searchTimeout.reset();
	channelSwitchTimeout.reset();
	switchChannel(1);
}

bool ProvisioningTarget::tick() {
	if (searchTimeout.elapsed()) {
		logger.info("Search timed out");
		return false;
	}

	switch (status) {
		case Status::WaitingForProvider: {
			if (channelSwitchTimeout.elapsed()) {
				if (currentChannel == 14) {
					switchChannel(1);
				} else {
					switchChannel(currentChannel + 1);
				}
			}
			channelSwitchTimeout.reset();
			break;
		}
		case Status::Authenticating: {
			break;
		}
		case Status::Connecting: {
			break;
		}
		case Status::SearchingForServer: {
			break;
		}
	}

	return true;
}

void ProvisioningTarget::handleMessage(
	uint8_t macAddress[6],
	const uint8_t* data,
	uint8_t length
) {
	auto packetId = static_cast<ProvisioningPackets::ProvisioningPacketId>(data[0]);

	switch (packetId) {
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningAvailable:
			handleProvisioningOffer(macAddress);
			break;
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningStart: {
			if (memcmp(macAddress, providerMac, sizeof(providerMac)) != 0) {
				break;
			}
			auto packet
				= *reinterpret_cast<const ProvisioningPackets::ProvisioningStart*>(data
				);
			// Ensure it's null terminated for security
			packet.wifiName[sizeof(packet.wifiName) - 1] = '\0';
			packet.wifiPassword[sizeof(packet.wifiPassword) - 1] = '\0';
			startConnection(packet.wifiName, packet.wifiPassword);
			break;
		}
		default:
			break;
	}
}

void ProvisioningTarget::stop() {}

void ProvisioningTarget::switchChannel(uint8_t channel) {
	logger.info("Switching to channel %d...", channel);
	wifi_set_channel(channel);
	currentChannel = channel;
}

void ProvisioningTarget::handleProvisioningOffer(uint8_t macAddress[6]) {
	logger.info("Received provisioning offer from " MACSTR, MAC2STR(macAddress));
	status = Status::Authenticating;
	addPeer(macAddress);
	memcpy(providerMac, macAddress, sizeof(providerMac));

	ProvisioningPackets::ProvisioningRequest packet;
	strcpy(packet.provisioningPassword, provisioningPassword);
	sendMessage(providerMac, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}

void ProvisioningTarget::startConnection(const char* ssid, const char* password) {
	// TODO:
	logger.info("Received wifi credentials SSID: %s, password: %s!", ssid, password);
}

}  // namespace SlimeVR::Network
