#include <Arduino.h>
#include <core_esp8266_features.h>

#include <cstdint>
#include <cstring>
#include <type_traits>

#include "GlobalVars.h"
#include "credentials.h"
#include "espnow.h"
#include "logging/Logger.h"
#include "network/wifihandler.h"
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
	addPeer(BroadcastMacAddress);
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
				channelSwitchTimeout.reset();
			}
			break;
		}
		case Status::ConnectionStarted: {
			sendAndWaitForAck(ProvisioningPackets::ProvisioningStatus{
				.status = ProvisioningPackets::ConnectionStatus::Connecting,
			});
			break;
		}
		case Status::Connecting: {
			if (WiFi.isConnected()) {
				sendAndWaitForAck(ProvisioningPackets::ProvisioningStatus{
					.status = ProvisioningPackets::ConnectionStatus::Connected
				});
				break;
			}

			if (wifiNetwork.getWiFiState()
				== WiFiNetwork::WiFiReconnectionStatus::Failed) {
				sendAndWaitForAck(ProvisioningPackets::ProvisioningFailed{
					.error = ProvisioningPackets::ConnectionError::ConnectionFailed,
				});
				break;
			}

			break;
		}
		case Status::SearchingForServer: {
			if (!searchingForServer) {
				serverSearchTimeout.reset();
				searchingForServer = true;
			}

			if (networkConnection.isConnected()) {
				sendAndWaitForAck(ProvisioningPackets::ProvisioningStatus{
					.status = ProvisioningPackets::ConnectionStatus::ServerFound,
				});
				break;
			}

			if (serverSearchTimeout.elapsed()) {
				sendAndWaitForAck(ProvisioningPackets::ProvisioningFailed{
					.error = ProvisioningPackets::ConnectionError::ServerNotFound,
				});
				break;
			}
			break;
		}
		case Status::Done:
			printf("Finished\n");
			return false;
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
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningStatusAck: {
			status = static_cast<Status>(static_cast<int>(status) + 1);
			waitingForAck = false;
			retryTimeout.reset();
			break;
		}
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningFailedAck: {
			status = Status::Done;
			break;
		}
		default:
			break;
	}
}

void ProvisioningTarget::switchChannel(uint8_t channel) {
	logger.info("Switching to channel %d...", channel);
	wifi_set_channel(channel);
	currentChannel = channel;

	esp_now_set_peer_channel(BroadcastMacAddress, channel);
}

void ProvisioningTarget::handleProvisioningOffer(uint8_t macAddress[6]) {
	if (status != Status::WaitingForProvider) {
		return;
	}
	logger.info("Received provisioning offer from " MACSTR, MAC2STR(macAddress));
	addPeer(macAddress);
	memcpy(providerMac, macAddress, sizeof(providerMac));

	ProvisioningPackets::ProvisioningRequest packet;
	strcpy(packet.provisioningPassword, provisioningPassword);
	sendMessage(providerMac, packet);
}

void ProvisioningTarget::startConnection(const char* ssid, const char* password) {
	logger.info(
		"Received WiFi credentials SSID %s and password length %zu! Connecting...",
		ssid,
		strlen(password)
	);

	wifiNetwork.setProvisionedWiFiCredentials(ssid, password);
	status = Status::ConnectionStarted;
	resetLastSendResult();
	sendMessage(
		providerMac,
		ProvisioningPackets::ProvisioningStatus{
			.status = ProvisioningPackets::ConnectionStatus::Connecting,
		}
	);
}

}  // namespace SlimeVR::Network
