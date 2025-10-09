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
#include <cstdint>
#include <cstring>

#include "GlobalVars.h"
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
		sendMessage(BroadcastMacAddress, ProvisioningPackets::ProvisioningAvailable{});
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
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningStatus: {
			auto packet
				= *reinterpret_cast<const ProvisioningPackets::ProvisioningStatus*>(data
				);
			addPeer(macAddress);
			sendMessage(macAddress, ProvisioningPackets::ProvisioningStatusAck{});
			removePeer(macAddress);
			networkConnection.sendProvisioningStatus(macAddress, packet.status);
			logger.info(
				"Tracker with mac address " MACSTR " reported status %d, %s",
				MAC2STR(macAddress),
				static_cast<uint8_t>(packet.status),
				ProvisioningPackets::statusToCstr(packet.status)
			);
			break;
		}
		case ProvisioningPackets::ProvisioningPacketId::ProvisioningFailed: {
			auto packet
				= *reinterpret_cast<const ProvisioningPackets::ProvisioningFailed*>(data
				);
			addPeer(macAddress);
			sendMessage(macAddress, ProvisioningPackets::ProvisioningFailedAck{});
			removePeer(macAddress);
			networkConnection.sendProvisioningFailed(macAddress, packet.error);
			logger.error(
				"Tracker with mac address " MACSTR " reported error %d, %s",
				MAC2STR(macAddress),
				static_cast<uint8_t>(packet.error),
				ProvisioningPackets::errorToCstr(packet.error)
			);
			break;
		}
		default:
			break;
	}
}

void ProvisioningProvider::handleProvisioningRequest(
	uint8_t macAddress[6],
	const char* password
) {
	if (strcmp(password, provisioningPassword) != 0) {
		return;
	}
	ProvisioningPackets::ProvisioningStart packet{};
	// TODO: swap with the getSSID/getPassword function once that gets merged in
	strcpy(packet.wifiName, wifiNetwork.getSSID().c_str());
	strcpy(packet.wifiPassword, wifiNetwork.getPassword().c_str());
	addPeer(macAddress);
	sendMessage(macAddress, packet);
	removePeer(macAddress);
	networkConnection.sendProvisioningNewTracker(macAddress);
	logger.info(
		"Provisioning tracker with mac address " MACSTR "...",
		MAC2STR(macAddress)
	);
}

}  // namespace SlimeVR::Network
