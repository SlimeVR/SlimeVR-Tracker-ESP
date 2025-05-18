/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain

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
#include "wifiprovisioning.h"

#if ESP8266
#include <espnow.h>
#elif ESP32
#include <esp_now.h>
#endif

#include <vector>

#include "GlobalVars.h"
#include "credentials.h"
#include "logging/Logger.h"
#include "wifihandler.h"

#ifndef MACSTR
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#endif

namespace SlimeVR {

#if ESP8266
void espnowReceiveCallback(uint8_t* macAddress, uint8_t* data, uint8_t length) {
	wifiProvisioning.handleMessage(macAddress, data, length);
}
#elif ESP32
void espnowReceiveCallback(
	const esp_now_recv_info_t* senderInfo,
	const uint8_t* data,
	int dataLen
) {
	wifiProvisioning.handleMessage(senderInfo->src_addr, data, dataLen);
}
#endif

bool WiFiProvisioning::startProvisioning() {
	if (status != ProvisioningStatus::Idle) {
		logger.error(
			"Can't start new provisioning process when a previous one is in progress!"
		);
		return false;
	}

	if (!wifiNetwork.isConnected()) {
		logger.error(
			"Can't start provisioning without being connected to a wifi network!"
		);
		return false;
	}

	if (strlen(provisioningPassword) == 0) {
		logger.error("No provisioning password set! Aborting");
		return false;
	}

	if (!initEspnow()) {
		return false;
	}

	logger.info("Starting wifi provisioning!");

	status = ProvisioningStatus::Provisioning;
	startMillis = millis();

	return true;
}

void WiFiProvisioning::stopProvisioning() {
	if (status != ProvisioningStatus::Provisioning) {
		return;
	}

	logger.info("Stopped provisioning");
	esp_now_deinit();
	esp_now_unregister_recv_cb();
	status = ProvisioningStatus::Idle;
}

bool WiFiProvisioning::startSearchForProvisioner() {
	if (status != ProvisioningStatus::Idle) {
		logger.error("Can't start new search when a previous one is in progress");
		return false;
	}

	if (strlen(provisioningPassword) == 0) {
		logger.info(
			"No provisioning password set, won't start searching for a provisioner"
		);
		return false;
	}

	if (!initEspnow()) {
		return false;
	}

	status = ProvisioningStatus::Searching;
	lastSearchSentMillis = millis();
	startMillis = millis();

	return true;
}

void WiFiProvisioning::stopSearchForProvisioner() {
	if (status != ProvisioningStatus::Searching) {
		return;
	}

	logger.info("Stopped searching for provisioner");
	esp_now_deinit();
	esp_now_unregister_recv_cb();
	status = ProvisioningStatus::Idle;
}

void WiFiProvisioning::tick() {
	uint32_t elapsedMillis = millis() - startMillis;
	if (status == ProvisioningStatus::Searching
		&& elapsedMillis >= static_cast<uint32_t>(SearchTimeoutSeconds * 1000)) {
		logger.info("Search timed out");
		stopSearchForProvisioner();
		return;
	}
	if (status == ProvisioningStatus::Provisioning
		&& elapsedMillis >= static_cast<uint32_t>(ProvisioningTimeoutSeconds * 1000)) {
		logger.info("Provisioning timed out");
		stopProvisioning();
		return;
	}

	if (status != ProvisioningStatus::Searching) {
		return;
	}

	if (millis() - lastSearchSentMillis
		< static_cast<uint32_t>(SearchIntervalSeconds * 1000)) {
		return;
	}

#if ESP8266
	esp_now_set_peer_channel(BroadcastMacAddress, nextSearchChannel);
#elif ESP32
	esp_now_peer_info_t peer;
	esp_now_get_peer(BroadcastMacAddress, &peer);
	peer.channel = nextSearchChannel;
	esp_now_mod_peer(&peer);
#endif

	logger.info("Searching on channel %d", nextSearchChannel);
	nextSearchChannel++;
	if (nextSearchChannel > 14) {
		nextSearchChannel = 1;
	}

	size_t passwordByteLength = strlen(provisioningPassword) + 1;
	std::vector<uint8_t> data;
	data.resize(1 + passwordByteLength);
	data[0] = static_cast<uint8_t>(ProvisioningMessage::Search);
	memcpy(data.data() + 1, provisioningPassword, passwordByteLength);
	auto result = esp_now_send(
		BroadcastMacAddress,
		data.data(),
		static_cast<uint8_t>(1 + passwordByteLength)
	);

	if (result != 0) {
		logger.error("Couldn't send: %d", result);
	}

	lastSearchSentMillis = millis();
}

bool WiFiProvisioning::initEspnow() {
	auto result = esp_now_init();
	if (result != 0) {
		logger.error("Couldn't start ESP-Now! Error: %d", result);
		return false;
	}

#if ESP8266

	result = esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
	if (result != 0) {
		logger.error("Couldn't set ESP-Now role! Error: %d", result);
		esp_now_deinit();
		return false;
	}

#endif

	result = addPeer(BroadcastMacAddress);
	if (result != 0) {
		logger.error("Couldn't register ESP-Now broadcast peer! Error: %d", result);
		esp_now_deinit();
		return false;
	}

	result = esp_now_register_recv_cb(espnowReceiveCallback);
	if (result != 0) {
		logger.error("Couldn't register ESP-Now receive callback! Error: %d", result);
		esp_now_deinit();
		return false;
	}

	return true;
}

bool WiFiProvisioning::isSearching() const {
	return status == ProvisioningStatus::Searching;
}

void WiFiProvisioning::handleMessage(
	uint8_t* macAddress,
	const uint8_t* data,
	uint8_t length
) {
	auto messageHeader = static_cast<ProvisioningMessage>(data[0]);

	if (status == ProvisioningStatus::Provisioning
		&& messageHeader == ProvisioningMessage::Search) {
		handleSearchMessage(macAddress, data, length);
		return;
	}

	if (status == ProvisioningStatus::Searching
		&& messageHeader == ProvisioningMessage::Reply) {
		handleReplyMessage(macAddress, data, length);
		return;
	}
}

void WiFiProvisioning::handleSearchMessage(
	uint8_t* macAddress,
	const uint8_t* data,
	uint8_t length
) {
	const char* password = reinterpret_cast<const char*>(&data[1]);
	if (strcmp(password, provisioningPassword) != 0) {
		logger.error(
			"Received a search message from " MACSTR " with an incorrect password!",
			MAC2STR(macAddress)
		);
		return;
	}

	logger.info(
		"Received a search message from " MACSTR ". Responding with WiFi credentials!",
		MAC2STR(macAddress)
	);

	addPeer(macAddress);

	String ssid = WiFi.SSID();
	String pass = WiFi.psk();
	size_t ssidByteLength = ssid.length() + 1;
	size_t passwordByteLength = pass.length() + 1;
	std::vector<uint8_t> sendData;
	sendData.resize(1 + ssidByteLength + passwordByteLength);
	sendData[0] = static_cast<uint8_t>(ProvisioningMessage::Reply);
	memcpy(sendData.data() + 1, ssid.c_str(), ssidByteLength);
	memcpy(sendData.data() + 1 + ssidByteLength, pass.c_str(), passwordByteLength);
	esp_now_send(
		macAddress,
		sendData.data(),
		static_cast<uint8_t>(1 + ssidByteLength + passwordByteLength)
	);

	delPeer(macAddress);
}

void WiFiProvisioning::handleReplyMessage(
	uint8_t* macAddress,
	const uint8_t* data,
	uint8_t length
) {
	auto* ssid = reinterpret_cast<const char*>(&data[1]);
	auto* pass = reinterpret_cast<const char*>(&data[1 + strlen(ssid) + 1]);

	logger.info(
		"Received a reply from " MACSTR " with SSID %s and password length %zu",
		MAC2STR(macAddress),
		ssid,
		strlen(pass)
	);

	wifiNetwork.setWiFiCredentials(ssid, pass);
}

uint8_t WiFiProvisioning::addPeer(uint8_t* macAddress) {
#if ESP8266
	return esp_now_add_peer(macAddress, ESP_NOW_ROLE_COMBO, 0, nullptr, 0);
#elif ESP32
	esp_now_peer_info_t peer{};
	memcpy(peer.peer_addr, macAddress, sizeof(uint8_t[6]));
	peer.channel = 0;
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;
	return esp_now_add_peer(&peer);
#endif
}

void WiFiProvisioning::delPeer(uint8_t* macAddress) { esp_now_del_peer(macAddress); }

}  // namespace SlimeVR
