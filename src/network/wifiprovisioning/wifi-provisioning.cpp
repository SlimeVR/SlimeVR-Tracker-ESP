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

#include <cstdint>
#include <memory>

#include "network/wifiprovisioning/provisioning-provider.h"
#include "network/wifiprovisioning/provisioning-target.h"
#include "wifi-provisioning.h"

#if ESP8266
#include <espnow.h>
#elif ESP32
#include <esp_now.h>
#endif

#include "../wifihandler.h"
#include "GlobalVars.h"
#include "credentials.h"
#include "logging/Logger.h"

namespace SlimeVR::Network {

#if ESP8266
void espnowReceiveCallback(uint8_t* macAddress, uint8_t* data, uint8_t length) {
	wifiProvisioning.handleMessage(macAddress, data, length);
}

void espnowSendCallback(uint8_t* macAddress, uint8_t status) {
	wifiProvisioning.handleSendResult(status == 0);
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
	if (!wifiNetwork.isConnected()) {
		logger.error(
			"Can't start provisioning without being connected to a Wi-Fi network!"
		);
		return false;
	}

	if (strlen(provisioningPassword) == 0) {
		logger.error("No provisioning password set! Aborting");
		return false;
	}

	if (role) {
		logger.error("The Wi-Fi provisioning module is busy");
		return false;
	}

	if (!initEspnow()) {
		return false;
	}

	logger.info("Starting Wi-Fi provisioning!");
	role = std::make_unique<ProvisioningProvider>(logger);
	role->init();

	return true;
}

void WiFiProvisioning::stopProvisioning() {
	if (!role) {
		return;
	}

	logger.info("Stopped provisioning");
	esp_now_deinit();
	esp_now_unregister_recv_cb();
	role.reset();
}

bool WiFiProvisioning::startSearchForProvider() {
	if (wifiNetwork.isConnected()) {
		logger.error("The device is already connected to Wi-Fi!");
		return false;
	}

	if (role) {
		logger.error("The Wi-Fi provisioning module is busy");
		return false;
	}

	if (strlen(provisioningPassword) == 0) {
		logger.info("No provisioning password set, won't start searching for a provider"
		);
		return false;
	}

	if (!initEspnow()) {
		return false;
	}

	role = std::make_unique<ProvisioningTarget>(logger);
	role->init();

	return true;
}

void WiFiProvisioning::stopSearchForProvider() {
	if (!role) {
		return;
	}

	logger.info("Stopped searching for provider");
	esp_now_deinit();
	esp_now_unregister_recv_cb();
	role.reset();
}

void WiFiProvisioning::tick() {
	if (!role) {
		return;
	}

	if (!role->tick()) {
		role.reset();
		esp_now_deinit();
		esp_now_unregister_recv_cb();
	}
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

	result = esp_now_register_recv_cb(espnowReceiveCallback);
	if (result != 0) {
		logger.error("Couldn't register ESP-Now receiver callback! Error: %d", result);
		esp_now_deinit();
		return false;
	}

	esp_now_register_send_cb(espnowSendCallback);

	return true;
}

void WiFiProvisioning::handleMessage(
	uint8_t* macAddress,
	const uint8_t* data,
	uint8_t length
) {
	role->handleMessage(macAddress, data, length);
}

void WiFiProvisioning::handleSendResult(bool success) {
	role->handleSendResult(success);
}

}  // namespace SlimeVR::Network
