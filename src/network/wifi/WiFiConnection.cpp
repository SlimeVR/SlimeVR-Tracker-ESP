/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 Gorbit99 & SlimeVR Contributors

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

#include "WiFiConnection.h"

#include <IPAddress.h>
#include <WiFiUdp.h>

#include <cstdint>
#include <memory>

#include "GlobalVars.h"

namespace SlimeVR::Network::WiFiComms {

void WiFiConnection::init() {
	logger.info("Setting up WiFi");
	WiFi.persistent(true);
	WiFi.mode(WIFI_STA);
	WiFi.hostname("SlimeVR FBT Tracker");
	logger.info(
		"Loaded credentials for SSID '%s' and pass length %d",
		getSSID().c_str(),
		getPassword().length()
	);

	transitionState(std::make_unique<SavedCredentialsAttemptState>(*this));

#if ESP8266
#if POWERSAVING_MODE == POWER_SAVING_NONE
	WiFi.setSleepMode(WIFI_NONE_SLEEP);
#elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
	WiFi.setSleepMode(WIFI_MODEM_SLEEP);
#elif POWERSAVING_MODE == POWER_SAVING_MODERATE
	WiFi.setSleepMode(WIFI_MODEM_SLEEP, 10);
#elif POWERSAVING_MODE == POWER_SAVING_MAXIMUM
	WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 10);
#error "MAX POWER SAVING NOT WORKING YET, please disable!"
#endif
#else
#if POWERSAVING_MODE == POWER_SAVING_NONE
	WiFi.setSleep(WIFI_PS_NONE);
#elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
	WiFi.setSleep(WIFI_PS_MIN_MODEM);
#elif POWERSAVING_MODE == POWER_SAVING_MODERATE \
	|| POWERSAVING_MODE == POWER_SAVING_MAXIMUM
	wifi_config_t conf;
	if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK) {
		conf.sta.listen_interval = 10;
		esp_wifi_set_config(WIFI_IF_STA, &conf);
		WiFi.setSleep(WIFI_PS_MAX_MODEM);
	} else {
		logger.error("Unable to get WiFi config, power saving not enabled!");
	}
#endif
#endif
}

void WiFiConnection::tick() {
	wifiProvisioning.upkeepProvisioning();

	currentState->tick();
}

void WiFiConnection::setWiFiCredentials(const char* SSID, const char* pass) {
	wifiProvisioning.stopProvisioning();
	transitionState(std::make_unique<ServerCredentialsAttemptState>(*this, SSID, pass));
}

void WiFiConnection::reset() {
	UDP.begin(serverPort);
	serverHost = IPAddress(255, 255, 255, 255);
}

bool WiFiConnection::isConnected() const {
	return currentState->toStatus() == WiFiReconnectionStatus::Success;
}

bool WiFiConnection::beginUDPPacket() {
	assert(isConnected());

	int r = UDP.beginPacket(serverHost, serverPort);
	if (r == 0) {
		// This *technically* should *never* fail, since the underlying UDP
		// library just returns 1.

		logger.warn("UDP beginPacket() failed");
	}

	return r > 0;
}

bool WiFiConnection::write(const uint8_t* data, size_t length) {
	assert(isConnected());

	return UDP.write(data, length) > 0;
}

bool WiFiConnection::endUDPPacket() {
	assert(isConnected());
	int r = UDP.endPacket();
	if (r == 0) {
		// This is usually just `ERR_ABRT` but the UDP client doesn't expose
		// the full error code to us, so we just have to live with it.

		// m_Logger.warn("UDP endPacket() failed");
	}
	return r > 0;
}

size_t WiFiConnection::readUDPPacket(uint8_t* buffer, size_t maxSize) {
	int packetSize = UDP.parsePacket();
	if (packetSize == 0) {
		return 0;
	}

	int len = UDP.read(buffer, maxSize);

#ifdef DEBUG_NETWORK
	logger.trace(
		"Received %d bytes from %s, port %d",
		packetSize,
		UDP.remoteIP().toString().c_str(),
		UDP.remotePort()
	);
	logger.traceArray("UDP packet contents: ", buffer, len);
#endif

	return static_cast<size_t>(len);
}

void WiFiConnection::setIPAddress(IPAddress&& address) { serverHost = address; }

IPAddress WiFiConnection::getIPAddress() const { return serverHost; }

void WiFiConnection::acceptHandshake() {
	serverHost = UDP.remoteIP();
	serverPort = UDP.remotePort();

	logger.debug(
		"Handshake successful, server is %s:%d",
		UDP.remoteIP().toString().c_str(),
		UDP.remotePort()
	);
}

WiFiConnection::WiFiReconnectionStatus WiFiConnection::getState() const {
	return currentState->toStatus();
}

void WiFiConnection::transitionState(std::unique_ptr<WiFiState>&& newState) {
	currentState = std::move(newState);
}

String WiFiConnection::getSSID() {
#if ESP8266
	return WiFi.SSID();
#else
	// Necessary, because without a WiFi.begin(), ESP32 is not kind enough to load the
	// SSID on its own, for whatever reason
	wifi_config_t wifiConfig;
	esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifiConfig);
	return {reinterpret_cast<char*>(wifiConfig.sta.ssid)};
#endif
}

String WiFiConnection::getPassword() {
#if ESP8266
	return WiFi.psk();
#else
	// Same as above
	wifi_config_t wifiConfig;
	esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifiConfig);
	return {reinterpret_cast<char*>(wifiConfig.sta.password)};
#endif
}

const char* WiFiConnection::statusToReasonString(wl_status_t status) {
	switch (status) {
		case WL_DISCONNECTED:
			return "Timeout";
#ifdef ESP8266
		case WL_WRONG_PASSWORD:
			return "Wrong password";
		case WL_CONNECT_FAILED:
			return "Connection failed";
#elif ESP32
		case WL_CONNECT_FAILED:
			return "Wrong password";
#endif

		case WL_NO_SSID_AVAIL:
			return "SSID not found";
		default:
			return "Unknown";
	}
}

WiFiConnection::WiFiFailureReason WiFiConnection::statusToFailure(wl_status_t status) {
	switch (status) {
		case WL_DISCONNECTED:
			return WiFiFailureReason::Timeout;
#ifdef ESP8266
		case WL_WRONG_PASSWORD:
			return WiFiFailureReason::WrongPassword;
#elif ESP32
		case WL_CONNECT_FAILED:
			return WiFiFailureReason::WrongPassword;
#endif

		case WL_NO_SSID_AVAIL:
			return WiFiFailureReason::SSIDNotFound;
		default:
			return WiFiFailureReason::Unknown;
	}
}

WiFiConnection::WiFiState::WiFiState(
	WiFiConnection& context,
	WiFiReconnectionStatus status
)
	: context{context}
	, status{status} {}

WiFiConnection::WiFiReconnectionStatus WiFiConnection::WiFiState::toStatus() const {
	return status;
}

WiFiConnection::ConnectingState::ConnectingState(
	WiFiConnection& context,
	WiFiReconnectionStatus status,
	const char* attemptName
)
	: WiFiState{context, status}
	, attemptName{attemptName} {}

void WiFiConnection::ConnectingState::reportWifiProgress() {
	if (context.wifiReportTimeout.elapsed()) {
		context.wifiReportTimeout.restart();
		Serial.print(".");
	}
}

void WiFiConnection::ConnectingState::tick() {
	this->reportWifiProgress();

	if (!setup) {
		if (!attempt(false)) {
			context.transitionState(nextState());
			return;
		}
		setup = true;
		return;
	}

	if (WiFi.status() == WL_CONNECTED) {
		context.transitionState(std::make_unique<ConnectedState>(context));
		return;
	}

	if (!wifiTimeout.elapsed() && WiFi.status() == WL_DISCONNECTED) {
		return;
	}

	showConnectionAttemptFailed(attemptName);

	if (WiFi.status() == WL_DISCONNECTED && !retriedOnG) {
		retriedOnG = true;
		if (!attempt(true)) {
			context.transitionState(nextState());
			return;
		}
		wifiTimeout.restart();

		return;
	}

	context.transitionState(nextState());
}

bool WiFiConnection::ConnectingState::tryConnecting(
	bool phyModeG,
	const char* SSID,
	const char* pass
) {
#if ESP8266
	if (phyModeG) {
		WiFi.setPhyMode(WIFI_PHY_MODE_11G);
		if constexpr (USE_ATTENUATION) {
			WiFi.setOutputPower(20.0 - ATTENUATION_G);
		}
	} else {
		WiFi.setPhyMode(WIFI_PHY_MODE_11N);
		if constexpr (USE_ATTENUATION) {
			WiFi.setOutputPower(20.0 - ATTENUATION_N);
		}
	}
#else
	if (phyModeG) {
		return false;
	}
#endif

	setStaticIPIfDefined();
	if (SSID == nullptr) {
		WiFi.begin();
	} else {
		WiFi.begin(SSID, pass);
	}
	return true;
}

void WiFiConnection::ConnectingState::setStaticIPIfDefined() {
#ifdef WIFI_USE_STATICIP
	const IPAddress ip(WIFI_STATIC_IP);
	const IPAddress gateway(WIFI_STATIC_GATEWAY);
	const IPAddress subnet(WIFI_STATIC_SUBNET);
	WiFi.config(ip, gateway, subnet);
#endif
}

void WiFiConnection::ConnectingState::showConnectionAttemptFailed(const char* type
) const {
	context.logger.error(
		"Can't connect from %s credentials, error: %d, reason: %s.",
		type,
		static_cast<int>(statusToFailure(WiFi.status())),
		statusToReasonString(WiFi.status())
	);
}

WiFiConnection::NotSetupState::NotSetupState(WiFiConnection& context)
	: WiFiState{context, WiFiReconnectionStatus::NotSetup} {}

void WiFiConnection::NotSetupState::tick() {
	// Do nothing
}

WiFiConnection::SavedCredentialsAttemptState::SavedCredentialsAttemptState(
	WiFiConnection& context
)
	: ConnectingState{context, WiFiReconnectionStatus::SavedAttempt, "saved"} {
	if (getSSID().length() == 0) {
		context.logger.debug("Skipping saved credentials attempt on 0-length SSID...");
		context.transitionState(
			std::make_unique<HardcodedCredentialsAttemptState>(context)
		);
		return;
	}
}

bool WiFiConnection::SavedCredentialsAttemptState::attempt(bool retry) {
	if (retry) {
		context.logger.debug("Trying saved credentials with PHY Mode G...");
	}

	return tryConnecting(retry);
}

std::unique_ptr<WiFiConnection::WiFiState>
WiFiConnection::SavedCredentialsAttemptState::nextState() {
	return std::make_unique<HardcodedCredentialsAttemptState>(context);
}

WiFiConnection::HardcodedCredentialsAttemptState::HardcodedCredentialsAttemptState(
	WiFiConnection& context
)
	: ConnectingState{context, WiFiReconnectionStatus::HardcodeAttempt, "hardcoded"} {}

bool WiFiConnection::HardcodedCredentialsAttemptState::attempt(bool retry) {
#if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
	if (retry) {
		context.logger.debug("Trying hardcoded credentials with PHY Mode G...");
	}

	// Don't need to save hardcoded credentials
	WiFi.persistent(false);
	auto result = tryConnecting(retry, WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
	WiFi.persistent(true);

	return result;
#else
	return false;
#endif
}

std::unique_ptr<WiFiConnection::WiFiState>
WiFiConnection::HardcodedCredentialsAttemptState::nextState() {
	return std::make_unique<FailedState>(context);
}

WiFiConnection::ServerCredentialsAttemptState::ServerCredentialsAttemptState(
	WiFiConnection& context,
	const char* SSID,
	const char* pass
)
	: ConnectingState{context, WiFiReconnectionStatus::ServerCredAttempt, "server"}
	, SSID{SSID}
	, pass{pass} {
	context.hadWifi = false;
}

bool WiFiConnection::ServerCredentialsAttemptState::attempt(bool retry) {
	return tryConnecting(retry, SSID, pass);
}

std::unique_ptr<WiFiConnection::WiFiState>
WiFiConnection::ServerCredentialsAttemptState::nextState() {
	return std::make_unique<FailedState>(context);
}

WiFiConnection::FailedState::FailedState(WiFiConnection& context)
	: WiFiState{context, WiFiReconnectionStatus::Failed} {}

void WiFiConnection::FailedState::tick() {
#if ESP8266
	if constexpr (USE_ATTENUATION) {
		WiFi.setOutputPower(20.0 - ATTENUATION_N);
	}
	WiFi.setPhyMode(WIFI_PHY_MODE_11N);
#endif
	// Start smart config
	if (!context.hadWifi && !WiFi.smartConfigDone() && wifiTimeout.elapsed()) {
		if (WiFi.status() != WL_IDLE_STATUS) {
			context.logger.error(
				"Can't connect from any credentials, error: %d, reason: %s.",
				static_cast<int>(statusToFailure(WiFi.status())),
				statusToReasonString(WiFi.status())
			);
			wifiTimeout.restart();
		}
		wifiProvisioning.startProvisioning();
	}
}

WiFiConnection::ConnectedState::ConnectedState(WiFiConnection& context)
	: WiFiState{context, WiFiReconnectionStatus::Success} {
	wifiProvisioning.stopProvisioning();
	statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, false);
	context.hadWifi = true;
	context.logger.info(
		"Connected successfully to SSID '%s', IP address %s",
		getSSID().c_str(),
		WiFi.localIP().toString().c_str()
	);
}

void WiFiConnection::ConnectedState::tick() {
	if (WiFi.status() == WL_CONNECTED) {
		return;
	}

	statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, true);
	context.logger.warn("Connection to WiFi lost, reconnecting...");
	context.transitionState(std::make_unique<SavedCredentialsAttemptState>(context));
}

}  // namespace SlimeVR::Network::WiFiComms
