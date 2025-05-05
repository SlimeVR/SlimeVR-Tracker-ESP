/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#include "GlobalVars.h"
#include "globals.h"
#if !ESP8266
#include "esp_wifi.h"
#endif

namespace SlimeVR {

void WiFiNetwork::reportWifiProgress() {
	if (lastWifiReportTime + 1000 < millis()) {
		lastWifiReportTime = millis();
		Serial.print(".");
	}
}

void WiFiNetwork::setStaticIPIfDefined() {
#ifdef WIFI_USE_STATICIP
	const IPAddress ip(WIFI_STATIC_IP);
	const IPAddress gateway(WIFI_STATIC_GATEWAY);
	const IPAddress subnet(WIFI_STATIC_SUBNET);
	WiFi.config(ip, gateway, subnet);
#endif
}

bool WiFiNetwork::isConnected() const {
	return wifiState == WiFiReconnectionStatus::Success;
}

void WiFiNetwork::setWiFiCredentials(const char* SSID, const char* pass) {
	wifiProvisioning.stopSearchForProvisioner();
	wifiProvisioning.stopProvisioning();
	WiFi.persistent(true);
	tryConnecting(false, SSID, pass);
	retriedOnG = false;
	// Reset state, will get back into provisioning if can't connect
	hadWifi = false;
	wifiState = WiFiReconnectionStatus::ServerCredAttempt;
}

IPAddress WiFiNetwork::getAddress() { return WiFi.localIP(); }

void WiFiNetwork::setUp() {
	// Don't need to save the already saved credentials or the hardcoded ones
	WiFi.persistent(false);
	wifiHandlerLogger.info("Setting up WiFi");
	WiFi.mode(WIFI_AP_STA);
	WiFi.hostname("SlimeVR FBT Tracker");
	wifiHandlerLogger.info(
		"Loaded credentials for SSID '%s' and pass length %d",
		WiFi.SSID().c_str(),
		WiFi.psk().length()
	);

	trySavedCredentials();

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
		wifiHandlerLogger.error("Unable to get WiFi config, power saving not enabled!");
	}
#endif
#endif
}

void WiFiNetwork::onConnected() {
	wifiState = WiFiReconnectionStatus::Success;
	wifiProvisioning.stopProvisioning();
	statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, false);
	hadWifi = true;
	wifiHandlerLogger.info(
		"Connected successfully to SSID '%s', IP address %s",
		WiFi.SSID().c_str(),
		WiFi.localIP().toString().c_str()
	);
	// Reset it, in case we just connected with server creds
	WiFi.persistent(false);
}

WiFiNetwork::WiFiReconnectionStatus WiFiNetwork::getWiFiState() { return wifiState; }

void WiFiNetwork::upkeep() {
	wifiProvisioning.tick();
	if (WiFi.status() == WL_CONNECTED) {
		if (!isConnected()) {
			onConnected();
			return;
		}

		if (millis() - lastRssiSample >= 2000) {
			lastRssiSample = millis();
			uint8_t signalStrength = WiFi.RSSI();
			networkConnection.sendSignalStrength(signalStrength);
		}
		return;
	}

	if (isConnected()) {
		statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, true);
		wifiHandlerLogger.warn("Connection to WiFi lost, reconnecting...");
		trySavedCredentials();
		return;
	}

	reportWifiProgress();
	if (millis() - wifiConnectionTimeout
			< static_cast<uint32_t>(WiFiTimeoutSeconds * 1000)
		&& WiFi.status() == WL_DISCONNECTED) {
		return;
	}

	switch (wifiState) {
		case WiFiReconnectionStatus::NotSetup:  // Wasn't set up
			return;
		case WiFiReconnectionStatus::SavedAttempt:  // Couldn't connect with
													// first set of
													// credentials
			if (!trySavedCredentials()) {
				tryHardcodedCredentials();
			}
			return;
		case WiFiReconnectionStatus::HardcodeAttempt:  // Couldn't connect with
													   // second set of credentials
			if (!tryHardcodedCredentials()) {
				wifiState = WiFiReconnectionStatus::Failed;
			}
			return;
		case WiFiReconnectionStatus::ServerCredAttempt:  // Couldn't connect with
														 // server-sent credentials.
			if (!tryServerCredentials()) {
				wifiState = WiFiReconnectionStatus::Failed;
			}
			return;
		case WiFiReconnectionStatus::Failed:  // Couldn't connect with second set of
											  // credentials or server credentials
			if (startedProvisioning) {
				return;
			}
			wifiHandlerLogger.error(
				"Can't connect from any credentials, error: %d, reason: %s.",
				static_cast<int>(statusToFailure(WiFi.status())),
				statusToReasonString(WiFi.status())
			);
			wifiHandlerLogger.info("Starting wifi provisioning");
			wifiProvisioning.startSearchForProvisioner();
			startedProvisioning = true;
			return;
	}
}

const char* WiFiNetwork::statusToReasonString(wl_status_t status) {
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

WiFiNetwork::WiFiFailureReason WiFiNetwork::statusToFailure(wl_status_t status) {
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

void WiFiNetwork::showConnectionAttemptFailed(const char* type) const {
	wifiHandlerLogger.error(
		"Can't connect from %s credentials, error: %d, reason: %s.",
		type,
		static_cast<int>(statusToFailure(WiFi.status())),
		statusToReasonString(WiFi.status())
	);
}

bool WiFiNetwork::trySavedCredentials() {
	if (WiFi.SSID().length() == 0) {
		wifiHandlerLogger.debug("Skipping saved credentials attempt on 0-length SSID..."
		);
		wifiState = WiFiReconnectionStatus::HardcodeAttempt;
		return false;
	}

	if (wifiState == WiFiReconnectionStatus::SavedAttempt) {
		showConnectionAttemptFailed("saved");

		if (WiFi.status() != WL_DISCONNECTED) {
			return false;
		}

		if (retriedOnG) {
			return false;
		}

		retriedOnG = true;
		wifiHandlerLogger.debug("Trying saved credentials with PHY Mode G...");
		return tryConnecting(true);
	}

	retriedOnG = false;

	wifiState = WiFiReconnectionStatus::SavedAttempt;
	return tryConnecting();
}

bool WiFiNetwork::tryHardcodedCredentials() {
#if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
	if (wifiState == WiFiReconnectionStatus::HardcodeAttempt) {
		showConnectionAttemptFailed("hardcoded");

		if (WiFi.status() != WL_DISCONNECTED) {
			return false;
		}

		if (retriedOnG) {
			return false;
		}

		retriedOnG = true;
		wifiHandlerLogger.debug("Trying hardcoded credentials with PHY Mode G...");
		return tryConnecting(true, WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
	}

	retriedOnG = false;

	wifiState = WiFiReconnectionStatus::HardcodeAttempt;
	return tryConnecting(false, WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
#else
	wifiState = WiFiReconnectionStatus::HardcodeAttempt;
	return false;
#endif
}

bool WiFiNetwork::tryServerCredentials() {
	if (WiFi.status() != WL_DISCONNECTED) {
		return false;
	}

	if (retriedOnG) {
		return false;
	}

	retriedOnG = true;

	return tryConnecting(true);
}

bool WiFiNetwork::tryConnecting(bool phyModeG, const char* SSID, const char* pass) {
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
	wifiConnectionTimeout = millis();
	return true;
}

}  // namespace SlimeVR
