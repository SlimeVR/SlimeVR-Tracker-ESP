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
#pragma once

#include "logging/Logger.h"
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

namespace SlimeVR {

class WiFiNetwork {
public:
	enum class WiFiReconnectionStatus {
		NotSetup = 0,
		SavedAttempt,
		HardcodeAttempt,
		ServerCredAttempt,
		Failed,
		Success
	};

	enum class WiFiFailureReason {
		Timeout = 0,
		SSIDNotFound = 1,
		WrongPassword = 2,
		Unknown = 3,
	};

	[[nodiscard]] bool isConnected() const;
	void setUp();
	void upkeep();
	void setWiFiCredentials(const char* SSID, const char* pass);
	static IPAddress getAddress();
	WiFiReconnectionStatus getWiFiState();

private:
	static constexpr float WiFiTimeoutSeconds = 11;

	void reportWifiProgress();
	void setStaticIPIfDefined();
	void onConnected();

	static String getSSID();
	static String getPassword();

	bool trySavedCredentials();
	bool tryHardcodedCredentials();
	bool tryServerCredentials();
	bool tryConnecting(
		bool phyModeG = false,
		const char* SSID = nullptr,
		const char* pass = nullptr
	);

	void showConnectionAttemptFailed(const char* type) const;

	static const char* statusToReasonString(wl_status_t status);
	static WiFiFailureReason statusToFailure(wl_status_t status);

	unsigned long lastWifiReportTime = 0;
	unsigned long wifiConnectionTimeout = millis();
	bool isWifiConnected = false;
	WiFiReconnectionStatus wifiState = WiFiReconnectionStatus::NotSetup;
	bool retriedOnG = false;
	bool hadWifi = false;
	unsigned long lastRssiSample = 0;

	uint8_t lastFailStatus = 0;

	SlimeVR::Logging::Logger wifiHandlerLogger{"WiFiHandler"};
};

/** Wifi Reconnection Statuses **/
}  // namespace SlimeVR
