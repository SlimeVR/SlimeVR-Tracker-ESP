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

#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>

#include <memory>

#include "logging/Logger.h"
#include "utils/Timeout.h"

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#endif

namespace SlimeVR::Network::WiFiComms {

class WiFiConnection {
public:
	void init();
	void tick();
	void setWiFiCredentials(const char* SSID, const char* pass);
	void reset();

	[[nodiscard]] bool isConnected() const;

	bool beginUDPPacket();
	bool write(const uint8_t* data, size_t length);
	bool endUDPPacket();

	size_t readUDPPacket(uint8_t* buffer, size_t maxSize);

	void setIPAddress(IPAddress&& address);
	[[nodiscard]] IPAddress getIPAddress() const;

	void acceptHandshake();

	[[nodiscard]] WiFiUDP& getUDPInstance();

	enum class WiFiReconnectionStatus {
		NotSetup = 0,
		SavedAttempt,
		HardcodeAttempt,
		ServerCredAttempt,
		Failed,
		Success
	};

	[[nodiscard]] WiFiReconnectionStatus getState() const;

private:
	constexpr static float WiFiTimeoutSeconds = 11;
	constexpr static uint64_t WiFiReportIntervalMillis = 1000;

	enum class WiFiFailureReason {
		Timeout = 0,
		SSIDNotFound = 1,
		WrongPassword = 2,
		Unknown = 3,
	};

	class WiFiState {
	public:
		WiFiState(WiFiConnection& context, WiFiReconnectionStatus status);

		virtual void tick() = 0;

		[[nodiscard]] WiFiReconnectionStatus toStatus() const;

	protected:
		WiFiConnection& context;
		WiFiReconnectionStatus status;
	};

	class ConnectingState : public WiFiState {
	public:
		ConnectingState(
			WiFiConnection& context,
			WiFiReconnectionStatus status,
			const char* attemptName
		);
		void tick() final;
		bool tryConnecting(
			bool phyModeG = false,
			const char* SSID = nullptr,
			const char* pass = nullptr
		);
		void setStaticIPIfDefined();
		void showConnectionAttemptFailed(const char* type) const;

	protected:
		virtual bool attempt(bool retry) = 0;
		virtual std::unique_ptr<WiFiState> nextState() = 0;

	private:
		void reportWifiProgress();

		Timeout wifiTimeout{static_cast<uint64_t>(WiFiTimeoutSeconds * 1000)};
		bool setup = false;
		bool retriedOnG = false;
		const char* attemptName;
	};

	class NotSetupState final : public WiFiState {
	public:
		explicit NotSetupState(WiFiConnection& context);
		void tick() final;
	};

	class SavedCredentialsAttemptState final : public ConnectingState {
	public:
		explicit SavedCredentialsAttemptState(WiFiConnection& context);
		bool attempt(bool retry) final;
		std::unique_ptr<WiFiState> nextState() final;

	private:
	};

	class HardcodedCredentialsAttemptState final : public ConnectingState {
	public:
		explicit HardcodedCredentialsAttemptState(WiFiConnection& context);
		bool attempt(bool retry) final;
		std::unique_ptr<WiFiState> nextState() final;
	};

	class ServerCredentialsAttemptState final : public ConnectingState {
	public:
		explicit ServerCredentialsAttemptState(
			WiFiConnection& context,
			const char* SSID,
			const char* pass
		);

	protected:
		bool attempt(bool retry) final;
		std::unique_ptr<WiFiState> nextState() final;

	private:
		const char* SSID;
		const char* pass;
	};

	class FailedState final : public WiFiState {
	public:
		explicit FailedState(WiFiConnection& context);
		void tick() final;

	private:
		Timeout wifiTimeout{static_cast<uint64_t>(WiFiTimeoutSeconds * 1000)};
	};

	class ConnectedState final : public WiFiState {
	public:
		explicit ConnectedState(WiFiConnection& context);
		void tick() final;

	private:
	};

	void transitionState(std::unique_ptr<WiFiState>&& newState);

	static String getSSID();
	static String getPassword();
	static const char* statusToReasonString(wl_status_t status);
	static WiFiFailureReason statusToFailure(wl_status_t status);

	WiFiUDP UDP;

	int serverPort = 6969;
	IPAddress serverHost = IPAddress(255, 255, 255, 255);

	std::unique_ptr<WiFiState> currentState;
	Timeout wifiReportTimeout{WiFiReportIntervalMillis};
	bool hadWifi = false;

	SlimeVR::Logging::Logger logger{"WiFiConnection"};
};

}  // namespace SlimeVR::Network::WiFiComms
