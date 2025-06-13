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
#ifndef SLIMEVR_WIFIPROVISIONING_H_
#define SLIMEVR_WIFIPROVISIONING_H_

#include "logging/Logger.h"

#if ESP32
#include <esp_now.h>
#endif

namespace SlimeVR {

class WiFiProvisioning {
public:
	bool startProvisioning();
	void stopProvisioning();

	bool startSearchForProvisioner();
	void stopSearchForProvisioner();

	void tick();

	[[nodiscard]] bool isSearching() const;

private:
	bool initEspnow();
	void handleMessage(uint8_t* macAddress, const uint8_t* data, uint8_t length);
	void handleSearchMessage(uint8_t* macAddress, const uint8_t* data, uint8_t length);
	void handleReplyMessage(uint8_t* macAddress, const uint8_t* data, uint8_t length);

	static uint8_t addPeer(uint8_t* macAddress);
	static void delPeer(uint8_t* macAddress);

	uint8_t BroadcastMacAddress[6]{0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	static constexpr float SearchIntervalSeconds = 0.1f;
	static constexpr float SearchTimeoutSeconds = 120.0f;
	static constexpr float ProvisioningTimeoutSeconds = 60.0f;

	enum class ProvisioningStatus { Idle, Provisioning, Searching };

	enum class ProvisioningMessage : uint8_t {
		Search = 0,
		Reply = 1,
	};

	ProvisioningStatus status = ProvisioningStatus::Idle;
	SlimeVR::Logging::Logger logger{"WiFiProvisioning"};
	uint32_t startMillis = 0;
	uint32_t lastSearchSentMillis = 0;
	uint8_t nextSearchChannel = 1;

#if ESP8266
	friend void espnowReceiveCallback(uint8_t*, uint8_t*, uint8_t);
#elif ESP32
	friend void espnowReceiveCallback(
		const esp_now_recv_info_t* senderInfo,
		const uint8_t* data,
		int dataLen
	);
#endif
};

}  // namespace SlimeVR

#endif  // SLIMEVR_WIFIPROVISIONING_H_
