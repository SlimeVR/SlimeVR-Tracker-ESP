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
#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "logging/Logger.h"
#include "provisioning-party.h"

#if ESP32
#include <esp_now.h>
#endif

namespace SlimeVR::Network {

class WiFiProvisioning {
public:
	bool startProvisioning();
	void stopProvisioning();

	bool startSearchForProvider();
	void stopSearchForProvider();

	void tick();

private:
	bool initEspnow();
	void handleMessage(uint8_t* macAddress, const uint8_t* data, uint8_t length);

	std::unique_ptr<ProvisioningParty> role;
	SlimeVR::Logging::Logger logger{"WiFiProvisioning"};

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

}  // namespace SlimeVR::Network
