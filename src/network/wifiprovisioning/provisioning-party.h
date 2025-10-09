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
#pragma once

#include <Arduino.h>

#include <cstdint>
#include <optional>
#if ESP8266
#include <espnow.h>
#elif ESP32
#include <esp_now.h>
#endif

#ifndef MACSTR
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#endif

#include "logging/Logger.h"
namespace SlimeVR::Network {

class ProvisioningParty {
public:
	explicit ProvisioningParty(SlimeVR::Logging::Logger& logger) noexcept;
	virtual ~ProvisioningParty() = default;
	virtual void init() = 0;
	virtual bool tick() = 0;
	virtual void
	handleMessage(uint8_t macAddress[6], const uint8_t* data, uint8_t length)
		= 0;

protected:
	uint8_t BroadcastMacAddress[6]{0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	void addPeer(uint8_t macAddress[6]) const;
	void removePeer(uint8_t macAddress[6]) const;

	template <typename Packet>
	bool sendMessage(uint8_t receiverMac[6], Packet data) const {
		return esp_now_send(
				   receiverMac,
				   reinterpret_cast<uint8_t*>(&data),
				   sizeof(Packet)
			   )
			== 0;
	}

	SlimeVR::Logging::Logger& logger;
};

}  // namespace SlimeVR::Network
