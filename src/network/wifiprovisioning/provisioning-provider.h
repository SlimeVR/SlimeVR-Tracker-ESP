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

#include <cstdint>

#include "logging/Logger.h"
#include "network/wifiprovisioning/provisioning-party.h"
#include "utils/timeout.h"
namespace SlimeVR::Network {

class ProvisioningProvider : public ProvisioningParty {
public:
	explicit ProvisioningProvider(SlimeVR::Logging::Logger& logger) noexcept;
	void init() final;
	bool tick() final;
	void handleMessage(uint8_t macAddress[6], const uint8_t* data, uint8_t length)
		final;

private:
	static constexpr float ProvisioningTimeoutSeconds = 60.0f;
	static constexpr float BroadcasstIntervalSeconds = 0.1f;

	void handleProvisioningRequest(uint8_t macAddress[6], const char* password);

	TimeOut provisioningTimeout{ProvisioningTimeoutSeconds};
	TimeOut messageTimeout{BroadcasstIntervalSeconds};
};

}  // namespace SlimeVR::Network
