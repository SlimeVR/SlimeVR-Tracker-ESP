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

class ProvisioningTarget : public ProvisioningParty {
public:
	explicit ProvisioningTarget(SlimeVR::Logging::Logger& logger) noexcept;
	void init() final;
	bool tick() final;
	void handleMessage(uint8_t macAddress[6], const uint8_t* data, uint8_t length)
		final;

private:
	static constexpr float ChannelSwitchSeconds = 0.5f;
	static constexpr float SearchTimeoutSeconds = 120.0f;
	static constexpr float RetryIntervalSeconds = 0.25f;
	static constexpr float ServerSearchTimeoutSeconds = 10.0f;

	enum class Status {
		WaitingForProvider,
		ConnectionStarted,
		Connecting,
		SearchingForServer,
		Done,
	};

	void switchChannel(uint8_t channel);
	void handleProvisioningOffer(uint8_t macAddress[6]);
	void startConnection(const char* ssid, const char* password);

	template <typename Packet>
	void sendAndWaitForAck(Packet packet) {
		if (waitingForAck && !retryTimeout.elapsed()) {
			return;
		}

		sendMessage(providerMac, packet);
		retryTimeout.reset();
		waitingForAck = true;
	}

	TimeOut searchTimeout{SearchTimeoutSeconds};
	TimeOut channelSwitchTimeout{ChannelSwitchSeconds};
	TimeOut retryTimeout{RetryIntervalSeconds};
	TimeOut serverSearchTimeout{ServerSearchTimeoutSeconds};
	uint8_t currentChannel = 0;
	Status status = Status::WaitingForProvider;
	uint8_t providerMac[6] = {};
	bool waitingForAck = false;
	bool searchingForServer = false;
};

}  // namespace SlimeVR::Network
