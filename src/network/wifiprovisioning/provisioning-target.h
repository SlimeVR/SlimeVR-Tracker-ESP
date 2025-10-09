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
