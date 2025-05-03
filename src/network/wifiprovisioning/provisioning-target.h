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
	void stop() final;

private:
	static constexpr float ChannelSwitchSeconds = 0.5f;
	static constexpr float SearchTimeoutSeconds = 120.0f;

	enum class Status {
		WaitingForProvider,
		Authenticating,
		Connecting,
		SearchingForServer,
	};

	void switchChannel(uint8_t channel);
	void handleProvisioningOffer(uint8_t macAddress[6]);
	void startConnection(const char* ssid, const char* password);

	TimeOut searchTimeout{SearchTimeoutSeconds};
	TimeOut channelSwitchTimeout{ChannelSwitchSeconds};
	uint8_t currentChannel = 0;
	Status status = Status::WaitingForProvider;
	uint8_t providerMac[6] = {};
};

}  // namespace SlimeVR::Network
