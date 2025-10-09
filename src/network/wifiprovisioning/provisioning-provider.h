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
