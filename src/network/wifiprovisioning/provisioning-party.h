#pragma once

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
