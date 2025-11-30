#pragma once

#include <IPAddress.h>
#include <WiFiUdp.h>

#include <cstdint>
#include <optional>

#include "logging/Logger.h"

namespace SlimeVR::Network {

class MDNSResolver {
public:
	explicit MDNSResolver(WiFiUDP& udp, SlimeVR::Logging::Logger& logger);
	void searchForMDNS();
	static bool isPacketMDNS(const uint8_t* buffer);
	std::optional<IPAddress> parseMDNSPacket(const uint8_t* buffer) const;

private:
	constexpr static float MDNSSearchIntervalSeconds = 5;
	const char* MDNSHostName = "slimevr-server";

	void sendMDNSQuery();

	WiFiUDP& udp;
	SlimeVR::Logging::Logger& logger;
	uint64_t lastMDNSQueryMillis = 0;
};

}  // namespace SlimeVR::Network
