#include <cstdint>
#include <optional>

#include "logging/Logger.h"
#include "provisioning-party.h"

namespace SlimeVR::Network {

ProvisioningParty::ProvisioningParty(SlimeVR::Logging::Logger& logger) noexcept
	: logger{logger} {}

void ProvisioningParty::addPeer(uint8_t macAddress[6]) const {
#if ESP8266
	esp_now_add_peer(macAddress, ESP_NOW_ROLE_COMBO, 0, nullptr, 0);
#elif ESP32
	esp_now_peer_info_t peer{};
	memcpy(peer.peer_addr, macAddress, sizeof(uint8_t[6]));
	peer.channel = 0;
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;
	esp_now_add_peer(&peer);
#endif
}

void ProvisioningParty::removePeer(uint8_t macAddress[6]) const {
	esp_now_del_peer(macAddress);
}

}  // namespace SlimeVR::Network
