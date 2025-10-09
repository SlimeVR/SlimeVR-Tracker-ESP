#include "provisioning-packets.h"

namespace SlimeVR::Network::ProvisioningPackets {

const char* statusToCstr(ConnectionStatus status) {
	switch (status) {
		case ConnectionStatus::Connecting:
			return "Connecting";
		case ConnectionStatus::Connected:
			return "Connected";
		case ConnectionStatus::ServerFound:
			return "Server Found";
	}
	return "Unknown";
}

const char* errorToCstr(ConnectionError error) {
	switch (error) {
		case ConnectionError::ConnectionFailed:
			return "Connection Failed";
		case ConnectionError::ServerNotFound:
			return "Server Not Found";
	}
	return "Unknown";
}

}  // namespace SlimeVR::Network::ProvisioningPackets
