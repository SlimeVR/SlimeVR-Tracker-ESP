#include "Status.h"

namespace SlimeVR {
namespace Status {
const char* statusToString(Status status) {
	switch (status) {
		case LOADING:
			return "LOADING";
		case LOW_BATTERY:
			return "LOW_BATTERY";
		case IMU_ERROR:
			return "IMU_ERROR";
		case WIFI_CONNECTING:
			return "WIFI_CONNECTING";
		case SERVER_CONNECTING:
			return "SERVER_CONNECTING";
		default:
			return "UNKNOWN";
	}
}
}  // namespace Status
}  // namespace SlimeVR
