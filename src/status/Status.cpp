#include "Status.h"

namespace SlimeVR
{
    namespace Status
    {
        const char *statusToString(Status status)
        {
            switch (status)
            {
            case LOADING:
                return "LOADING";
            case LOW_BATTERY:
                return "LOW_BATTERY";
            case IMU_ERROR:
                return "IMU_ERROR";
            case WIFI_CONNECTING:
                return "WIFI_CONNECTING";
            case SERVER_SEARCHING:
                return "SERVER_SEARCHING";
            case SERVER_CONNECTING:
                return "SERVER_CONNECTING";
            case BATTERY_CHARGING:
                return "BATTERY_CHARGING";
            case BATTERY_CHARGE_COMPLETE:
                return "BATTERY_CHARGE_COMPLETE";
            default:
                return "UNKNOWN";
            }
        }
    }
}
