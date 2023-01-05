#include "StatusManager.h"

namespace SlimeVR
{
    namespace Status
    {
        void StatusManager::setStatus(Status status, bool value)
        {
            if (value)
            {
                if (m_Status & status)
                {
                    return;
                }

                m_Logger.trace("Added status %s", statusToString(status));

                m_Status |= status;
            }
            else
            {
                if (!(m_Status & status))
                {
                    return;
                }

                m_Logger.trace("Removed status %s", statusToString(status));

                m_Status &= ~status;
            }
        }

        bool StatusManager::hasStatus(Status status)
        {
            return (m_Status & status) == status;
        }

        String StatusManager::getStatusString()
        {
            if(m_Status == 0)
            {
                return "NONE";
            }
            String statusString;
            if(hasStatus(Status::LOADING))
            {
                statusString = "LOADING";
            }
            if(hasStatus(Status::LOW_BATTERY))
            {
                if(!statusString.isEmpty())
                    statusString += "|";
                statusString += "LOW_BATTERY";
            }
            if(hasStatus(Status::IMU_ERROR))
            {
                if(!statusString.isEmpty())
                    statusString += "|";
                statusString += "IMU_ERROR";
            }
            if(hasStatus(Status::WIFI_CONNECTING))
            {
                if(!statusString.isEmpty())
                    statusString += "|";
                statusString += "WIFI_CONNECTING";
            }
            if(hasStatus(Status::SERVER_CONNECTING))
            {
                if(!statusString.isEmpty())
                    statusString += "|";
                statusString += "SERVER_CONNECTING";
            }
            if(statusString.isEmpty())
                statusString = "UNKNOWN";
            return statusString;
        }
    }
}
