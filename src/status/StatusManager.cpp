#include "StatusManager.h"

namespace SlimeVR
{
    namespace Status
    {
        void StatusManager::setStatus(Status status, bool value)
        {
            if (value)
            {
                m_Logger.trace("Removed status %s", statusToString(status));

                m_Status |= status;
            }
            else
            {
                m_Logger.trace("Added status %s", statusToString(status));

                m_Status &= ~status;
            }
        }

        bool StatusManager::hasStatus(Status status)
        {
            return (m_Status & status) == status;
        }
    }
}
