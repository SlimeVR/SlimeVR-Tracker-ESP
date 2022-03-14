#ifndef STATUS_STATUSMANAGER_H
#define STATUS_STATUSMANAGER_H

#include "Status.h"
#include "logging/Logger.h"

namespace SlimeVR
{
    namespace Status
    {
        class StatusManager
        {
        public:
            void setStatus(Status status, bool value);
            bool hasStatus(Status status);

        private:
            uint32_t m_Status;

            Logging::Logger m_Logger = Logging::Logger("StatusManager");
        };
    }
}

#endif
