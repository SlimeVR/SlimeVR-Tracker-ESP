#ifndef STATUS_STATUSMANAGER_H
#define STATUS_STATUSMANAGER_H

#include "Status.h"
#include "logging/Logger.h"

namespace SlimeVR {
namespace Status {
class StatusManager {
public:
	void setStatus(Status status, bool value);
	bool hasStatus(Status status);
	uint32_t getStatus() { return m_Status; };

private:
	uint32_t m_Status;

	enum class Logs {
		AddedStatus = 0,
		RemovedStatus = 1,
	};
	Logging::Logger<Logs> m_Logger{"StatusManager", "status"};
};
}  // namespace Status
}  // namespace SlimeVR

#endif
