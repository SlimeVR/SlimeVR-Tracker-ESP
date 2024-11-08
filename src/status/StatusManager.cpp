#include "StatusManager.h"

namespace SlimeVR {
namespace Status {
void StatusManager::setStatus(Status status, bool value) {
	if (value) {
		if (m_Status & status) {
			return;
		}

		m_Logger.trace("Added status %s", statusToString(status));

		m_Status |= status;
	} else {
		if (!(m_Status & status)) {
			return;
		}

		m_Logger.trace("Removed status %s", statusToString(status));

		m_Status &= ~status;
	}
}

bool StatusManager::hasStatus(Status status) { return (m_Status & status) == status; }
}  // namespace Status
}  // namespace SlimeVR
