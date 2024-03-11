#include <Arduino.h>

#include "GlobalVars.h"

namespace SlimeVR {
namespace Logging {
bool getRemoteCmdConncted() { return networkRemoteCmd.isConnected() && networkConnection.isConnected(); }

Stream& getRemoteCmdStream() { return networkRemoteCmd.getStream(); }
}  // namespace Logging
}  // namespace SlimeVR
