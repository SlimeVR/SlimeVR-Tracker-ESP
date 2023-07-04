#include <Arduino.h>
#include "GlobalVars.h"

namespace SlimeVR
{
  namespace Logging
  {
	bool getRemoteCmdConncted()
	{
		return networkRemoteCmd.isConnected();
	}

	Stream & getRemoteCmdStream()
	{
		return networkRemoteCmd.getStream();
	}
  }
}
