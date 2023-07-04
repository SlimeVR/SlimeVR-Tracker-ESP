#ifndef LOGGING_RETOMELOGHELPER_H
#define LOGGING_RETOMELOGHELPER_H

#include <Arduino.h>

namespace SlimeVR
{
  namespace Logging
  {
	bool getRemoteCmdConncted();
	Stream & getRemoteCmdStream();
  }
}

#endif // LOGGING_RETOMELOGHELPER_H
