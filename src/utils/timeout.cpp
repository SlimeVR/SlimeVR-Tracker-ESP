#include <Arduino.h>

#include "timeout.h"

namespace SlimeVR {

TimeOut::TimeOut(float lengthSeconds)
	: lengthMillis{static_cast<uint64_t>(lengthSeconds * 1000)} {}

void TimeOut::reset() { startMillis = millis(); }

bool TimeOut::elapsed() const { return millis() - startMillis >= lengthMillis; }

}  // namespace SlimeVR
