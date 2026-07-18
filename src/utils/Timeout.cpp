#include "Timeout.h"

Timeout::Timeout(uint64_t durationMillis)
	: durationMillis{durationMillis} {}

void Timeout::restart() { startMillis = millis(); }

bool Timeout::elapsed() const { return millis() - startMillis >= durationMillis; }
