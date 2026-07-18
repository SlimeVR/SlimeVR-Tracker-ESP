#pragma once

#include <Arduino.h>

#include <cstdint>

class Timeout {
public:
	explicit Timeout(uint64_t durationMillis);
	void restart();
	[[nodiscard]] bool elapsed() const;

private:
	uint64_t startMillis = millis();
	uint64_t durationMillis;
};
