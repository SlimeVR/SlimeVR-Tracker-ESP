#pragma once

#include <cstdint>

namespace SlimeVR {

class TimeOut {
public:
	TimeOut(float lengthSeconds);
	void reset();
	[[nodiscard]] bool elapsed() const;

private:
	uint64_t lengthMillis;
	uint64_t startMillis = 0;
};

}  // namespace SlimeVR
