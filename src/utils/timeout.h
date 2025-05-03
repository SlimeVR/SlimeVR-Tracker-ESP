#pragma once

#include "c_types.h"
namespace SlimeVR {

class TimeOut {
public:
	TimeOut(float lengthSeconds);
	void reset();
	bool elapsed() const;

private:
	uint64 lengthMillis;
	uint64 startMillis = 0;
};

}  // namespace SlimeVR
