/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 Gorbit99, unlogisch04 & SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <Arduino.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "../logging/Logger.h"

namespace SlimeVR::Debugging {

class Benchmark {
public:
	Benchmark(const char* name);
	Benchmark(const Benchmark& other) = delete;
	Benchmark(Benchmark&& other) = delete;
	Benchmark& operator=(const Benchmark& other) = delete;
	Benchmark& operator=(Benchmark&& other) = delete;

	void before();
	void after();

private:
	static constexpr float ReportsIntervalSeconds = 10.0f;

	void printReport();
	void reset();

	uint32_t lastReportMillis = millis();

	uint64_t currentMeasurementStartMicros = 0;

	uint64_t totalTimeTakenMicros = 0;
	uint64_t minTimeTakenMicros = std::numeric_limits<uint64_t>::max();
	uint64_t maxTimeTakenMicros = 0;
	uint32_t measurementCount = 0;

	const char* name;

	SlimeVR::Logging::Logger m_Logger = SlimeVR::Logging::Logger("Benchmark");
};

}  // namespace SlimeVR::Debugging
