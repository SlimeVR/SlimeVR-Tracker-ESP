/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors
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

#include <cstdint>

#include "logging/Logger.h"

namespace SlimeVR::Debugging {

struct TimingMeasurement {
	unsigned long avg = 0;
	unsigned long min = -1;
	unsigned long max = 0;
	unsigned long timeTaken = 0;
	float timeTakenPercent = 0;
	unsigned long timeTotal = 0;
	unsigned long count = 0;
	unsigned long start = 0;
	unsigned long end = 0;
};

/*
 * Usage:
 *
 * std::vector<const char*> timingNames = {
 *    "tpsCounter.update()",  <- 0
 *    "globalTimer.tick()"    <- 1
 * }
 * TimeTakenMeasurer measurer{timingNames};
 *
 * ...
 *
 * measurer.before(0);
 * thing to measure
 * measurer.after(0);
 */
class TimeTakenMeasurer {
public:
	explicit TimeTakenMeasurer(const std::vector<const char*>& names) {
		for (const auto& name : names) {
			this->names.push_back(name);
			this->timings.push_back({0, 2 ^ 64, 0, 0, 0, 0, 0, 0});
			//this->timingPoints.push_back({0, 0});
		}
	}

	void before(int measurement);
	void after(int measurement);
	void calculate();


private:
	void nextPeriod();
	void report();
	static constexpr float SecondsBetweenReports = 10.0f;
	unsigned long lastTimingsPrint = 0;

	std::vector<String> names;
	std::vector<TimingMeasurement> timings;

	std::vector<TimingMeasurement> pasttimings;
	SlimeVR::Logging::Logger m_Logger = SlimeVR::Logging::Logger("TimeTaken");
	unsigned long  lastTimeTakenReportMillis = 0;
};

}  // namespace SlimeVR::Debugging
