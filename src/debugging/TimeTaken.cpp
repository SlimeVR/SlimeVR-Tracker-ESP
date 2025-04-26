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

#include "TimeTaken.h"

namespace SlimeVR::Debugging {

void TimeTakenMeasurer::before(int measurement) { timings[measurement].start = micros(); }
void TimeTakenMeasurer::after(int measurement) { timings[measurement].end = micros(); }

void TimeTakenMeasurer::calculate() {
	for (auto& timing : timings) {
		if (timing.start == 0 && timing.end == 0) {
			continue;  // Skip if not measured
		}
		unsigned long d = timing.end - timing.start;
		timing.min = min(timing.min, d);
		timing.max = max(timing.max, d);
		timing.timeTaken += d;
		timing.count++;
		timing.start = 0;
		timing.end = 0;
	}
	if (lastTimingsPrint < millis() - SecondsBetweenReports * 1000) {
		nextPeriod();
		report();
	}
}

void TimeTakenMeasurer::nextPeriod() {
	// Calculate the time since the last report
	unsigned long sinceLastReportMillis = millis() - lastTimingsPrint;
	lastTimingsPrint = millis();
	for (auto& timing : timings) {
		timing.timeTakenPercent = static_cast<float>(timing.timeTaken) / 1e3f
										/ static_cast<float>(sinceLastReportMillis) * 100;
		timing.timeTotal = sinceLastReportMillis;
		timing.avg = timing.timeTaken / max(timing.count, 1ul);
	}

	// Copy the values to the past measurement
	pasttimings = timings;

	// Reset the current measurement
	for (auto& timing : timings) {
		timing.avg = 0;
		timing.min = -1;
		timing.max = 0;
		timing.timeTaken = 0;
		timing.count = 0;
		timing.start = 0;
		timing.end = 0;
		timing.timeTakenPercent = 0;
		timing.timeTotal = 0;
	}
}

void TimeTakenMeasurer::report() {
	unsigned long totalTime = 0;
	unsigned long count = 0;
    for (size_t i = 0; i < pasttimings.size(); ++i) {
        const auto& timing = pasttimings[i];
        const auto& name = names[i];
		m_Logger.info("%-24s | avg: %5lu us | min: %5lu us | max: %5lu us | time taken: %5lu ms or %5.2f%% of %lu ms count: %lu",
			name.c_str(),
			timing.avg,
			timing.min,
			timing.max,
			static_cast<unsigned long>(timing.timeTaken / 1000),
			timing.timeTakenPercent,
			timing.timeTotal,
			timing.count
		);
		totalTime = max(timing.timeTotal, totalTime);
		count = max (timing.count, count);
	}
	m_Logger.info("Time Total: %lu ms Loops: %lu PrintReport Time: %lu ms", totalTime, count, millis()-lastTimingsPrint);
}

}  // namespace SlimeVR::Debugging
