#include "Benchmark.h"

#include <limits>

namespace SlimeVR::Debugging {

Benchmark::Benchmark(const char* name)
	: name{name} {}

void Benchmark::before() {
#if DEBUG_MEASURE_TIME_TAKEN
	currentMeasurementStartMicros = micros();
#endif
}

void Benchmark::after() {
#if DEBUG_MEASURE_TIME_TAKEN
	auto timeTakenMicros = micros() - currentMeasurementStartMicros;

	totalTimeTakenMicros += timeTakenMicros;
	minTimeTakenMicros = std::min(minTimeTakenMicros, timeTakenMicros);
	maxTimeTakenMicros = std::max(maxTimeTakenMicros, timeTakenMicros);
	measurementCount++;

	auto timeSinceLastReport = millis() - lastReportMillis;
	if (timeSinceLastReport >= static_cast<uint32_t>(ReportsIntervalSeconds * 1000)) {
		printReport();
		reset();
	}
#endif
}

void Benchmark::printReport() {
#if DEBUG_MEASURE_TIME_TAKEN
	if (measurementCount == 0) {
		return;
	}

	auto timeSinceLastReport = millis() - lastReportMillis;
	uint64_t average = totalTimeTakenMicros / measurementCount;
	float timeTakenPercent = static_cast<float>(totalTimeTakenMicros) / 1000.0f
						   / timeSinceLastReport * 100.0f;
	m_Logger.info(
		"%-24s | "
		"avg: %5llu us | "
		"min: %5llu us | "
		"max: %5llu us | "
		"time taken: %5llu ms or %5.2f%% of %lu ms | count: %u",
		name,
		average,
		minTimeTakenMicros,
		maxTimeTakenMicros,
		totalTimeTakenMicros / 1000,
		timeTakenPercent,
		timeSinceLastReport,
		measurementCount
	);

	lastReportMillis = millis();
#endif
}

void Benchmark::reset() {
#if DEBUG_MEASURE_TIME_TAKEN
	totalTimeTakenMicros = 0;
	minTimeTakenMicros = std::numeric_limits<uint64_t>::max();
	maxTimeTakenMicros = 0;
	measurementCount = 0;
#endif
}

}  // namespace SlimeVR::Debugging
