#include "Benchmark.h"

#include <limits>

#include "../debug.h"

namespace SlimeVR::Debugging {

Benchmark::Benchmark(const char* name)
	: name{name} {}

void Benchmark::enable() {
	lastReportMillis = millis();
	totalLoops = 0;
	enabled = true;
}

void Benchmark::disable() {
	for (auto* instance : benchmarkInstances) {
		instance->reset();
	}
	enabled = false;
}

bool Benchmark::status() { return enabled; }

void Benchmark::before() {
	if (!enabled) {
		return;
	}
	if (!registered) {
		benchmarkInstances.push_back(this);
		registered = true;
	}

	currentMeasurementStartMicros = micros();
}

void Benchmark::after() {
	if (!enabled) {
		return;
	}
	auto timeTakenMicros = micros() - currentMeasurementStartMicros;

	totalTimeTakenMicros += timeTakenMicros;
	minTimeTakenMicros = std::min(minTimeTakenMicros, timeTakenMicros);
	maxTimeTakenMicros = std::max(maxTimeTakenMicros, timeTakenMicros);
	measurementCount++;
}

void Benchmark::tick() {
	if (!enabled) {
		return;
	}
	totalLoops++;

	auto timeSinceLastReport = millis() - lastReportMillis;
	if (timeSinceLastReport < static_cast<uint32_t>(ReportsIntervalSeconds * 1000)) {
		return;
	}

	auto printStartMicros = micros();
	for (auto* instance : benchmarkInstances) {
		instance->printReport();
		instance->reset();
	}
	auto elapsedMicros = micros() - printStartMicros;
	logger.info(
		"Time total: %lu ms, loops: %u, report print time: %lu us",
		timeSinceLastReport,
		totalLoops,
		elapsedMicros
	);

	lastReportMillis = millis();
	totalLoops = 0;
}

void Benchmark::printReport() const {
	if (measurementCount == 0) {
		return;
	}

	auto timeSinceLastReport = millis() - lastReportMillis;
	uint64_t average = totalTimeTakenMicros / measurementCount;
	float timeTakenPercent = static_cast<float>(totalTimeTakenMicros) / 1000.0f
						   / timeSinceLastReport * 100.0f;

	logger.info(
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
}

void Benchmark::reset() {
	totalTimeTakenMicros = 0;
	minTimeTakenMicros = std::numeric_limits<uint64_t>::max();
	maxTimeTakenMicros = 0;
	measurementCount = 0;
}

uint32_t Benchmark::lastReportMillis = millis();

std::vector<Benchmark*> Benchmark::benchmarkInstances{};

SlimeVR::Logging::Logger Benchmark::logger("Benchmark");

uint32_t Benchmark::totalLoops = 0;

bool Benchmark::enabled = DEBUG_MEASURE_TIME_TAKEN;

}  // namespace SlimeVR::Debugging
