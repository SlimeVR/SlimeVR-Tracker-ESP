#include "TempGradientCalculator.h"

#include <functional>

TemperatureGradientCalculator::TemperatureGradientCalculator(
	const std::function<void(float gradient)>& callback
)
	: callback{callback} {}

void TemperatureGradientCalculator::feedSample(float sample, float timeStep) {
	tempSum += sample * timeStep;
}

void TemperatureGradientCalculator::tick() {
	uint64_t nextCheck
		= lastAverageSentMillis + static_cast<uint64_t>(AveragingTimeSeconds * 1e3);

	if (millis() < nextCheck) {
		return;
	}

	lastAverageSentMillis = nextCheck;
	float average = tempSum / AveragingTimeSeconds;
	callback((average - lastTempAverage) / AveragingTimeSeconds);
	lastTempAverage = average;
	tempSum = 0;
}
