#include "TPSCounter.h"

void TPSCounter::reset()
{
	_lastUpdate = _lastAverageUpdate = millis();
	_tps = _averagedTps = 0.0;
	_averageUpdatesCounter = 0;
}

void TPSCounter::update()
{
	long time = millis();
	long sinceLastUpdate = millis() - _lastUpdate;
	long sinceAvgLastUpdate = millis() - _lastAverageUpdate;
	_lastUpdate = time;
	_tps = 1000.0 / static_cast<float>(sinceLastUpdate);
	if(sinceAvgLastUpdate > 1000)
	{
		_lastAverageUpdate = time;
		_averagedTps = 1000.0 / static_cast<float>(sinceAvgLastUpdate) * _averageUpdatesCounter;
		_averageUpdatesCounter = 0;
	}
	_averageUpdatesCounter++;
}

float TPSCounter::getAveragedTPS()
{
	return _averagedTps;
}

float TPSCounter::getTPS()
{
	return _tps;
}
