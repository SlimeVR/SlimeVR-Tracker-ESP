/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Eiren Rain & SlimeVR contributors

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
#include "TPSCounter.h"

void TPSCounter::reset() {
	_lastUpdate = _lastAverageUpdate = millis();
	_tps = _averagedTps = 0.0;
	_averageUpdatesCounter = 0;
}

void TPSCounter::update() {
	long time = millis();
	long sinceLastUpdate = millis() - _lastUpdate;
	long sinceAvgLastUpdate = millis() - _lastAverageUpdate;
	_lastUpdate = time;
	_tps = 1000.0 / static_cast<float>(sinceLastUpdate);
	if (sinceAvgLastUpdate > 1000) {
		_lastAverageUpdate = time;
		_averagedTps
			= 1000.0 / static_cast<float>(sinceAvgLastUpdate) * _averageUpdatesCounter;
		_averageUpdatesCounter = 0;
	}
	_averageUpdatesCounter++;
}

float TPSCounter::getAveragedTPS() { return _averagedTps; }

float TPSCounter::getTPS() { return _tps; }
