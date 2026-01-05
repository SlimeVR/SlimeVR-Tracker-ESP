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

#include <Arduino.h>

#include <cstdint>
#include <functional>
#include <vector>

#include "GlobalVars.h"

#ifdef ON_OFF_BUTTON_PIN

class OnOffButton {
public:
	void setup();
	void tick();
	void onBeforeSleep(std::function<void()> callback);
	void signalTrackerMoved();

	static OnOffButton& getInstance();

private:
	OnOffButton() = default;
	static OnOffButton instance;

	static constexpr float longPressSeconds = 1.0f;
	static constexpr float batteryBadTimeoutSeconds = 10.0f;

	bool getButton();
	void signalPressStart();
	void emitOnBeforeSleep();
	void goToSleep();

	bool buttonPressed = false;
	uint64_t buttonPressStartMillis = 0;
	uint64_t buttonCircularBuffer = 0;
	std::vector<std::function<void()>> callbacks;
	bool batteryBad = false;
	uint64_t batteryBadSinceMillis = 0;
	bool wasReleasedInitially = false;

	uint64_t lastActivityMillis = 0;

	friend void IRAM_ATTR buttonInterruptHandler();
};

#endif
