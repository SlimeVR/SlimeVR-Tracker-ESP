/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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

#include "./logging/Logger.h"

#include <cstdint>
#include <functional>
#include <vector>
#include <esp_timer.h>
#include <esp_system.h>

namespace SlimeVR {

class ResetCounter {
public:
	void setup();
	void onResetCount(std::function<void(uint32_t)> callback);

private:
	void signalResetDelay();
	void signalResetTimeout();
	void invokeResetCountCallbacks();

	std::vector<std::function<void(uint32_t)>> resetCountCallbacks;
	uint32_t resetCount = 0;

	esp_timer_handle_t delayTimerHandle;
	esp_timer_handle_t timeoutTimerHandle;

	Logging::Logger m_Logger = Logging::Logger("ESPNowConnection");

	static constexpr float resetDelaySeconds = 0.05f;
	static constexpr float resetTimeoutSeconds = 3.0f;

	friend void resetDelayTimerCallback(void *);
	friend void resetTimeoutTimerCallback(void *);
};

} // namespace SlimeVR
