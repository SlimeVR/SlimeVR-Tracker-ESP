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

#include "ResetCounter.h"

#include "GlobalVars.h"

namespace SlimeVR {

#if ESP32
void resetDelayTimerCallback(void *) {
    resetCounter.signalResetDelay();
}

void resetTimeoutTimerCallback(void *) {
    resetCounter.signalResetTimeout();
}
#endif

void ResetCounter::setup() {
#if ESP32

    if (esp_timer_early_init() != ESP_OK) {
        m_Logger.error("Couldn't initialize timer!");
        return;
    }

    esp_timer_create_args_t delayArgs{
        .callback = resetDelayTimerCallback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "RESET COUNTER DELAY",
        .skip_unhandled_events = true,
    };

    if (esp_timer_create(&delayArgs, &delayTimerHandle) != ESP_OK) {
        m_Logger.error("Couldn't create delay timer!");
        return;
    }
    if (esp_timer_start_once(delayTimerHandle, resetDelaySeconds * 1e6) != ESP_OK) {
        m_Logger.error("Couldn't start delay timer!");
        return;
    }

    esp_timer_create_args_t timeoutArgs{
        .callback = resetTimeoutTimerCallback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "RESET COUNTER TIMEOUT",
        .skip_unhandled_events = true,
    };

    if (esp_timer_create(&timeoutArgs, &timeoutTimerHandle) != ESP_OK) {
        m_Logger.error("Couldn't create timeout timer!");
        return;
    }
    if (esp_timer_start_once(timeoutTimerHandle, resetTimeoutSeconds * 1e6) != ESP_OK) {
        m_Logger.error("Couldn't start timeout timer!");
        return;
    }

#elif ESP8266

	timerStartMillis = millis();
	signalResetDelay();

#endif
}

void ResetCounter::update() {
#if ESP8266

	if (timeoutElapsed) {
		return;
	}

	uint32_t elapsedMillis = millis() - timerStartMillis;

	if (elapsedMillis < resetTimeoutSeconds * 1e3) {
		return;
	}

	signalResetTimeout();
	timeoutElapsed = true;

#endif
}

void ResetCounter::signalResetDelay() {
    if (!configuration.loadResetCount(&resetCount)) {
        resetCount = 0;
    }
    resetCount++;
    configuration.saveResetCount(resetCount);

#if ESP32
    esp_timer_delete(delayTimerHandle);
#endif
}

void ResetCounter::signalResetTimeout() {
    configuration.saveResetCount(0);

	if (resetCount > 1) {
		invokeResetCountCallbacks();
	}

#if ESP32
    esp_timer_delete(timeoutTimerHandle);
#endif
}

void ResetCounter::onResetCount(std::function<void(uint32_t)> callback) {
    resetCountCallbacks.push_back(callback);
}

void ResetCounter::invokeResetCountCallbacks() {
    for (auto &callback : resetCountCallbacks) {
        callback(resetCount);
    }
}

} // namespace SlimeVR
