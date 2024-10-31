#include "ResetCounter.h"

#include "GlobalVars.h"

namespace SlimeVR {

#ifdef ESP32
void resetDelayTimerCallback(void *) {
    resetCounter.signalResetDelay();
}

void resetTimeoutTimerCallback(void *) {
    resetCounter.signalResetTimeout();
}
#endif

void ResetCounter::setup() {
#ifdef ESP32

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

#elif defined ESP8266

	timerStartMillis = millis();
	signalResetDelay();

#endif
}

void ResetCounter::update() {
#ifdef ESP8266

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

#ifdef ESP32
    esp_timer_delete(delayTimerHandle);
#endif
}

void ResetCounter::signalResetTimeout() {
    configuration.saveResetCount(0);

	if (resetCount > 1) {
		invokeResetCountCallbacks();
	}

#ifdef ESP32
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
