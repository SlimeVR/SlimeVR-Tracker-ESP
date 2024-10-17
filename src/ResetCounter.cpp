#include "ResetCounter.h"

#include "GlobalVars.h"

namespace SlimeVR {

void resetDelayTimerCallback(void *) {
    resetCounter.signalResetDelay();
}

void resetTimeoutTimerCallback(void *) {
    resetCounter.signalResetTimeout();
}

void ResetCounter::setup() {
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
    delayTimerHandle;
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
    timeoutTimerHandle;
    if (esp_timer_create(&timeoutArgs, &timeoutTimerHandle) != ESP_OK) {
        m_Logger.error("Couldn't create timeout timer!");
        return;
    }
    if (esp_timer_start_once(timeoutTimerHandle, resetTimeoutSeconds * 1e6) != ESP_OK) {
        m_Logger.error("Couldn't start timeout timer!");
        return;
    }
}

void ResetCounter::signalResetDelay() {
    if (!configuration.loadResetCount(&resetCount)) {
        resetCount = 0;
    }
    resetCount++;
    configuration.saveResetCount(resetCount);

    esp_timer_delete(delayTimerHandle);
}

void ResetCounter::signalResetTimeout() {
    configuration.saveResetCount(0);

	if (resetCount > 1) {
		invokeResetCountCallbacks();
	}

    esp_timer_delete(timeoutTimerHandle);
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
