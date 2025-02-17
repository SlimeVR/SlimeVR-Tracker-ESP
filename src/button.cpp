#include "button.h"

#include <climits>

#if ESP32
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#endif

#include "GlobalVars.h"

#ifdef ON_OFF_BUTTON_PIN

void IRAM_ATTR buttonInterruptHandler() {
	if (OnOffButton::getInstance().buttonPressed) {
		return;
	}

	OnOffButton::getInstance().signalPressStart();
}

void OnOffButton::setup() {
#if ESP8266
	digitalWrite(D0, LOW);
	pinMode(D0, OUTPUT);
	pinMode(ON_OFF_BUTTON_PIN, INPUT);
#endif

#if ESP32
	pinMode(
		ON_OFF_BUTTON_PIN,
		BUTTON_ACTIVE_LEVEL == 0 ? INPUT_PULLUP : INPUT_PULLDOWN
	);

	esp_deep_sleep_enable_gpio_wakeup(
		1 << ON_OFF_BUTTON_PIN,
		BUTTON_ACTIVE_LEVEL == 0 ? ESP_GPIO_WAKEUP_GPIO_LOW : ESP_GPIO_WAKEUP_GPIO_HIGH
	);

#ifdef BUTTON_IMU_ENABLE_PIN
	pinMode(BUTTON_IMU_ENABLE_PIN, OUTPUT);
	gpio_hold_dis(static_cast<gpio_num_t>(BUTTON_IMU_ENABLE_PIN));
	digitalWrite(BUTTON_IMU_ENABLE_PIN, BUTTON_IMU_ENABLE_ACTIVE_LEVEL);
#endif

	gpio_deep_sleep_hold_en();
#endif

	attachInterrupt(
		ON_OFF_BUTTON_PIN,
		buttonInterruptHandler,
		BUTTON_ACTIVE_LEVEL == 0 ? FALLING : RISING
	);
}

void OnOffButton::tick() {
#ifdef BUTTON_AUTO_SLEEP_TIME_SECONDS
	uint64_t autoSleepElapsed = millis() - lastActivityMillis;

	if (autoSleepElapsed >= BUTTON_AUTO_SLEEP_TIME_SECONDS * 1e3) {
		goToSleep();
	}
#endif

#if defined(BUTTON_BATTERY_VOLTAGE_THRESHOLD) && BATTERY_MONITOR == BAT_EXTERNAL
	if (battery.getVoltage() >= BUTTON_BATTERY_VOLTAGE_THRESHOLD) {
		batteryBad = false;
	} else if (!batteryBad) {
		batteryBad = true;
		batteryBadSinceMillis = millis();
	}

	if (batteryBad) {
		uint64_t batteryBadElapsed = millis() - batteryBadSinceMillis;

		if (batteryBadElapsed >= batteryBadTimeoutSeconds * 1e3) {
			goToSleep();
		}
	}
#endif

	if (!buttonPressed) {
		return;
	}

	uint64_t timeTaken = millis() - buttonPressStartMillis;

	if (getButton() && timeTaken < longPressSeconds * 1e3) {
		return;
	}

	if (timeTaken >= longPressSeconds * 1e3) {
		goToSleep();
	}

	buttonPressed = false;
}

void OnOffButton::onBeforeSleep(std::function<void()> callback) {
	callbacks.push_back(callback);
}

void OnOffButton::signalTrackerMoved() { lastActivityMillis = millis(); }

OnOffButton& OnOffButton::getInstance() { return instance; }

bool OnOffButton::getButton() {
	static constexpr uint8_t circularBufferBitCount
		= sizeof(buttonCircularBuffer) * CHAR_BIT;

	bool isPressed = digitalRead(ON_OFF_BUTTON_PIN) == BUTTON_ACTIVE_LEVEL;
	buttonCircularBuffer = buttonCircularBuffer << 1 | isPressed;

	auto popCount = __builtin_popcount(buttonCircularBuffer);
	return popCount >= circularBufferBitCount / 2;
}

void OnOffButton::signalPressStart() {
	buttonPressed = true;
	buttonPressStartMillis = millis();
	buttonCircularBuffer = UINT64_MAX;
}

void OnOffButton::emitOnBeforeSleep() {
	for (auto& callback : callbacks) {
		callback();
	}
}

void OnOffButton::goToSleep() {
	emitOnBeforeSleep();

#if defined(BUTTON_IMU_ENABLE_PIN) && ESP32
	digitalWrite(BUTTON_IMU_ENABLE_PIN, LOW);
	gpio_hold_en(static_cast<gpio_num_t>(BUTTON_IMU_ENABLE_PIN));
#endif

	ledManager.pattern(100, 100, 3);

	while (buttonPressed && getButton())
		;

#if ESP8266
	ESP.deepSleep(0);
#elif ESP32
	esp_deep_sleep_start();
#endif
}

OnOffButton OnOffButton::instance;

#endif
