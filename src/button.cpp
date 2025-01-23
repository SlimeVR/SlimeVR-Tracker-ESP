#include "button.h"

#include <driver/rtc_io.h>
#include <esp_sleep.h>

#include <climits>

#ifdef ON_OFF_BUTTON_PIN

void IRAM_ATTR buttonInterruptHandler() {
	if (OnOffButton::getInstance().buttonPressed) {
		return;
	}

	OnOffButton::getInstance().signalPressStart();
}

void OnOffButton::setup() {
	pinMode(
		ON_OFF_BUTTON_PIN,
		BUTTON_ACTIVE_LEVEL == 0 ? INPUT_PULLUP : INPUT_PULLDOWN
	);

	attachInterrupt(
		ON_OFF_BUTTON_PIN,
		buttonInterruptHandler,
		BUTTON_ACTIVE_LEVEL == 0 ? FALLING : RISING
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
}

void OnOffButton::tick() {
	uint64_t elapsed = millis() - lastActivityMillis;

	if (elapsed >= BUTTON_AUTO_SLEEP_TIME_SECONDS * 1e3) {
		goToSleep();
	}

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

#ifdef BUTTON_IMU_ENABLE_PIN
	digitalWrite(BUTTON_IMU_ENABLE_PIN, LOW);
	gpio_hold_en(static_cast<gpio_num_t>(BUTTON_IMU_ENABLE_PIN));
#endif

	ledManager.pattern(100, 100, 3);

	while (buttonPressed && getButton())
		;

	esp_deep_sleep_start();
}

OnOffButton OnOffButton::instance;

#endif
