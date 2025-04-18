#include "onOffButton.h"

#include "globalVars.h"

void SlimeVR::OnOffButton::setup() {
#ifdef ON_OFF_BUTTON

#ifdef ESP8266
	digitalWrite(D0, LOW);
	pinMode(D0, OUTPUT);
	pinMode(ON_OFF_BUTTON, INPUT);
#endif

#ifdef ESP32
	pinMode(
		ON_OFF_BUTTON,
		ON_OFF_BUTTON_ACTIVE_LEVEL == LOW ? INPUT_PULLUP : INPUT_PULLDOWN
	);
	esp_deep_sleep_enable_gpio_wakeup(
		1 << ON_OFF_BUTTON,
		ON_OFF_BUTTON_ACTIVE_LEVEL == LOW ? ESP_GPIO_WAKEUP_GPIO_LOW
										  : ESP_GPIO_WAKEUP_GPIO_HIGH
	);
#endif

#endif
}

void SlimeVR::OnOffButton::update() {
#ifdef ON_OFF_BUTTON

	if (digitalRead(ON_OFF_BUTTON) != ON_OFF_BUTTON_ACTIVE_LEVEL) {
		return;
	}

	uint32_t ringBuffer = 0;
	long startTime = millis();
	while (millis() - startTime < ON_OFF_BUTTON_HOLD_TIME_MS) {
		ringBuffer <<= 1;
		ringBuffer |= digitalRead(ON_OFF_BUTTON) != ON_OFF_BUTTON_ACTIVE_LEVEL;

		int popCount = __builtin_popcount(ringBuffer);
		if (popCount > 16) {
			return;
		}
		delay(1);
	}

	ledManager.off();
	for (int i = 0; i < 3; i++) {
		ledManager.on();
		delay(100);
		ledManager.off();
		delay(100);
	}

	ringBuffer = 0;
	while (__builtin_popcount(ringBuffer) <= 16) {
		ringBuffer <<= 1;
		ringBuffer |= digitalRead(ON_OFF_BUTTON) != ON_OFF_BUTTON_ACTIVE_LEVEL;
		delay(1);
	}

	const auto& sensors = sensorManager.getSensors();
	for (auto& sensor : sensors) {
		sensor->deinitialize();
	}

#ifdef ESP8266
	ESP.deepSleep(0);
#endif

#ifdef ESP32
	esp_deep_sleep_start();
#endif
#endif
}
