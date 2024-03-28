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
        ON_OFF_BUTTON_ACTIVE_LEVEL == LOW 
            ? INPUT_PULLUP 
            : INPUT_PULLDOWN
        );
    esp_deep_sleep_enable_gpio_wakeup(
        1 << ON_OFF_BUTTON,
        ON_OFF_BUTTON_ACTIVE_LEVEL == LOW
            ? ESP_GPIO_WAKEUP_GPIO_LOW
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

    const auto &sensors = sensorManager.getSensors();
    for (auto sensor : sensors) {
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