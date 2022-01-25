/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#ifndef SLIMEVR_LEDMGR_H_
#define SLIMEVR_LEDMGR_H_

#include <Arduino.h>
#include "globals.h"

#define LED_STATUS_SERVER_CONNECTING 2
#define LED_STATUS_WIFI_CONNECTING 4
#define LED_STATUS_LOW_BATTERY 128
#define LED_STATUS_IMU_ERROR 256

#define DEFAULT_LENGTH 300
#define DEFAULT_GAP 500
#define DEFAULT_INTERVAL 3000

#define STANDBUY_LENGTH DEFAULT_LENGTH
#define IMU_ERROR_LENGTH DEFAULT_LENGTH
#define IMU_ERROR_INTERVAL 1000
#define IMU_ERROR_COUNT 5
#define LOW_BATTERY_LENGTH DEFAULT_LENGTH
#define LOW_BATTERY_INTERVAL 300
#define LOW_BATTERY_COUNT 1
#define WIFI_CONNECTING_LENGTH DEFAULT_LENGTH
#define WIFI_CONNECTING_INTERVAL 3000
#define WIFI_CONNECTING_COUNT 3
#define SERVER_CONNECTING_LENGTH DEFAULT_LENGTH
#define SERVER_CONNECTING_INTERVAL 3000
#define SERVER_CONNECTING_COUNT 2

namespace LEDManager {
    
    enum Stage {
        OFF, ON, GAP, INTERVAL
    };
    void on(uint8_t pin);
    void off(uint8_t pin);
    void blink(uint8_t pin, unsigned long time, uint8_t direction = LOW);
    void pattern(uint8_t pin, unsigned long timeon, unsigned long timeoff, int times, uint8_t direction = LOW);
    void signalAssert();
    void setLedStatus(uint32_t status);
    void unsetLedStatus(uint32_t status);
    void ledStatusUpdate();
}

#endif // SLIMEVR_LEDMGR_H_