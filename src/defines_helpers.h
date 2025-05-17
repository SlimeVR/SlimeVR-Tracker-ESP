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

#include <cstdint>

#include "pins_arduino.h"

#ifndef PIN_IMU_SDA
#define SDA(pin) constexpr uint8_t PIN_IMU_SDA = pin;
#else
#define SDA(pin)
#endif

#ifndef PIN_IMU_SCL
#define SCL(pin) constexpr uint8_t PIN_IMU_SCL = pin;
#else
#define SCL(pin)
#endif

#ifndef PIN_IMU_INT
#define INT(pin) constexpr uint8_t PIN_IMU_INT = pin;
#else
#define INT(pin)
#endif

#ifndef PIN_IMU_INT_2
#define INT2(pin) constexpr uint8_t PIN_IMU_INT_2 = pin;
#else
#define INT2(pin)
#endif

#ifndef PIN_BATTERY_LEVEL
#define BATTERY(pin) constexpr uint8_t PIN_BATTERY_LEVEL = pin;
#else
#define BATTERY(pin)
#endif

#ifndef LED_PIN
#define LED(pin) const uint8_t LED_PIN = pin;
#else
#define LED(pin)
#endif

#ifndef LED_PIN
extern const uint8_t __attribute__((weak)) LED_PIN;
#endif

#ifndef BATTERY_SHIELD_RESISTANCE
#define BATTERY_SHIELD_R(value) constexpr float BATTERY_SHIELD_RESISTANCE = value;
#else
#define BATTERY_SHIELD_R(value)
#endif

#ifndef BATTERY_SHIELD_R1
#define BATTERY_R1(value) constexpr float BATTERY_SHIELD_R1 = value;
#else
#define BATTERY_R1(value)
#endif

#ifndef BATTERY_SHIELD_R2
#define BATTERY_R2(value) constexpr float BATTERY_SHIELD_R2 = value;
#else
#define BATTERY_R2(value)
#endif

#ifndef LED_INVERTED
#define INVERTED_LED(value) const bool LED_INVERTED = value;
#else
#define INVERTED_LED(value)
#endif

#ifndef LED_INVERTED
extern const bool __attribute__((weak)) LED_INVERTED;
#endif
