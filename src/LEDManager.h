/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 TheDevMinerTV

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
#ifndef SLIMEVR_LEDMANAGER_H
#define SLIMEVR_LEDMANAGER_H

#include <Arduino.h>

#include "globals.h"
#include "logging/Logger.h"

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

namespace SlimeVR {
enum LEDStage { OFF, ON, GAP, INTERVAL };

class LEDManager {
public:
	LEDManager(uint8_t pin)
		: m_Pin(pin) {}

	void setup();

	/*!
	 *  @brief Turns the LED on
	 */
	void on();

	/*!
	 *  @brief Turns the LED off
	 */
	void off();

	/*!
	 *  @brief Blink the LED for [time]ms. *Can* cause lag
	 *  @param time Amount of ms to turn the LED on
	 */
	void blink(unsigned long time);

	/*!
	 *  @brief Show a pattern on the LED. *Can* cause lag
	 *  @param timeon Amount of ms to turn the LED on
	 *  @param timeoff Amount of ms to turn the LED off
	 *  @param times Amount of times to display the pattern
	 */
	void pattern(unsigned long timeon, unsigned long timeoff, int times);

	void update();

private:
	uint8_t m_CurrentCount = 0;
	unsigned long m_Timer = 0;
	LEDStage m_CurrentStage = OFF;
	unsigned long m_LastUpdate = millis();

	uint8_t m_Pin;

	Logging::Logger m_Logger = Logging::Logger("LEDManager");
};
}  // namespace SlimeVR

#endif
