/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain

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
// ================================================
// See docs for configuration options and examples:
// https://docs.slimevr.dev/firmware/configuring-project.html#2-configuring-definesh
// ================================================

// Set parameters of IMU and board used
#define IMU IMU_ICM45686
#define SECOND_IMU IMU
#define BOARD BOARD_CUSTOM
#define IMU_ROTATION DEG_270
#define SECOND_IMU_ROTATION DEG_270

#define PRIMARY_IMU_OPTIONAL false
#define SECONDARY_IMU_OPTIONAL true

#define MAX_SENSORS_COUNT 2
#define TRACKER_TYPE TrackerType::TRACKER_TYPE_SVR_ROTATION

#ifndef SENSOR_DESC_LIST
#define SENSOR_DESC_LIST                       \
	SENSOR_DESC_ENTRY(                         \
		IMU,                                   \
		PRIMARY_IMU_ADDRESS_ONE,               \
		IMU_ROTATION,                          \
		DIRECT_WIRE(PIN_IMU_SCL, PIN_IMU_SDA), \
		PRIMARY_IMU_OPTIONAL,                  \
		DIRECT_PIN(PIN_IMU_INT),               \
		0                                      \
	)                                          \
	SENSOR_DESC_ENTRY(                         \
		SECOND_IMU,                            \
		SECONDARY_IMU_ADDRESS_TWO,             \
		SECOND_IMU_ROTATION,                   \
		DIRECT_WIRE(PIN_IMU_SCL, PIN_IMU_SDA), \
		SECONDARY_IMU_OPTIONAL,                \
		DIRECT_PIN(PIN_IMU_INT_2),             \
		0                                      \
	)
#endif

#define BATTERY_MONITOR BAT_EXTERNAL

#define ON_OFF_BUTTON_PIN 1
#define PIN_IMU_SDA 5
#define PIN_IMU_SCL 6
#define PIN_IMU_INT 255
#define PIN_IMU_INT_2 255
#define PIN_BATTERY_LEVEL 3
#define LED_PIN 0
#define LED_INVERTED true
#define BATTERY_SHIELD_RESISTANCE 0
#define BATTERY_SHIELD_R1 150
#define BATTERY_SHIELD_R2 150
