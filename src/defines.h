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
#ifndef IMU
#define IMU IMU_AUTO
#endif
#ifndef SECOND_IMU
#define SECOND_IMU IMU_AUTO
#endif
#ifndef BOARD
#define BOARD BOARD_SLIMEVR_V1_2
#endif
#ifndef IMU_ROTATION
#define IMU_ROTATION DEG_270
#endif
#ifndef SECOND_IMU_ROTATION
#define SECOND_IMU_ROTATION DEG_270
#endif

#ifndef PRIMARY_IMU_OPTIONAL
#define PRIMARY_IMU_OPTIONAL false
#endif
#ifndef SECONDARY_IMU_OPTIONAL
#define SECONDARY_IMU_OPTIONAL true
#endif

// Set I2C address here or directly in IMU_DESC_ENTRY for each IMU used
// If not set, default address is used based on the IMU and Sensor ID
// #define PRIMARY_IMU_ADDRESS_ONE 0x4a
// #define SECONDARY_IMU_ADDRESS_TWO 0x4b

#ifndef BATTERY_MONITOR
// Battery monitoring options (comment to disable):
//   BAT_EXTERNAL for ADC pin,
//   BAT_INTERNAL for internal - can detect only low battery,
//   BAT_MCP3021 for external ADC connected over I2C
#define BATTERY_MONITOR BAT_EXTERNAL
#endif

// --- OVERRIDES FOR DEFAULT PINS

// #define PIN_IMU_SDA 14
// #define PIN_IMU_SCL 12
// #define PIN_IMU_INT 16
// #define PIN_IMU_INT_2 13
// #define PIN_BATTERY_LEVEL 17
// #define LED_PIN 2
// #define LED_INVERTED true
// #define BATTERY_SHILED_RESISTANCE 0
// #define BATTERY_SHIELD_R1 10
// #define BATTERY_SHIELD_R2 40.2

// ------------------------------
