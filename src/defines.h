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
#define IMU IMU_AUTO
#define SECOND_IMU IMU_AUTO
#define BOARD BOARD_SLIMEVR_V1_2
#define IMU_ROTATION DEG_270
#define SECOND_IMU_ROTATION DEG_270

#define PRIMARY_IMU_OPTIONAL false
#define SECONDARY_IMU_OPTIONAL true

#if BOARD != BOARD_GLOVE_IMU_SLIMEVR_DEV
#define MAX_SENSORS_COUNT 2
#define TRACKER_TYPE TrackerType::TRACKER_TYPE_SVR_ROTATION
// Set I2C address here or directly in IMU_DESC_ENTRY for each IMU used
// If not set, default address is used based on the IMU and Sensor ID
// #define PRIMARY_IMU_ADDRESS_ONE 0x4a
// #define SECONDARY_IMU_ADDRESS_TWO 0x4b

// Axis mapping example
/*
#include "sensors/axisremap.h"
#define BMI160_QMC_REMAP AXIS_REMAP_BUILD(AXIS_REMAP_USE_Y, AXIS_REMAP_USE_XN,
AXIS_REMAP_USE_Z, \ AXIS_REMAP_USE_YN, AXIS_REMAP_USE_X, AXIS_REMAP_USE_Z)

SENSOR_DESC_ENTRY(IMU_BMP160, PRIMARY_IMU_ADDRESS_ONE, IMU_ROTATION, PIN_IMU_SCL,
PIN_IMU_SDA, PRIMARY_IMU_OPTIONAL, BMI160_QMC_REMAP) \
*/

#ifndef SENSOR_DESC_LIST
#if BOARD == BOARD_SLIMEVR_V1_2
#define SENSOR_DESC_LIST                                             \
	SENSOR_DESC_ENTRY(                                               \
		IMU,                                                         \
		DIRECT_SPI(24'000'000, MSBFIRST, SPI_MODE3, DIRECT_PIN(15)), \
		IMU_ROTATION,                                                \
		NO_WIRE,                                                     \
		PRIMARY_IMU_OPTIONAL,                                        \
		DIRECT_PIN(PIN_IMU_INT),                                     \
		0                                                            \
	)                                                                \
	SENSOR_DESC_ENTRY(                                               \
		SECOND_IMU,                                                  \
		SECONDARY_IMU_ADDRESS_TWO,                                   \
		SECOND_IMU_ROTATION,                                         \
		DIRECT_WIRE(PIN_IMU_SCL, PIN_IMU_SDA),                       \
		SECONDARY_IMU_OPTIONAL,                                      \
		DIRECT_PIN(PIN_IMU_INT_2),                                   \
		0                                                            \
	)
#else
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
#endif
#else

// Predefines for the GLOVE
#ifndef SENSOR_DESC_LIST
#define MAX_SENSORS_COUNT 10
#define TRACKER_TYPE TrackerType::TRACKER_TYPE_SVR_GLOVE_LEFT
#define GLOVE_SIDE GLOVE_LEFT
#define PRIMARY_IMU_ADDRESS_ONE 0x4a
#define SECONDARY_IMU_ADDRESS_TWO 0x4b

#define SENSOR_DESC_LIST                                 \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		(PRIMARY_IMU_ADDRESS_ONE ^ 0x02),                \
		IMU_ROTATION,                                    \
		DIRECT_WIRE(PIN_IMU_SCL, PIN_IMU_SDA),           \
		false,                                           \
		MCP_PIN(MCP_GPA6),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		(SECONDARY_IMU_ADDRESS_TWO ^ 0x02),              \
		IMU_ROTATION,                                    \
		DIRECT_WIRE(PIN_IMU_SCL, PIN_IMU_SDA),           \
		true,                                            \
		MCP_PIN(MCP_GPA5),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		PRIMARY_IMU_ADDRESS_ONE,                         \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 0), \
		true,                                            \
		MCP_PIN(MCP_GPB0),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		SECONDARY_IMU_ADDRESS_TWO,                       \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 0), \
		true,                                            \
		MCP_PIN(MCP_GPB1),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		PRIMARY_IMU_ADDRESS_ONE,                         \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 1), \
		true,                                            \
		MCP_PIN(MCP_GPB2),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		SECONDARY_IMU_ADDRESS_TWO,                       \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 1), \
		true,                                            \
		MCP_PIN(MCP_GPB3),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		PRIMARY_IMU_ADDRESS_ONE,                         \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 2), \
		true,                                            \
		MCP_PIN(MCP_GPB4),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		SECONDARY_IMU_ADDRESS_TWO,                       \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 2), \
		true,                                            \
		MCP_PIN(MCP_GPB5),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		PRIMARY_IMU_ADDRESS_ONE,                         \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 3), \
		true,                                            \
		MCP_PIN(MCP_GPB6),                               \
		0                                                \
	)                                                    \
	SENSOR_DESC_ENTRY(                                   \
		IMU,                                             \
		SECONDARY_IMU_ADDRESS_TWO,                       \
		IMU_ROTATION,                                    \
		PCA_WIRE(PIN_IMU_SCL, PIN_IMU_SDA, PCA_ADDR, 3), \
		true,                                            \
		MCP_PIN(MCP_GPA1),                               \
		0                                                \
	)

#if GLOVE_SIDE == GLOVE_LEFT
#define SENSOR_INFO_LIST                                                    \
	SENSOR_INFO_ENTRY(0, SensorPosition::POSITION_LEFT_HAND)                \
	SENSOR_INFO_ENTRY(1, SensorPosition::POSITION_LEFT_LITTLE_INTERMEDIATE) \
	SENSOR_INFO_ENTRY(2, SensorPosition::POSITION_LEFT_RING_INTERMEDIATE)   \
	SENSOR_INFO_ENTRY(3, SensorPosition::POSITION_LEFT_RING_DISTAL)         \
	SENSOR_INFO_ENTRY(4, SensorPosition::POSITION_LEFT_MIDDLE_INTERMEDIATE) \
	SENSOR_INFO_ENTRY(5, SensorPosition::POSITION_LEFT_MIDDLE_DISTAL)       \
	SENSOR_INFO_ENTRY(6, SensorPosition::POSITION_LEFT_INDEX_INTERMEDIATE)  \
	SENSOR_INFO_ENTRY(7, SensorPosition::POSITION_LEFT_INDEX_DISTAL)        \
	SENSOR_INFO_ENTRY(8, SensorPosition::POSITION_LEFT_THUMB_PROXIMAL)      \
	SENSOR_INFO_ENTRY(9, SensorPosition::POSITION_LEFT_THUMB_DISTAL)
#elif GLOVE_SDIE == GLOVE_RIGHT
#define SENSOR_INFO_LIST                                                     \
	SENSOR_INFO_ENTRY(0, SensorPosition::POSITION_RIGHT_HAND)                \
	SENSOR_INFO_ENTRY(1, SensorPosition::POSITION_RIGHT_LITTLE_INTERMEDIATE) \
	SENSOR_INFO_ENTRY(2, SensorPosition::POSITION_RIGHT_RING_INTERMEDIATE)   \
	SENSOR_INFO_ENTRY(3, SensorPosition::POSITION_RIGHT_RING_DISTAL)         \
	SENSOR_INFO_ENTRY(4, SensorPosition::POSITION_RIGHT_MIDDLE_INTERMEDIATE) \
	SENSOR_INFO_ENTRY(5, SensorPosition::POSITION_RIGHT_MIDDLE_DISTAL)       \
	SENSOR_INFO_ENTRY(6, SensorPosition::POSITION_RIGHT_INDEX_INTERMEDIATE)  \
	SENSOR_INFO_ENTRY(7, SensorPosition::POSITION_RIGHT_INDEX_DISTAL)        \
	SENSOR_INFO_ENTRY(8, SensorPosition::POSITION_RIGHT_THUMB_PROXIMAL)      \
	SENSOR_INFO_ENTRY(9, SensorPosition::POSITION_RIGHT_THUMB_DISTAL)
#else  // GLOVE_SDIE
#error "Glove side not defined"
#endif  // GLOVE_SDIE

#endif  // SENSOR_DESC_LIST
#endif  // BOARD != BOARD_GLOVE_IMU_SLIMEVR_DEV

// Battery monitoring options (comment to disable):
//   BAT_EXTERNAL for ADC pin,
//   BAT_INTERNAL for internal - can detect only low battery,
//   BAT_MCP3021 for external ADC connected over I2C
#define BATTERY_MONITOR BAT_EXTERNAL

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
