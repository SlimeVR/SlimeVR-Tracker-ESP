/*
 * LED configuration:
 * Configuration Priority 1 = Highest:
 * 1. LED_PIN
 * 2. LED_BUILTIN
 *
 *   LED_PIN
 *     - Number or Symbol (D1,..) of the Output
 *     - To turn off the LED, set LED_PIN to LED_OFF
 *   LED_INVERTED
 *     - false for output 3.3V on high
 *     - true for pull down to GND on high
 */

/*
 * D1 Mini boards with ESP8266 have internal resistors. For these boards you only have
 * to adjust BATTERY_SHIELD_RESISTANCE. For other boards you can now adjust the other
 * resistor values. The diagram looks like this:
 *   (Battery)--- [BATTERY_SHIELD_RESISTANCE] ---(INPUT_BOARD)--- [BATTERY_SHIELD_R2]
 *   ---(ESP32_INPUT)--- [BATTERY_SHIELD_R1] --- (GND)
 * BATTERY_SHIELD_R(180)
 * 130k BatteryShield, 180k SlimeVR or fill in
 * external resistor value in kOhm BATTERY_R1(100)
 * Board voltage divider resistor Ain to GND in kOhm BATTERY_R2(220)
 * Board voltage divider resistor Ain to INPUT_BOARD in kOhm
 */

#include "defines_helpers.h"

// Default IMU pinouts and definitions for default tracker types

#if BOARD != BOARD_GLOVE_IMU_SLIMEVR_DEV
// Defaunlt definitions for normal 2-sensor trackers
#ifndef MAX_SENSORS_COUNT
#define MAX_SENSORS_COUNT 2
#endif
#ifndef TRACKER_TYPE
#define TRACKER_TYPE TrackerType::TRACKER_TYPE_SVR_ROTATION
#endif

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
#define SENSOR_DESC_LIST                             \
	SENSOR_DESC_ENTRY(                               \
		IMU,                                         \
		DIRECT_PIN(15),                              \
		IMU_ROTATION,                                \
		DIRECT_SPI(24'000'000, MSBFIRST, SPI_MODE3), \
		PRIMARY_IMU_OPTIONAL,                        \
		DIRECT_PIN(PIN_IMU_INT),                     \
		0                                            \
	)                                                \
	SENSOR_DESC_ENTRY(                               \
		SECOND_IMU,                                  \
		SECONDARY_IMU_ADDRESS_TWO,                   \
		SECOND_IMU_ROTATION,                         \
		DIRECT_WIRE(PIN_IMU_SCL, PIN_IMU_SDA),       \
		SECONDARY_IMU_OPTIONAL,                      \
		DIRECT_PIN(PIN_IMU_INT_2),                   \
		0                                            \
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
#else  // BOARD == BOARD_GLOVE_IMU_SLIMEVR_DEV

SDA(1)
SCL(0)
#define PCA_ADDR 0x70
INT(16)
INT2(13)
BATTERY(3)
LED(2)
INVERTED_LED(true)
BATTERY_SHIELD_R(0)
BATTERY_R1(10)
BATTERY_R2(40.2)

#include "glove_default.h"

#endif  // BOARD != BOARD_GLOVE_IMU_SLIMEVR_DEV
