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

// Board-specific configurations
#if BOARD == BOARD_SLIMEVR

SDA(14)
SCL(12)
INT(16)
INT2(13)
BATTERY(17)
LED(2)
INVERTED_LED(true)
BATTERY_SHIELD_R(0)
BATTERY_R1(10)
BATTERY_R2(40.2)

#elif BOARD == BOARD_SLIMEVR_V1_2

SDA(4)
SCL(5)
INT(2)
INT2(16)
BATTERY(17)
LED(2)
INVERTED_LED(true)
BATTERY_SHIELD_R(0)
BATTERY_R1(10)
BATTERY_R2(40.2)

#elif BOARD == BOARD_SLIMEVR_LEGACY || BOARD == BOARD_SLIMEVR_DEV

SDA(4)
SCL(5)
INT(10)
INT2(13)
BATTERY(17)
LED(2)
INVERTED_LED(true)
BATTERY_SHIELD_R(0)
BATTERY_R1(10)
BATTERY_R2(40.2)

#elif BOARD == BOARD_NODEMCU || BOARD == BOARD_WEMOSD1MINI

SDA(D2)
SCL(D1)
INT(D5)
INT2(D6)
BATTERY(A0)
BATTERY_SHIELD_R(180)
BATTERY_R1(100)
BATTERY_R2(220)

#elif BOARD == BOARD_ESP01

SDA(2)
SCL(0)
INT(255)
INT2(255)
BATTERY(255)
LED(LED_OFF)
INVERTED_LED(false)

#elif BOARD == BOARD_TTGO_TBASE

SDA(5)
SCL(4)
INT(14)
INT2(13)
BATTERY(A0)

#elif BOARD == BOARD_CUSTOM

// Define pins by the examples above

#elif BOARD == BOARD_WROOM32

SDA(21)
SCL(22)
INT(23)
INT2(25)
BATTERY(36)

#elif BOARD == BOARD_LOLIN_C3_MINI

SDA(5)
SCL(4)
INT(6)
INT2(8)
BATTERY(3)
LED(7)

#elif BOARD == BOARD_BEETLE32C3

SDA(8)
SCL(9)
INT(6)
INT2(7)
BATTERY(3)
LED(10)
INVERTED_LED(false)

#elif BOARD == BOARD_ESP32C3DEVKITM1 || BOARD == BOARD_ESP32C6DEVKITC1

SDA(5)
SCL(4)
INT(6)
INT2(7)
BATTERY(3)
LED(LED_OFF)

#elif BOARD == BOARD_WEMOSWROOM02

SDA(2)
SCL(14)
INT(0)
INT2(4)
BATTERY(A0)
LED(16)
INVERTED_LED(true)

#elif BOARD == BOARD_XIAO_ESP32C3

SDA(6)
SCL(7)  // D5
INT(5)  // D3
INT2(10)  // D10
LED(4)  // D2
INVERTED_LED(false)
BATTERY(2)  // D0 / A0
BATTERY_SHIELD_R(0)
BATTERY_R1(100)
BATTERY_R2(100)

#elif BOARD == BOARD_GLOVE_IMU_SLIMEVR_DEV

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

#endif
