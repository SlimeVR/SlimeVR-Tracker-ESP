#include <cstdint>

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
#define LED(pin) constexpr uint8_t LED_PIN = pin;
#else
#define LED(pin)
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
#define INVERTED_LED(value) constexpr bool LED_INVERTED = value;
#else
#define INVERTED_LED(value)
#endif
