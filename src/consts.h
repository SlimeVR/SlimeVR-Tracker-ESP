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
#ifndef SLIMEVR_CONSTS_H_
#define SLIMEVR_CONSTS_H_

// List of constants used in other places

enum class ImuID {
	Unknown = 0,
	MPU9250,
	MPU6500,
	BNO080,
	BNO085,
	BNO055,
	MPU6050,
	BNO086,
	BMI160,
	ICM20948,
	ICM42688,
	BMI270,
	LSM6DS3TRC,
	LSM6DSV,
	LSM6DSO,
	LSM6DSR,
	Empty = 255
};

#define IMU_UNKNOWN ErroneousSensor
#define IMU_MPU9250 MPU9250Sensor
#define IMU_MPU6500 MPU6050Sensor
#define IMU_BNO080 BNO080Sensor
#define IMU_BNO085 BNO085Sensor
#define IMU_BNO055 BNO055Sensor
#define IMU_MPU6050 MPU6050Sensor
#define IMU_BNO086 BNO086Sensor
#define IMU_BMI160 BMI160Sensor
#define IMU_ICM20948 ICM20948Sensor
#define IMU_ICM42688 SoftFusionICM42688
#define IMU_BMI270 SoftFusionBMI270
#define IMU_LSM6DS3TRC SoftFusionLSM6DS3TRC
#define IMU_LSM6DSV SoftFusionLSM6DSV
#define IMU_LSM6DSO SoftFusionLSM6DSO
#define IMU_LSM6DSR SoftFusionLSM6DSR
#define IMU_MPU6050_SF SoftFusionMPU6050

#define IMU_DEV_RESERVED 250  // Reserved, should not be used in any release firmware

#define BOARD_UNKNOWN 0
#define BOARD_SLIMEVR_LEGACY 1
#define BOARD_SLIMEVR_DEV 2
#define BOARD_NODEMCU 3
#define BOARD_CUSTOM 4
#define BOARD_WROOM32 5
#define BOARD_WEMOSD1MINI 6
#define BOARD_TTGO_TBASE 7
#define BOARD_ESP01 8
#define BOARD_SLIMEVR 9
#define BOARD_LOLIN_C3_MINI 10
#define BOARD_BEETLE32C3 11
#define BOARD_ES32C3DEVKITM1 12
#define BOARD_OWOTRACK 13  // Only used by owoTrack mobile app
#define BOARD_WRANGLER 14  // Only used by wrangler app
#define BOARD_MOCOPI 15  // Used by mocopi/moslime
#define BOARD_WEMOSWROOM02 16
#define BOARD_XIAO_ESP32C3 17
#define BOARD_HARITORA 18 // Used by Haritora/SlimeTora
#define BOARD_ES32C6DEVKITC1 19
#define BOARD_DEV_RESERVED 250 // Reserved, should not be used in any release firmware

#define BAT_EXTERNAL 1
#define BAT_INTERNAL 2
#define BAT_MCP3021 3
#define BAT_INTERNAL_MCP3021 4

#define LED_OFF 255

#define POWER_SAVING_LEGACY 0  // No sleeping, but PS enabled
#define POWER_SAVING_NONE 1  // No sleeping, no PS => for connection issues
#define POWER_SAVING_MINIMUM 2  // Sleeping and PS => default
#define POWER_SAVING_MODERATE \
	3  // Sleeping and better PS => might miss broadcasts, use at own risk
#define POWER_SAVING_MAXIMUM \
	4  // Actual CPU sleeping, currently has issues with disconnecting

// Send rotation/acceleration data as separate frames.
// PPS: 1470 @ 5+1, 1960 @ 5+3
#define PACKET_BUNDLING_DISABLED 0
// Less packets. Pack data per sensor and send asap.
// Compared to PACKET_BUNDLING_DISABLED, reduces PPS by ~54% for 5+1, by ~63% for 5+3
// setups. PPS: 680 @ 5+1, 740 @ 5+3
#define PACKET_BUNDLING_LOWLATENCY 1
// Even less packets, if more than 1 sensor - wait for data from all sensors or until
// timeout, then send. Compared to PACKET_BUNDLING_LOWLATENCY, reduces PPS by ~5% for
// 5+1, by ~15% for 5+3 setups. PPS: 650 @ 5+1, 650 @ 5+3
#define PACKET_BUNDLING_BUFFERED 2

// Get radian for a given angle from 0° to 360° (2*PI*r, solve for r given an angle,
// range -180° to 180°)
#define DEG_X(deg) ((((deg) < 180.0f ? 0 : 360.0f) - (deg)) * PI / 180.0f)

#define DEG_0 DEG_X(0.0f)
#define DEG_90 DEG_X(90.0f)
#define DEG_180 DEG_X(180.0f)
#define DEG_270 DEG_X(270.0f)

#define CONST_EARTH_GRAVITY 9.80665

#define ACCEL_CALIBRATION_METHOD_SKIP 0
#define ACCEL_CALIBRATION_METHOD_ROTATION 1
#define ACCEL_CALIBRATION_METHOD_6POINT 2

#define BMI160_MAG_TYPE_HMC 1
#define BMI160_MAG_TYPE_QMC 2

#define MCU_UNKNOWN 0
#define MCU_ESP8266 1
#define MCU_ESP32 2
#define MCU_OWOTRACK_ANDROID 3  // Only used by owoTrack mobile app
#define MCU_WRANGLER 4  // Only used by wrangler app
#define MCU_OWOTRACK_IOS 5  // Only used by owoTrack mobile app
#define MCU_ESP32_C3 6
#define MCU_MOCOPI 7  // Used by mocopi/moslime
#define MCU_HARITORA 8  // Used by Haritora/SlimeTora
#define MCU_DEV_RESERVED 250  // Reserved, should not be used in any release firmware

#ifdef ESP8266
#define HARDWARE_MCU MCU_ESP8266
#elif defined(ESP32)
#define HARDWARE_MCU MCU_ESP32
#else
#define HARDWARE_MCU MCU_UNKNOWN
#endif

#define CURRENT_CONFIGURATION_VERSION 1

#endif  // SLIMEVR_CONSTS_H_
