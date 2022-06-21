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

#ifndef __LSM6DS3IMU_H__
#define __LSM6DS3IMU_H__

#include "Arduino.h"
#include "I2Cdev.h"
/*************** Device Register  *******************/
#define LSM6DS3_TEST_PAGE 0X00
#define LSM6DS3_RAM_ACCESS 0X01
#define LSM6DS3_SENSOR_SYNC_TIME 0X04
#define LSM6DS3_SENSOR_SYNC_EN 0X05
#define LSM6DS3_FIFO_CTRL1 0X06
#define LSM6DS3_FIFO_CTRL2 0X07
#define LSM6DS3_FIFO_CTRL3 0X08
#define LSM6DS3_FIFO_CTRL4 0X09
#define LSM6DS3_FIFO_CTRL5 0X0A
#define LSM6DS3_ORIENT_CFG_G 0X0B
#define LSM6DS3_REFERENCE_G 0X0C
#define LSM6DS3_INT1_CTRL 0X0D
#define LSM6DS3_INT2_CTRL 0X0E
#define LSM6DS3_WHO_AM_I_REG 0X0F
#define LSM6DS3_CTRL1_XL 0X10
#define LSM6DS3_CTRL2_G 0X11
#define LSM6DS3_CTRL3_C 0X12
#define LSM6DS3_CTRL4_C 0X13
#define LSM6DS3_CTRL5_C 0X14
#define LSM6DS3_CTRL6_G 0X15
#define LSM6DS3_CTRL7_G 0X16
#define LSM6DS3_CTRL8_XL 0X17
#define LSM6DS3_CTRL9_XL 0X18
#define LSM6DS3_CTRL10_C 0X19
#define LSM6DS3_MASTER_CONFIG 0X1A
#define LSM6DS3_WAKE_UP_SRC 0X1B
#define LSM6DS3_TAP_SRC 0X1C
#define LSM6DS3_D6D_SRC 0X1D
#define LSM6DS3_STATUS_REG 0X1E
#define LSM6DS3_OUT_TEMP_L 0X20
#define LSM6DS3_OUT_TEMP_H 0X21
// Gyro Registers L_G
#define LSM6DS3_OUTX_L_G 0X22
#define LSM6DS3_OUTX_H_G 0X23
#define LSM6DS3_OUTY_L_G 0X24
#define LSM6DS3_OUTY_H_G 0X25
#define LSM6DS3_OUTZ_L_G 0X26
#define LSM6DS3_OUTZ_H_G 0X27
// Accelerometer Registers L_G
#define LSM6DS3_OUTX_L_XL 0X28
#define LSM6DS3_OUTX_H_XL 0X29
#define LSM6DS3_OUTY_L_XL 0X2A
#define LSM6DS3_OUTY_H_XL 0X2B
#define LSM6DS3_OUTZ_L_XL 0X2C
#define LSM6DS3_OUTZ_H_XL 0X2D
// Sensor Hub
#define LSM6DS3_SENSORHUB1_REG 0X2E
#define LSM6DS3_SENSORHUB2_REG 0X2F
#define LSM6DS3_SENSORHUB3_REG 0X30
#define LSM6DS3_SENSORHUB4_REG 0X31
#define LSM6DS3_SENSORHUB5_REG 0X32
#define LSM6DS3_SENSORHUB6_REG 0X33
#define LSM6DS3_SENSORHUB7_REG 0X34
#define LSM6DS3_SENSORHUB8_REG 0X35
#define LSM6DS3_SENSORHUB9_REG 0X36
#define LSM6DS3_SENSORHUB10_REG 0X37
#define LSM6DS3_SENSORHUB11_REG 0X38
#define LSM6DS3_SENSORHUB12_REG 0X39
/********************************************************************************
 * Register      : CTRL1_XL
 * Address       : 0X10
 * Bit Group Name: BW_XL
 * Permission    : RW
 *******************************************************************************/
#define LSM6DS3_BW_XL_400Hz 0x00
#define LSM6DS3_BW_XL_200Hz 0x01
#define LSM6DS3_BW_XL_100Hz 0x02
#define LSM6DS3_BW_XL_50Hz 0x03

#define LSM6DS3_FS_XL_2g 0x00
#define LSM6DS3_FS_XL_16g 0x04
#define LSM6DS3_FS_XL_4g 0x08
#define LSM6DS3_FS_XL_8g 0x0C

#define LSM6DS3_ODR_XL_POWER_DOWN 0x00
#define LSM6DS3_ODR_XL_13Hz 0x10
#define LSM6DS3_ODR_XL_26Hz 0x20
#define LSM6DS3_ODR_XL_52Hz 0x30
#define LSM6DS3_ODR_XL_104Hz 0x40
#define LSM6DS3_ODR_XL_208Hz 0x50
#define LSM6DS3_ODR_XL_416Hz 0x60
#define LSM6DS3_ODR_XL_833Hz 0x70
#define LSM6DS3_ODR_XL_1660Hz 0x80
#define LSM6DS3_ODR_XL_3330Hz 0x90
#define LSM6DS3_ODR_XL_6660Hz 0xA0
#define LSM6DS3_ODR_XL_13330Hz 0xB0
/********************************************************************************
 * Register      : CTRL2_G
 * Address       : 0X11
 *******************************************************************************/
#define LSM6DS3_FS_G_125dps 0x02
#define LSM6DS3_FS_G_245dps 0x00
#define LSM6DS3_FS_G_500dps 0x04
#define LSM6DS3_FS_G_1000dps 0x08
#define LSM6DS3_FS_G_2000dps 0x0C

#define LSM6DS3_ODR_G_POWER_DOWN 0x00
#define LSM6DS3_ODR_G_13Hz 0x10
#define LSM6DS3_ODR_G_26Hz 0x20
#define LSM6DS3_ODR_G_52Hz 0x30
#define LSM6DS3_ODR_G_104Hz 0x40
#define LSM6DS3_ODR_G_208Hz 0x50
#define LSM6DS3_ODR_G_416Hz 0x60
#define LSM6DS3_ODR_G_833Hz 0x70
#define LSM6DS3_ODR_G_1660Hz 0x80
/*******************************************************************************
 * Register      : CTRL3_C
 * Address       : 0X12
 *******************************************************************************/
#define LSM6DS3_SW_RESET_NORMAL_MODE 0x00
#define LSM6DS3_SW_RESET_RESET_DEVICE 0x01
#define LSM6DS3_BOOT_NORMAL_MODE 0x00
#define LSM6DS3_BOOT_REBOOT_MODE 0x80
/********************************************************************************
 * Register      : CTRL4_C
 * Address       : 0X13
 *******************************************************************************/
#define LSM6DS3_BW_SCAL_ODR_DISABLED 0x00
#define LSM6DS3_BW_SCAL_ODR_ENABLED 0x80
class LSM6DS3
{
public:
	LSM6DS3();
	~LSM6DS3() = default;
	bool testConnection();
	uint8_t getDeviceID();
	void resetDevice();
	void initialize(uint8_t addr);
	void getRawAccelX(int16_t *);
	void getRawAccelY(int16_t *);
	void getRawAccelZ(int16_t *);
	void getRawGyroX(int16_t *);
	void getRawGyroY(int16_t *);
	void getRawGyroZ(int16_t *);
	void getAcceleration(int16_t *x, int16_t *y, int16_t *z);
	void getRotation(int16_t *x, int16_t *y, int16_t *z);
	void getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
	int16_t getRawTemperature();
	float getTemperatureF();
	float getTemperature();
	float gscale;

protected:
	uint8_t devAddr;
	uint8_t buffer[12];
};
#endif