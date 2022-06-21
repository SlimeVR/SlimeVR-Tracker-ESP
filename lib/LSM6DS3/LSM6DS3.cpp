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

#include "LSM6DS3.h"

// Settings
#define LSM6DS3_ACC_BANDWIDTH LSM6DS3_BW_XL_400Hz	 // Hz.  Can be: 50, 100, 200, 400
#define LSM6DS3_ACC_RANGE LSM6DS3_FS_XL_2g			 // Max G force readable.  Can be: 2, 4, 8, 16
#define LSM6DS3_ACC_SAMPLE_RATE LSM6DS3_ODR_XL_208Hz // Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
#define LSM6DS3_GRYO_RANGE LSM6DS3_FS_G_125dps		 // Max deg/s.  Can be: 125, 245, 500, 1000, 2000
#define LSM6DS3_GYRO_SAMPLE_RATE LSM6DS3_ODR_G_208Hz // Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666

//****************************************************************************//
//  Configuration section
//****************************************************************************//
LSM6DS3::LSM6DS3()
{
	gscale = ((125 * pow(2, LSM6DS3_GRYO_RANGE >> 1)) / 32768.0) * (PI / 180.0);
}

void LSM6DS3::initialize(uint8_t addr)
{
	devAddr = addr;
	/* Issue a soft-reset to bring the device into a clean state */
	resetDevice();
	// Check the structure values to determine how to setup the device
	uint8_t temp = 0;
	temp = LSM6DS3_ACC_BANDWIDTH | LSM6DS3_ACC_RANGE | LSM6DS3_ACC_SAMPLE_RATE;
	I2Cdev::writeByte(devAddr, LSM6DS3_CTRL1_XL, temp);
	temp = LSM6DS3_BW_SCAL_ODR_ENABLED;
	I2Cdev::writeByte(devAddr, LSM6DS3_CTRL4_C, temp);
	temp = LSM6DS3_GRYO_RANGE | LSM6DS3_GYRO_SAMPLE_RATE;
	I2Cdev::writeByte(devAddr, LSM6DS3_CTRL2_G, temp);
	// enable sensorhub??
	I2Cdev::writeByte(devAddr, LSM6DS3_RAM_ACCESS, 0b00000001);
	I2Cdev::writeByte(devAddr, LSM6DS3_CTRL10_C, 0b00100000);
	I2Cdev::writeByte(devAddr, LSM6DS3_MASTER_CONFIG, 0b10000000);
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
void LSM6DS3::getRawAccelX(int16_t *x_raw)
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUTX_L_XL, 2, buffer);
	*x_raw = ((buffer[1]) << 8) | buffer[0];
}
void LSM6DS3::getRawAccelY(int16_t *y_raw)
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUTY_L_XL, 2, buffer);
	*y_raw = ((buffer[1]) << 8) | buffer[0];
}
void LSM6DS3::getRawAccelZ(int16_t *z_raw)
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUTZ_L_XL, 2, buffer);
	*z_raw = ((buffer[1]) << 8) | buffer[0];
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//

void LSM6DS3::getRawGyroX(int16_t *x_raw)
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUTX_L_G, 2, buffer);
	*x_raw = ((buffer[1]) << 8) | buffer[0];
}
void LSM6DS3::getRawGyroY(int16_t *y_raw)
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUTY_L_G, 2, buffer);
	*y_raw = ((buffer[1]) << 8) | buffer[0];
}
void LSM6DS3::getRawGyroZ(int16_t *z_raw)
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUTZ_L_G, 2, buffer);
	*z_raw = ((buffer[1]) << 8) | buffer[0];
}

//****************************************************************************//
//
//  Combined motion section
//
//****************************************************************************//

void LSM6DS3::getRotation(int16_t *x, int16_t *y, int16_t *z)
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUTX_L_G, 6, buffer);
	*x = (((buffer[1]) << 8) | buffer[0]);
	*y = (((buffer[3]) << 8) | buffer[2]);
	*z = (((buffer[5]) << 8) | buffer[4]);
}

void LSM6DS3::getAcceleration(int16_t *x, int16_t *y, int16_t *z)
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUTX_L_XL, 6, buffer);
	*x = (((buffer[7]) << 8) | buffer[6]);
	*y = (((buffer[9]) << 8) | buffer[8]);
	*z = (((buffer[11]) << 8) | buffer[10]);
}

void LSM6DS3::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUTX_L_G, 12, buffer);
	*gx = (((buffer[1]) << 8) | buffer[0]);
	*gy = (((buffer[3]) << 8) | buffer[2]);
	*gz = (((buffer[5]) << 8) | buffer[4]);
	*ax = (((buffer[7]) << 8) | buffer[6]);
	*ay = (((buffer[9]) << 8) | buffer[8]);
	*az = (((buffer[11]) << 8) | buffer[10]);
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//

int16_t LSM6DS3::getRawTemperature()
{
	I2Cdev::readBytes(devAddr, LSM6DS3_OUT_TEMP_L, 2, buffer);
	return ((buffer[1]) << 8) | buffer[0];
}

float LSM6DS3::getTemperature()
{
	float output = (float)getRawTemperature() / 16; // divide by 16 to scale
	output += 25;									// Add 25 degrees to remove offset
	return output;
}

float LSM6DS3::getTemperatureF()
{
	float output = (float)getRawTemperature() / 16; // divide by 16 to scale
	output = (output + 25 * 9) / 5 + 32;
	return output;
}

//****************************************************************************//
//
//  Utility section
//
//****************************************************************************//

bool LSM6DS3::testConnection()
{
	uint8_t deviceId = getDeviceID();
	return deviceId == 0x69;
}

uint8_t LSM6DS3::getDeviceID()
{
	I2Cdev::readByte(devAddr, LSM6DS3_WHO_AM_I_REG, buffer);
	return buffer[0];
}

void LSM6DS3::resetDevice()
{
	I2Cdev::writeByte(devAddr, LSM6DS3_RAM_ACCESS, LSM6DS3_SW_RESET_NORMAL_MODE);
	delay(500);
}