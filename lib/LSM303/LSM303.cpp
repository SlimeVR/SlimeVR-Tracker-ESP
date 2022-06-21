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

// +------------------+------------+-------------+---------------+---------+-----------+---------+-----------+------------+-------------+-----------+-------------+
// |      Model       | LSM303AGR  | LSM303AGRTR |  LSM303DLHCTR | LSM303C | LSM303CTR | LSM303D | LSM303DLH | LSM303DLHC | LSM303DLHTR | LSM303DLM | LSM303DLMTR |
// +------------------+------------+-------------+---------------+---------+-----------+---------+-----------+------------+-------------+-----------+-------------+
// | I2C Acceleration |            |             |               |         |           | 3C      | 30        |            | 30          | 30        | 30          |
// | I2C Acceleration | 32         | 32          | 32            | 3A      | 3A        | 3A      | 32        | 32         | 32          | 32        | 32          |
// | Address          | 28         | 28          | 28            | 28      | 28        | 28      | 28        | 28         | 28          | 28        | 28          |
// | Order            | XYZ        | XYZ         | XYZ           | XYZ     | XYZ       | XYZ     | XYZ       | XYZ        | XYZ         | XYZ       | XYZ         |
// | Axis             | LH         | LH          | LH            | LH      | LH        | LH      | LH        | LH         | LH          | LH        | LH          |
// | Mag              | 3C         | 3C          | 3C            | 3C      | 3C        |         |           | 3C         | 3C          | 3C        | 3C          |
// | Address          | 68         | 68          | 3             | 28      | 28        | 8       | 3         | 3          | 3           | 3         | 3           |
// | Order            | XYZ        | XYZ         | XZY           | XYZ     | XYZ       | XYZ     | XYZ       | XYZ        | XYZ         | XYZ       | XYZ         |
// | Axis             | LH         | LH          | HL            | LH      | LH        | LH      | HL        | HL         | HL          | HL        | HL          |
// | Supported		  | No         | No	         | No            | No      | No        | Yes     | Yes       | Yes        | No          | Yes       | No          |
// +------------------+------------+-------------+---------------+---------+-----------+---------+-----------+------------+-------------+-----------+-------------+

#include "LSM303.h"

LSM303::LSM303()
{
	gyro = gyro_none;
	accMag = accMag_none;
	gscale = (250 / 32768.0) * (PI / 180.0);
}

void LSM303::initialize(uint8_t addr)
{
	devAddr = addr;
	/* Find the gyros location */
	findGyro();
	/* Find the gccelerometer and magnetometers location */
	findAccelMag();
	/* Issue a soft-reset to bring the device into a clean state */
	resetDevice();
}

void LSM303::resetGyro()
{
	if (gyro == gyro_L3GD20H)
		I2Cdev::writeByte(gyro_address, L3G_LOW_ODR, 0x00);
	I2Cdev::writeByte(gyro_address, L3G_CTRL4, 0x00);
	I2Cdev::writeByte(gyro_address, L3G_CTRL1, 0xCF);
}

uint8_t LSM303::getGyroID()
{
	uint8_t id = 0;
	I2Cdev::readByte(gyro_address, L3G_WHO_AM_I, &id);
	return id;
}

void LSM303::findGyro()
{
	uint8_t id = 0xFF;
	I2Cdev::readByte(L3GD20_SA0_HIGH_ADDRESS, L3G_WHO_AM_I, &id);
	switch (id)
	{
	case (L3GD20H_WHO_ID):
		gyro_address = L3GD20_SA0_HIGH_ADDRESS;
		gyro = gyro_L3GD20H;
		return;
	case (L3GD20_WHO_ID):
		gyro_address = L3GD20_SA0_HIGH_ADDRESS;
		gyro = gyro_L3GD20;
		return;
	}
	I2Cdev::readByte(L3GD20_SA0_LOW_ADDRESS, L3G_WHO_AM_I, &id);
	switch (id)
	{
	case (L3GD20H_WHO_ID):
		gyro_address = L3GD20_SA0_LOW_ADDRESS;
		gyro = gyro_L3GD20H;
		return;
	case (L3GD20_WHO_ID):
		gyro_address = L3GD20_SA0_LOW_ADDRESS;
		gyro = gyro_L3GD20;
		return;
	}
	I2Cdev::readByte(L3G4200D_SA0_HIGH_ADDRESS, L3G_WHO_AM_I, &id);
	if (id == L3G4200D_WHO_ID)
	{
		gyro_address = L3G4200D_SA0_HIGH_ADDRESS;
		gyro = gyro_L3G4200D;
		return;
	}
	I2Cdev::readByte(L3G4200D_SA0_LOW_ADDRESS, L3G_WHO_AM_I, &id);
	if (id == L3G4200D_WHO_ID)
	{
		gyro_address = L3G4200D_SA0_LOW_ADDRESS;
		gyro = gyro_L3G4200D;
		return;
	}
	gyro = gyro_none;
}

void LSM303::findAccelMag()
{
	uint8_t id = 0xFF;
	bool set = false;
	// accMag_D
	I2Cdev::readByte(LSM303D_SA0_HIGH_ADDRESS, LSM303_WHO_AM_I, &id);
	if (id == LSM303D_WHO_ID && !set)
	{
		accMag = accMag_LSM303D;
		acc_address = mag_address = LSM303D_SA0_HIGH_ADDRESS;
		mag_order = XYZ;
		mag_start_reg = LSM303D_OUT_MAG_START;
		set = true;
	}

	I2Cdev::readByte(LSM303D_SA0_LOW_ADDRESS, LSM303_WHO_AM_I, &id);
	if (id == LSM303D_WHO_ID && !set)
	{
		accMag = accMag_LSM303D;
		acc_address = mag_address = LSM303D_SA0_LOW_ADDRESS;
		mag_order = XYZ;
		mag_start_reg = LSM303D_OUT_MAG_START;
		set = true;
	}

	I2Cdev::readByte(LSM303DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS, LSM303_CTRL1, &id);
	if (id != -1 && !set)
	{
		acc_address = LSM303DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS;
		mag_address = LSM303DLHC_DLM_DLH_MAG_ADDRESS;
		accMag = LSM303DLM_WHO_ID == id ? accMag_LSM303DLHC : accMag_LSM303DLH;
		mag_order = XYZ | 0b01000000;
		mag_start_reg = LSM303DLHC_DLM_DLH_OUT_MAG_START;
		set = true;
	}

	I2Cdev::readByte(LSM303DLM_DLH_ACC_SA0_LOW_ADDRESS, LSM303_CTRL1, &id);
	if (id != -1 && !set)
	{
		acc_address = LSM303DLM_DLH_ACC_SA0_LOW_ADDRESS;
		mag_address = LSM303DLHC_DLM_DLH_MAG_ADDRESS;
		accMag = LSM303DLM_WHO_ID == id ? accMag_LSM303DLM : accMag_LSM303DLH;
		mag_order = XYZ | 0b01000000;
		mag_start_reg = LSM303DLHC_DLM_DLH_OUT_MAG_START;
		set = true;
	}
	if (set)
		return;
	accMag = accMag_none;
}

/*****************************************************************************

							Accelerometer section

******************************************************************************/

void LSM303::getAcceleration(int16_t *output_buffer)
{
	I2Cdev::readBytes(acc_address, LSM303_OUT_ACC_START | 0x80 | 0x80, 6, buffer);
	output_buffer[0] = (int16_t)((buffer[1] << 8) | buffer[0]) >> 4;
	output_buffer[1] = (int16_t)((buffer[3] << 8) | buffer[2]) >> 4;
	output_buffer[2] = (int16_t)((buffer[5] << 8) | buffer[4]) >> 4;
}

/*****************************************************************************

							Gyroscope section

******************************************************************************/

void LSM303::getRotation(int16_t *output_buffer)
{
	I2Cdev::readBytes(gyro_address, L3G_OUT_GYRO_START | 0x80, 6, buffer);
	output_buffer[0] = (int16_t)(buffer[1] << 8 | buffer[0]);
	output_buffer[1] = (int16_t)(buffer[3] << 8 | buffer[2]);
	output_buffer[2] = (int16_t)(buffer[5] << 8 | buffer[4]);
}

/*****************************************************************************

							Magnetometer section

******************************************************************************/

void LSM303::getMagnetometer(int16_t *output_buffer)
{
	I2Cdev::readBytes(mag_address, mag_start_reg | 0x80, 6, buffer);
	if ((mag_order >> 6) & 0b01)
	{
		output_buffer[(mag_order & 0b11)] = (int16_t)(buffer[0] << 8 | buffer[1]);
		output_buffer[(mag_order & 0b1100) >> 2] = (int16_t)(buffer[2] << 8 | buffer[3]);
		output_buffer[(mag_order & 0b110000) >> 4] = (int16_t)(buffer[4] << 8 | buffer[5]);
	}
	else
	{
		output_buffer[(mag_order & 0b11)] = (int16_t)(buffer[1] << 8 | buffer[0]);
		output_buffer[(mag_order & 0b1100) >> 2] = (int16_t)(buffer[3] << 8 | buffer[2]);
		output_buffer[(mag_order & 0b110000) >> 4] = (int16_t)(buffer[5] << 8 | buffer[4]);
	}
}

/*****************************************************************************

							Combined motion section

******************************************************************************/

void LSM303::getMotion6(int16_t *output_buffer)
{
	getAcceleration(output_buffer);
	getRotation(&output_buffer[3]);
}

void LSM303::getMotion9(int16_t *output_buffer)
{
	getAcceleration(output_buffer);
	getRotation(&output_buffer[3]);
	getMagnetometer(&output_buffer[6]);
}

/*****************************************************************************

							Utility section

******************************************************************************/

bool LSM303::testConnection()
{
	return gyro != gyro_none && accMag != accMag_none;
}

uint8_t LSM303::getAccMagID()
{
	uint8_t id = 0;
	I2Cdev::readByte(acc_address, LSM303_WHO_AM_I, &id);
	return id;
}

void LSM303::resetDevice()
{
	resetGyro();
	resetAccelMag();
	setRange();
}

void LSM303::resetAccelMag()
{
	if (accMag == accMag_LSM303D)
	{
		I2Cdev::writeByte(acc_address, LSM303_CTRL2, 0x00);
		I2Cdev::writeByte(acc_address, LSM303_CTRL1, 0x57);
		I2Cdev::writeByte(acc_address, LSM303_CTRL5, 0x64);
		I2Cdev::writeByte(acc_address, LSM303_CTRL6, 0x20);
		I2Cdev::writeByte(acc_address, LSM303_CTRL7, 0x00);
	}
	else
	{
		if (accMag == accMag_LSM303DLHC)
		{
			I2Cdev::writeByte(acc_address, LSM303_CTRL4, 0x08);
			I2Cdev::writeByte(acc_address, LSM303_CTRL1, 0x47);
		}
		else // DLM, DLH
		{
			I2Cdev::writeByte(acc_address, LSM303_CTRL4, 0x00);
			I2Cdev::writeByte(acc_address, LSM303_CTRL1, 0x27);
		}
		I2Cdev::writeByte(mag_address, LSM303_CRA_REG_M, 0x9C);
		I2Cdev::writeByte(mag_address, LSM303_CRB_REG_M, 0x20);
		I2Cdev::writeByte(mag_address, LSM303_MR_REG_M, 0x00);
	}
}

uint8_t LSM303::getGyroAddress()
{
	return gyro_address;
}

uint8_t LSM303::getAccAddress()
{
	return acc_address;
}

uint8_t LSM303::getMagAddress()
{
	return mag_address;
}

uint8_t LSM303::getAccMagType()
{
	return accMag;
}

void LSM303::setRange()
{
	I2Cdev::writeByte(mag_address, LSM303_CRB_REG_M, 0x20);
	int16_t output_buffer[3] = {0};
	for (uint8_t i = 1; i < 8; i++)
	{
		getMagnetometer(output_buffer);
		if ((output_buffer[0] >= 2040) | (output_buffer[0] <= -2040) | (output_buffer[1] >= 2040) | (output_buffer[1] <= -2040) | (output_buffer[2] >= 2040) | (output_buffer[2] <= -2040))
		{
			I2Cdev::writeByte(mag_address, LSM303_CRB_REG_M, i * 0x20);
		}
		delay(10);
		getMagnetometer(output_buffer);
	}
}

float LSM303::getTemperatureC()
{
	uint8_t temperature = 0;
	I2Cdev::readByte(gyro_address, L3G_OUT_TEMP, &temperature);
	return (float)temperature / 8;
}