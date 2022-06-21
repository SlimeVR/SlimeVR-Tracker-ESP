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

#ifndef __LSM303IMU_H__
#define __LSM303IMU_H__

#include "Arduino.h"
#include "I2Cdev.h"

/*************** Gyro  Addresses L3GD20H, L3GD20, L3G4200D *******************/
#define L3GD20_SA0_HIGH_ADDRESS 0b1101011 // also applies to D20H
#define L3GD20_SA0_LOW_ADDRESS 0b1101010  // also applies to D20H
#define L3G4200D_SA0_HIGH_ADDRESS 0b1101001
#define L3G4200D_SA0_LOW_ADDRESS 0b1101000

/*************** Gyro  Who_am_i values *******************/
#define L3GD20H_WHO_ID 0xD7
#define L3GD20_WHO_ID 0xD4
#define L3G4200D_WHO_ID 0xD3

/*************** Device Registers L3GD20H, L3GD20, L3G4200D *******************/
#define L3G_WHO_AM_I 0x0F
#define L3G_CTRL1 0x20
#define L3G_CTRL2 0x21
#define L3G_CTRL3 0x22
#define L3G_CTRL4 0x23
#define L3G_CTRL5 0x24

#define L3G_REFERENCE 0x25
#define L3G_OUT_TEMP 0x26
#define L3G_STATUS 0x27
#define L3G_OUT_GYRO_START 0x28
#define L3G_OUT_X_L 0x28
#define L3G_OUT_X_H 0x29
#define L3G_OUT_Y_L 0x2A
#define L3G_OUT_Y_H 0x2B
#define L3G_OUT_Z_L 0x2C
#define L3G_OUT_Z_H 0x2D
#define L3G_LOW_ODR 0x39 // LSM303 only

/*************** Device Registers LSM303 *******************/
#define LSM303D_SA0_LOW_ADDRESS 0b0011110
#define LSM303DLHC_DLM_DLH_MAG_ADDRESS 0b0011110
#define LSM303DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS 0b0011001
#define LSM303D_SA0_HIGH_ADDRESS 0b0011101
#define LSM303DLM_DLH_ACC_SA0_LOW_ADDRESS 0b0011000

#define LSM303_WHO_AM_I 0x0F // 00000111   rw
#define LSM303D_WHO_ID 0x49
#define LSM303DLM_WHO_ID 0x3C

#define LSM303_CTRL0 0x1F	 // D
#define LSM303_CTRL1 0x20	 // D, DLH, DLM, DLHC
#define LSM303_CTRL2 0x21	 // D, DLH, DLM, DLHC
#define LSM303_CTRL3 0x22	 // D, DLH, DLM, DLHC
#define LSM303_CTRL4 0x23	 // D, DLH, DLM, DLHC
#define LSM303_CTRL5 0x24	 // D, DLH, DLM, DLHC
#define LSM303_CTRL6 0x25	 // D, DLH, DLM, DLHC
#define LSM303_CTRL7 0x26	 // D, DLH, DLM, DLHC
#define LSM303_STATUS_A 0x27 // D, DLH, DLM, DLHC

#define LSM303_CRA_REG_M 0x00 // DLH, DLM, DLHC
#define LSM303_CRB_REG_M 0x01 // DLH, DLM, DLHC
#define LSM303_MR_REG_M 0x02  // DLH, DLM, DLHC

#define LSM303_SR_REG_M 0x09  // DLH, DLM, DLHC
#define LSM303_IRA_REG_M 0x0A // DLH, DLM, DLHC
#define LSM303_IRB_REG_M 0x0B // DLH, DLM, DLHC
#define LSM303_IRC_REG_M 0x0C // DLH, DLM, DLHC

#define LSM303_TEMP_OUT_H_M 0x31 // DLHC
#define LSM303_TEMP_OUT_L_M 0x32 // DLHC

#define LSM303_OUT_ACC_START 0x28
#define LSM303_OUT_X_L_A 0x28
#define LSM303_OUT_X_H_A 0x29
#define LSM303_OUT_Y_L_A 0x2A
#define LSM303_OUT_Y_H_A 0x2B
#define LSM303_OUT_Z_L_A 0x2C
#define LSM303_OUT_Z_H_A 0x2D

#define LSM303DLHC_DLM_DLH_OUT_MAG_START 0x03
#define LSM303D_OUT_MAG_START 0x08

#define LSM303DLH_OUT_X_L_M 0x04
#define LSM303DLM_OUT_X_L_M 0x04
#define LSM303DLHC_OUT_X_L_M 0x04
#define LSM303D_OUT_X_L_M 0x08
#define LSM303DLH_OUT_X_H_M 0x03
#define LSM303DLM_OUT_X_H_M 0x03
#define LSM303DLHC_OUT_X_H_M 0x03
#define LSM303D_OUT_X_H_M 0x09

#define LSM303DLH_OUT_Y_H_M 0x05
#define LSM303DLM_OUT_Y_H_M 0x07
#define LSM303DLHC_OUT_Y_H_M 0x07
#define LSM303D_OUT_Y_H_M 0x0B
#define LSM303DLH_OUT_Y_L_M 0x06
#define LSM303DLHC_OUT_Y_L_M 0x08
#define LSM303DLM_OUT_Y_L_M 0x08
#define LSM303D_OUT_Y_L_M 0x0A

#define LSM303DLH_OUT_Z_H_M 0x07
#define LSM303DLH_OUT_Z_L_M 0x08
#define LSM303DLM_OUT_Z_H_M 0x05
#define LSM303DLM_OUT_Z_L_M 0x06
#define LSM303DLHC_OUT_Z_H_M 0x05
#define LSM303DLHC_OUT_Z_L_M 0x06
#define LSM303D_OUT_Z_L_M 0x0C
#define LSM303D_OUT_Z_H_M 0x0D

#define LSM303_REGISTER_MAG_TEMP_OUT_H_M 0x31
#define LSM303_REGISTER_MAG_TEMP_OUT_L_M 0x32
// reg types
#define XYZ 0b00100100
#define XZY 0b00011000

class LSM303
{
public:
	LSM303(); // You can changes the IMU settings here
	~LSM303() = default;
	float gscale;

	enum gyro_type
	{
		gyro_none,
		gyro_L3G4200D,
		gyro_L3GD20,
		gyro_L3GD20H
	};

	enum accMag_type
	{
		accMag_none,
		accMag_LSM303DLM,
		accMag_LSM303DLHC,
		accMag_LSM303D,
		accMag_LSM303DLH
	};

	// Utility
	void initialize(uint8_t addr); // initializes the
	bool testConnection();		   // Checks if device_id matches what we expect
	void resetGyro();
	void resetAccelMag();
	void resetDevice(); // Resets the IMU
	void findGyro();
	void findAccelMag();
	void readGyro();
	uint8_t getGyroID(); // Retreives the device_id
	uint8_t getAccMagID();

	// calcs
	float getTemperatureC();
	void getAcceleration(int16_t *output_buffer);
	void getRotation(int16_t *output_buffer);
	void getMotion6(int16_t *output_buffer);
	void getMotion9(int16_t *output_buffer);
	void getMagnetometer(int16_t *output_buffer);
	void setRange();

	uint8_t getGyroAddress();
	uint8_t getAccAddress();
	uint8_t getMagAddress();
	uint8_t getAccMagType();

protected:
	uint8_t devAddr;
	uint8_t buffer[6];
	uint8_t gyro_address, acc_address, mag_address;
	uint8_t mag_order, mag_start_reg;
	gyro_type gyro;
	accMag_type accMag;
};
#endif
