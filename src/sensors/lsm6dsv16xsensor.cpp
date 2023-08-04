/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 Eiren Rain & SlimeVR contributors

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

#include "sensors/lsm6dsv16xsensor.h"

#include "GlobalVars.h"
#include "customConversions.h"
#include "lsm6dsv16xsensor.h"
#include "utils.h"

#define INTERRUPTFREE  // TODO: change based on int pin number (255 = interruptFree)
#define DEBUG_SENSOR

volatile bool imuEvent = false;

void IRAM_ATTR interruptHandler() { imuEvent = true; }

void LSM6DSV16XSensor::motionSetup() {
	errorCounter = 0;  // Either subtract to the error counter or handle the error
	if (imu.begin() == LSM6DSV16X_ERROR) {
		m_Logger.fatal(
			"Can't connect to %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	uint8_t deviceId = 0;
	if (imu.ReadID(&deviceId) == LSM6DSV16X_ERROR) {
		m_Logger.fatal(
			"The IMU returned an error when reading the device ID of: 0x%02x",
			deviceId
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	if (deviceId != LSM6DSV16X_ID) {
		m_Logger.fatal(
			"The IMU returned an invalid device ID of: 0x%02x when it should have "
			"been: 0x%02x",
			deviceId,
			LSM6DSV16X_ID
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	// printf("\n\n\n%s Self Test started on 0x%02x.(Disabled)",
	// getIMUNameByType(sensorType), addr); Test if
	// (imu.Test_IMU(LSM6DSV16X_XL_ST_NEGATIVE, LSM6DSV16X_GY_ST_NEGATIVE) ==
	// LSM6DSV16X_ERROR) { 	m_Logger.fatal( 		"The IMU returned an error during
	// the self test"
	//	);
	// ledManager.pattern(50, 50, 200);
	// return;
	//}
	printf(
		"\nConnected to %s on 0x%02x. IMU test passed ",
		getIMUNameByType(sensorType),
		addr
	);

	uint8_t status = 0;

	// Enable Block Data Update
	status |= imu.Enable_Block_Data_Update();
	status |= imu.Set_Auto_Increment(true);

	// Set maximums
	status |= imu.Set_X_FS(LSM6DSV16X_ACCEL_MAX);
	status |= imu.Set_G_FS(LSM6DSV16X_GYRO_MAX);

	// Set data rate
	status |= imu.Set_X_ODR(LSM6DSV16X_FIFO_DATA_RATE);
	status |= imu.Set_G_ODR(LSM6DSV16X_FIFO_DATA_RATE);
	status |= imu.Set_SFLP_ODR(LSM6DSV16X_FIFO_DATA_RATE);

	status |= imu.FIFO_Set_X_BDR(LSM6DSV16X_FIFO_DATA_RATE);

	status |= imu.Set_T_ODR(LSM6DSV16X_FIFO_TEMP_DATA_RATE);

	// Enable IMU
	status |= imu.Enable_X();
	status |= imu.Enable_G();

	// Set FIFO size
	status |= imu.FIFO_Set_Watermark_Level(LSM6DSV16X_FIFO_MAX_SIZE);

	// Set FIFO mode to "continuous", so old data gets thrown away
	status |= imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

	// Set FIFO SFLP Batch
	// NOTE: might not need all of this
	status |= imu.FIFO_Set_SFLP_Batch(true, false, false);
	// Enable Game Rotation Fusion
	status |= imu.Enable_Game_Rotation();

	// Set GBias
	// status |= imu.Set_G_Bias(0, 0, 0);

#ifndef INTERRUPTFREE
	attachInterrupt(m_IntPin, interruptHandler, RISING);

	errorCounter -= imu.Enable_Single_Tap_Detection(LSM6DSV16X_INT1_PIN);
	errorCounter -= imu.Enable_Double_Tap_Detection(LSM6DSV16X_INT1_PIN);
#endif

	if (status != LSM6DSV16X_OK) {
		m_Logger.fatal(
			"%d Error(s) occured enabling imu features on %s at address 0x%02x",
			errorCounter,
			getIMUNameByType(sensorType),
			addr
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	lastReset = 0;
	lastData = millis();
	working = true;
	configured = true;
}

void LSM6DSV16XSensor::motionLoop() {
	lsm6dsv16x_fifo_status_t fifo_status;
	if (imu.FIFO_Get_Status(&fifo_status) != LSM6DSV16X_OK) {
		m_Logger.error(
			"Error getting FIFO status on %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		errorCounter++;
		return;
	}

	// m_Logger.info("FIFO status: %d", fifo_status.fifo_level);

	if (fifo_status.fifo_level < 1) {
		return;
	}

	uint16_t fifo_samples = 0;
	if (imu.FIFO_Get_Num_Samples(&fifo_samples) == LSM6DSV16X_ERROR) {
		m_Logger.error(
			"Error getting number of samples in FIFO on %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		errorCounter++;
		return;
	}

	for (uint16_t i = 0; i < fifo_samples; i++) {
		uint8_t tag;

		if (imu.FIFO_Get_Tag(&tag) != LSM6DSV16X_OK) {
			m_Logger.error(
				"Failed to get FIFO data tag on %s at address 0x%02x",
				getIMUNameByType(sensorType),
				addr
			);
			continue;
		}

		uint8_t data[6];
		// printf("\n\n\nimu.FIFO_Get_Data(data)");
		imu.FIFO_Get_Data(data);
		if (tag == 1) {  // gyro
			continue;
		}
		if (tag == 2) {  // accel
			continue;
		}
		if (tag == 3) {  // temp
			continue;
		}

		if (tag == 0x13) {  // SFLP game rotation vector
			float x = Conversions::convertBytesToFloat(data[0], data[1]);
			float y = Conversions::convertBytesToFloat(data[2], data[3]);
			float z = Conversions::convertBytesToFloat(data[4], data[5]);

			fusedRotation = fusedRotationToQuaternion(x, y, z);

			lastReset = 0;
			lastData = millis();

			if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES
				|| !lastFusedRotationSent.equalsWithEpsilon(fusedRotation)) {
				newFusedRotation = true;
				lastFusedRotationSent = fusedRotation;
			}
			continue;
		}

		if (tag == 0x16) {  // SFLP gyroscope bias
			continue;
		}

		if (tag == 0x17) {  // SFLP gravity vector
			continue;
		}

		if (tag == 0x19) {  // sensor hub nack
			continue;
		}
	}
}

SensorStatus LSM6DSV16XSensor::getSensorState() {
	// TODO: this may need to be redone, errorCounter gets reset at the end of the loop
	return errorCounter > 0 ? SensorStatus::SENSOR_ERROR
		 : isWorking()      ? SensorStatus::SENSOR_OK
							: SensorStatus::SENSOR_OFFLINE;
}

Quat LSM6DSV16XSensor::fusedRotationToQuaternion(float x, float y, float z) {
	float length2 = x * x + y * y + z * z;

	if (length2 > 1) {
		float length = sqrt(length2);
		x /= length;
		y /= length;
		z /= length;
		length2 = 1;
	}

	float w = sqrt(1 - length2);
	return Quat(x, y, z, w);
}

void LSM6DSV16XSensor::sendData() {
	if (newFusedRotation) {
		newFusedRotation = false;
		networkConnection.sendRotationData(
			sensorId,
			&fusedRotation,
			DATA_TYPE_NORMAL,
			calibrationAccuracy
		);

#ifdef DEBUG_SENSOR
		m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(fusedRotation));
#endif
	}

#if SEND_ACCELERATION
	if (newAcceleration)  // Returns acceleration in G's
	{
		newAcceleration = false;
		networkConnection.sendSensorAcceleration(sensorId, acceleration);
	}
#endif

	if (tap != 0)  // chip supports tap and double tap
	{
		networkConnection.sendSensorTap(sensorId, tap);
		tap = 0;
	}
}

void LSM6DSV16XSensor::startCalibration(int calibrationType) {
	// These IMU are factory calibrated.
	// The register might be able to be changed but it could break the device
	// I don't think we will need to mess with them
}
