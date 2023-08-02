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
#include "utils.h"

#define INTERRUPTFREE  // TODO: change based on int pin number (255 = interruptFree)
#define DEBUG_SENSOR

constexpr uint8_t SFLP_GAME_EN_BIT = 1 << 1;

volatile bool imuEvent = false;

void IRAM_ATTR interruptHandler() { imuEvent = true; }

void LSM6DSV16XSensor::motionSetup() {
#ifdef DEBUG_SENSOR
	// TODO: Should anything go here
#endif
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

	m_Logger.info("Connected to %s on 0x%02x. ", getIMUNameByType(sensorType), addr);

	uint8_t status = 0;

	// Restore defaults
	status |= imu.Reset_Set(LSM6DSV16X_RESET_CTRL_REGS);

	// Set maximums
	status |= imu.Set_X_FS(LSM6DSV16X_ACCEL_MAX);
	status |= imu.Set_G_FS(LSM6DSV16X_GYRO_MAX);

	// Set FIFO size
	status |= imu.FIFO_Set_Watermark_Level(LSM6DSV16X_FIFO_MAX_ENTRIES);

	// Set FIFO SFLP Batch
	// NOTE: might not need all of this
	status |= imu.FIFO_Set_SFLP_Batch(true, true, true);

	// Set FIFO mode to "continuous", so old data gets thrown away
	status |= imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

	// Set data rate
	status |= imu.Set_X_ODR(LSM6DSV16X_FIFO_DATA_RATE);
	status |= imu.Set_G_ODR(LSM6DSV16X_FIFO_DATA_RATE);
	status |= imu.Set_SFLP_ODR(LSM6DSV16X_FIFO_DATA_RATE);

	// Enable Game Rotation Fusion
	status |= imu.Enable_Game_Rotation();

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
	uint16_t fifo_samples;
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

		if (tag == 1 || tag == 2) {
			continue;
		}

		m_Logger.info("Got tag %d", tag);
	}
}

SensorStatus LSM6DSV16XSensor::getSensorState() {
	// TODO: this may need to be redone, errorCounter gets reset at the end of the loop
	return errorCounter > 0 ? SensorStatus::SENSOR_ERROR
		 : isWorking()      ? SensorStatus::SENSOR_OK
							: SensorStatus::SENSOR_OFFLINE;
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
