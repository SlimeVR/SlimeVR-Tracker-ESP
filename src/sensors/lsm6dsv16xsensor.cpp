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

LSM6DSV16XSensor::LSM6DSV16XSensor(
	uint8_t id,
	uint8_t type,
	uint8_t address,
	float rotation,
	uint8_t sclPin,
	uint8_t sdaPin,
	uint8_t intPin
)
	: Sensor("LSM6DSV16XSensor", type, id, address, rotation, sclPin, sdaPin)
	, imu(&Wire, addr << 1)  // We shift the address left 1 to work with the library
	, m_IntPin(intPin){};

void LSM6DSV16XSensor::motionSetup() {
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

#ifdef SELF_TEST_ON_INIT
	m_Logger.info(
		"%s Self Test started on 0x%02x.(Disabled)",
		getIMUNameByType(sensorType),
		addr
	);

	if (imu.Test_IMU(LSM6DSV16X_XL_ST_NEGATIVE, LSM6DSV16X_GY_ST_NEGATIVE)
		== LSM6DSV16X_ERROR) {
		m_Logger.fatal("The IMU returned an error during the self test");
		ledManager.pattern(50, 50, 200);
		return;
	}
	m_Logger.info("Self Test Passed");
#endif

	m_Logger.info("Connected to %s on 0x%02x", getIMUNameByType(sensorType), addr);

	uint8_t status = 0;

	status |= imu.begin();

	// Restore defaults
	status |= imu.Reset_Set(LSM6DSV16X_RESET_CTRL_REGS);

	// Enable Block Data Update
	status |= imu.Enable_Block_Data_Update();
	status |= imu.Set_Auto_Increment(true);

	// Set maximums
	status |= imu.Set_X_FS(LSM6DSV16X_ACCEL_MAX);
	status |= imu.Set_G_FS(LSM6DSV16X_GYRO_MAX);

	// Set data rate
	status |= imu.Set_X_ODR(LSM6DSV16X_FIFO_DATA_RATE);
	status |= imu.Set_SFLP_ODR(LSM6DSV16X_FIFO_DATA_RATE);
	status |= imu.FIFO_Set_X_BDR(LSM6DSV16X_FIFO_DATA_RATE);
	status |= imu.Set_T_ODR(LSM6DSV16X_FIFO_TEMP_DATA_RATE);

	// Enable IMU
	status |= imu.Enable_X();
	status |= imu.Enable_G();

	// Set FIFO mode to "continuous", so old data gets thrown away
	status |= imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
	status |= imu.FIFO_Set_SFLP_Batch(true, false, false);

	// Enable Game Rotation Fusion
	status |= imu.Enable_Game_Rotation();

	// Set GBias
	// status |= imu.Set_G_Bias(0, 0, 0);

#ifdef INTERRUPT_FOR_SLEEP
	attachInterrupt(m_IntPin, interruptHandler, RISING);

	errorCounter -= imu.Enable_Single_Tap_Detection(LSM6DSV16X_INT1_PIN);
	errorCounter -= imu.Enable_Double_Tap_Detection(LSM6DSV16X_INT1_PIN);
#endif

	if (status != LSM6DSV16X_OK) {
		m_Logger.fatal(
			"Errors occured enabling imu features on %s at address 0x%02x",
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
	if (lastData + 1000 < millis() && configured) {  // Errors
		m_Logger.error(
			"The %s at address 0x%02x, has not responded in the last second",
			getIMUNameByType(sensorType),
			addr
		);
		// statusManager.setStatus(SlimeVR::Status::IMU_ERROR, true);
		// working = false;
		lastData = millis();  // reset time counter for error message

#ifdef REINIT_ON_FAILURE
		if (reinitOnFailAttempts < REINIT_RETRY_MAX_ATTEMPTS) {
			motionSetup();
		} else {
			m_Logger.error(
				"The %s at address 0x%02x, could not be revived",
				getIMUNameByType(sensorType),
				addr
			);
		}

		reinitOnFailAttempts++;  // buf overflow will make it try again in about 4 min
#endif
	}

	uint16_t fifo_samples = 0;
	if (imu.FIFO_Get_Num_Samples(&fifo_samples) == LSM6DSV16X_ERROR) {
		m_Logger.error(
			"Error getting number of samples in FIFO on %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
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
		imu.FIFO_Get_Data(data);
		switch (tag) {
			case lsm6dsv16x_fifo_out_raw_t::
				LSM6DSV16X_XL_NC_TAG:  // accel
#if SEND_ACCELERATION
									   // int32_t accelerometerInt[3];
				// errorCounter -= imu.Get_X_Axes(accelerometerInt);
				// acceleration[0] = accelerometerInt[0] * 0.01F; //convert from mg to g
				// acceleration[1] = accelerometerInt[1] * 0.01F;
				// acceleration[2] = accelerometerInt[2] * 0.01F;
				acceleration[0]
					= Conversions::convertBytesToFloat(data[0], data[1]) * sensitivity;
				acceleration[1]
					= Conversions::convertBytesToFloat(data[2], data[3]) * sensitivity;
				acceleration[2]
					= Conversions::convertBytesToFloat(data[4], data[5]) * sensitivity;
				newAcceleration = true;
#endif  // SEND_ACCELERATION
				break;
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_TEMPERATURE_TAG:  // temp
				// TODO: send temperature data to the server
				break;
			case lsm6dsv16x_fifo_out_raw_t::
				LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG: {  // SFLP game rotation
															 // vector
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
				break;
			}
		}
	}
}

SensorStatus LSM6DSV16XSensor::getSensorState() {
	return isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
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
