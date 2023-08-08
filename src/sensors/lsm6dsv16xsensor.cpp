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

void IRAM_ATTR interruptHandler() {}

#ifdef LSM6DSV16X_INTERRUPT
volatile bool imuEvent = false;

void IRAM_ATTR interruptHandler() { imuEvent = true; }
#endif

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
			"The %s at 0x%02x returned an error when reading the device ID of: 0x%02x",
			getIMUNameByType(sensorType),
			addr,
			deviceId
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	if (deviceId != LSM6DSV16X_ID) {
		m_Logger.fatal(
			"The %s at 0x%02x returned an invalid device ID of: 0x%02x when it should "
			"have been: 0x%02x",
			getIMUNameByType(sensorType),
			addr,
			deviceId,
			LSM6DSV16X_ID
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	imu.Enable_6D_Orientation(LSM6DSV16X_INT2_PIN);
	uint8_t isFaceDown;
	imu.Get_6D_Orientation_ZL(&isFaceDown);

#ifndef LSM6DSV16X_NO_SELF_TEST_ON_FACEDOWN
	if (isFaceDown) {
		if (runSelfTest() != LSM6DSV16X_OK) {
			m_Logger.fatal(
				"The %s at 0x%02x returned an error during the self test "
				"(maybe it wasn't on a flat surface?)",
				getIMUNameByType(sensorType),
				addr
			);

			ledManager.pattern(50, 50, 200);
			return;
		}
	}
#endif  // LSM6DSV16X_NO_SELF_TEST_ON_FACEDOWN

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
	status |= imu.Set_G_ODR(LSM6DSV16X_FIFO_DATA_RATE);
	status |= imu.Set_SFLP_ODR(LSM6DSV16X_FIFO_DATA_RATE);
	status |= imu.FIFO_Set_X_BDR(60);

	// Enable IMU
	status |= imu.Enable_X();
	status |= imu.Enable_G();

	status |= imu.Set_X_Filter_Mode(1, LSM6DSV16X_XL_LIGHT);
	status |= imu.Set_G_Filter_Mode(1, LSM6DSV16X_XL_LIGHT);

	//status |= imu.Set_X_Filter_Mode(0, LSM6DSV16X_XL_ULTRA_LIGHT);
	//status |= imu.Set_G_Filter_Mode(0, LSM6DSV16X_XL_ULTRA_LIGHT);

	// Set FIFO mode to "continuous", so old data gets thrown away
	status |= imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

	// Enable Fusion
	status |= imu.FIFO_Set_SFLP_Batch(true, true, false);
	status |= imu.Enable_Game_Rotation();

	// Calibration
	if (isFaceDown) {
		startCalibration(0);
	}
	status |= imu.Disable_6D_Orientation();

	// status |= imu.beginPreconfigured();

#ifdef LSM6DSV16X_INTERRUPT
	attachInterrupt(m_IntPin, interruptHandler, RISING);
	status |= imu.Enable_Single_Tap_Detection(LSM6DSV16X_INT1_PIN);
	status |= imu.Enable_Double_Tap_Detection(LSM6DSV16X_INT1_PIN);
#endif  // LSM6DSV16X_INTERRUPT

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
	lastTempRead = millis();
}

constexpr float mgPerG = 1000.0f;
constexpr float mdpsPerDps = 1000.0f;

void LSM6DSV16XSensor::motionLoop() {
	if (millis() - lastTempRead > LSM6DSV16X_TEMP_READ_INTERVAL * 1000) {
		lastTempRead = millis();

		int16_t rawTemp;
		if (imu.Get_Temperature_Raw(&rawTemp) != LSM6DSV16X_OK) {
			m_Logger.error(
				"Error getting temperature on %s at address 0x%02x",
				getIMUNameByType(sensorType),
				addr
			);
		} else {
			float actualTemp = lsm6dsv16x_from_lsb_to_celsius(rawTemp);
			if (fabsf(actualTemp - temperature) > 0.01f) {
				temperature = actualTemp;
				newTemperature = true;
			}
		}
	}

	if (lastData + 1000 < millis() && configured) {  // Errors
		m_Logger.error(
			"The %s at address 0x%02x, has not responded in the last second",
			getIMUNameByType(sensorType),
			addr
		);
		statusManager.setStatus(SlimeVR::Status::IMU_ERROR, true);
		working = false;
		lastData = millis();  // reset time counter for error message

#ifdef REINIT_ON_FAILURE  // Most likely will not fix the imu (maybe remove)
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
#endif  // REINIT_ON_FAILURE
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

		switch (tag) {
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_XL_NC_TAG: {  // accel
#if SEND_ACCELERATION
				int32_t rawAccelerations[3];
				if (imu.FIFO_Get_X_Axes(rawAccelerations) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get accelerometer data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					continue;
				}
				acceleration[0] = (rawAccelerations[0] / mgPerG) - gravity[0];
				acceleration[1] = (rawAccelerations[1] / mgPerG) - gravity[1];
				acceleration[2] = (rawAccelerations[2] / mgPerG) - gravity[2];
				acceleration[0] *= CONST_EARTH_GRAVITY;
				acceleration[1] *= CONST_EARTH_GRAVITY;
				acceleration[2] *= CONST_EARTH_GRAVITY;

				newAcceleration = true;
#endif  // SEND_ACCELERATION
				break;
			}
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG: {
				float data[3];
				if (imu.FIFO_Get_Game_Vector(data) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get game rotation vector on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					continue;
				}

				fusedRotation = fusedRotationToQuaternion(data[0], data[1], data[2])
							  * sensorOffset * gyroBias;

				lastReset = 0;
				lastData = millis();

				if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES
					|| !lastFusedRotationSent.equalsWithEpsilon(fusedRotation)) {
					newFusedRotation = true;
					lastFusedRotationSent = fusedRotation;
				}
				break;
			}
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG: {
				if (imu.FIFO_Get_Gravity_Vector(gravity) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get gravity vector on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					continue;
				}
				gravity[0] /= mgPerG;
				gravity[1] /= mgPerG;
				gravity[2] /= mgPerG;
				break;
			}
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG: {
				float gbias[3];
				if (imu.FIFO_Get_GBias_Vector(gbias) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get gyro bias on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					continue;
				}

				gbias[0] /= mdpsPerDps;
				gbias[1] /= mdpsPerDps;
				gbias[2] /= mdpsPerDps;

				// printf("\nx: %f, y: %f, z: %f", gbias[0], gbias[1], gbias[2]);
				break;
			}
			default: {  // We don't use the data so remove from fifo
				uint8_t data[6];
				imu.FIFO_Get_Data(data);
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

LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::runSelfTest() {
	m_Logger.info(
		"%s Self Test started on address: 0x%02x",
		getIMUNameByType(sensorType),
		addr
	);

	if (imu.Test_IMU(LSM6DSV16X_XL_ST_NEGATIVE, LSM6DSV16X_GY_ST_NEGATIVE)
		== LSM6DSV16X_ERROR) {
		return LSM6DSV16X_ERROR;
	}
	m_Logger.info(
		"%s Self Test Passed on address: 0x%02x",
		getIMUNameByType(sensorType),
		addr
	);

	return LSM6DSV16X_OK;
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
#endif  // DEBUG_SENSOR
	}

#if SEND_ACCELERATION
	if (newAcceleration)  // Returns acceleration in G's
	{
		newAcceleration = false;
		networkConnection.sendSensorAcceleration(sensorId, acceleration);
	}
#endif  // SEND_ACCELERATION

	if (tap != 0)  // chip supports tap and double tap
	{
		networkConnection.sendSensorTap(sensorId, tap);
		tap = 0;
	}

	if (newTemperature) {
		newTemperature = false;
		networkConnection.sendTemperature(sensorId, temperature);
	}
}

void LSM6DSV16XSensor::startCalibration(int calibrationType) {
	// Supported Calibrations: Accelerometer offset and Gyroscope Bias
	// Currently only second is used

	m_Logger.info("Flip right side up in the next 5 seconds to start calibration.");
	delay(5000);
	uint8_t isFaceUp;
	imu.Get_6D_Orientation_ZH(&isFaceUp);

	if (!isFaceUp) {
		return;
	}

	// give warning if temp is low ? (is this needed)?
	m_Logger.info("Leave still for 30 seconds");
	delay(2000);

	double gbiasAvg[] = {0, 0, 0};
	uint16_t dataCountGbias = 0;

	while (dataCountGbias < 4000) {
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

			switch (tag) {
				case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG: {
					if (dataCountGbias < 100) {
						dataCountGbias++;
						uint8_t data[6];
						imu.FIFO_Get_Data(data);
						break;
					}
					float gbias[3];
					if (imu.FIFO_Get_GBias_Vector(gbias) != LSM6DSV16X_OK) {
						m_Logger.error(
							"Failed to get gyro bias on %s at address 0x%02x",
							getIMUNameByType(sensorType),
							addr
						);
						continue;
					}

					gbiasAvg[0] += gbias[0] / mdpsPerDps;
					gbiasAvg[1] += gbias[1] / mdpsPerDps;
					gbiasAvg[2] += gbias[2] / mdpsPerDps;
					dataCountGbias++;
					break;
				}
				default: {  // We don't use the data so remove from fifo
					uint8_t data[6];
					imu.FIFO_Get_Data(data);
				}
			}
		}
	}
	m_Logger.info("Gbias x: %f, y: %f, z: %f", gbiasAvg[0], gbiasAvg[1], gbiasAvg[2]);
	dataCountGbias -= 100;

	gbiasAvg[0] = gbiasAvg[0] / dataCountGbias;
	gbiasAvg[1] = gbiasAvg[1] / dataCountGbias;
	gbiasAvg[2] = gbiasAvg[2] / dataCountGbias;

	m_Logger.info("Gbias Calibration Data, X: %f, Y: %f, Z: %f", gbiasAvg[0], gbiasAvg[1], gbiasAvg[2]);

	
	//imu.Set_SFLP_GBIAS(0.00, 0.245062, 0.150119);
	imu.Set_SFLP_GBIAS(gbiasAvg[0], gbiasAvg[1], gbiasAvg[2]);
	delay(10000);
}
