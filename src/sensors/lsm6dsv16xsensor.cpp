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
	imu.Get_6D_Orientation_ZL(&isFaceDown
	);  // IMU rotation could be different (IMU upside down)

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

	int8_t status = 0;

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

	status
		|= imu.Set_SFLP_ODR(LSM6DSV16X_FIFO_DATA_RATE);  // Linear accel or full fusion
	status |= imu.FIFO_Set_X_BDR(LSM6DSV16X_FIFO_DATA_RATE);

#ifdef LSM6DSV16X_ESP_FUSION
	status |= imu.FIFO_Set_G_BDR(LSM6DSV16X_FIFO_DATA_RATE);
#endif

	status |= imu.FIFO_Set_Timestamp_Decimation(
		lsm6dsv16x_fifo_timestamp_batch_t::LSM6DSV16X_TMSTMP_DEC_1
	);

	// Enable IMU
	status |= imu.Enable_Timestamp();
	status |= imu.Enable_X();
	status |= imu.Enable_G();

	// status |= imu.Set_X_Filter_Mode(1, LSM6DSV16X_XL_LIGHT);
	// status |= imu.Set_G_Filter_Mode(1, LSM6DSV16X_XL_LIGHT);

	// status |= imu.Set_X_Filter_Mode(0, LSM6DSV16X_XL_ULTRA_LIGHT);
	// status |= imu.Set_G_Filter_Mode(0, LSM6DSV16X_XL_ULTRA_LIGHT);

	// Set FIFO mode to "continuous", so old data gets thrown away
	status |= imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

#if defined(LSM6DSV16X_ONBOARD_FUSION) \
	&& defined(LSM6DSV16X_ESP_FUSION   \
	)  // Group all the data together //set the watermark level here for nrf sleep
	status |= imu.FIFO_Set_SFLP_Batch(
		true,
		true,
		false
	);  // Calculate Linear acceleration works great on imu
	status |= imu.Enable_Game_Rotation();

	// Calibration
	if (isFaceDown) {
		startCalibration(0);  // can not calibrate onboard fusion
	}
	loadIMUCalibration();  // accel into imu and gryo into ram
#elif defined(LSM6DSV16X_ONBOARD_FUSION)
	status |= imu.FIFO_Set_SFLP_Batch(
		true,
		true,
		false
	);  // Calculate Linear acceleration works great on imu
	status |= imu.Enable_Game_Rotation();
#elif defined(LSM6DSV16X_ESP_FUSION)
	status |= imu.FIFO_Set_SFLP_Batch(
		false,
		true,
		false
	);  // Calculate Linear acceleration works great on imu

	// Calibration
	if (isFaceDown) {
		startCalibration(0);  // can not calibrate onboard fusion
	}
	loadIMUCalibration();  // accel into imu and gryo into ram
#endif

	status |= imu.Disable_6D_Orientation();

	// status |= imu.beginPreconfigured();

#ifdef LSM6DSV16X_INTERRUPT
	attachInterrupt(m_IntPin, interruptHandler, RISING);
	status |= imu.Enable_Single_Tap_Detection(LSM6DSV16X_INT1_PIN
	);  // Tap requires an interrupt
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

#if defined(LSM6DSV16X_ONBOARD_FUSION) \
	&& defined(LSM6DSV16X_ESP_FUSION   \
	)  // Group all the data together //set the watermark level here for nrf sleep
	if (fifo_samples < 5) {  // X BDR, G BDR, Game, Gravity, Timestamp
		return;
	}
#elif defined(LSM6DSV16X_ONBOARD_FUSION)
	if (fifo_samples < 4) {  // X BDR, GAME and Gravity, Timestamp
		return;
	}
#elif defined(LSM6DSV16X_ESP_FUSION)
	if (fifo_samples < 4) {  // X BDR, G BDR, Gravity, Timestamp
		return;
	}
#endif

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
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_TIMESTAMP_TAG: {
				if (i != 0) {
					return;  // Group data together
				}
				previousDataTime = currentDataTime;
				if (imu.FIFO_Get_Timestamp(&currentDataTime)) {
					m_Logger.error(
						"Failed to get timestamp data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					continue;
				}

				// newTemperature = false;
				newFusedGameRotation = false;
				newRawAcceleration = false;
				newRawGyro = false;
				newGravityVector = false;

				m_Logger.info("timestamp: %d", currentDataTime);
				break;
			}

			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_XL_NC_TAG: {  // accel
#if SEND_ACCELERATION || defined(LSM6DSV16X_ESP_FUSION)
				int32_t acceleration[3];
				if (imu.FIFO_Get_X_Axes(acceleration) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get accelerometer data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					continue;
				}
				rawAcceleration[0] = (rawAcceleration[0] / mgPerG);
				rawAcceleration[1] = (rawAcceleration[1] / mgPerG);
				rawAcceleration[2] = (rawAcceleration[2] / mgPerG);

				newRawAcceleration = true;
				m_Logger.info("accel");
#endif  // SEND_ACCELERATION
				break;
			}
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG: {
				if (imu.FIFO_Get_Gravity_Vector(gravityVector) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get gravity vector on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					continue;
				}
				gravityVector[0] /= mgPerG;
				gravityVector[1] /= mgPerG;
				gravityVector[2] /= mgPerG;
#ifdef LSM6DSV16X_ESP_FUSION
				gravityVector[0]
					-= accelerationOffset[0];  // The SFLP block does not use the
											   // accelerometer calibration
				gravityVector[1] -= accelerationOffset[1];
				gravityVector[2] -= accelerationOffset[2];
#endif
				newGravityVector = true;
				m_Logger.info("gravity");
				break;
			}

#ifdef LSM6DSV16X_ESP_FUSION
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_GY_NC_TAG: {
				int32_t angularVelocity[3];
				if (imu.FIFO_Get_G_Axes(angularVelocity) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get accelerometer data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					continue;
				}
				rawGyro[0] = angularVelocity[0] / mdpsPerDps;
				rawGyro[1] = angularVelocity[1] / mdpsPerDps;
				rawGyro[2] = angularVelocity[2] / mdpsPerDps;

				newRawGyro = true;
				m_Logger.info("gyro");
				break;
			}
#endif

#ifdef LSM6DSV16X_ONBOARD_FUSION
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG: {
				if (imu.FIFO_Get_Game_Vector(fusedGameRotation) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get game rotation vector on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					continue;
				}

				newFusedGameRotation = true;
				m_Logger.info("game");
				break;
			}
#endif

			default: {  // We don't use the data so remove from fifo
				uint8_t data[6];
				imu.FIFO_Get_Data(data);
			}
		}
	}
	m_Logger.info("\n\n");

	if (newRawAcceleration && newGravityVector) {
		acceleration[0]
			*= (rawAcceleration[0] - gravityVector[0]) * CONST_EARTH_GRAVITY;
		acceleration[1]
			*= (rawAcceleration[1] - gravityVector[1]) * CONST_EARTH_GRAVITY;
		acceleration[2]
			*= (rawAcceleration[2] - gravityVector[2]) * CONST_EARTH_GRAVITY;

		newAcceleration = true;
	}

	if (newFusedGameRotation) {
		fusedRotation = fusedRotationToQuaternion(
			fusedGameRotation[0],
			fusedGameRotation[1],
			fusedGameRotation[2]
		);
		if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES
			|| !lastFusedRotationSent.equalsWithEpsilon(fusedRotation)) {
			newFusedRotation = true;
			lastFusedRotationSent = fusedRotation;
		}
		lastData = millis();
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

LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::loadIMUCalibration() {
	int8_t status = 0;
	status |= imu.Enable_X_User_Offset();
	status |= imu.Set_X_User_Offset(
		accelerationOffset[0],
		accelerationOffset[1],
		accelerationOffset[2]
	);
	return (LSM6DSV16XStatusTypeDef)status;
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
	/*
		double gbiasAvg[] = {0, 0, 0};
		float startingQuat[3];
		double quatDiffAvg[] = {0, 0, 0};
		uint16_t dataCountGbias = 0;
		uint16_t dataCountQuatDiff = 0;

		while (dataCountGbias < 4000 && dataCountQuatDiff < 4000) {
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
						int32_t rawAccelerations[3];
						if (imu.FIFO_Get_X_Axes(rawAccelerations) != LSM6DSV16X_OK) {
							m_Logger.error(
								"Failed to get accelerometer data on %s at address
	   0x%02x", getIMUNameByType(sensorType), addr
							);
							continue;
						}
						acceleration[0] = (rawAccelerations[0] / mgPerG) - gravity[0];
						acceleration[1] = (rawAccelerations[1] / mgPerG) - gravity[1];
						acceleration[2] = (rawAccelerations[2] / mgPerG) - gravity[2];
						acceleration[0] *= CONST_EARTH_GRAVITY;
						acceleration[1] *= CONST_EARTH_GRAVITY;
						acceleration[2] *= CONST_EARTH_GRAVITY;

						// m_Logger.info(
						//	"Acceleration x: %f, y: %f, z: %f",
						//	acceleration[0],
						//	acceleration[1],
						//	acceleration[2]
						//);
						break;
					}
					case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
	   { if (dataCountGbias < 100) { dataCountGbias++; uint8_t data[6];
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
					case lsm6dsv16x_fifo_out_raw_t::
						LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG: {
						if (dataCountQuatDiff < 99) {
							dataCountQuatDiff++;
							uint8_t data[6];
							imu.FIFO_Get_Data(data);
							break;
						}

						float data[3];
						if (imu.FIFO_Get_Game_Vector(data) != LSM6DSV16X_OK) {
							m_Logger.error(
								"Failed to get game rotation vector on %s at address "
								"0x%02x",
								getIMUNameByType(sensorType),
								addr
							);
							continue;
						}

						if (dataCountQuatDiff == 99) {
							startingQuat[0] = data[0];
							startingQuat[1] = data[1];
							startingQuat[2] = data[2];
						}

						quatDiffAvg[0] += (startingQuat[0] - data[0]);
						quatDiffAvg[1] += (startingQuat[1] - data[1]);
						quatDiffAvg[2] += (startingQuat[2] - data[2]);

						m_Logger.info(
							"Game Diff X: %f, Y: %f, Z: %f",
							(startingQuat[0] - data[0]),
							(startingQuat[1] - data[1]),
							(startingQuat[2] - data[2])
						);
						startingQuat[0] = data[0];
						startingQuat[1] = data[1];
						startingQuat[2] = data[2];
						dataCountQuatDiff++;
						break;
					}
					default: {  // We don't use the data so remove from fifo
						uint8_t data[6];
						imu.FIFO_Get_Data(data);
					}
				}
			}
		}
		m_Logger.info("Gbias x: %f, y: %f, z: %f", gbiasAvg[0], gbiasAvg[1],
	   gbiasAvg[2]); m_Logger.info( "quatAvg x: %f, y: %f, z: %f", quatDiffAvg[0],
			quatDiffAvg[1],
			quatDiffAvg[2]
		);
		dataCountGbias -= 100;
		dataCountQuatDiff -= 100;

		gbiasAvg[0] /= dataCountGbias;
		gbiasAvg[1] /= dataCountGbias;
		gbiasAvg[2] /= dataCountGbias;

		quatDiffAvg[0] /= dataCountGbias;
		quatDiffAvg[1] /= dataCountGbias;
		quatDiffAvg[2] /= dataCountGbias;

		m_Logger.info(
			"Gbias Calibration Data, X: %f, Y: %f, Z: %f",
			gbiasAvg[0],
			gbiasAvg[1],
			gbiasAvg[2]
		);

		m_Logger.info(
			"Quat Diff Calibration Data, X: %f, Y: %f, Z: %f",
			quatDiffAvg[0],
			quatDiffAvg[1],
			quatDiffAvg[2]
		);
		delay(10000);
	*/
}
