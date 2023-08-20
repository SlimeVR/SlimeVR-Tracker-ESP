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
) : Sensor("LSM6DSV16XSensor", type, id, address, rotation, sclPin, sdaPin), 
	  imu(&Wire, addr << 1),  // We shift the address left 1 to work with the library
	  m_IntPin(intPin),
	  sfusion(
		  1.0f / LSM6DSV16X_FIFO_DATA_RATE,
		  1.0f / LSM6DSV16X_FIFO_DATA_RATE,
		  -1.0f
	  ){};

void LSM6DSV16XSensor::motionSetup() {
	RestDetectionParams restDetectionParams;
	restDetectionParams.restMinTimeMicros = 3 * 1e6;
	//restDetectionParams.restThAcc = 0.01f;
	//restDetectionParams.restThGyr = 0.15f; //dps
	restDetectionParams.restThAcc = 0.05f;
	restDetectionParams.restThGyr = 0.25f; //dps
	sfusion.updateRestDetectionParameters(restDetectionParams);

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

	int8_t status = 0;

	status |= imu.Enable_6D_Orientation(LSM6DSV16X_INT2_PIN);
	uint8_t isFaceDown;
	// TODO: IMU rotation could be different (IMU upside down then isFaceUp)
	status |= imu.Get_6D_Orientation_ZL(&isFaceDown);  

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

	status |= imu.Set_SFLP_ODR(LSM6DSV16X_FIFO_DATA_RATE);  // Linear accel or full fusion
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

#if defined(LSM6DSV16X_ONBOARD_FUSION) && defined(LSM6DSV16X_ESP_FUSION)
	status |= imu.FIFO_Set_SFLP_Batch(true, true, false);
	status |= imu.Enable_Game_Rotation();
	loadIMUCalibration();

	// Calibration
	if (isFaceDown) {
		startCalibration(0);  // can not calibrate onboard fusion
	}
#elif defined(LSM6DSV16X_ONBOARD_FUSION)
	status |= imu.FIFO_Set_SFLP_Batch(true, true, false);
	status |= imu.Enable_Game_Rotation();
#elif defined(LSM6DSV16X_ESP_FUSION)
	status |= imu.FIFO_Set_SFLP_Batch(false, true, false);
	status |= imu.Enable_Game_Rotation();
	loadIMUCalibration();

	// Calibration
	if (isFaceDown) {
		startCalibration(0);  // can not calibrate onboard fusion
	}
#endif

	status |= imu.Disable_6D_Orientation();

	// status |= imu.beginPreconfigured();

#ifdef LSM6DSV16X_INTERRUPT
	attachInterrupt(m_IntPin, interruptHandler, RISING);
	status |= imu.Enable_Single_Tap_Detection(LSM6DSV16X_INT1_PIN);  // Tap requires an interrupt
	status |= imu.Enable_Double_Tap_Detection(LSM6DSV16X_INT1_PIN);
#endif  // LSM6DSV16X_INTERRUPT
	status |= imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);  // add to lib but we need to
	status |= imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);  // get / keep track of current fifo type

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
constexpr float dpsPerRad = 57.295779578552f;

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

	// Group all the data together //set the watermark level for nrf sleep
	if (fifo_samples < LSM6DSV16X_FIFO_FRAME_SIZE) {
		return;
	}

	readFifo(fifo_samples);

	if (newRawAcceleration && newGravityVector) {
		acceleration[0] = (rawAcceleration[0] - gravityVector[0]) * CONST_EARTH_GRAVITY;
		acceleration[1] = (rawAcceleration[1] - gravityVector[1]) * CONST_EARTH_GRAVITY;
		acceleration[2] = (rawAcceleration[2] - gravityVector[2]) * CONST_EARTH_GRAVITY;

		newAcceleration = true;
		newGravityVector = false;
	}

#if defined(LSM6DSV16X_ONBOARD_FUSION) && defined(LSM6DSV16X_ESP_FUSION)
	// TODO: fusion of fusion stuff
	fusedRotation = sfusion.getQuaternionQuat();
	lastFusedRotationSent = fusedRotation;
	newFusedRotation = true;
	newFusedGameRotation = false;
	lastData = millis();
#elif defined(LSM6DSV16X_ONBOARD_FUSION)
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
		newFusedGameRotation = false;
	}
#elif defined(LSM6DSV16X_ESP_FUSION)
		fusedRotation = sfusion.getQuaternionQuat();
		if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES || !lastFusedRotationSent.equalsWithEpsilon(fusedRotation)) {
			newFusedRotation = true;
			lastFusedRotationSent = fusedRotation;
		}
		lastData = millis();
#endif

#ifdef LSM6DSV16X_ESP_FUSION
	if (newRawAcceleration && newRawGyro) {
		sfusion.update6D(
			rawAcceleration,
			rawGyro,
			(double)(currentDataTime - previousDataTime) * LSM6DSV16X_TIMESTAMP_LSB
		);
		newRawAcceleration = false;
		newRawGyro = false;
	}
#endif
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

	if (imu.Test_IMU(LSM6DSV16X_XL_ST_NEGATIVE, LSM6DSV16X_GY_ST_NEGATIVE) == LSM6DSV16X_ERROR) {
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
	SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
    // If no compatible calibration data is found, the calibration data will just be zero-ed out
    switch (sensorCalibration.type) {
    case SlimeVR::Configuration::CalibrationConfigType::LSM6DSV16X:
        m_Calibration = sensorCalibration.data.lsm6dsv16x;
        break;

    case SlimeVR::Configuration::CalibrationConfigType::NONE:
        m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
        m_Logger.info("Calibration is advised");
        break;

    default:
        m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
        m_Logger.info("Calibration is advised");
    }

#ifdef LSM6DSV16X_ACCEL_OFFSET_CAL
	int8_t status = 0;
	status |= imu.Set_X_User_Offset(
		m_Calibration.A_off[0],
		m_Calibration.A_off[1],
		m_Calibration.A_off[2]
	);
	status |= imu.Enable_X_User_Offset();
	return (LSM6DSV16XStatusTypeDef)status;
#endif
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

// Used for calibration (Blocking)
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::readNextFifoFrame() {
	uint16_t fifo_samples = 0;
	while (fifo_samples < LSM6DSV16X_FIFO_FRAME_SIZE) {
		if (imu.FIFO_Get_Num_Samples(&fifo_samples) == LSM6DSV16X_ERROR) {
		m_Logger.error(
			"Error getting number of samples in FIFO on %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		return LSM6DSV16X_ERROR;
		}
	}
	return readFifo(LSM6DSV16X_FIFO_FRAME_SIZE);
}

	LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::readFifo(uint16_t fifo_samples) {
		for (uint16_t i = 0; i < fifo_samples; i++) {
		uint8_t tag;
		if (imu.FIFO_Get_Tag(&tag) != LSM6DSV16X_OK) {
			m_Logger.error(
				"Failed to get FIFO data tag on %s at address 0x%02x",
				getIMUNameByType(sensorType),
				addr
			);
			return LSM6DSV16X_ERROR;
		}

		switch (tag) {
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_TIMESTAMP_TAG: {
				if (i % LSM6DSV16X_FIFO_FRAME_SIZE != 0) {
					return LSM6DSV16X_OK;  // If we are not requesting a full data set
										   // then stop reading
				}
				previousDataTime = currentDataTime;
				if (imu.FIFO_Get_Timestamp(&currentDataTime)) {
					m_Logger.error(
						"Failed to get timestamp data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV16X_ERROR;
				}

				// newTemperature = false;
				newRawAcceleration = false;
				newGravityVector = false;

#ifdef LSM6DSV16X_ONBOARD_FUSION
				newFusedGameRotation = false;
#endif
#ifdef LSM6DSV16X_ESP_FUSION
				newRawGyro = false;
#endif
				break;
			}

#if SEND_ACCELERATION || defined(LSM6DSV16X_ESP_FUSION)
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_XL_NC_TAG: {  // accel

				int32_t acceleration[3];
				if (imu.FIFO_Get_X_Axes(acceleration) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get accelerometer data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV16X_ERROR;
				}
				rawAcceleration[0] = (acceleration[0] / mgPerG);
				rawAcceleration[1] = (acceleration[1] / mgPerG);
				rawAcceleration[2] = (acceleration[2] / mgPerG);

				newRawAcceleration = true;
				break;
			}
#endif  // SEND_ACCELERATION

			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG: {
				if (imu.FIFO_Get_Gravity_Vector(gravityVector) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get gravity vector on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV16X_ERROR;
				}
				gravityVector[0] /= mgPerG;
				gravityVector[1] /= mgPerG;
				gravityVector[2] /= mgPerG;
				newGravityVector = true;

#if defined(LSM6DSV16X_ESP_FUSION) && defined(LSM6DSV16X_ACCEL_OFFSET_CAL)
				// The SFLP block does not use the accelerometer calibration
				gravityVector[0] += m_Calibration.G_off[0];
				gravityVector[1] += m_Calibration.G_off[1];
				gravityVector[2] += m_Calibration.G_off[2];
#endif
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
					return LSM6DSV16X_ERROR;
				}
				
				rawGyro[0] = (float)angularVelocity[0] / mdpsPerDps;
				rawGyro[1] = (float)angularVelocity[1] / mdpsPerDps;
				rawGyro[2] = (float)angularVelocity[2] / mdpsPerDps;		

				// convert to rads/s
				rawGyro[0] /= dpsPerRad;
				rawGyro[1] /= dpsPerRad;
				rawGyro[2] /= dpsPerRad;

#ifdef LSM6DSV16X_GYRO_OFFSET_CAL			
				rawGyro[0] -= m_Calibration.G_off[0];
				rawGyro[1] -= m_Calibration.G_off[1];
				rawGyro[2] -= m_Calibration.G_off[2];
#endif
#ifdef LSM6DSV16X_GYRO_SCALE_CAL
				rawGyro[0] *= m_Calibration.G_scale[0];
				rawGyro[1] *= m_Calibration.G_scale[1];
				rawGyro[2] *= m_Calibration.G_scale[2];
#endif

				newRawGyro = true;
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
					return LSM6DSV16X_ERROR;
				}

				newFusedGameRotation = true;
				break;
			}
#endif

			default: {  // We don't use the data so remove from fifo
				uint8_t data[6];
				if (imu.FIFO_Get_Data(data) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get unwanted data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV16X_ERROR;
				}
			}
		}
	}
	return LSM6DSV16X_OK;
}

void LSM6DSV16XSensor::startCalibration(int calibrationType) {
	m_Logger.info("Flip right side up in the next 5 seconds to start calibration.");
	delay(5000);
	uint8_t isFaceUp;
	imu.Get_6D_Orientation_ZH(&isFaceUp);

	if (!isFaceUp) {
		m_Logger.info("This is your last chance to flip the tracker over. I am warning you");
		delay(5000);
		uint8_t isFaceDown;
		imu.Get_6D_Orientation_ZL(&isFaceDown);

		if (!isFaceDown) {
			return;
		}
		testGyroScaleCalibration();
		return;
	}

#ifdef LSM6DSV16X_GYRO_OFFSET_CAL
	ledManager.on();
	constexpr uint16_t calibrationSamples = 300;
	// Reset values
	float tempGxyz[3] = {0, 0, 0};
	
	m_Calibration.G_off[0] = 0.0f;
	m_Calibration.G_off[1] = 0.0f;
	m_Calibration.G_off[2] = 0.0f;

	float tempGscale[3];
	tempGscale[0] = m_Calibration.G_scale[0];
	tempGscale[1] = m_Calibration.G_scale[1];
	tempGscale[2] = m_Calibration.G_scale[2];
	
	m_Calibration.G_scale[0] = 1.0f;
	m_Calibration.G_scale[1] = 1.0f;
	m_Calibration.G_scale[2] = 1.0f;
	

	// Wait for sensor to calm down before calibration
	m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
	delay(2000);

	imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);  // add to lib but we need to
	imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);  // get / keep track of current fifo type
	waitForRest();
	uint16_t count = 0;
	while (count < calibrationSamples) {
		readNextFifoFrame();
		if (newRawGyro) {
			tempGxyz[0] += rawGyro[0];
			tempGxyz[1] += rawGyro[1];
			tempGxyz[2] += rawGyro[2];
			count++;
		}
	}


	m_Calibration.G_off[0] = tempGxyz[0] / calibrationSamples;
	m_Calibration.G_off[1] = tempGxyz[1] / calibrationSamples;
	m_Calibration.G_off[2] = tempGxyz[2] / calibrationSamples;

	m_Calibration.G_scale[0] = tempGscale[0];
	m_Calibration.G_scale[1] = tempGscale[1];
	m_Calibration.G_scale[2] = tempGscale[2];

#ifdef DEBUG_SENSOR
	m_Logger.trace(
		"Gyro calibration results: %f %f %f",
		tempGxyz[0],
		tempGxyz[1],
		tempGxyz[2]
	);
#endif
#endif


#ifdef LSM6DSV16X_ACCEL_OFFSET_CAL
	//Accelerometer Calibration
	MagnetoCalibration* magneto = new MagnetoCalibration();
	imu.Disable_X_User_Offset();

	m_Logger.info(
		"Put the device into 6 unique orientations (all sides), leave it still and do "
		"not hold/touch for 3 seconds each"
	);
	constexpr uint8_t ACCEL_CALIBRATION_DELAY_SEC = 3;
	ledManager.on();
	for (uint8_t i = ACCEL_CALIBRATION_DELAY_SEC; i > 0; i--) {
		m_Logger.info("%i...", i);
		delay(1000);
	}
	ledManager.off();

	constexpr uint16_t expectedPositions = 6;
	constexpr uint16_t numSamplesPerPosition = 96;

	uint16_t numPositionsRecorded = 0;
	uint16_t numCurrentPositionSamples = 0;
	bool waitForMotion = false;

	float* accelCalibrationChunk = new float[numSamplesPerPosition * 3];
	ledManager.pattern(100, 100, 6);
	ledManager.on();
	m_Logger.info("Gathering accelerometer data...");
	m_Logger.info(
		"Waiting for position %i, leave the device as is...",
		numPositionsRecorded + 1
	);

	imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);  // add to lib but we need to
	imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);  // get / keep track of current fifo type
	while (true) {
		uint16_t fifo_samples = 0;
		if (imu.FIFO_Get_Num_Samples(&fifo_samples) == LSM6DSV16X_ERROR) {
			m_Logger.error(
				"Error getting number of samples in FIFO on %s at address 0x%02x",
				getIMUNameByType(sensorType),
				addr
			);
			return;
		}
		

		// Group all the data together //set the watermark level here for nrf sleep
		if (fifo_samples < LSM6DSV16X_FIFO_FRAME_SIZE) {
			continue;
		}

		readFifo(fifo_samples);

		// in microseconds
		if (!newRawAcceleration) {
			continue;
		}

		sfusion.updateAcc(
			rawAcceleration,
			(currentDataTime - previousDataTime) * LSM6DSV16X_TIMESTAMP_LSB
		);
		newRawAcceleration = false;

		if (waitForMotion) {
			if (!sfusion.getRestDetected()) {
				waitForMotion = false;
			}
			continue;
		}

		if (sfusion.getRestDetected()) {
			const uint16_t i = numCurrentPositionSamples * 3;
			accelCalibrationChunk[i + 0] = rawAcceleration[0];
			accelCalibrationChunk[i + 1] = rawAcceleration[1];
			accelCalibrationChunk[i + 2] = rawAcceleration[2];
			numCurrentPositionSamples++;

			if (numCurrentPositionSamples >= numSamplesPerPosition) {
				for (int i = 0; i < numSamplesPerPosition; i++) {
					magneto->sample(
						accelCalibrationChunk[i * 3 + 0],
						accelCalibrationChunk[i * 3 + 1],
						accelCalibrationChunk[i * 3 + 2]
					);
				}
				numPositionsRecorded++;
				numCurrentPositionSamples = 0;
				if (numPositionsRecorded < expectedPositions) {
					ledManager.pattern(50, 50, 2);
					ledManager.on();
					m_Logger.info(
						"Recorded, waiting for position %i...",
						numPositionsRecorded + 1
					);
					waitForMotion = true;
				}
			}
		} else {
			numCurrentPositionSamples = 0;
		}

		if (numPositionsRecorded >= expectedPositions) {
			break;
		}
	}
	ledManager.off();
	m_Logger.debug("Calculating accelerometer calibration data...");
	delete[] accelCalibrationChunk;

	float A_BAinv[4][3];
	magneto->current_calibration(A_BAinv);
	delete magneto;

	m_Logger.debug("Finished calculating accelerometer calibration");

	m_Calibration.A_off[0] = A_BAinv[0][0];
	m_Calibration.A_off[1] = A_BAinv[0][1];
	m_Calibration.A_off[2] = A_BAinv[0][2];

#ifdef DEBUG_SENSOR
	m_Logger.trace(
		"Accel calibration results: %f %f %f",
		A_BAinv[0][0],
		A_BAinv[0][1],
		A_BAinv[0][2]
	);
#endif
#endif

	saveCalibration();
	ledManager.off();
	m_Logger.info("Calibration finished, enjoy");
}

void LSM6DSV16XSensor::printGyroScaleCalibration() {
	m_Logger.info("");
	m_Logger.info("Gyro Scale Calibration Values for %s on 0x%02x", getIMUNameByType(sensorType), addr);
	m_Logger.info("X Scale: %f", m_Calibration.G_scale[0]);
	m_Logger.info("Y Scale: %f", m_Calibration.G_scale[1]);
	m_Logger.info("Z Scale: %f", m_Calibration.G_scale[2]);

	m_Logger.info("Gyro Offset Calibration Values for %s on 0x%02x", getIMUNameByType(sensorType), addr);
	m_Logger.info("X Scale: %f", m_Calibration.G_off[0]);
	m_Logger.info("Y Scale: %f", m_Calibration.G_off[1]);
	m_Logger.info("Z Scale: %f", m_Calibration.G_off[2]);

	m_Logger.info("Accel Offset Calibration Values for %s on 0x%02x", getIMUNameByType(sensorType), addr);
	m_Logger.info("X Scale: %f", m_Calibration.A_off[0]);
	m_Logger.info("Y Scale: %f", m_Calibration.A_off[1]);
	m_Logger.info("Z Scale: %f", m_Calibration.A_off[2]);
}

void LSM6DSV16XSensor::setGyroScaleCalibration(float xScale, float yScale, float zScale) {
	m_Logger.info("Gyro Scale Calibration Updated");
	m_Calibration.G_scale[0] = xScale;
	m_Calibration.G_scale[1] = yScale;
	m_Calibration.G_scale[2] = zScale;
	saveCalibration();
}

void LSM6DSV16XSensor::resetGyroScaleCalibration() {
	m_Calibration.G_scale[0] = 1.0f;
	m_Calibration.G_scale[1] = 1.0f;
	m_Calibration.G_scale[2] = 1.0f;
	saveCalibration();
}


void LSM6DSV16XSensor::testGyroScaleCalibration() {
#ifdef LSM6DSV16X_GYRO_SCALE_CAL
	//Gyroscope scale
	m_Logger.info("");
	m_Logger.info("");
	m_Logger.info(
		"Congrats you made it to the advance calibrations, let the tracker sit still and don't "
		"touch"
	);

	printGyroScaleCalibration();

	Vector3 rawRotations[4];
	uint8_t count = 0;

	ledManager.off();
	waitForRest();

	m_Logger.info("");
	m_Logger.info(
		"\tStep 1: Move the tracker to a corner or edge that you can get it in the "
		"same position every time"
	);
	m_Logger.info("\tStep 2: Let the tracker rest until the light turns on");
	m_Logger.info(
		"\tStep 3: Rotate the tracker in one axis 360 degrees and align with the "
		"previous edge."
	);
	m_Logger.info("\t\tNOTE: the light will turn off after you start moving it");

	m_Logger.info(
		"\tStep 4: Repeat step 2 and 3 in the same axis going the opposite direction"
	);

	ledManager.on();
	waitForMovement();

	delay(2000);
	imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);  // add to lib but we need to
	imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);  // get / keep track of current fifo type

	waitForRest();
	while (count < 2) {
		ledManager.on();  // The user should rotate
		m_Logger.info("Rotate the tracker 360 degrees");

		waitForMovement();
		rawRotations[count * 2] = sfusion.getQuaternionQuat().get_euler() * dpsPerRad;
		ledManager.off();

		waitForRest();
		rawRotations[count * 2 + 1] = sfusion.getQuaternionQuat().get_euler() * dpsPerRad;
		count++;
	}

	m_Logger.info(
		"Drift First Rotation: %f, %f, %f",
		rawRotations[0].x - rawRotations[1].x,
		rawRotations[0].y - rawRotations[1].y,
		rawRotations[0].z - rawRotations[1].z
	);

	m_Logger.info(
		"Drift Second Rotation: %f, %f, %f",
		rawRotations[2].x - rawRotations[3].x,
		rawRotations[2].y - rawRotations[3].y,
		rawRotations[2].z - rawRotations[3].z
	);

	m_Logger.info(
		"Drift Initial - Final: %f, %f, %f",
		rawRotations[0].x - rawRotations[3].x,
		rawRotations[0].y - rawRotations[3].y,
		rawRotations[0].z - rawRotations[3].z
	);
	lastData = millis();
#endif
}


void LSM6DSV16XSensor::saveCalibration() {
	m_Logger.debug("Saving the calibration data");

	SlimeVR::Configuration::CalibrationConfig calibration;
	calibration.type = SlimeVR::Configuration::CalibrationConfigType::LSM6DSV16X;
	calibration.data.lsm6dsv16x = m_Calibration;
	configuration.setCalibration(sensorId, calibration);
	configuration.save();
}

void LSM6DSV16XSensor::apply6DToRestDetection() {
	if (newRawGyro && newRawAcceleration) {
		sfusion.update6D(
			rawAcceleration,
			rawGyro,
			(double)(currentDataTime - previousDataTime) * LSM6DSV16X_TIMESTAMP_LSB
		);
	}
	newRawAcceleration = false;
}

void LSM6DSV16XSensor::waitForRest() {
	while (!sfusion.getRestDetected()) {
		readNextFifoFrame();
		apply6DToRestDetection();
	}
}
void LSM6DSV16XSensor::waitForMovement() {
	while (sfusion.getRestDetected()) {
		readNextFifoFrame();
		apply6DToRestDetection();
	}
}
