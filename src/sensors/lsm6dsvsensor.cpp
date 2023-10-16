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

#include "sensors/lsm6dsvsensor.h"

#include "GlobalVars.h"
#include "customConversions.h"
#include "lsm6dsvsensor.h"
#include "utils.h"


#ifdef LSM6DSV_INTERRUPT
volatile bool imuEvent = false;
void IRAM_ATTR interruptHandler() { imuEvent = true; }
#endif


LSM6DSVSensor::LSM6DSVSensor(
	uint8_t id,
	uint8_t type,
	uint8_t address,
	float rotation,
	uint8_t sclPin,
	uint8_t sdaPin,
	uint8_t intPin
) : Sensor("LSM6DSVSensor", type, id, address, rotation, sclPin, sdaPin), 
	  imu(&Wire, addr << 1),  // We shift the address left 1 to work with the library
	  m_IntPin(intPin)
#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
	  , sfusion(
		  1.0f / LSM6DSV_FIFO_DATA_RATE,
		  1.0f / LSM6DSV_FIFO_DATA_RATE,
		  -1.0f )
#endif
	  {};

void LSM6DSVSensor::motionSetup() {
	uint8_t deviceId = 0;
	if (imu.ReadID(&deviceId) == LSM6DSV16X_ERROR) {
		m_Logger.fatal(
			"The %s at 0x%02x returned an error when reading the device ID of: 0x%02x",
			getIMUNameByType(sensorType),
			addr,
			deviceId
		);
		ledManager.pattern(50, 50, 200);
		status = LSM6DSV16X_ERROR;
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

	status |= imu.Enable_6D_Orientation(LSM6DSV16X_INT2_PIN);
	uint8_t isFaceDown;
	// TODO: IMU rotation could be different (IMU upside down then isFaceUp)
	status |= imu.Get_6D_Orientation_ZL(&isFaceDown);  

#ifndef LSM6DSV_NO_SELF_TEST_ON_FACEDOWN
	if (isFaceDown) {
		if (runSelfTest() != LSM6DSV16X_OK) {
			m_Logger.fatal(
				"The %s at 0x%02x returned an error during the self test "
				"(maybe it wasn't on a flat surface?)",
				getIMUNameByType(sensorType),
				addr
			);
			ledManager.pattern(50, 50, 200);
			status = LSM6DSV16X_ERROR;
			return;
		}
	}
#endif  // LSM6DSV_NO_SELF_TEST_ON_FACEDOWN

	m_Logger.info("Connected to %s on 0x%02x", getIMUNameByType(sensorType), addr);

	status |= imu.begin();

	// Restore defaults
	status |= imu.Reset_Set(LSM6DSV16X_RESET_CTRL_REGS);

	// Enable Block Data Update
	status |= imu.Enable_Block_Data_Update();
	status |= imu.Set_Auto_Increment(true);

	// Set maximums
	status |= imu.Set_X_FS(LSM6DSV_ACCEL_MAX);
	status |= imu.Set_G_FS(LSM6DSV_GYRO_MAX);

	// Set data rate
	status |= imu.Set_X_ODR(LSM6DSV_GYRO_ACCEL_RATE, LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE);
	status |= imu.Set_G_ODR(LSM6DSV_GYRO_ACCEL_RATE, LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE);

	status |= imu.FIFO_Set_X_BDR(LSM6DSV_FIFO_DATA_RATE);

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
	status |= imu.FIFO_Set_G_BDR(LSM6DSV_FIFO_DATA_RATE);
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

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ONBOARD)
	status |= imu.FIFO_Set_SFLP_Batch(true, true, false);
	status |= imu.Set_SFLP_ODR(LSM6DSV_FIFO_DATA_RATE);
	status |= imu.Enable_SFLP_Block();
#endif
	


#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
	loadIMUCalibration();

	// Calibration
	if (isFaceDown) {
		startCalibration(0);  // can not calibrate onboard fusion
	}
#endif

	status |= imu.Disable_6D_Orientation();

	// status |= imu.beginPreconfigured();

	

#ifdef LSM6DSV_INTERRUPT
	attachInterrupt(m_IntPin, interruptHandler, RISING);
	status |= imu.Enable_Single_Tap_Detection(LSM6DSV16X_INT1_PIN); // Tap recommends an interrupt
#else
	status |= imu.Enable_Single_Tap_Detection(LSM6DSV16X_INT2_PIN); //Just poll to see if an event happened jank but works
#endif  // LSM6DSV_INTERRUPT

	status |= imu.Set_Tap_Threshold(LSM6DSV_TAP_THRESHOLD);
	status |= imu.Set_Tap_Shock_Time(LSM6DSV_TAP_SHOCK_TIME);
	status |= imu.Set_Tap_Quiet_Time(LSM6DSV_TAP_QUITE_TIME);

	status |= imu.FIFO_Reset();
	
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
constexpr uint8_t fifoFramSize = 4; // X BDR, (G BDR || Game), Gravity, Timestamp

void LSM6DSVSensor::motionLoop() {
#ifdef LSM6DSV_INTERRUPT
	if (imuEvent) {
		LSM6DSV16X_Event_Status_t eventStatus;
		status |= imu.Get_X_Event_Status(&eventStatus);

		if (eventStatus.TapStatus) {
			tap++;
		}
		imuEvent = false;
	}
#else
	LSM6DSV_Event_Status_t eventStatus;
	status |= imu.Get_X_Event_Status(&eventStatus);

	if (eventStatus.TapStatus) {
		tap++;
	}
#endif

	if (millis() - lastTempRead > LSM6DSV_TEMP_READ_INTERVAL * 1000) {
		lastTempRead = millis();

		int16_t rawTemp;
		if (imu.Get_Temperature_Raw(&rawTemp) != LSM6DSV16X_OK) {
			m_Logger.error(
				"Error getting temperature on %s at address 0x%02x",
				getIMUNameByType(sensorType),
				addr
			);
			status = LSM6DSV16X_ERROR;
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
	}

	uint16_t fifo_samples = 0;
	if (imu.FIFO_Get_Num_Samples(&fifo_samples) == LSM6DSV16X_ERROR) {
		m_Logger.error(
			"Error getting number of samples in FIFO on %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		status = LSM6DSV16X_ERROR;
		return;
	}

	// Group all the data together //set the watermark level for nrf sleep
	// TODO: add the interupt code to this
	
	if (fifo_samples < fifoFramSize) {
		return;
	}

	readFifo(fifo_samples);

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
	fusedRotation = sfusion.getQuaternionQuat() * sensorOffset;
	if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES || !lastFusedRotationSent.equalsWithEpsilon(fusedRotation)) {
		newFusedRotation = true;
		lastFusedRotationSent = fusedRotation;
	}
	lastData = millis();

	sfusion.getLinearAcc(acceleration);
	newAcceleration = true;

	if (newRawAcceleration && newRawGyro) {
		sfusion.update6D(
			rawAcceleration,
			rawGyro,
			lsm6dsv16x_from_lsb_to_nsec(currentDataTime - previousDataTime) * 1e-9
		);
		newRawAcceleration = false;
		newRawGyro = false;
	}
#endif

#ifdef DEBUG_SENSOR
	Vector3 rotation = lastFusedRotationSent.get_euler() * dpsPerRad;
	m_Logger.trace(",%f,%f,%f,%d,%f,%d,%f,%f,%f,%f,%f,%f",
		rotation.x,
		rotation.y,
		rotation.z,
		currentDataTime,
		lsm6dsv16x_from_lsb_to_nsec(currentDataTime - previousDataTime) * 1e-9,
		millis(),
		rawGyro[0],
		rawGyro[1],
		rawGyro[2],
		rawAcceleration[0],
		rawAcceleration[1],
		rawAcceleration[2]);
#endif // DEBUG_SENSOR
}

SensorStatus LSM6DSVSensor::getSensorState() {
	return isWorking()
			 ? (status == LSM6DSV16X_OK ? SensorStatus::SENSOR_OK
										: SensorStatus::SENSOR_ERROR)
			 : SensorStatus::SENSOR_OFFLINE;
}

Quat LSM6DSVSensor::fusedRotationToQuaternion(float x, float y, float z) {
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

LSM6DSV16XStatusTypeDef LSM6DSVSensor::runSelfTest() {
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

void LSM6DSVSensor::sendData() {
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

	if (tap != 0)
	{
		networkConnection.sendSensorTap(sensorId, tap);
		tap = 0;
	}

	if (newTemperature) {
		newTemperature = false;
		networkConnection.sendTemperature(sensorId, temperature);
	}
}

LSM6DSV16XStatusTypeDef LSM6DSVSensor::readFifo(uint16_t fifo_samples) {
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
				if (i % fifoFramSize != 0) {
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

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
				newRawGyro = false;
#endif
				break;
			}

			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_XL_NC_TAG: {  // accel

				int32_t intAcceleration[3];
				if (imu.FIFO_Get_X_Axes(intAcceleration) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get accelerometer data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV16X_ERROR;
				}
				rawAcceleration[0] = (intAcceleration[0] / mgPerG);
				rawAcceleration[1] = (intAcceleration[1] / mgPerG);
				rawAcceleration[2] = (intAcceleration[2] / mgPerG);

				newRawAcceleration = true;
				break;
			}

			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG: {
				float gravityVector[3];
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

				SlimeVR::Sensors::SensorFusion::calcLinearAcc(rawAcceleration, gravityVector, acceleration);
				newAcceleration = true;
				break;
			}

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
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

#ifdef LSM6DSV_GYRO_OFFSET_CAL			
				rawGyro[0] -= m_Calibration.G_off[0];
				rawGyro[1] -= m_Calibration.G_off[1];
				rawGyro[2] -= m_Calibration.G_off[2];
#endif
#ifdef LSM6DSV_GYRO_SCALE_CAL
				rawGyro[0] *= m_Calibration.G_sensitivity[0];
				rawGyro[1] *= m_Calibration.G_sensitivity[1];
				rawGyro[2] *= m_Calibration.G_sensitivity[2];
#endif

				newRawGyro = true;
				break;
			}
#endif

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ONBOARD)
			case lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG: {
				float fusedGameRotation[3];
				if (imu.FIFO_Get_Game_Vector(fusedGameRotation) != LSM6DSV16X_OK) {
					m_Logger.error(
						"Failed to get game rotation vector on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV16X_ERROR;
				}

				fusedRotation = fusedRotationToQuaternion(
									fusedGameRotation[0],
									fusedGameRotation[1],
									fusedGameRotation[2]
								)
							  * sensorOffset;
				if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES
					|| !lastFusedRotationSent.equalsWithEpsilon(fusedRotation)) {
					newFusedRotation = true;
					lastFusedRotationSent = fusedRotation;
				}
				lastData = millis();
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



#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
// Used for calibration (Blocking)
LSM6DSV16XStatusTypeDef LSM6DSVSensor::readNextFifoFrame() {
	uint16_t fifo_samples = 0;
	while (fifo_samples < fifoFramSize) {
		if (imu.FIFO_Get_Num_Samples(&fifo_samples) == LSM6DSV16X_ERROR) {
		m_Logger.error(
			"Error getting number of samples in FIFO on %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		return LSM6DSV16X_ERROR;
		}
	}
	return readFifo(fifoFramSize);
}

LSM6DSV16XStatusTypeDef LSM6DSVSensor::loadIMUCalibration() {
	SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
    // If no compatible calibration data is found, the calibration data will just be zero-ed out
    switch (sensorCalibration.type) {
    case SlimeVR::Configuration::CalibrationConfigType::LSM6DSV:
        m_Calibration = sensorCalibration.data.lsm6dsv;
        break;

    case SlimeVR::Configuration::CalibrationConfigType::NONE:
        m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
        m_Logger.info("Calibration is advised");
        break;

    default:
        m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
        m_Logger.info("Calibration is advised");
    }

#ifdef LSM6DSV_ACCEL_OFFSET_CAL
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

#ifdef LSM6DSV_ACCEL_OFFSET_CAL
void LSM6DSVSensor::calibrateAccel() {
	m_Logger.info(
		"Accel offset calibration started on sensor #%d of type %s at address 0x%02x",
		getSensorId(),
		getIMUNameByType(sensorType),
		addr
	);
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

	imu.FIFO_Reset();
	while (true) {
		uint16_t fifo_samples = 0;
		if (imu.FIFO_Get_Num_Samples(&fifo_samples) == LSM6DSV16X_ERROR) {
			m_Logger.error(
				"Error getting number of samples in FIFO on %s at address 0x%02x",
				getIMUNameByType(sensorType),
				addr
			);
			status = LSM6DSV16X_ERROR;
			return;
		}
		

		// Group all the data together //set the watermark level here for nrf sleep
		if (fifo_samples < fifoFramSize) {
			continue;
		}

		readFifo(fifo_samples);

		// in microseconds
		if (!newRawAcceleration) {
			continue;
		}

		sfusion.updateAcc(
			rawAcceleration,
			lsm6dsv16x_from_lsb_to_nsec(currentDataTime - previousDataTime) * 1e-9
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
	saveCalibration();

	imu.Set_X_User_Offset(
		m_Calibration.A_off[0],
		m_Calibration.A_off[1],
		m_Calibration.A_off[2]
	);
	lastData = millis();

#ifdef DEBUG_SENSOR
	m_Logger.trace(
		"Accel calibration results: %f %f %f",
		A_BAinv[0][0],
		A_BAinv[0][1],
		A_BAinv[0][2]
	);
#endif
}
#endif

#ifdef LSM6DSV_GYRO_OFFSET_CAL
void LSM6DSVSensor::calibrateGyro() {
	m_Logger.info(
		"Gyro offset calibration started on sensor #%d of type %s at address 0x%02x",
		getSensorId(),
		getIMUNameByType(sensorType),
		addr
	);
	ledManager.on();
	constexpr uint16_t calibrationSamples = 300;
	// Reset values
	float tempGxyz[3] = {0, 0, 0};
	
	m_Calibration.G_off[0] = 0.0f;
	m_Calibration.G_off[1] = 0.0f;
	m_Calibration.G_off[2] = 0.0f;

	float tempGsensitivity[3];
	tempGsensitivity[0] = m_Calibration.G_sensitivity[0];
	tempGsensitivity[1] = m_Calibration.G_sensitivity[1];
	tempGsensitivity[2] = m_Calibration.G_sensitivity[2];
	
	m_Calibration.G_sensitivity[0] = 1.0f;
	m_Calibration.G_sensitivity[1] = 1.0f;
	m_Calibration.G_sensitivity[2] = 1.0f;
	

	// Wait for sensor to calm down before calibration
	m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
	delay(2000);

	imu.FIFO_Reset();
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
	ledManager.off();
	m_Calibration.G_off[0] = tempGxyz[0] / calibrationSamples;
	m_Calibration.G_off[1] = tempGxyz[1] / calibrationSamples;
	m_Calibration.G_off[2] = tempGxyz[2] / calibrationSamples;

	m_Calibration.G_sensitivity[0] = tempGsensitivity[0];
	m_Calibration.G_sensitivity[1] = tempGsensitivity[1];
	m_Calibration.G_sensitivity[2] = tempGsensitivity[2];
	saveCalibration();
	lastData = millis();
	

#ifdef DEBUG_SENSOR
	m_Logger.trace(
		"Gyro calibration results: %f %f %f",
		tempGxyz[0],
		tempGxyz[1],
		tempGxyz[2]
	);
#endif
}
#endif


#ifdef LSM6DSV_GYRO_SENSITIVITY_CAL //Redo based on the bmi implementation
void LSM6DSVSensor::calibrateGyroSensitivity() {
	m_Logger.info(
		"Gyro sensitivity calibration started on sensor #%d of type %s at address 0x%02x",
		getSensorId(),
		getIMUNameByType(sensorType),
		addr
	);

	m_Calibration.G_sensitivity[0] = 1.0f;
	m_Calibration.G_sensitivity[1] = 1.0f;
	m_Calibration.G_sensitivity[2] = 1.0f;

	imu.Enable_6D_Orientation(LSM6DSV16X_INT2_PIN);
	Vector3 rawRotationInit;
	Vector3 rawRotationFinal;
	uint8_t count = 0;
	float gyroCount[3]; //Use this to determine the axis spun
	float calculatedScale[3] = {1.0f, 1.0f, 1.0f};

	ledManager.off();

	m_Logger.info("");
	m_Logger.info(
		"\tStep 1: Move the tracker to a corner or edge that you can get it in the "
		"same position every time"
	);
	m_Logger.info("\tStep 2: Let the tracker rest until the light turns on");
	m_Logger.info(
		"\tStep 3: Rotate the tracker about one axis %d full rotations and align with the "
		"previous edge.",
		LSM6DSV_GYRO_SENSITIVITY_SPINS
	);
	m_Logger.info("\t\tNOTE: the light will turn off after you start moving it");

	m_Logger.info(
		"\tStep 4: Repeat step 1 - 3 in the other 2 axis. When the light is on you should move it"
	);

	waitForRest();
	while (count < 3) {
		ledManager.on();
		waitForMovement(); //position tracker
		ledManager.off();
		waitForRest();

		ledManager.on();  // The user should rotate
		m_Logger.info("Rotate the tracker %d times", LSM6DSV_GYRO_SENSITIVITY_SPINS);
		gyroCount[0] = 0.0f;
		gyroCount[1] = 0.0f;
		gyroCount[2] = 0.0f;
		rawRotationInit = sfusion.getQuaternionQuat().get_euler() * dpsPerRad;
		waitForMovement();
		ledManager.off();

		

		while (!sfusion.getRestDetected()) { //wait for rest
			readNextFifoFrame();
			gyroCount[0] += rawGyro[0];
			gyroCount[1] += rawGyro[1];
			gyroCount[2] += rawGyro[2];
			apply6DToRestDetection();
		}

		uint8_t isUp;
		rawRotationFinal = sfusion.getQuaternionQuat().get_euler() * dpsPerRad;

		if (abs(gyroCount[0]) > abs(gyroCount[1]) && abs(gyroCount[0]) > abs(gyroCount[2])) { //Spun in X
			imu.Get_6D_Orientation_XH(&isUp);
			if((!isUp && gyroCount[0] > 0) || (isUp && gyroCount[0] < 0)) {
				calculatedScale[0] = (1.0 / (1.0 - ((rawRotationInit.y - rawRotationFinal.y)/(360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("X, Diff: %f", rawRotationInit.y - rawRotationFinal.y);
			}
			else {
				calculatedScale[0] = (1.0 / (1.0 - ((rawRotationFinal.y - rawRotationInit.y)/(360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("-X, Diff: %f", rawRotationFinal.y - rawRotationInit.y);
			}
		}

		else if (abs(gyroCount[1]) > abs(gyroCount[0]) && abs(gyroCount[1]) > abs(gyroCount[2])) { //Spun in Y
			imu.Get_6D_Orientation_YH(&isUp);
			if((isUp && gyroCount[1] > 0) || (!isUp && gyroCount[1] < 0)) {
				calculatedScale[1] = (1.0 / (1.0 - ((rawRotationInit.y - rawRotationFinal.y)/(360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("Y, Diff: %f", rawRotationInit.y - rawRotationFinal.y);
			}
			else {
				calculatedScale[1] = (1.0 / (1.0 - ((rawRotationFinal.y - rawRotationInit.y)/(360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("-Y, Diff: %f", rawRotationFinal.y - rawRotationInit.y);
			}
		}

		else if (abs(gyroCount[2]) > abs(gyroCount[0]) && abs(gyroCount[2]) > abs(gyroCount[1])) { //Spun in Z
			imu.Get_6D_Orientation_ZH(&isUp);
			if((isUp && gyroCount[2] > 0) || (!isUp && gyroCount[2] < 0)) {
				calculatedScale[2] = (1.0 / (1.0 - ((rawRotationInit.y - rawRotationFinal.y)/(360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("Z, Diff: %f", rawRotationInit.y - rawRotationFinal.y);
			}
			else {
				calculatedScale[2] = (1.0 / (1.0 - ((rawRotationFinal.y - rawRotationInit.y)/(360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("-Z, Diff: %f", rawRotationFinal.y - rawRotationInit.y);
			}
		}
		count++;
	}
	m_Calibration.G_sensitivity[0] = calculatedScale[0];
	m_Calibration.G_sensitivity[1] = calculatedScale[1];
	m_Calibration.G_sensitivity[2] = calculatedScale[2];
	saveCalibration();

	m_Logger.debug(
		"Gyro Sensitivity calibration results: %f %f %f",
		calculatedScale[0],
		calculatedScale[1],
		calculatedScale[2]
	);

	imu.Disable_6D_Orientation();

	lastData = millis();

#ifdef DEBUG_SENSOR
	m_Logger.trace(
		"Gyro Sensitivity calibration results: %f %f %f",
		calculatedScale[0],
		calculatedScale[1],
		calculatedScale[2]
	);
#endif
}
#endif

void LSM6DSVSensor::startCalibration(int calibrationType) {
	m_Logger.info("Flip right side up in the next 5 seconds to start calibration.");
	delay(5000);
	uint8_t isFaceUp;
	imu.Get_6D_Orientation_ZH(&isFaceUp);

	if (!isFaceUp) {
		m_Logger.info("Flip the tracker over now for gyro sensitivity calibration");
		delay(5000);
		imu.Get_6D_Orientation_ZH(&isFaceUp);

		if (!isFaceUp) {
			return;
		}
		calibrateGyroSensitivity();
		return;
	}


#ifdef LSM6DSV_GYRO_OFFSET_CAL
	calibrateGyro();
#endif

#ifdef LSM6DSV_ACCEL_OFFSET_CAL
	calibrateAccel();
#endif
	m_Logger.info("Calibration finished, enjoy");
}

void LSM6DSVSensor::printCalibration() {
	m_Logger.info("Sensor #%d Calibration Data", getSensorId());
	m_Logger.info("  Accel Offset Calibration Values for %s on 0x%02x", getIMUNameByType(sensorType), addr);
	m_Logger.info("    X Scale: %f", m_Calibration.A_off[0]);
	m_Logger.info("    Y Scale: %f", m_Calibration.A_off[1]);
	m_Logger.info("    Z Scale: %f", m_Calibration.A_off[2]);

	m_Logger.info("  Gyro Offset Calibration Values for %s on 0x%02x", getIMUNameByType(sensorType), addr);
	m_Logger.info("    X Scale: %f", m_Calibration.G_off[0]);
	m_Logger.info("    Y Scale: %f", m_Calibration.G_off[1]);
	m_Logger.info("    Z Scale: %f", m_Calibration.G_off[2]);

	m_Logger.info("  Gyro Sensitivity Calibration Values for %s on 0x%02x", getIMUNameByType(sensorType), addr);
	m_Logger.info("    X Scale: %f", m_Calibration.G_sensitivity[0]);
	m_Logger.info("    Y Scale: %f", m_Calibration.G_sensitivity[1]);
	m_Logger.info("    Z Scale: %f", m_Calibration.G_sensitivity[2]);
}

void LSM6DSVSensor::resetCalibration() {
	m_Logger.info("Sensor #%d Calibration Reset", getSensorId());
	m_Calibration.A_off[0] = 0.0f;
	m_Calibration.A_off[1] = 0.0f;
	m_Calibration.A_off[2] = 0.0f;

	m_Calibration.G_off[0] = 0.0f;
	m_Calibration.G_off[1] = 0.0f;
	m_Calibration.G_off[2] = 0.0f;

	m_Calibration.G_sensitivity[0] = 1.0f;
	m_Calibration.G_sensitivity[1] = 1.0f;
	m_Calibration.G_sensitivity[2] = 1.0f;
	saveCalibration();
}

void LSM6DSVSensor::saveCalibration() {
	m_Logger.debug("Saving the calibration data");

	SlimeVR::Configuration::CalibrationConfig calibration;
	calibration.type = SlimeVR::Configuration::CalibrationConfigType::LSM6DSV;
	calibration.data.lsm6dsv = m_Calibration;
	configuration.setCalibration(sensorId, calibration);
	configuration.save();
}

void LSM6DSVSensor::apply6DToRestDetection() {
	if (newRawGyro && newRawAcceleration) {
		sfusion.update6D(
			rawAcceleration,
			rawGyro,
			lsm6dsv16x_from_lsb_to_nsec(currentDataTime - previousDataTime) * 1e-9 //seconds
		);
	}
	newRawAcceleration = false;
}

void LSM6DSVSensor::waitForRest() {
	while (!sfusion.getRestDetected()) {
		readNextFifoFrame();
		apply6DToRestDetection();
	}
}
void LSM6DSVSensor::waitForMovement() {
	while (sfusion.getRestDetected()) {
		readNextFifoFrame();
		apply6DToRestDetection();
	}
}
#endif // (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
