/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 jojos38

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

#include "bmi323sensor.h"

constexpr float GyroSensitivity = 32.768f;
constexpr float AccelSensitivity = 4096.0f;
constexpr double GScale = ((32768. / GyroSensitivity) / 32768.) * (PI / 180.0);
constexpr double AScale = CONST_EARTH_GRAVITY / AccelSensitivity;

double gScaleX = GScale;
double gScaleY = GScale;
double gScaleZ = GScale;

/**
 * @brief This function converts lsb to meter per second squared for 16 bit
 * accelerometer at range 2G, 4G, 8G or 16G.
 */
static float lsbToMps2(int16_t val) { return AScale * val; }

/**
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
/*static float lsbToDps(int16_t val) {
	return GScale * val;
}*/

/**
 * @brief Thanks Github Copilot
 */
int8_t BMI323Sensor::i2cRead(
	uint8_t registerAddress,
	uint8_t* registerData,
	uint32_t length,
	void* interfacePointer
) {
	uint8_t address = *static_cast<uint8_t*>(interfacePointer);

	// Read data from the sensor
	Wire.beginTransmission(address);
	Wire.write(registerAddress);
	Wire.endTransmission();
	Wire.requestFrom(address, length);
	for (auto i = 0u; i < length; i++) {
		registerData[i] = Wire.read();
	}

	return 0;
}

/**
 * @brief Thanks Github Copilot
 */
int8_t BMI323Sensor::i2cWrite(
	uint8_t registerAddress,
	const uint8_t* registerData,
	uint32_t length,
	void* interfacePointer
) {
	uint8_t dev_addr = *static_cast<uint8_t*>(interfacePointer);

	// Write data to the sensor
	Wire.beginTransmission(dev_addr);
	Wire.write(registerAddress);
	for (auto i = 0u; i < length; i++) {
		Wire.write(registerData[i]);
	}
	Wire.endTransmission();

	return 0;
}

/**
 * @brief Almost thanks Github Copilot
 */
void BMI323Sensor::delayUs(uint32_t period, void*) { delay(period / 1000); }

/**
 * @brief Extracts the next frame from a FIFO frame
 * @note The indexes (accelIndex, gyroIndex, tempIndex, timeIndex) needs to
 * be reset before the first call
 */
uint8_t BMI323Sensor::extractFrame(
	uint8_t* data,
	uint8_t index,
	float* accelData,
	float* gyroData,
	float* tempData
) {
	uint8_t dataValidity = ACCEL_VALID | GYRO_VALID | TEMP_VALID;

	// Unpack accelerometer
	uint8_t accelIndex = index * frameLength + BMI323::DUMMY_BYTE;
	int16_t isValid = (int16_t)((data[accelIndex + 1] << 8) | data[accelIndex]);
	if (isValid == BMI323::FIFO_ACCEL_DUMMY_FRAME) {
		dataValidity &= ~ACCEL_VALID;
	} else {
		accelData[0] = lsbToMps2(isValid);
		accelData[1]
			= lsbToMps2((int16_t)((data[accelIndex + 3] << 8) | data[accelIndex + 2]));
		accelData[2]
			= lsbToMps2((int16_t)((data[accelIndex + 5] << 8) | data[accelIndex + 4]));
		// accelData.sensor_time = (int16_t)((data[accelIndex + 13] << 8) |
		// data[accelIndex + 12]);
	}

	// Unpack gyroscope data
	uint8_t gyroIndex = accelIndex + BMI323::LENGTH_FIFO_ACCEL;
	isValid = (int16_t)((data[gyroIndex + 1] << 8) | data[gyroIndex]);
	if (isValid == BMI323::FIFO_GYRO_DUMMY_FRAME) {
		dataValidity &= ~GYRO_VALID;
	} else {
#if BMI323_USE_TEMP_CAL
		gyroData[0] = isValid;
		gyroData[1] = (int16_t)((data[gyroIndex + 3] << 8) | data[gyroIndex + 2]);
		gyroData[2] = (int16_t)((data[gyroIndex + 5] << 8) | data[gyroIndex + 4]);
#else
		gyroData[0] = (isValid)*gScaleX;
		gyroData[1]
			= ((int16_t)((data[gyroIndex + 3] << 8) | data[gyroIndex + 2])) * gScaleY;
		gyroData[2]
			= ((int16_t)((data[gyroIndex + 5] << 8) | data[gyroIndex + 4])) * gScaleZ;
#endif
		// gyroData.sensor_time = (int16_t)((data[gyroIndex + 7] << 8) | data[gyroIndex
		// + 6]);
	}

	// Unpack temperature data
	uint8_t tempIndex = gyroIndex + BMI323::LENGTH_FIFO_GYRO;
	uint16_t isTempValid = (uint16_t)((data[tempIndex + 1] << 8) | data[tempIndex]);
	if (isTempValid == BMI323::FIFO_TEMP_DUMMY_FRAME) {
		dataValidity &= ~TEMP_VALID;
	} else {
		tempData[0] = ((int16_t)((data[tempIndex + 1] << 8) | data[tempIndex]) / 512.0f)
					+ 23.0f;
	}

	return dataValidity;
}

/**
 * @brief Print calibration data
 */
void BMI323Sensor::printCalibrationData() {
	m_Logger.debug("Gyroscope calibration data:");
	m_Logger.debug(String(
					   "  Gyro offset: " + String(m_calibration.G_O[0]) + ", "
					   + String(m_calibration.G_O[1]) + ", "
					   + String(m_calibration.G_O[2])
	)
					   .c_str());
	m_Logger.debug(String(
					   "  Gyro gain: " + String(m_calibration.G_G[0]) + ", "
					   + String(m_calibration.G_G[1]) + ", "
					   + String(m_calibration.G_G[2])
	)
					   .c_str());
	m_Logger.debug("Magnometer calibration matrix:");
	m_Logger.debug("{");
	for (int i = 0; i < 3; i++) {
		m_Logger.debug(String(
						   "  " + String(m_calibration.M_B[i]) + ", "
						   + String(m_calibration.M_Ainv[i][0]) + ", "
						   + String(m_calibration.M_Ainv[i][1]) + ", "
						   + String(m_calibration.M_Ainv[i][2])
		)
						   .c_str());
	}
	m_Logger.debug("}");
	m_Logger.debug(
		String("Temperature calibration data:" + String(m_calibration.temperature))
			.c_str()
	);
}

void BMI323Sensor::motionSetup() {
	bool error = false;
	bool calibrate = false;
	int8_t result;

	m_configuration.setup();

	// Initialize the sensor
	result = bmi323.initI2C();
	if (result == BMI323::SUCCESS) {
		m_Logger.info(
			String("BMI323 Initialized on address 0x" + String(address, HEX)).c_str()
		);
	} else {
		m_Logger.info("BMI323 Initialization failed");
		error = true;
	}

	// Get the calibration data
	m_Logger.info("Loading calibration data");
	struct SlimeVR::Configuration::CalibrationConfig calibrationConfig
		= m_configuration.getCalibration(sensorId);
	m_calibration = calibrationConfig.data.bmi323;

	// Apply the calibration data
	if (m_calibration.G_G[0] != 0 || m_calibration.G_G[1] != 0
		|| m_calibration.G_G[2] != 0 || m_calibration.G_O[0] != 0
		|| m_calibration.G_O[1] != 0 || m_calibration.G_O[2] != 0) {
		m_Logger.info("Calibration data found");
		printCalibrationData();
		result = bmi323.setGyroOffsetGain(m_calibration.G_O, m_calibration.G_G);
		if (result == BMI323::SUCCESS) {
			m_Logger.info("BMI323 Calibration data applied");
		} else {
			m_Logger.error("BMI323 Calibration data apply failed");
			error = true;
		}
	} else {
		m_Logger.warn(
			"No calibration data found, please calibrate the sensor it only takes a "
			"few seconds"
		);
		calibrate = true;
	}

	// Set gyroscope configuration
	result = bmi323.setGyroConfig(
		BMI323::GYRO_ODR_400HZ,
		BMI323::GYRO_BANDWIDTH_ODR_HALF,
		BMI323::GYRO_MODE_HIGH_PERF,
		BMI323::GYRO_RANGE_1000DPS,
		BMI323::GYRO_AVG_2
	);
	if (result == BMI323::SUCCESS) {
		m_Logger.info("BMI323 Gyroscope configured");
	} else {
		m_Logger.error("BMI323 Gyroscope configuration failed");
		error = true;
	}

	// Set accelerometer configuration
	result = bmi323.setAccelConfig(
		BMI323::ACCEL_ODR_200HZ,
		BMI323::ACCEL_BANDWIDTH_ODR_HALF,
		BMI323::ACCEL_MODE_HIGH_PERF,
		BMI323::ACCEL_RANGE_8G,
		BMI323::ACCEL_AVG_2
	);
	if (result == BMI323::SUCCESS) {
		m_Logger.info("BMI323 Accelerometer configured");
	} else {
		m_Logger.error("BMI323 Accelerometer configuration failed");
		error = true;
	}

	// Set FIFO configuration
	bmi323.setFifoConfig(
		BMI323::FIFO_ACCEL_EN | BMI323::FIFO_GYRO_EN | BMI323::FIFO_TEMP_EN,
		BMI323::ENABLE
	);
	if (result == BMI323::SUCCESS) {
		m_Logger.info("BMI323 FIFO enabled");
	} else {
		m_Logger.error("BMI323 FIFO enable failed");
		error = true;
	}

	// Calculate variables for BMI323 FIFO
	frameLength = BMI323::LENGTH_FIFO_ACCEL + BMI323::LENGTH_FIFO_GYRO
				+ BMI323::LENGTH_TEMPERATURE;

#if BMI323_USE_BMM350
	// BMM350
	bmm350Address = (this->address == BMI323::ADDRESS_I2C_PRIMARY)
					  ? BMM350::ADDRESS_I2C_PRIMARY
					  : BMM350::ADDRESS_I2C_SECONDARY;
	result = bmm350.initI2C();
	if (result == BMM350::SUCCESS) {
		m_Logger.info(
			String("BMM350 initialized on address 0x" + String(bmm350Address, HEX))
				.c_str()
		);
	} else {
		m_Logger.info("BMM350 initialization failed");
		error = true;
	}

	// Configure the sensor
	result = bmm350.seOdrAvg(BMM350::ODR_25HZ, BMM350::AVG_4);
	if (result == BMM350::SUCCESS) {
		m_Logger.info("BMM350 configured");
	} else {
		m_Logger.error("BMM350 configuration failed");
		error = true;
	}

	// Disable interrupt
	result = bmm350.setInterruptEnabled(BMM350::DISABLE);
	if (result == BMM350::SUCCESS) {
		m_Logger.info("BMM350 interrupt disabled");
	} else {
		m_Logger.error("BMM350 interrupt disable failed");
		error = true;
	}

	// Set power mode
	result = bmm350.setPowermode(BMM350::NORMAL_MODE);
	if (result == BMM350::SUCCESS) {
		m_Logger.info("BMM350 power mode set");
	} else {
		m_Logger.error("BMM350 power mode set failed");
		error = true;
	}
#endif

#if BMI323_USE_TEMP_CAL
	// allocate temperature memory after calibration because OOM
	gyroTempCalibrator = new GyroTemperatureCalibrator(
		SlimeVR::Configuration::CalibrationConfigType::BMI323,
		sensorId,
		GyroSensitivity,
		(uint32_t)(0.2f / (1.0f / 200.0f))
	);

	gyroTempCalibrator->loadConfig(GyroSensitivity);
	if (gyroTempCalibrator->config.hasCoeffs) {
		gyroTempCalibrator->approximateOffset(
			m_calibration.temperature,
			GOxyzStaticTempCompensated
		);
	}
#endif

#if BMI323_USE_SENSCAL
	String localDevice = WiFi.macAddress();
	for (auto const& offsets : sensitivityOffsets) {
		if (!localDevice.equals(offsets.mac)) {
			continue;
		}
		if (offsets.sensorId != sensorId) {
			continue;
		}
#define BMI323_CALCULATE_SENSITIVTY_MUL(degrees) \
	(1.0 / (1.0 - ((degrees) / (360.0 * offsets.spins))))
		gScaleX = GScale * BMI323_CALCULATE_SENSITIVTY_MUL(offsets.x);
		gScaleY = GScale * BMI323_CALCULATE_SENSITIVTY_MUL(offsets.y);
		gScaleZ = GScale * BMI323_CALCULATE_SENSITIVTY_MUL(offsets.z);
		m_Logger.debug(
			"Custom sensitivity offset enabled: %s %s",
			offsets.mac,
			offsets.sensorId == SENSORID_PRIMARY ? "primary" : "aux"
		);
	}
#endif

	if (calibrate) {
		startCalibration(0);
	}

	// Tell SlimeVR that the sensor is working and ready
	if (!error) {
		m_status = SensorStatus::SENSOR_OK;
		working = true;
		m_Logger.info("BMI323 initialized");
	} else {
		m_status = SensorStatus::SENSOR_ERROR;
		working = false;
		m_Logger.error("BMI323 initialization failed");
	}
}

void BMI323Sensor::motionLoop() {
	// Get available data length in the FIFO
	uint16_t availableFifoLength;
	bmi323.getFifoLength(&availableFifoLength);

	/* if (availableFifoLength >= 127) {
		m_Logger.error("FIFO OVERFLOW");
	} */

	// Get the current time
	uint32_t timeMicros = micros();
	bool restDetected = m_sfusion.getRestDetected();

	// If there is enough data in the FIFO to get at least a single frame
	if (availableFifoLength >= frameLength) {
		// Make sure the length that we read is a multiple of the frame length
		uint16_t fifoReadLength = availableFifoLength
			= std::min(
				  availableFifoLength,
				  static_cast<uint16_t>(I2C_BUFFER_LENGTH - BMI323::DUMMY_BYTE)
			  ) / frameLength
				* frameLength
			+ BMI323::DUMMY_BYTE;
		int8_t result = bmi323.readFifoData(fifoData, fifoReadLength);
		if (result == BMI323::SUCCESS) {
			// Feed sensor fusion
			const uint8_t frameCount = fifoReadLength / frameLength;
			for (int i = 0; i < frameCount; i++) {
				uint8_t dataValidity
					= extractFrame(fifoData, i, accelData, gyroData, &temperatureData);

				if (dataValidity & ACCEL_VALID) {
					// m_Logger.trace(String("Accel: " + String(accelData[0]) + ", " + String(accelData[1]) + ", " + String(accelData[2])).c_str());
					m_sfusion.updateAcc(accelData, -1);
				}

				if (dataValidity & GYRO_VALID) {
					// m_Logger.trace(String("Gyro: " + String(gyroData[0]) + ", " + String(gyroData[1]) + ", " + String(gyroData[2])).c_str());
#if BMI323_USE_TEMP_CAL
					gyroTempCalibrator->updateGyroTemperatureCalibration(
						temperatureData,
						restDetected,
						gyroData[0],
						gyroData[1],
						gyroData[2]
					);

					float GOxyz[3];
					if (gyroTempCalibrator->approximateOffset(temperatureData, GOxyz)) {
						gyroData[0] = (gyroData[0]
									   - GOxyz[0] /*- GOxyzStaticTempCompensated[0]*/)
									* gScaleX;
						gyroData[1] = (gyroData[1]
									   - GOxyz[1] /*- GOxyzStaticTempCompensated[1]*/)
									* gScaleY;
						gyroData[2] = (gyroData[2]
									   - GOxyz[2] /*- GOxyzStaticTempCompensated[2]*/)
									* gScaleZ;
					} else {
						gyroData[0] = (gyroData[0]) * gScaleX;
						gyroData[1] = (gyroData[1]) * gScaleY;
						gyroData[2] = (gyroData[2]) * gScaleZ;
					}
#endif

					m_sfusion.updateGyro(gyroData, -1);
				}
			}
		} else {
			m_Logger.error("FIFO data read failed");
		}
	}

	// Check if data needs to be sent (120 Hz)
	if (timeMicros - lastRotationSendTime > rotationSendInterval) {
		lastRotationSendTime = timeMicros;

		// Rotation
		fusedRotation = m_sfusion.getQuaternionQuat();
		if (!fusedRotation.equalsWithEpsilon(lastFusedRotation)) {
			newFusedRotation = true;

			// Acceleration
			sensor_real_t const* linearAcc = m_sfusion.getLinearAcc();
			acceleration[0] = linearAcc[0];
			acceleration[1] = linearAcc[1];
			acceleration[2] = linearAcc[2];
			newAcceleration = true;

			lastFusedRotation = fusedRotation;
		}
	}

	// Check if temperature needs to be sent (2 Hz)
	if (timeMicros - lastTempSendTime > tempSendInterval) {
		lastTempSendTime = timeMicros;
		networkConnection.sendTemperature(sensorId, temperatureData);

		// Rest detect for auto-calibration
		if (timeMicros - lastAutomaticCalibration > autoCalibrationInterval) {
			if (restDetected) {
				autoCalibrationRestSecondsTimer += uint32_t(tempSendInterval / 1000000);
				if (autoCalibrationRestSecondsTimer >= autoCalibrationRestSeconds) {
					m_Logger.info("Auto calibration starting");
					// this->startCalibration(1);
					autoCalibrationRestSecondsTimer = 0;
					lastAutomaticCalibration = timeMicros;
				}
			} else {
				autoCalibrationRestSecondsTimer = 0;
			}
		}
	}

// Check if mag needs to be fused
#if BMI323_USE_BMM350
	if (timeMicros - lastMagFusionTime > magFusionInterval) {
		lastMagFusionTime = timeMicros;

		float magData[4] = {0};
		bmm350.getCompensatedMagData(magData);
		applyMagCalibrationAndScale(magData);
		float remappedAxes[3] = {magData[1], magData[0], -magData[2]};
		m_sfusion.updateMag(remappedAxes, -1);

		// m_Logger.trace(String("Mag: " + String(remappedAxes[0]) + ", " + String(remappedAxes[1]) + ", " + String(remappedAxes[2])).c_str());
	}
#endif
}

void BMI323Sensor::applyMagCalibrationAndScale(float Mxyz[3]) {
	// apply offsets and scale factors from Magneto
	float temp[3];
	for (uint8_t i = 0; i < 3; i++) {
		temp[i] = (Mxyz[i] - m_calibration.M_B[i]);
	}
	Mxyz[0] = m_calibration.M_Ainv[0][0] * temp[0]
			+ m_calibration.M_Ainv[0][1] * temp[1]
			+ m_calibration.M_Ainv[0][2] * temp[2];
	Mxyz[1] = m_calibration.M_Ainv[1][0] * temp[0]
			+ m_calibration.M_Ainv[1][1] * temp[1]
			+ m_calibration.M_Ainv[1][2] * temp[2];
	Mxyz[2] = m_calibration.M_Ainv[2][0] * temp[0]
			+ m_calibration.M_Ainv[2][1] * temp[1]
			+ m_calibration.M_Ainv[2][2] * temp[2];
}

void BMI323Sensor::startCalibration(int calibrationType) {
#if BMI323_USE_BMM350
	if (calibrationType == 0) {
		// with DMP, we just need mag data
		constexpr int calibrationSamples = 600;

		// Blink calibrating led before user should rotate the sensor
		m_Logger.info(
			"Gently rotate the device while it's gathering magnetometer data, "
			"calibration will start in 3 seconds..."
		);
		delay(3000);
		ledManager.pattern(15, 300, 3000 / 310);
		MagnetoCalibration* magneto = new MagnetoCalibration();
		for (int i = 0; i < calibrationSamples; i++) {
			ledManager.on();

			float magData[4];
			bmm350.getCompensatedMagData(magData);
			magneto->sample((double)magData[0], (double)magData[1], (double)magData[2]);
			m_Logger.debug(String(
							   "Sample " + String(i) + "/" + String(calibrationSamples)
							   + " - " + String(magData[0]) + ", " + String(magData[1])
							   + ", " + String(magData[2])
			)
							   .c_str());

			ledManager.off();
			delay(50);
		}
		m_Logger.debug("Calculating calibration data...");

		float M_BAinv[4][3];
		magneto->current_calibration(M_BAinv);
		delete magneto;

		for (int i = 0; i < 3; i++) {
			m_calibration.M_B[i] = M_BAinv[0][i];
			m_calibration.M_Ainv[0][i] = M_BAinv[1][i];
			m_calibration.M_Ainv[1][i] = M_BAinv[2][i];
			m_calibration.M_Ainv[2][i] = M_BAinv[3][i];
		}
	}
#endif

	// Calibrate sensitivity (ALWAYS FIRST)
	m_Logger.info(
		"Calibrating gyroscope sensitivity in 5 seconds... Please do not move the "
		"device"
	);
	if (calibrationType == 0) {
		delay(5000);
	}
	ledManager.on();
	struct BMI323::SelfCalibResult calibSensitivityResult;
	uint8_t result = bmi323.performGyroCalibration(
		BMI323::CALIBRATION_SENSITIVITY,
		BMI323::CALIBRATION_APPLY_TRUE,
		&calibSensitivityResult
	);
	if (result == BMI323::SUCCESS) {
		m_Logger.info("Gyroscope sensitivity calibration done");
	} else {
		m_Logger.error("Gyroscope sensitivity calibration failed");
	}

	// Calibrate offset
	m_Logger.info("Calibrating gyroscope offset... Please do not move the device");
	struct BMI323::SelfCalibResult calibOffsetResult;
	result = bmi323.performGyroCalibration(
		BMI323::CALIBRATION_OFFSET,
		BMI323::CALIBRATION_APPLY_TRUE,
		&calibOffsetResult
	);
	if (result == BMI323::SUCCESS) {
		m_Logger.info("Gyroscope offset calibration done");
	} else {
		m_Logger.error("Gyroscope offset calibration failed");
	}

	// Save results
	uint16_t offset[3];
	uint8_t gain[3];
	bmi323.getGyroOffsetGain(offset, gain);

	if (result == BMI323::SUCCESS) {
		// Save the calibration data
		m_Logger.info("Saving calibration data");
		m_calibration.G_O[0] = offset[0];
		m_calibration.G_O[1] = offset[1];
		m_calibration.G_O[2] = offset[2];

		m_calibration.G_G[0] = gain[0];
		m_calibration.G_G[1] = gain[1];
		m_calibration.G_G[2] = gain[2];

		struct SlimeVR::Configuration::CalibrationConfig calibrationConfig;
		calibrationConfig.type = SlimeVR::Configuration::CalibrationConfigType::BMI323;
		calibrationConfig.data.bmi323 = m_calibration;
		m_configuration.setCalibration(sensorId, calibrationConfig);
		m_configuration.save();

		printCalibrationData();
	} else {
		m_Logger.error("Calibration data save failed");
	}

	m_Logger.info("Calibration done");
	ledManager.off();
}

void BMI323Sensor::saveTemperatureCalibration() {
	gyroTempCalibrator->saveConfig();
	m_Logger.info("Temperature configuration saved");
}
