/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 S.J. Remington & SlimeVR contributors

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

#include "bmi160sensor.h"

#include <hmc5883l.h>
#include <qmc5883l.h>

#include <map>

#include "GlobalVars.h"

void BMI160Sensor::initHMC(BMI160MagRate magRate) {
	/* Configure MAG interface and setup mode */
	/* Set MAG interface normal power mode */
	imu.setRegister(BMI160_RA_CMD, BMI160_CMD_MAG_MODE_NORMAL);
	delay(60);

	/* Enable MAG interface */
	imu.setRegister(BMI160_RA_IF_CONF, BMI160_IF_CONF_MODE_PRI_AUTO_SEC_MAG);
	delay(1);

	imu.setMagDeviceAddress(HMC_DEVADDR);
	delay(3);
	imu.setRegister(BMI160_RA_MAG_IF_1_MODE, BMI160_MAG_SETUP_MODE);
	delay(3);

	/* Configure HMC5883L Sensor */
	imu.setMagRegister(
		HMC_RA_CFGA,
		HMC_CFGA_DATA_RATE_75 | HMC_CFGA_AVG_SAMPLES_8 | HMC_CFGA_BIAS_NORMAL
	);
	imu.setMagRegister(HMC_RA_CFGB, HMC_CFGB_GAIN_1_30);
	imu.setMagRegister(HMC_RA_MODE, HMC_MODE_HIGHSPEED | HMC_MODE_READ_CONTINUOUS);

	imu.setRegister(BMI160_RA_MAG_IF_2_READ_RA, HMC_RA_DATA);
	imu.setRegister(BMI160_RA_MAG_CONF, magRate);
	delay(3);
	imu.setRegister(BMI160_RA_MAG_IF_1_MODE, BMI160_MAG_DATA_MODE_6);
}

void BMI160Sensor::initQMC(BMI160MagRate magRate) {
	/* Configure MAG interface and setup mode */
	/* Set MAG interface normal power mode */
	imu.setRegister(BMI160_RA_CMD, BMI160_CMD_MAG_MODE_NORMAL);
	delay(60);

	/* Enable MAG interface */
	imu.setRegister(BMI160_RA_IF_CONF, BMI160_IF_CONF_MODE_PRI_AUTO_SEC_MAG);
	delay(1);

	imu.setMagDeviceAddress(QMC_DEVADDR);
	delay(3);
	imu.setRegister(BMI160_RA_MAG_IF_1_MODE, BMI160_MAG_SETUP_MODE);
	delay(3);

	/* Configure QMC5883L Sensor */
	imu.setMagRegister(QMC_RA_RESET, 1);
	delay(3);
	imu.setMagRegister(
		QMC_RA_CONTROL,
		QMC_CFG_MODE_CONTINUOUS | QMC_CFG_ODR_200HZ | QMC_CFG_RNG_8G | QMC_CFG_OSR_512
	);

	imu.setRegister(BMI160_RA_MAG_IF_2_READ_RA, QMC_RA_DATA);
	imu.setRegister(BMI160_RA_MAG_CONF, magRate);
	delay(3);
	imu.setRegister(BMI160_RA_MAG_IF_1_MODE, BMI160_MAG_DATA_MODE_6);
}

void BMI160Sensor::motionSetup() {
	// initialize device
	imu.initialize(
		addr,
		BMI160_GYRO_RATE,
		BMI160_GYRO_RANGE,
		BMI160_GYRO_FILTER_MODE,
		BMI160_ACCEL_RATE,
		BMI160_ACCEL_RANGE,
		BMI160_ACCEL_FILTER_MODE
	);
#if !USE_6_AXIS
#if BMI160_MAG_TYPE == BMI160_MAG_TYPE_HMC
	initHMC(BMI160_MAG_RATE);
#elif BMI160_MAG_TYPE == BMI160_MAG_TYPE_QMC
	initQMC(BMI160_MAG_RATE);
#else
	static_assert(false, "Mag is enabled but BMI160_MAG_TYPE not set in defines");
#endif
#endif

	if (!imu.testConnection()) {
		m_Logger.fatal(
			"Can't connect to BMI160 (reported device ID 0x%02x) at address 0x%02x",
			imu.getDeviceID(),
			addr
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	m_Logger.info(
		"Connected to BMI160 (reported device ID 0x%02x) at address 0x%02x",
		imu.getDeviceID(),
		addr
	);

	// Initialize the configuration
	{
		SlimeVR::Configuration::SensorConfig sensorConfig
			= configuration.getSensor(sensorId);
		// If no compatible calibration data is found, the calibration data will just be
		// zero-ed out
		switch (sensorConfig.type) {
			case SlimeVR::Configuration::SensorConfigType::BMI160:
				m_Config = sensorConfig.data.bmi160;
				break;

			case SlimeVR::Configuration::SensorConfigType::NONE:
				m_Logger.warn(
					"No calibration data found for sensor %d, ignoring...",
					sensorId
				);
				m_Logger.info("Calibration is advised");
				break;

			default:
				m_Logger.warn(
					"Incompatible calibration data found for sensor %d, ignoring...",
					sensorId
				);
				m_Logger.info("Calibration is advised");
		}
	}

	int16_t ax, ay, az;
	getRemappedAcceleration(&ax, &ay, &az);
	float g_az = (float)az / BMI160_ACCEL_TYPICAL_SENSITIVITY_LSB;
	if (g_az < -0.75f) {
		ledManager.on();

		m_Logger.info("Flip front to confirm start calibration");
		delay(5000);
		getRemappedAcceleration(&ax, &ay, &az);
		g_az = (float)az / BMI160_ACCEL_TYPICAL_SENSITIVITY_LSB;
		if (g_az > 0.75f) {
			m_Logger.debug("Starting calibration...");
			startCalibration(0);
		}

		ledManager.off();
	}

	{
#define IS_INT16_CLIPPED(value) (value == INT16_MIN || value == INT16_MAX)
		const bool anyClipped
			= IS_INT16_CLIPPED(ax) || IS_INT16_CLIPPED(ay) || IS_INT16_CLIPPED(az);
		const bool anyZero = ax == 0 || ay == 0 || az == 0;
		if (anyClipped || anyZero) {
			m_Logger.warn("---------------- WARNING -----------------");
			m_Logger.warn("One or more accelerometer axes may be dead");
			m_Logger.warn(
				"Acceleration: %i %i %i (Z = %f G)",
				ax,
				ay,
				az,
				(float)az / BMI160_ACCEL_TYPICAL_SENSITIVITY_LSB
			);
			m_Logger.warn("---------------- WARNING -----------------");
		}
	}

	// allocate temperature memory after calibration because OOM
	gyroTempCalibrator = new GyroTemperatureCalibrator(
		SlimeVR::Configuration::SensorConfigType::BMI160,
		sensorId,
		BMI160_GYRO_TYPICAL_SENSITIVITY_LSB,
		BMI160_TEMP_CALIBRATION_REQUIRED_SAMPLES_PER_STEP
	);

#if BMI160_USE_TEMPCAL
	gyroTempCalibrator->loadConfig(BMI160_GYRO_TYPICAL_SENSITIVITY_LSB);
	if (gyroTempCalibrator->config.hasCoeffs) {
		float GOxyzAtTemp[3];
		gyroTempCalibrator->approximateOffset(m_Config.temperature, GOxyzAtTemp);
		for (uint32_t i = 0; i < 3; i++) {
			GOxyzStaticTempCompensated[i] = m_Config.G_off[i] - GOxyzAtTemp[i];
		}
	}
#endif

#if BMI160_USE_SENSCAL
	{
		String localDevice = WiFi.macAddress();
		for (auto const& offsets : sensitivityOffsets) {
			if (!localDevice.equals(offsets.mac)) {
				continue;
			}
			if (offsets.sensorId != sensorId) {
				continue;
			}

#define BMI160_CALCULATE_SENSITIVTY_MUL(degrees) \
	(1.0 / (1.0 - ((degrees) / (360.0 * offsets.spins))))

			gscaleX = BMI160_GSCALE * BMI160_CALCULATE_SENSITIVTY_MUL(offsets.x);
			gscaleY = BMI160_GSCALE * BMI160_CALCULATE_SENSITIVTY_MUL(offsets.y);
			gscaleZ = BMI160_GSCALE * BMI160_CALCULATE_SENSITIVTY_MUL(offsets.z);
			m_Logger.debug(
				"Custom sensitivity offset enabled: %s %s",
				offsets.mac,
				offsets.sensorId == SENSORID_PRIMARY ? "primary" : "aux"
			);
		}
	}
#endif

	isGyroCalibrated = hasGyroCalibration();
	isAccelCalibrated = hasAccelCalibration();
#if !USE_6_AXIS
	isMagCalibrated = hasMagCalibration();
#endif
	m_Logger.info(
		"Calibration data for gyro: %s",
		isGyroCalibrated ? "found" : "not found"
	);
	m_Logger.info(
		"Calibration data for accel: %s",
		isAccelCalibrated ? "found" : "not found"
	);
#if !USE_6_AXIS
	m_Logger.info(
		"Calibration data for mag: %s",
		isMagCalibrated ? "found" : "not found"
	);
#endif

	imu.setFIFOHeaderModeEnabled(true);
	imu.setGyroFIFOEnabled(true);
	imu.setAccelFIFOEnabled(true);
#if !USE_6_AXIS
	imu.setMagFIFOEnabled(true);
#endif
	delay(4);
	imu.resetFIFO();
	delay(2);

	uint8_t err;
	if (imu.getErrReg(&err)) {
		if (err & BMI160_ERR_MASK_CHIP_NOT_OPERABLE) {
			m_Logger.fatal("Fatal error: chip not operable");
			return;
		} else if (err & BMI160_ERR_MASK_ERROR_CODE) {
			m_Logger.error("Error code 0x%02x", err);
		} else {
			m_Logger.info("Initialized");
		}
	} else {
		m_Logger.error("Failed to get error register value");
	}

	working = true;
}

void BMI160Sensor::motionLoop() {
#if ENABLE_INSPECTION
	{
		int16_t rX, rY, rZ, aX, aY, aZ;
		getRemappedRotation(&rX, &rY, &rZ);
		getRemappedAcceleration(&aX, &aY, &aZ);

		networkConnection.sendInspectionRawIMUData(
			sensorId,
			rX,
			rY,
			rZ,
			255,
			aX,
			aY,
			aZ,
			255,
			0,
			0,
			0,
			255
		);
	}
#endif

	{
		uint32_t now = micros();
		constexpr uint32_t BMI160_TARGET_SYNC_INTERVAL_MICROS = 25000;
		uint32_t elapsed = now - lastClockPollTime;
		if (elapsed >= BMI160_TARGET_SYNC_INTERVAL_MICROS) {
			lastClockPollTime = now - (elapsed - BMI160_TARGET_SYNC_INTERVAL_MICROS);

			const uint32_t nextLocalTime1 = micros();
			uint32_t rawSensorTime;
			if (imu.getSensorTime(&rawSensorTime)) {
				localTime0 = localTime1;
				localTime1 = nextLocalTime1;
				syncLatencyMicros = (micros() - localTime1) * 0.3;
				sensorTime0 = sensorTime1;
				sensorTime1 = rawSensorTime;
				if ((sensorTime0 > 0 || localTime0 > 0)
					&& (sensorTime1 > 0 || sensorTime1 > 0)) {
					// handle 24 bit overflow
					double remoteDt = sensorTime1 >= sensorTime0
										? sensorTime1 - sensorTime0
										: (sensorTime1 + 0xFFFFFF) - sensorTime0;
					double localDt = localTime1 - localTime0;
					const double nextSensorTimeRatio
						= localDt / (remoteDt * BMI160_TIMESTAMP_RESOLUTION_MICROS);

					// handle sdk lags and time travel
					if (round(nextSensorTimeRatio) == 1.0) {
						sensorTimeRatio = nextSensorTimeRatio;

						if (round(sensorTimeRatioEma) != 1.0) {
							sensorTimeRatioEma = sensorTimeRatio;
						}

						constexpr double EMA_APPROX_SECONDS = 1.0;
						constexpr uint32_t EMA_SAMPLES
							= (EMA_APPROX_SECONDS / 3 * 1e6)
							/ BMI160_TARGET_SYNC_INTERVAL_MICROS;
						sensorTimeRatioEma -= sensorTimeRatioEma / EMA_SAMPLES;
						sensorTimeRatioEma += sensorTimeRatio / EMA_SAMPLES;

						sampleDtMicros = BMI160_ODR_GYR_MICROS * sensorTimeRatioEma;
						samplesSinceClockSync = 0;
					}
				}
			}

			getTemperature(&temperature);
			optimistic_yield(100);
		}
	}

	{
		uint32_t now = micros();
		constexpr uint32_t BMI160_TARGET_POLL_INTERVAL_MICROS = 6000;
		uint32_t elapsed = now - lastPollTime;
		if (elapsed >= BMI160_TARGET_POLL_INTERVAL_MICROS) {
			lastPollTime = now - (elapsed - BMI160_TARGET_POLL_INTERVAL_MICROS);

#if BMI160_DEBUG
			uint32_t start = micros();
			readFIFO();
			uint32_t end = micros();
			cpuUsageMicros += end - start;
			if (!lastCpuUsagePrinted) {
				lastCpuUsagePrinted = end;
			}
			if (end - lastCpuUsagePrinted > 1e6) {
				bool restDetected = sfusion.getRestDetected();

				m_Logger.debug(
					"readFIFO took %0.4f ms, read gyr %i acc %i mag %i rest %i resets "
					"%i readerrs %i type " SENSOR_FUSION_TYPE_STRING,
					((float)cpuUsageMicros / 1e3f),
					gyrReads,
					accReads,
					magReads,
					restDetected,
					numFIFODropped,
					numFIFOFailedReads
				);

				cpuUsageMicros = 0;
				lastCpuUsagePrinted = end;
				gyrReads = 0;
				accReads = 0;
				magReads = 0;
			}
#else
			readFIFO();
#endif
			optimistic_yield(100);
			if (!sfusion.isUpdated()) {
				return;
			}
			hadData = true;
			sfusion.clearUpdated();
		}
	}

	{
		uint32_t now = micros();
		constexpr float maxSendRateHz = 2.0f;
		constexpr uint32_t sendInterval = 1.0f / maxSendRateHz * 1e6;
		uint32_t elapsed = now - lastTemperaturePacketSent;
		if (elapsed >= sendInterval) {
			lastTemperaturePacketSent = now - (elapsed - sendInterval);
#if BMI160_TEMPCAL_DEBUG
			uint32_t isCalibrating = gyroTempCalibrator->isCalibrating() ? 10000 : 0;
			networkConnection.sendTemperature(
				sensorId,
				isCalibrating + 10000 + (gyroTempCalibrator->config.samplesTotal * 100)
					+ temperature
			);
#else
			networkConnection.sendTemperature(sensorId, temperature);
#endif
			optimistic_yield(100);
		}
	}

	{
		uint32_t now = micros();
		constexpr float maxSendRateHz = 120.0f;
		constexpr uint32_t sendInterval = 1.0f / maxSendRateHz * 1e6;
		uint32_t elapsed = now - lastRotationPacketSent;
		if (elapsed >= sendInterval) {
			lastRotationPacketSent = now - (elapsed - sendInterval);

			setFusedRotation(sfusion.getQuaternionQuat());
			setAcceleration(sfusion.getLinearAccVec());

			optimistic_yield(100);
		}
	}
}

void BMI160Sensor::readFIFO() {
	if (!imu.getFIFOCount(&fifo.length)) {
#if BMI160_DEBUG
		numFIFOFailedReads++;
#endif
		return;
	}

	if (fifo.length <= 1) {
		return;
	}
	if (fifo.length > sizeof(fifo.data)) {
#if BMI160_DEBUG
		numFIFODropped++;
#endif
		imu.resetFIFO();
		return;
	}
	std::fill(fifo.data, fifo.data + fifo.length, 0);
	if (!imu.getFIFOBytes(fifo.data, fifo.length)) {
#if BMI160_DEBUG
		numFIFOFailedReads++;
#endif
		return;
	}

	optimistic_yield(100);

	int16_t gx, gy, gz;
	int16_t ax, ay, az;
	bool gnew, anew;
#if !USE_6_AXIS
	int16_t mx, my, mz;
	bool mnew;
#endif

	uint8_t header;
	for (uint32_t i = 0; i < fifo.length;) {
#define BMI160_FIFO_FRAME_ENSURE_BYTES_AVAILABLE(len) \
	{                                                 \
		if (i + len > fifo.length)                    \
			break;                                    \
	}
		BMI160_FIFO_FRAME_ENSURE_BYTES_AVAILABLE(1);

		// ignore interrupt tags in header
		header = fifo.data[i] & 0b11111100;
		i++;

		if (header == BMI160_FIFO_HEADER_CTL_SKIP_FRAME) {
			BMI160_FIFO_FRAME_ENSURE_BYTES_AVAILABLE(BMI160_FIFO_SKIP_FRAME_LEN);
			break;
		} else if (header == BMI160_FIFO_HEADER_CTL_SENSOR_TIME) {
			BMI160_FIFO_FRAME_ENSURE_BYTES_AVAILABLE(BMI160_FIFO_SENSOR_TIME_LEN);
			i += BMI160_FIFO_SENSOR_TIME_LEN;
		} else if (header == BMI160_FIFO_HEADER_CTL_INPUT_CONFIG) {
			BMI160_FIFO_FRAME_ENSURE_BYTES_AVAILABLE(BMI160_FIFO_INPUT_CONFIG_LEN);
			i += BMI160_FIFO_INPUT_CONFIG_LEN;
		} else if (header & BMI160_FIFO_HEADER_DATA_FRAME_BASE) {
			if (!(header & BMI160_FIFO_HEADER_DATA_FRAME_MASK_HAS_DATA)) {
				break;
			}
			gnew = false;
			anew = false;
#if !USE_6_AXIS
			mnew = false;
#endif

			// mag
			if (header & BMI160_FIFO_HEADER_DATA_FRAME_FLAG_M) {
				BMI160_FIFO_FRAME_ENSURE_BYTES_AVAILABLE(BMI160_FIFO_M_LEN);
#if !USE_6_AXIS
				getMagnetometerXYZFromBuffer(&fifo.data[i], &mx, &my, &mz);
				mnew = true;
#endif
				i += BMI160_FIFO_M_LEN;
			}

			// bmi160 -> 0 lsb 1 msb
			// gyro
			if (header & BMI160_FIFO_HEADER_DATA_FRAME_FLAG_G) {
				BMI160_FIFO_FRAME_ENSURE_BYTES_AVAILABLE(BMI160_FIFO_G_LEN);
				gx = ((int16_t)fifo.data[i + 1] << 8) | fifo.data[i + 0];
				gy = ((int16_t)fifo.data[i + 3] << 8) | fifo.data[i + 2];
				gz = ((int16_t)fifo.data[i + 5] << 8) | fifo.data[i + 4];
				gnew = true;
				i += BMI160_FIFO_G_LEN;
			}

			// bmi160 -> 0 lsb 1 msb
			// accel
			if (header & BMI160_FIFO_HEADER_DATA_FRAME_FLAG_A) {
				BMI160_FIFO_FRAME_ENSURE_BYTES_AVAILABLE(BMI160_FIFO_A_LEN);
				ax = ((int16_t)fifo.data[i + 1] << 8) | fifo.data[i + 0];
				ay = ((int16_t)fifo.data[i + 3] << 8) | fifo.data[i + 2];
				az = ((int16_t)fifo.data[i + 5] << 8) | fifo.data[i + 4];
				anew = true;
				i += BMI160_FIFO_A_LEN;
			}

// gyro callback updates fusion and must be last
#if !USE_6_AXIS
			if (mnew) {
				onMagRawSample(BMI160_ODR_MAG_MICROS, mx, my, mz);
			}
#endif
			if (anew) {
				onAccelRawSample(BMI160_ODR_ACC_MICROS, ax, ay, az);
			}
			if (gnew) {
				constexpr uint32_t alignmentBitmask
					= ~(0xFFFFFFFF << (16 - BMI160_GYRO_RATE));
				uint32_t alignmentOffset = (sensorTime1 & alignmentBitmask)
										 * BMI160_TIMESTAMP_RESOLUTION_MICROS
										 * sensorTimeRatioEma;

				timestamp0 = timestamp1;
				timestamp1 = (localTime1 - alignmentOffset - syncLatencyMicros)
						   + (++samplesSinceClockSync) * sampleDtMicros;
				int32_t dtMicros = timestamp1 - timestamp0;

				constexpr float invPeriod = 1.0f / BMI160_ODR_GYR_MICROS;
				int32_t sampleOffset = round((float)dtMicros * invPeriod) - 1;
				if (abs(sampleOffset) > 3) {
					dtMicros = sampleDtMicros;
				} else if (sampleOffset != 0) {
					dtMicros -= sampleOffset * sampleDtMicros;
				}

				onGyroRawSample(dtMicros, gx, gy, gz);
			}
		} else {
			break;
		}
	}
}

void BMI160Sensor::onGyroRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z) {
#if BMI160_DEBUG
	gyrReads++;
#endif

#if BMI160_USE_TEMPCAL
	bool restDetected = sfusion.getRestDetected();
	gyroTempCalibrator
		->updateGyroTemperatureCalibration(temperature, restDetected, x, y, z);
#endif

	sensor_real_t gyroCalibratedStatic[3];
	gyroCalibratedStatic[0]
		= (sensor_real_t)((((double)x - m_Config.G_off[0]) * gscaleX));
	gyroCalibratedStatic[1]
		= (sensor_real_t)((((double)y - m_Config.G_off[1]) * gscaleY));
	gyroCalibratedStatic[2]
		= (sensor_real_t)((((double)z - m_Config.G_off[2]) * gscaleZ));

#if BMI160_USE_TEMPCAL
	float GOxyz[3];
	if (gyroTempCalibrator->approximateOffset(temperature, GOxyz)) {
		Gxyz[0] = (sensor_real_t)((
			((double)x - GOxyz[0] - GOxyzStaticTempCompensated[0]) * gscaleX
		));
		Gxyz[1] = (sensor_real_t)((
			((double)y - GOxyz[1] - GOxyzStaticTempCompensated[1]) * gscaleY
		));
		Gxyz[2] = (sensor_real_t)((
			((double)z - GOxyz[2] - GOxyzStaticTempCompensated[2]) * gscaleZ
		));
	} else
#endif
	{
		Gxyz[0] = gyroCalibratedStatic[0];
		Gxyz[1] = gyroCalibratedStatic[1];
		Gxyz[2] = gyroCalibratedStatic[2];
	}
	remapGyroAccel(&Gxyz[0], &Gxyz[1], &Gxyz[2]);

	sfusion.updateGyro(Gxyz, (sensor_real_t)dtMicros * 1.0e-6);

	optimistic_yield(100);
}
void BMI160Sensor::onAccelRawSample(
	uint32_t dtMicros,
	int16_t x,
	int16_t y,
	int16_t z
) {
#if BMI160_DEBUG
	accReads++;
#endif

	Axyz[0] = (sensor_real_t)x;
	Axyz[1] = (sensor_real_t)y;
	Axyz[2] = (sensor_real_t)z;
	applyAccelCalibrationAndScale(Axyz);
	remapGyroAccel(&Axyz[0], &Axyz[1], &Axyz[2]);
	lastAxyz[0] = Axyz[0];
	lastAxyz[1] = Axyz[1];
	lastAxyz[2] = Axyz[2];

	sfusion.updateAcc(Axyz, (sensor_real_t)dtMicros * 1.0e-6);

	optimistic_yield(100);
}
void BMI160Sensor::onMagRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z) {
#if BMI160_DEBUG
	magReads++;
#endif

#if !USE_6_AXIS
	Mxyz[0] = (sensor_real_t)x;
	Mxyz[1] = (sensor_real_t)y;
	Mxyz[2] = (sensor_real_t)z;
	applyMagCalibrationAndScale(Mxyz);
	remapMagnetometer(&Mxyz[0], &Mxyz[1], &Mxyz[2]);
	sfusion.updateMag(Mxyz);
#endif
}

void BMI160Sensor::printTemperatureCalibrationState() {
	const auto degCtoF = [](float degC) { return (degC * 9.0f / 5.0f) + 32.0f; };

	m_Logger.info("Sensor %i temperature calibration state:", sensorId);
	m_Logger
		.info("  current temp: %0.4f C (%0.4f F)", temperature, degCtoF(temperature));
	auto printTemperatureRange = [&](const char* label, float min, float max) {
		m_Logger.info(
			"  %s: min %0.4f C max %0.4f C (min %0.4f F max %0.4f F)",
			label,
			min,
			max,
			degCtoF(min),
			degCtoF(max)
		);
	};
	printTemperatureRange("total range", TEMP_CALIBRATION_MIN, TEMP_CALIBRATION_MAX);
	printTemperatureRange(
		"calibrated range",
		gyroTempCalibrator->config.minTemperatureRange,
		gyroTempCalibrator->config.maxTemperatureRange
	);
	m_Logger.info(
		"  done: %0.1f%",
		gyroTempCalibrator->config.getCalibrationDonePercent()
	);
}
void BMI160Sensor::printDebugTemperatureCalibrationState() {
	m_Logger.info(
		"Sensor %i gyro odr %f hz, sensitivity %f lsb",
		sensorId,
		BMI160_ODR_GYR_HZ,
		BMI160_GYRO_TYPICAL_SENSITIVITY_LSB
	);
	m_Logger.info("Sensor %i temperature calibration matrix (tempC x y z):", sensorId);
	m_Logger.info("BUF %i %i", sensorId, TEMP_CALIBRATION_BUFFER_SIZE);
	m_Logger.info("SENS %i %f", sensorId, BMI160_GYRO_TYPICAL_SENSITIVITY_LSB);
	m_Logger.info("DATA %i", sensorId);
	for (int i = 0; i < TEMP_CALIBRATION_BUFFER_SIZE; i++) {
		m_Logger.info(
			"%f %f %f %f",
			gyroTempCalibrator->config.samples[i].t,
			gyroTempCalibrator->config.samples[i].x,
			gyroTempCalibrator->config.samples[i].y,
			gyroTempCalibrator->config.samples[i].z
		);
	}
	m_Logger.info("END %i", sensorId);
	m_Logger.info(
		"y = %f + (%fx) + (%fxx) + (%fxxx)",
		UNPACK_VECTOR_ARRAY(gyroTempCalibrator->config.cx),
		gyroTempCalibrator->config.cx[3]
	);
	m_Logger.info(
		"y = %f + (%fx) + (%fxx) + (%fxxx)",
		UNPACK_VECTOR_ARRAY(gyroTempCalibrator->config.cy),
		gyroTempCalibrator->config.cy[3]
	);
	m_Logger.info(
		"y = %f + (%fx) + (%fxx) + (%fxxx)",
		UNPACK_VECTOR_ARRAY(gyroTempCalibrator->config.cz),
		gyroTempCalibrator->config.cz[3]
	);
}
void BMI160Sensor::saveTemperatureCalibration() { gyroTempCalibrator->saveConfig(); }

bool BMI160Sensor::getTemperature(float* out) {
// Middle value is 23 degrees C (0x0000)
#define BMI160_ZERO_TEMP_OFFSET 23
	// Temperature per step from -41 + 1/2^9 degrees C (0x8001) to 87 - 1/2^9 degrees C
	// (0x7FFF)
	constexpr float TEMP_STEP = 128. / 65535;
	int16_t temp;
	if (imu.getTemperature(&temp)) {
		*out = (temp * TEMP_STEP) + BMI160_ZERO_TEMP_OFFSET;
		return true;
	}
	return false;
}

void BMI160Sensor::applyAccelCalibrationAndScale(sensor_real_t Axyz[3]) {
	// apply offsets (bias) and scale factors from Magneto
	if (isAccelCalibrated) {
#if useFullCalibrationMatrix == true
		float tmp[3];
		for (uint8_t i = 0; i < 3; i++) {
			tmp[i] = (Axyz[i] - m_Config.A_B[i]);
		}
		Axyz[0] = m_Config.A_Ainv[0][0] * tmp[0] + m_Config.A_Ainv[0][1] * tmp[1]
				+ m_Config.A_Ainv[0][2] * tmp[2];
		Axyz[1] = m_Config.A_Ainv[1][0] * tmp[0] + m_Config.A_Ainv[1][1] * tmp[1]
				+ m_Config.A_Ainv[1][2] * tmp[2];
		Axyz[2] = m_Config.A_Ainv[2][0] * tmp[0] + m_Config.A_Ainv[2][1] * tmp[1]
				+ m_Config.A_Ainv[2][2] * tmp[2];
#else
		for (uint8_t i = 0; i < 3; i++) {
			Axyz[i] = (Axyz[i] - calibration->A_B[i]);
		}
#endif
	}
	Axyz[0] *= BMI160_ASCALE;
	Axyz[1] *= BMI160_ASCALE;
	Axyz[2] *= BMI160_ASCALE;
}

void BMI160Sensor::applyMagCalibrationAndScale(sensor_real_t Mxyz[3]) {
#if !USE_6_AXIS
// apply offsets and scale factors from Magneto
#if useFullCalibrationMatrix == true
	float temp[3];
	for (uint8_t i = 0; i < 3; i++) {
		temp[i] = (Mxyz[i] - m_Config.M_B[i]);
	}
	Mxyz[0] = m_Config.M_Ainv[0][0] * temp[0] + m_Config.M_Ainv[0][1] * temp[1]
			+ m_Config.M_Ainv[0][2] * temp[2];
	Mxyz[1] = m_Config.M_Ainv[1][0] * temp[0] + m_Config.M_Ainv[1][1] * temp[1]
			+ m_Config.M_Ainv[1][2] * temp[2];
	Mxyz[2] = m_Config.M_Ainv[2][0] * temp[0] + m_Config.M_Ainv[2][1] * temp[1]
			+ m_Config.M_Ainv[2][2] * temp[2];
#else
	for (i = 0; i < 3; i++) {
		Mxyz[i] = (Mxyz[i] - m_Config.M_B[i]);
	}
#endif
#endif
}

bool BMI160Sensor::hasGyroCalibration() {
	for (int i = 0; i < 3; i++) {
		if (m_Config.G_off[i] != 0.0) {
			return true;
		}
	}
	return false;
}

bool BMI160Sensor::hasAccelCalibration() {
	for (int i = 0; i < 3; i++) {
		if (m_Config.A_B[i] != 0.0 || m_Config.A_Ainv[0][i] != 0.0
			|| m_Config.A_Ainv[1][i] != 0.0 || m_Config.A_Ainv[2][i] != 0.0) {
			return true;
		}
	}
	return false;
}

bool BMI160Sensor::hasMagCalibration() {
	for (int i = 0; i < 3; i++) {
		if (m_Config.M_B[i] != 0.0 || m_Config.M_Ainv[0][i] != 0.0
			|| m_Config.M_Ainv[1][i] != 0.0 || m_Config.M_Ainv[2][i] != 0.0) {
			return true;
		}
	}
	return false;
}

void BMI160Sensor::startCalibration(int calibrationType) {
	ledManager.on();

	maybeCalibrateGyro();
	maybeCalibrateAccel();
	maybeCalibrateMag();

	m_Logger.debug("Saving the calibration data");

	SlimeVR::Configuration::SensorConfig config;
	config.type = SlimeVR::Configuration::SensorConfigType::BMI160;
	config.data.bmi160 = m_Config;
	configuration.setSensor(sensorId, config);
	configuration.save();

	m_Logger.debug("Saved the calibration data");

	m_Logger.info("Calibration data gathered, exiting calibration mode in...");
	constexpr uint8_t POST_CALIBRATION_DELAY_SEC = 3;
	ledManager.on();
	for (uint8_t i = POST_CALIBRATION_DELAY_SEC; i > 0; i--) {
		m_Logger.info("%i...", i);
		delay(1000);
	}
}

void BMI160Sensor::maybeCalibrateGyro() {
#ifndef BMI160_CALIBRATION_GYRO_SECONDS
	static_assert(false, "BMI160_CALIBRATION_GYRO_SECONDS not set in defines");
#endif

#if BMI160_CALIBRATION_GYRO_SECONDS == 0
	m_Logger.debug("Skipping gyro calibration");
	return;
#endif

	// Wait for sensor to calm down before calibration
	constexpr uint8_t GYRO_CALIBRATION_DELAY_SEC = 3;
	constexpr float GYRO_CALIBRATION_DURATION_SEC = BMI160_CALIBRATION_GYRO_SECONDS;
	m_Logger.info(
		"Put down the device and wait for baseline gyro reading calibration (%.1f "
		"seconds)",
		GYRO_CALIBRATION_DURATION_SEC
	);
	ledManager.on();
	for (uint8_t i = GYRO_CALIBRATION_DELAY_SEC; i > 0; i--) {
		m_Logger.info("%i...", i);
		delay(1000);
	}
	ledManager.off();

	if (!getTemperature(&temperature)) {
		m_Logger.error("Error: can't read temperature");
	}
	m_Config.temperature = temperature;

#ifdef DEBUG_SENSOR
	m_Logger.trace("Calibration temperature: %f", temperature);
#endif

	if (!imu.getGyroDrdy()) {
		m_Logger.error("Fatal error: gyroscope drdy = 0 (dead?)");
		return;
	}

	ledManager.pattern(100, 100, 3);
	ledManager.on();
	m_Logger.info("Gyro calibration started...");

	constexpr uint16_t gyroCalibrationSamples
		= GYRO_CALIBRATION_DURATION_SEC / (BMI160_ODR_GYR_MICROS / 1e6);
	int32_t rawGxyz[3] = {0};
	for (int i = 0; i < gyroCalibrationSamples; i++) {
		imu.waitForGyroDrdy();

		int16_t gx, gy, gz;
		imu.getRotation(&gx, &gy, &gz);
		rawGxyz[0] += gx;
		rawGxyz[1] += gy;
		rawGxyz[2] += gz;
	}
	ledManager.off();
	m_Config.G_off[0] = ((double)rawGxyz[0]) / gyroCalibrationSamples;
	m_Config.G_off[1] = ((double)rawGxyz[1]) / gyroCalibrationSamples;
	m_Config.G_off[2] = ((double)rawGxyz[2]) / gyroCalibrationSamples;

#ifdef DEBUG_SENSOR
	m_Logger.trace(
		"Gyro calibration results: %f %f %f",
		UNPACK_VECTOR_ARRAY(m_Config.G_off)
	);
#endif
}

void BMI160Sensor::maybeCalibrateAccel() {
#ifndef BMI160_ACCEL_CALIBRATION_METHOD
	static_assert(false, "BMI160_ACCEL_CALIBRATION_METHOD not set in defines");
#endif

#if BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_SKIP
	m_Logger.debug("Skipping accelerometer calibration");
	return;
#endif

	MagnetoCalibration* magneto = new MagnetoCalibration();

// Blink calibrating led before user should rotate the sensor
#if BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_ROTATION
	m_Logger.info("After 3 seconds, Gently rotate the device while it's gathering data"
	);
#elif BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_6POINT
	m_Logger.info(
		"Put the device into 6 unique orientations (all sides), leave it still and do "
		"not hold/touch for 3 seconds each"
	);
#endif
	constexpr uint8_t ACCEL_CALIBRATION_DELAY_SEC = 3;
	ledManager.on();
	for (uint8_t i = ACCEL_CALIBRATION_DELAY_SEC; i > 0; i--) {
		m_Logger.info("%i...", i);
		delay(1000);
	}
	ledManager.off();

#if BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_ROTATION
	uint16_t accelCalibrationSamples = 200;
	ledManager.pattern(100, 100, 6);
	delay(100);
	ledManager.on();
	m_Logger.debug("Gathering accelerometer data...");
	for (int i = 0; i < accelCalibrationSamples; i++) {
		int16_t ax, ay, az;
		imu.getAcceleration(&ax, &ay, &az);
		magneto->sample(ax, ay, az);

		delay(100);
	}
	ledManager.off();
	m_Logger.debug("Calculating accelerometer calibration data...");
#elif BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_6POINT
	RestDetectionParams calibrationRestDetectionParams;
	calibrationRestDetectionParams.restMinTime = 3;
	calibrationRestDetectionParams.restThAcc = 0.25f;
	RestDetection calibrationRestDetection(
		calibrationRestDetectionParams,
		BMI160_ODR_GYR_MICROS * 1.0e-6,
		BMI160_ODR_ACC_MICROS * 1.0e-6
	);

	constexpr uint16_t expectedPositions = 6;
	constexpr uint16_t numSamplesPerPosition = 96;

	uint16_t numPositionsRecorded = 0;
	uint16_t numCurrentPositionSamples = 0;
	bool waitForMotion = true;

	float* accelCalibrationChunk = new float[numSamplesPerPosition * 3];
	ledManager.pattern(100, 100, 6);
	ledManager.on();
	m_Logger.info("Gathering accelerometer data...");
	m_Logger.info(
		"Waiting for position %i, you can leave the device as is...",
		numPositionsRecorded + 1
	);
	while (true) {
		int16_t ax, ay, az;
		imu.getAcceleration(&ax, &ay, &az);
		sensor_real_t scaled[3];
		scaled[0] = ax * BMI160_ASCALE;
		scaled[1] = ay * BMI160_ASCALE;
		scaled[2] = az * BMI160_ASCALE;

		calibrationRestDetection.updateAcc(BMI160_ODR_ACC_MICROS * 1.0e-6, scaled);

		if (waitForMotion) {
			if (!calibrationRestDetection.getRestDetected()) {
				waitForMotion = false;
			}
			delayMicroseconds(BMI160_ODR_ACC_MICROS);
			continue;
		}

		if (calibrationRestDetection.getRestDetected()) {
			const uint16_t i = numCurrentPositionSamples * 3;
			accelCalibrationChunk[i + 0] = ax;
			accelCalibrationChunk[i + 1] = ay;
			accelCalibrationChunk[i + 2] = az;
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

		delayMicroseconds(BMI160_ODR_ACC_MICROS);
	}
	ledManager.off();
	m_Logger.debug("Calculating accelerometer calibration data...");
	delete[] accelCalibrationChunk;
#endif

	float A_BAinv[4][3];
	magneto->current_calibration(A_BAinv);
	delete magneto;

	m_Logger.debug("Finished calculating accelerometer calibration");
	m_Logger.debug("Accelerometer calibration matrix:");
	m_Logger.debug("{");
	for (int i = 0; i < 3; i++) {
		m_Config.A_B[i] = A_BAinv[0][i];
		m_Config.A_Ainv[0][i] = A_BAinv[1][i];
		m_Config.A_Ainv[1][i] = A_BAinv[2][i];
		m_Config.A_Ainv[2][i] = A_BAinv[3][i];
		m_Logger.debug(
			"  %f, %f, %f, %f",
			A_BAinv[0][i],
			A_BAinv[1][i],
			A_BAinv[2][i],
			A_BAinv[3][i]
		);
	}
	m_Logger.debug("}");
}

void BMI160Sensor::maybeCalibrateMag() {
#if !USE_6_AXIS
#ifndef BMI160_CALIBRATION_MAG_SECONDS
	static_assert(false, "BMI160_CALIBRATION_MAG_SECONDS not set in defines");
#endif

#if BMI160_CALIBRATION_MAG_SECONDS == 0
	m_Logger.debug("Skipping magnetometer calibration");
	return;
#endif

	MagnetoCalibration* magneto = new MagnetoCalibration();

	constexpr uint8_t MAG_CALIBRATION_DELAY_SEC = 3;
	constexpr float MAG_CALIBRATION_DURATION_SEC = BMI160_CALIBRATION_MAG_SECONDS;
	m_Logger.info(
		"After 3 seconds, rotate the device in figure 8 pattern while it's gathering "
		"data (%.1f seconds)",
		MAG_CALIBRATION_DURATION_SEC
	);
	for (uint8_t i = MAG_CALIBRATION_DELAY_SEC; i > 0; i--) {
		m_Logger.info("%i...", i);
		delay(1000);
	}
	ledManager.pattern(100, 100, 9);
	delay(100);
	ledManager.on();
	m_Logger.debug("Gathering magnetometer data...");

	constexpr float SAMPLE_DELAY_MS = 100.0f;
	constexpr uint16_t magCalibrationSamples
		= MAG_CALIBRATION_DURATION_SEC / (SAMPLE_DELAY_MS / 1e3f);

	uint8_t magdata[6];
	for (int i = 0; i < magCalibrationSamples; i++) {
		ledManager.on();

		int16_t mx, my, mz;
		imu.getMagnetometerXYZBuffer(magdata);
		getMagnetometerXYZFromBuffer(magdata, &mx, &my, &mz);
		magneto->sample(mx, my, mz);

		ledManager.off();
		delay(SAMPLE_DELAY_MS);
	}
	ledManager.off();
	m_Logger.debug("Calculating magnetometer calibration data...");

	float M_BAinv[4][3];
	magneto->current_calibration(M_BAinv);
	delete magneto;

	m_Logger.debug("[INFO] Magnetometer calibration matrix:");
	m_Logger.debug("{");
	for (int i = 0; i < 3; i++) {
		m_Config.M_B[i] = M_BAinv[0][i];
		m_Config.M_Ainv[0][i] = M_BAinv[1][i];
		m_Config.M_Ainv[1][i] = M_BAinv[2][i];
		m_Config.M_Ainv[2][i] = M_BAinv[3][i];
		m_Logger.debug(
			"  %f, %f, %f, %f",
			M_BAinv[0][i],
			M_BAinv[1][i],
			M_BAinv[2][i],
			M_BAinv[3][i]
		);
	}
	m_Logger.debug("}");
#endif
}

void BMI160Sensor::remapGyroAccel(
	sensor_real_t* x,
	sensor_real_t* y,
	sensor_real_t* z
) {
	remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), x, y, z);
}

void BMI160Sensor::remapMagnetometer(
	sensor_real_t* x,
	sensor_real_t* y,
	sensor_real_t* z
) {
	remapAllAxis(AXIS_REMAP_GET_ALL_MAG(axisRemap), x, y, z);
}

void BMI160Sensor::getRemappedRotation(int16_t* x, int16_t* y, int16_t* z) {
	imu.getRotation(x, y, z);
	remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), x, y, z);
}
void BMI160Sensor::getRemappedAcceleration(int16_t* x, int16_t* y, int16_t* z) {
	imu.getAcceleration(x, y, z);
	remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), x, y, z);
}

void BMI160Sensor::getMagnetometerXYZFromBuffer(
	uint8_t* data,
	int16_t* x,
	int16_t* y,
	int16_t* z
) {
#if BMI160_MAG_TYPE == BMI160_MAG_TYPE_HMC
	// hmc5883l -> 0 msb 1 lsb
	// XZY order
	*x = ((int16_t)data[0] << 8) | data[1];
	*z = ((int16_t)data[2] << 8) | data[3];
	*y = ((int16_t)data[4] << 8) | data[5];
#elif BMI160_MAG_TYPE == BMI160_MAG_TYPE_QMC
	// qmc5883l -> 0 lsb 1 msb
	// XYZ order
	*x = ((int16_t)data[1] << 8) | data[0];
	*y = ((int16_t)data[3] << 8) | data[2];
	*z = ((int16_t)data[5] << 8) | data[4];
#endif
}
