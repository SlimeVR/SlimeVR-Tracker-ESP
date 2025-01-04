/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Tailsy13 & SlimeVR Contributors

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

#pragma once

#include "../SensorFusionRestDetect.h"
#include "../sensor.h"
#include "GlobalVars.h"

namespace SlimeVR::Sensors {

template <template <typename I2CImpl> typename T, typename I2CImpl>
class SoftFusionSensor : public Sensor {
	using imu = T<I2CImpl>;

	static constexpr bool Uses32BitSensorData
		= requires(imu& i) { i.Uses32BitSensorData; };

	using RawSensorT =
		typename std::conditional<Uses32BitSensorData, int32_t, int16_t>::type;
	using RawVectorT = std::array<RawSensorT, 3>;

	static constexpr auto UpsideDownCalibrationInit = true;
	static constexpr auto GyroCalibDelaySeconds = 5;
	static constexpr auto GyroCalibSeconds = 5;
	static constexpr auto SampleRateCalibDelaySeconds = 1;
	static constexpr auto SampleRateCalibSeconds = 5;

	static constexpr auto AccelCalibDelaySeconds = 3;
	static constexpr auto AccelCalibRestSeconds = 3;

	static constexpr double GScale
		= ((32768. / imu::GyroSensitivity) / 32768.) * (PI / 180.0);
	static constexpr double AScale = CONST_EARTH_GRAVITY / imu::AccelSensitivity;

	static constexpr bool HasMotionlessCalib
		= requires(imu& i) { typename imu::MotionlessCalibrationData; };
	static constexpr size_t MotionlessCalibDataSize() {
		if constexpr (HasMotionlessCalib) {
			return sizeof(typename imu::MotionlessCalibrationData);
		} else {
			return 0;
		}
	}

	bool detected() const {
		const auto value = m_sensor.i2c.readReg(imu::Regs::WhoAmI::reg);
		if (imu::Regs::WhoAmI::value != value) {
			m_Logger.error(
				"Sensor not detected, expected reg 0x%02x = 0x%02x but got 0x%02x",
				imu::Regs::WhoAmI::reg,
				imu::Regs::WhoAmI::value,
				value
			);
			return false;
		}

		return true;
	}

	void sendTempIfNeeded() {
		uint32_t now = micros();
		constexpr float maxSendRateHz = 2.0f;
		constexpr uint32_t sendInterval = 1.0f / maxSendRateHz * 1e6;
		uint32_t elapsed = now - m_lastTemperaturePacketSent;
		if (elapsed >= sendInterval) {
			const float temperature = m_sensor.getDirectTemp();
			m_lastTemperaturePacketSent = now - (elapsed - sendInterval);
			networkConnection.sendTemperature(sensorId, temperature);
		}
	}

	void recalcFusion() {
		m_fusion = SensorFusionRestDetect(
			m_calibration.G_Ts,
			m_calibration.A_Ts,
			m_calibration.M_Ts
		);
	}

	void processAccelSample(const RawSensorT xyz[3], const sensor_real_t timeDelta) {
		sensor_real_t accelData[]
			= {static_cast<sensor_real_t>(xyz[0]),
			   static_cast<sensor_real_t>(xyz[1]),
			   static_cast<sensor_real_t>(xyz[2])};

		float tmp[3];
		for (uint8_t i = 0; i < 3; i++) {
			tmp[i] = (accelData[i] - m_calibration.A_B[i]);
		}

		accelData[0]
			= (m_calibration.A_Ainv[0][0] * tmp[0] + m_calibration.A_Ainv[0][1] * tmp[1]
			   + m_calibration.A_Ainv[0][2] * tmp[2])
			* AScale;
		accelData[1]
			= (m_calibration.A_Ainv[1][0] * tmp[0] + m_calibration.A_Ainv[1][1] * tmp[1]
			   + m_calibration.A_Ainv[1][2] * tmp[2])
			* AScale;
		accelData[2]
			= (m_calibration.A_Ainv[2][0] * tmp[0] + m_calibration.A_Ainv[2][1] * tmp[1]
			   + m_calibration.A_Ainv[2][2] * tmp[2])
			* AScale;

		m_fusion.updateAcc(accelData, m_calibration.A_Ts);
	}

	void processGyroSample(const RawSensorT xyz[3], const sensor_real_t timeDelta) {
		const sensor_real_t scaledData[] = {
			static_cast<sensor_real_t>(
				GScale * (static_cast<sensor_real_t>(xyz[0]) - m_calibration.G_off[0])
			),
			static_cast<sensor_real_t>(
				GScale * (static_cast<sensor_real_t>(xyz[1]) - m_calibration.G_off[1])
			),
			static_cast<sensor_real_t>(
				GScale * (static_cast<sensor_real_t>(xyz[2]) - m_calibration.G_off[2])
			)
		};
		m_fusion.updateGyro(scaledData, m_calibration.G_Ts);
	}

	void eatSamplesForSeconds(const uint32_t seconds) {
		const auto targetDelay = millis() + 1000 * seconds;
		auto lastSecondsRemaining = seconds;
		while (millis() < targetDelay) {
#ifdef ESP8266
			ESP.wdtFeed();
#endif
			auto currentSecondsRemaining = (targetDelay - millis()) / 1000;
			if (currentSecondsRemaining != lastSecondsRemaining) {
				m_Logger.info("%d...", currentSecondsRemaining + 1);
				lastSecondsRemaining = currentSecondsRemaining;
			}
			m_sensor.bulkRead(
				[](const RawSensorT xyz[3], const sensor_real_t timeDelta) {},
				[](const RawSensorT xyz[3], const sensor_real_t timeDelta) {}
			);
		}
	}

	std::pair<RawVectorT, RawVectorT> eatSamplesReturnLast(const uint32_t milliseconds
	) {
		RawVectorT accel = {0};
		RawVectorT gyro = {0};
		const auto targetDelay = millis() + milliseconds;
		while (millis() < targetDelay) {
			m_sensor.bulkRead(
				[&](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					accel[0] = xyz[0];
					accel[1] = xyz[1];
					accel[2] = xyz[2];
				},
				[&](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					gyro[0] = xyz[0];
					gyro[1] = xyz[1];
					gyro[2] = xyz[2];
				}
			);
			yield();
		}
		return std::make_pair(accel, gyro);
	}

public:
	static constexpr auto TypeID = imu::Type;
	static constexpr uint8_t Address = imu::Address;

	SoftFusionSensor(
		uint8_t id,
		uint8_t i2cAddress,
		float rotation,
		uint8_t sclPin,
		uint8_t sdaPin,
		uint8_t
	)
		: Sensor(imu::Name, imu::Type, id, i2cAddress, rotation, sclPin, sdaPin)
		, m_fusion(imu::GyrTs, imu::AccTs, imu::MagTs)
		, m_sensor(I2CImpl(i2cAddress), m_Logger) {}
	~SoftFusionSensor() {}

	void motionLoop() override final {
		sendTempIfNeeded();

		// read fifo updating fusion
		uint32_t now = micros();
		constexpr uint32_t targetPollIntervalMicros = 6000;
		uint32_t elapsed = now - m_lastPollTime;
		if (elapsed >= targetPollIntervalMicros) {
			m_lastPollTime = now - (elapsed - targetPollIntervalMicros);
			m_sensor.bulkRead(
				[&](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					processAccelSample(xyz, timeDelta);
				},
				[&](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					processGyroSample(xyz, timeDelta);
				}
			);
			optimistic_yield(100);
			if (!m_fusion.isUpdated()) {
				return;
			}
			hadData = true;
			m_fusion.clearUpdated();
		}

		// send new fusion values when time is up
		now = micros();
		constexpr float maxSendRateHz = 120.0f;
		constexpr uint32_t sendInterval = 1.0f / maxSendRateHz * 1e6;
		elapsed = now - m_lastRotationPacketSent;
		if (elapsed >= sendInterval) {
			m_lastRotationPacketSent = now - (elapsed - sendInterval);

			setFusedRotation(m_fusion.getQuaternionQuat());
			setAcceleration(m_fusion.getLinearAccVec());
			optimistic_yield(100);
		}
	}

	void motionSetup() override final {
		if (!detected()) {
			m_status = SensorStatus::SENSOR_ERROR;
			return;
		}

		SlimeVR::Configuration::SensorConfig sensorCalibration
			= configuration.getSensor(sensorId);

		// If no compatible calibration data is found, the calibration data will just be
		// zero-ed out
		if (sensorCalibration.type == SlimeVR::Configuration::SensorConfigType::SFUSION
			&& (sensorCalibration.data.sfusion.ImuType == imu::Type)
			&& (sensorCalibration.data.sfusion.MotionlessDataLen
				== MotionlessCalibDataSize())) {
			m_calibration = sensorCalibration.data.sfusion;
			recalcFusion();
		} else if (sensorCalibration.type == SlimeVR::Configuration::SensorConfigType::NONE) {
			m_Logger.warn(
				"No calibration data found for sensor %d, ignoring...",
				sensorId
			);
			m_Logger.info("Calibration is advised");
		} else {
			m_Logger.warn(
				"Incompatible calibration data found for sensor %d, ignoring...",
				sensorId
			);
			m_Logger.info("Please recalibrate");
		}

		bool initResult = false;

		if constexpr (HasMotionlessCalib) {
			typename imu::MotionlessCalibrationData calibData;
			std::memcpy(&calibData, m_calibration.MotionlessData, sizeof(calibData));
			initResult = m_sensor.initialize(calibData);
		} else {
			initResult = m_sensor.initialize();
		}

		if (!initResult) {
			m_Logger.error("Sensor failed to initialize!");
			m_status = SensorStatus::SENSOR_ERROR;
			return;
		}

		m_status = SensorStatus::SENSOR_OK;
		working = true;
		[[maybe_unused]] auto lastRawSample = eatSamplesReturnLast(1000);
		if constexpr (UpsideDownCalibrationInit) {
			auto gravity = static_cast<sensor_real_t>(
				AScale * static_cast<sensor_real_t>(lastRawSample.first[2])
			);
			m_Logger.info(
				"Gravity read: %.1f (need < -7.5 to start calibration)",
				gravity
			);
			if (gravity < -7.5f) {
				ledManager.on();
				m_Logger.info("Flip front in 5 seconds to start calibration");
				lastRawSample = eatSamplesReturnLast(5000);
				gravity = static_cast<sensor_real_t>(
					AScale * static_cast<sensor_real_t>(lastRawSample.first[2])
				);
				if (gravity > 7.5f) {
					m_Logger.debug("Starting calibration...");
					startCalibration(0);
				} else {
					m_Logger.info("Flip not detected. Skipping calibration.");
				}

				ledManager.off();
			}
		}
	}

	void startCalibration(int calibrationType) override final {
		if (calibrationType == 0) {
			// ALL
			calibrateSampleRate();
			if constexpr (HasMotionlessCalib) {
				typename imu::MotionlessCalibrationData calibData;
				m_sensor.motionlessCalibration(calibData);
				std::memcpy(
					m_calibration.MotionlessData,
					&calibData,
					sizeof(calibData)
				);
			}
			// Gryoscope offset calibration can only happen after any motionless
			// gyroscope calibration, otherwise we are calculating the offset based
			// on an incorrect starting point
			calibrateGyroOffset();
			calibrateAccel();
		} else if (calibrationType == 1) {
			calibrateSampleRate();
		} else if (calibrationType == 2) {
			calibrateGyroOffset();
		} else if (calibrationType == 3) {
			calibrateAccel();
		} else if (calibrationType == 4) {
			if constexpr (HasMotionlessCalib) {
				typename imu::MotionlessCalibrationData calibData;
				m_sensor.motionlessCalibration(calibData);
				std::memcpy(
					m_calibration.MotionlessData,
					&calibData,
					sizeof(calibData)
				);
			} else {
				m_Logger.info("Sensor doesn't provide any custom motionless calibration"
				);
			}
		}

		saveCalibration();
	}

	void saveCalibration() {
		m_Logger.debug("Saving the calibration data");
		SlimeVR::Configuration::SensorConfig calibration;
		calibration.type = SlimeVR::Configuration::SensorConfigType::SFUSION;
		calibration.data.sfusion = m_calibration;
		configuration.setSensor(sensorId, calibration);
		configuration.save();
	}

	void calibrateGyroOffset() {
		// Wait for sensor to calm down before calibration
		m_Logger.info(
			"Put down the device and wait for baseline gyro reading calibration (%d "
			"seconds)",
			GyroCalibDelaySeconds
		);
		ledManager.on();
		eatSamplesForSeconds(GyroCalibDelaySeconds);
		ledManager.off();

		m_calibration.temperature = m_sensor.getDirectTemp();
		m_Logger.trace("Calibration temperature: %f", m_calibration.temperature);

		ledManager.pattern(100, 100, 3);
		ledManager.on();
		m_Logger.info("Gyro calibration started...");

		int32_t sumXYZ[3] = {0};
		const auto targetCalib = millis() + 1000 * GyroCalibSeconds;
		uint32_t sampleCount = 0;

		while (millis() < targetCalib) {
#ifdef ESP8266
			ESP.wdtFeed();
#endif
			m_sensor.bulkRead(
				[](const RawSensorT xyz[3], const sensor_real_t timeDelta) {},
				[&sumXYZ,
				 &sampleCount](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					sumXYZ[0] += xyz[0];
					sumXYZ[1] += xyz[1];
					sumXYZ[2] += xyz[2];
					++sampleCount;
				}
			);
		}

		ledManager.off();
		m_calibration.G_off[0] = ((double)sumXYZ[0]) / sampleCount;
		m_calibration.G_off[1] = ((double)sumXYZ[1]) / sampleCount;
		m_calibration.G_off[2] = ((double)sumXYZ[2]) / sampleCount;

		m_Logger.info(
			"Gyro offset after %d samples: %f %f %f",
			sampleCount,
			UNPACK_VECTOR_ARRAY(m_calibration.G_off)
		);
	}

	void calibrateAccel() {
		auto magneto = std::make_unique<MagnetoCalibration>();
		m_Logger.info(
			"Put the device into 6 unique orientations (all sides), leave it still and "
			"do not hold/touch for %d seconds each",
			AccelCalibRestSeconds
		);
		ledManager.on();
		eatSamplesForSeconds(AccelCalibDelaySeconds);
		ledManager.off();

		RestDetectionParams calibrationRestDetectionParams;
		calibrationRestDetectionParams.restMinTime = AccelCalibRestSeconds;
		calibrationRestDetectionParams.restThAcc = 0.25f;

		RestDetection calibrationRestDetection(
			calibrationRestDetectionParams,
			imu::GyrTs,
			imu::AccTs
		);

		constexpr uint16_t expectedPositions = 6;
		constexpr uint16_t numSamplesPerPosition = 96;

		uint16_t numPositionsRecorded = 0;
		uint16_t numCurrentPositionSamples = 0;
		bool waitForMotion = true;

		auto accelCalibrationChunk
			= std::make_unique<float[]>(numSamplesPerPosition * 3);
		ledManager.pattern(100, 100, 6);
		ledManager.on();
		m_Logger.info("Gathering accelerometer data...");
		m_Logger.info(
			"Waiting for position %i, you can leave the device as is...",
			numPositionsRecorded + 1
		);
		bool samplesGathered = false;
		while (!samplesGathered) {
#ifdef ESP8266
			ESP.wdtFeed();
#endif
			m_sensor.bulkRead(
				[&](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					const sensor_real_t scaledData[]
						= {static_cast<sensor_real_t>(
							   AScale * static_cast<sensor_real_t>(xyz[0])
						   ),
						   static_cast<sensor_real_t>(
							   AScale * static_cast<sensor_real_t>(xyz[1])
						   ),
						   static_cast<sensor_real_t>(
							   AScale * static_cast<sensor_real_t>(xyz[2])
						   )};

					calibrationRestDetection.updateAcc(imu::AccTs, scaledData);
					if (waitForMotion) {
						if (!calibrationRestDetection.getRestDetected()) {
							waitForMotion = false;
						}
						return;
					}

					if (calibrationRestDetection.getRestDetected()) {
						const uint16_t i = numCurrentPositionSamples * 3;
						accelCalibrationChunk[i + 0] = xyz[0];
						accelCalibrationChunk[i + 1] = xyz[1];
						accelCalibrationChunk[i + 2] = xyz[2];
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
						samplesGathered = true;
					}
				},
				[](const RawSensorT xyz[3], const sensor_real_t timeDelta) {}
			);
		}
		ledManager.off();
		m_Logger.debug("Calculating accelerometer calibration data...");
		accelCalibrationChunk.reset();

		float A_BAinv[4][3];
		magneto->current_calibration(A_BAinv);

		m_Logger.debug("Finished calculating accelerometer calibration");
		m_Logger.debug("Accelerometer calibration matrix:");
		m_Logger.debug("{");
		for (int i = 0; i < 3; i++) {
			m_calibration.A_B[i] = A_BAinv[0][i];
			m_calibration.A_Ainv[0][i] = A_BAinv[1][i];
			m_calibration.A_Ainv[1][i] = A_BAinv[2][i];
			m_calibration.A_Ainv[2][i] = A_BAinv[3][i];
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

	void calibrateSampleRate() {
		m_Logger.debug(
			"Calibrating IMU sample rate in %d second(s)...",
			SampleRateCalibDelaySeconds
		);
		ledManager.on();
		eatSamplesForSeconds(SampleRateCalibDelaySeconds);

		uint32_t accelSamples = 0;
		uint32_t gyroSamples = 0;

		const auto calibTarget = millis() + 1000 * SampleRateCalibSeconds;
		m_Logger.debug("Counting samples now...");
		uint32_t currentTime;
		while ((currentTime = millis()) < calibTarget) {
			m_sensor.bulkRead(
				[&accelSamples](
					const RawSensorT xyz[3],
					const sensor_real_t timeDelta
				) { accelSamples++; },
				[&gyroSamples](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					gyroSamples++;
				}
			);
			yield();
		}

		const auto millisFromStart
			= currentTime - (calibTarget - 1000 * SampleRateCalibSeconds);
		m_Logger.debug(
			"Collected %d gyro, %d acc samples during %d ms",
			gyroSamples,
			accelSamples,
			millisFromStart
		);
		m_calibration.A_Ts = millisFromStart / (accelSamples * 1000.0);
		m_calibration.G_Ts = millisFromStart / (gyroSamples * 1000.0);

		m_Logger.debug(
			"Gyro frequency %fHz, accel frequency: %fHz",
			1.0 / m_calibration.G_Ts,
			1.0 / m_calibration.A_Ts
		);
		ledManager.off();

		// fusion needs to be recalculated
		recalcFusion();
	}

	SensorStatus getSensorState() override final { return m_status; }

	SensorFusionRestDetect m_fusion;
	T<I2CImpl> m_sensor;
	SlimeVR::Configuration::SoftFusionSensorConfig m_calibration
		= {// let's create here transparent calibration that doesn't affect input data
		   .ImuType = {imu::Type},
		   .MotionlessDataLen = {MotionlessCalibDataSize()},
		   .A_B = {0.0, 0.0, 0.0},
		   .A_Ainv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
		   .M_B = {0.0, 0.0, 0.0},
		   .M_Ainv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
		   .G_off = {0.0, 0.0, 0.0},
		   .temperature = 0.0,
		   .A_Ts = imu::AccTs,
		   .G_Ts = imu::GyrTs,
		   .M_Ts = imu::MagTs,
		   .G_Sens = {1.0, 1.0, 1.0},
		   .MotionlessData = {}
		};

	SensorStatus m_status = SensorStatus::SENSOR_OFFLINE;
	uint32_t m_lastPollTime = micros();
	uint32_t m_lastRotationPacketSent = 0;
	uint32_t m_lastTemperaturePacketSent = 0;
};

}  // namespace SlimeVR::Sensors
