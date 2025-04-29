/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Tailsy13, Gorbit99 & SlimeVR Contributors

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

#include <cstring>
#include <vector>

#include "GlobalVars.h"
#include "configuration/SensorConfig.h"
#include "logging/Logger.h"
#include "motionprocessing/RestDetection.h"
#include "motionprocessing/types.h"
#include "sensors/SensorFusionRestDetect.h"
#include "sensors/softfusion/CalibrationBase.h"

namespace SlimeVR::Sensor {

template <typename IMU, typename RawSensorT, typename RawVectorT>
class SoftfusionCalibrator : public CalibrationBase<IMU, RawSensorT, RawVectorT> {
public:
	static constexpr bool HasUpsideDownCalibration = true;

	using Base = CalibrationBase<IMU, RawSensorT, RawVectorT>;

	SoftfusionCalibrator(
		Sensors::SensorFusionRestDetect& fusion,
		IMU& sensor,
		uint8_t sensorId,
		SlimeVR::Logging::Logger& logger,
		float TempTs,
		float AScale,
		float GScale,
		SensorToggleState& toggles
	)
		: Base{fusion, sensor, sensorId, logger, TempTs, AScale, GScale, toggles} {
		calibration.T_Ts = TempTs;
	}

	void startCalibration(
		int calibrationType,
		const Base::EatSamplesFn& eatSamplesForSeconds,
		const Base::ReturnLastFn& eatSamplesReturnLast
	) final {
		if (calibrationType == 0) {
			// ALL
			calibrateSampleRate(eatSamplesForSeconds);
			if constexpr (Base::HasMotionlessCalib) {
				typename IMU::MotionlessCalibrationData calibData;
				sensor.motionlessCalibration(calibData);
				std::memcpy(calibration.MotionlessData, &calibData, sizeof(calibData));
			}
			// Gryoscope offset calibration can only happen after any motionless
			// gyroscope calibration, otherwise we are calculating the offset based
			// on an incorrect starting point
			calibrateGyroOffset(eatSamplesReturnLast);
			calibrateAccel(eatSamplesForSeconds);
		} else if (calibrationType == 1) {
			calibrateSampleRate(eatSamplesForSeconds);
		} else if (calibrationType == 2) {
			calibrateGyroOffset(eatSamplesReturnLast);
		} else if (calibrationType == 3) {
			calibrateAccel(eatSamplesForSeconds);
		} else if (calibrationType == 4) {
			if constexpr (Base::HasMotionlessCalib) {
				typename IMU::MotionlessCalibrationData calibData;
				sensor.motionlessCalibration(calibData);
				std::memcpy(calibration.MotionlessData, &calibData, sizeof(calibData));
			} else {
				logger.info("Sensor doesn't provide any custom motionless calibration");
			}
		}

		saveCalibration();
	}

	bool calibrationMatches(const Configuration::SensorConfig& sensorCalibration
	) final {
		return sensorCalibration.type
				== SlimeVR::Configuration::SensorConfigType::SFUSION
			&& (sensorCalibration.data.sfusion.ImuType == IMU::Type)
			&& (sensorCalibration.data.sfusion.MotionlessDataLen
				== Base::MotionlessCalibDataSize());
	}

	void assignCalibration(const Configuration::SensorConfig& sensorCalibration) final {
		calibration = sensorCalibration.data.sfusion;

		if (!toggles.getToggle(SensorToggles::CalibrationEnabled)) {
			for (size_t i = 0; i < 3; i++) {
				calibration.A_B[i] = 0;
				calibration.A_Ainv[i][0] = 0;
				calibration.A_Ainv[i][1] = 0;
				calibration.A_Ainv[i][2] = 0;
				calibration.M_B[i] = 0;
				calibration.M_Ainv[i][0] = 0;
				calibration.M_Ainv[i][1] = 0;
				calibration.M_Ainv[i][2] = 0;
				calibration.G_off[i] = 0;
				calibration.G_Sens[i] = 0;
			}
		}

		Base::recalcFusion();
	}

	void scaleAccelSample(sensor_real_t accelSample[3]) final {
		float tmp[3];
		for (uint8_t i = 0; i < 3; i++) {
			tmp[i] = (accelSample[i] - calibration.A_B[i]);
		}

		accelSample[0]
			= (calibration.A_Ainv[0][0] * tmp[0] + calibration.A_Ainv[0][1] * tmp[1]
			   + calibration.A_Ainv[0][2] * tmp[2])
			* AScale;
		accelSample[1]
			= (calibration.A_Ainv[1][0] * tmp[0] + calibration.A_Ainv[1][1] * tmp[1]
			   + calibration.A_Ainv[1][2] * tmp[2])
			* AScale;
		accelSample[2]
			= (calibration.A_Ainv[2][0] * tmp[0] + calibration.A_Ainv[2][1] * tmp[1]
			   + calibration.A_Ainv[2][2] * tmp[2])
			* AScale;
	}

	float getAccelTimestep() final { return calibration.A_Ts; }

	void scaleGyroSample(sensor_real_t gyroSample[3]) final {
		gyroSample[0] = static_cast<sensor_real_t>(
			GScale * (gyroSample[0] - calibration.G_off[0])
		);
		gyroSample[1] = static_cast<sensor_real_t>(
			GScale * (gyroSample[1] - calibration.G_off[1])
		);
		gyroSample[2] = static_cast<sensor_real_t>(
			GScale * (gyroSample[2] - calibration.G_off[2])
		);
	}

	float getGyroTimestep() final { return calibration.G_Ts; }

	float getTempTimestep() final { return calibration.T_Ts; }

	const uint8_t* getMotionlessCalibrationData() final {
		return calibration.MotionlessData;
	}

private:
	static constexpr auto GyroCalibDelaySeconds = 5;
	static constexpr auto GyroCalibSeconds = 5;

	static constexpr auto SampleRateCalibDelaySeconds = 1;
	static constexpr auto SampleRateCalibSeconds = 5;

	static constexpr auto AccelCalibDelaySeconds = 3;
	static constexpr auto AccelCalibRestSeconds = 3;

	void saveCalibration() {
		logger.debug("Saving the calibration data");
		SlimeVR::Configuration::SensorConfig calibration{};
		calibration.type = SlimeVR::Configuration::SensorConfigType::SFUSION;
		calibration.data.sfusion = this->calibration;
		configuration.setSensor(sensorId, calibration);
		configuration.save();
	}

	void calibrateGyroOffset(const Base::ReturnLastFn& eatSamplesReturnLast) {
		if (!toggles.getToggle(SensorToggles::CalibrationEnabled)) {
			return;
		}

		// Wait for sensor to calm down before calibration
		logger.info(
			"Put down the device and wait for baseline gyro reading calibration (%d "
			"seconds)",
			GyroCalibDelaySeconds
		);
		ledManager.on();
		auto lastSamples = eatSamplesReturnLast(GyroCalibDelaySeconds);
		ledManager.off();

		calibration.temperature = std::get<2>(lastSamples) / IMU::TemperatureSensitivity
								+ IMU::TemperatureBias;
		logger.trace("Calibration temperature: %f", calibration.temperature);

		ledManager.pattern(100, 100, 3);
		ledManager.on();
		logger.info("Gyro calibration started...");

		int32_t sumXYZ[3] = {0};
		const auto targetCalib = millis() + 1000 * GyroCalibSeconds;
		uint32_t sampleCount = 0;

		while (millis() < targetCalib) {
#ifdef ESP8266
			ESP.wdtFeed();
#endif
			sensor.bulkRead(
				[](const RawSensorT xyz[3], const sensor_real_t timeDelta) {},
				[&sumXYZ,
				 &sampleCount](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					sumXYZ[0] += xyz[0];
					sumXYZ[1] += xyz[1];
					sumXYZ[2] += xyz[2];
					++sampleCount;
				},
				[](const int16_t rawTemp, const sensor_real_t timeDelta) {}
			);
		}

		ledManager.off();
		calibration.G_off[0]
			= static_cast<float>(sumXYZ[0]) / static_cast<float>(sampleCount);
		calibration.G_off[1]
			= static_cast<float>(sumXYZ[1]) / static_cast<float>(sampleCount);
		calibration.G_off[2]
			= static_cast<float>(sumXYZ[2]) / static_cast<float>(sampleCount);

		logger.info(
			"Gyro offset after %d samples: %f %f %f",
			sampleCount,
			UNPACK_VECTOR_ARRAY(calibration.G_off)
		);
	}

	void calibrateAccel(const Base::EatSamplesFn& eatSamplesForSeconds) {
		if (!toggles.getToggle(SensorToggles::CalibrationEnabled)) {
			return;
		}

		auto magneto = std::make_unique<MagnetoCalibration>();
		logger.info(
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
			IMU::GyrTs,
			IMU::AccTs
		);

		constexpr uint16_t expectedPositions = 6;
		constexpr uint16_t numSamplesPerPosition = 96;

		uint16_t numPositionsRecorded = 0;
		uint16_t numCurrentPositionSamples = 0;
		bool waitForMotion = true;

		std::vector<float> accelCalibrationChunk;
		accelCalibrationChunk.resize(numSamplesPerPosition * 3);
		ledManager.pattern(100, 100, 6);
		ledManager.on();
		logger.info("Gathering accelerometer data...");
		logger.info(
			"Waiting for position %i, you can leave the device as is...",
			numPositionsRecorded + 1
		);
		bool samplesGathered = false;
		while (!samplesGathered) {
#ifdef ESP8266
			ESP.wdtFeed();
#endif
			sensor.bulkRead(
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

					calibrationRestDetection.updateAcc(IMU::AccTs, scaledData);
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
								logger.info(
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
				[](const RawSensorT xyz[3], const sensor_real_t timeDelta) {},
				[](const int16_t rawTemp, const sensor_real_t timeDelta) {}
			);
		}
		ledManager.off();
		logger.debug("Calculating accelerometer calibration data...");
		accelCalibrationChunk.resize(0);

		float A_BAinv[4][3];
		magneto->current_calibration(A_BAinv);

		logger.debug("Finished calculating accelerometer calibration");
		logger.debug("Accelerometer calibration matrix:");
		logger.debug("{");
		for (int i = 0; i < 3; i++) {
			calibration.A_B[i] = A_BAinv[0][i];
			calibration.A_Ainv[0][i] = A_BAinv[1][i];
			calibration.A_Ainv[1][i] = A_BAinv[2][i];
			calibration.A_Ainv[2][i] = A_BAinv[3][i];
			logger.debug(
				"  %f, %f, %f, %f",
				A_BAinv[0][i],
				A_BAinv[1][i],
				A_BAinv[2][i],
				A_BAinv[3][i]
			);
		}
		logger.debug("}");
	}

	void calibrateSampleRate(const Base::EatSamplesFn& eatSamplesForSeconds) {
		logger.debug(
			"Calibrating IMU sample rate in %d second(s)...",
			SampleRateCalibDelaySeconds
		);
		ledManager.on();
		eatSamplesForSeconds(SampleRateCalibDelaySeconds);

		uint32_t accelSamples = 0;
		uint32_t gyroSamples = 0;
		uint32_t tempSamples = 0;

		const auto calibTarget = millis() + 1000 * SampleRateCalibSeconds;
		logger.debug("Counting samples now...");
		uint32_t currentTime;
		while ((currentTime = millis()) < calibTarget) {
			sensor.bulkRead(
				[&](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					accelSamples++;
				},
				[&](const RawSensorT xyz[3], const sensor_real_t timeDelta) {
					gyroSamples++;
				},
				[&](const int16_t rawTemp, const sensor_real_t timeDelta) {
					tempSamples++;
				}
			);
			yield();
		}

		const auto millisFromStart = static_cast<float>(
			currentTime - (calibTarget - 1000 * SampleRateCalibSeconds)
		);
		logger.debug(
			"Collected %d gyro, %d acc samples during %d ms",
			gyroSamples,
			accelSamples,
			millisFromStart
		);
		calibration.A_Ts
			= millisFromStart / (static_cast<float>(accelSamples) * 1000.0f);
		calibration.G_Ts
			= millisFromStart / (static_cast<float>(gyroSamples) * 1000.0f);
		calibration.T_Ts
			= millisFromStart / (static_cast<float>(tempSamples) * 1000.0f);

		logger.debug(
			"Gyro frequency %fHz, accel frequency: %fHz, temperature frequency: %fHz",
			1.0 / calibration.G_Ts,
			1.0 / calibration.A_Ts,
			1.0 / calibration.T_Ts
		);
		ledManager.off();

		// fusion needs to be recalculated
		Base::recalcFusion();
	}

	SlimeVR::Configuration::SoftFusionSensorConfig calibration = {
		// let's create here transparent calibration that doesn't affect input data
		.ImuType = {IMU::Type},
		.MotionlessDataLen = {Base::MotionlessCalibDataSize()},
		.A_B = {0.0, 0.0, 0.0},
		.A_Ainv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
		.M_B = {0.0, 0.0, 0.0},
		.M_Ainv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
		.G_off = {0.0, 0.0, 0.0},
		.temperature = 0.0,
		.A_Ts = IMU::AccTs,
		.G_Ts = IMU::GyrTs,
		.M_Ts = IMU::MagTs,
		.G_Sens = {1.0, 1.0, 1.0},
		.MotionlessData = {},
		.T_Ts = 0,
	};

private:
	using Base::AScale;
	using Base::GScale;
	using Base::logger;
	using Base::sensor;
	using Base::sensorId;
	using Base::toggles;
};

}  // namespace SlimeVR::Sensor
