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

#include <PinInterface.h>

#include <cstdint>
#include <cstring>

#include "../../GlobalVars.h"
#include "../../sensorinterface/SensorInterface.h"
#include "../RestCalibrationDetector.h"
#include "../sensor.h"
#include "TempGradientCalculator.h"
#include "imuconsts.h"
#include "motionprocessing/types.h"
#include "sensors/SensorFusion.h"
#include "sensors/softfusion/magdriver.h"

namespace SlimeVR::Sensors {

template <typename SensorType, template <typename IMU> typename Calibrator>
class SoftFusionSensor : public Sensor {
	using Consts = IMUConsts<SensorType>;
	using RawSensorT = typename Consts::RawSensorT;

	using Calib = Calibrator<SensorType>;
	static constexpr auto UpsideDownCalibrationInit = Calib::HasUpsideDownCalibration;

	float lastReadTemperature = 0;
	uint32_t lastTempPollTime = micros();

	bool detected() const {
		const auto value
			= m_sensor.m_RegisterInterface.readReg(SensorType::Regs::WhoAmI::reg);
		if constexpr (requires { SensorType::Regs::WhoAmI::values.size(); }) {
			for (auto possible : SensorType::Regs::WhoAmI::values) {
				if (value == possible) {
					return true;
				}
			}
			// this assumes there are only 2 values in the array
			m_Logger.error(
				"Sensor not detected, expected reg 0x%02x = [0x%02x, 0x%02x] but got "
				"0x%02x",
				SensorType::Regs::WhoAmI::reg,
				SensorType::Regs::WhoAmI::values[0],
				SensorType::Regs::WhoAmI::values[1],
				value
			);
			return false;
		} else {
			if (value == SensorType::Regs::WhoAmI::value) {
				return true;
			}
			m_Logger.error(
				"Sensor not detected, expected reg 0x%02x = 0x%02x but got 0x%02x",
				SensorType::Regs::WhoAmI::reg,
				SensorType::Regs::WhoAmI::value,
				value
			);
			return false;
		}
	}

	void sendData() final {
		Sensor::sendData();
		sendTempIfNeeded();
	}

	void sendTempIfNeeded() {
		uint32_t now = micros();
		constexpr float maxSendRateHz = 2.0f;
		constexpr uint32_t sendInterval = 1.0f / maxSendRateHz * 1e6;
		uint32_t elapsed = now - m_lastTemperaturePacketSent;
		if (elapsed >= sendInterval) {
			m_lastTemperaturePacketSent = now - (elapsed - sendInterval);
			networkConnection.sendTemperature(sensorId, lastReadTemperature);
		}
	}

	TemperatureGradientCalculator tempGradientCalculator{[&](float gradient) {
		m_fusion.updateBiasForgettingTime(
			calibrator.getZROChange() / std::fabs(gradient)
		);
	}};

	void processAccelSample(const RawSensorT xyz[3], const sensor_real_t timeDelta) {
		sensor_real_t accelData[]
			= {static_cast<sensor_real_t>(xyz[0]),
			   static_cast<sensor_real_t>(xyz[1]),
			   static_cast<sensor_real_t>(xyz[2])};

		calibrator.scaleAccelSample(accelData);

		m_fusion.updateAcc(accelData, calibrator.getAccelTimestep());

		calibrator.provideAccelSample(xyz);
	}

	void processGyroSample(const RawSensorT xyz[3], const sensor_real_t timeDelta) {
		sensor_real_t gyroData[]
			= {static_cast<sensor_real_t>(xyz[0]),
			   static_cast<sensor_real_t>(xyz[1]),
			   static_cast<sensor_real_t>(xyz[2])};
		calibrator.scaleGyroSample(gyroData);
		m_fusion.updateGyro(gyroData, calibrator.getGyroTimestep());

		calibrator.provideGyroSample(xyz);
	}

	void
	processTempSample(const int16_t rawTemperature, const sensor_real_t timeDelta) {
		if constexpr (!Consts::DirectTempReadOnly) {
			const float scaledTemperature
				= SensorType::TemperatureBias
				+ static_cast<float>(rawTemperature)
					  * (1.0 / SensorType::TemperatureSensitivity);

			lastReadTemperature = scaledTemperature;
			if (toggles.getToggle(SensorToggles::TempGradientCalibrationEnabled)) {
				tempGradientCalculator.feedSample(
					lastReadTemperature,
					calibrator.getTempTimestep()
				);
			}

			calibrator.provideTempSample(lastReadTemperature);
		}
	}

public:
	static constexpr auto TypeID = SensorType::Type;
	static constexpr uint8_t Address = SensorType::Address;

	SoftFusionSensor(
		uint8_t id,
		RegisterInterface& registerInterface,
		float rotation,
		SlimeVR::SensorInterface* sensorInterface = nullptr,
		PinInterface* intPin = nullptr,
		uint8_t = 0
	)
		: Sensor(
			SensorType::Name,
			SensorType::Type,
			id,
			registerInterface,
			rotation,
			sensorInterface
		)
		, m_fusion(
			  SensorType::SensorVQFParams,
			  SensorType::GyrTs,
			  SensorType::AccTs,
			  SensorType::MagTs
		  )
		, m_sensor(registerInterface, m_Logger) {}
	~SoftFusionSensor() override = default;

	void checkSensorTimeout() {
		uint32_t now = millis();
		constexpr uint32_t sensorTimeoutMillis = 2e3;  // 2 seconds
		if (m_lastRotationUpdateMillis + sensorTimeoutMillis > now) {
			return;
		}

		working = false;
		m_status = SensorStatus::SENSOR_ERROR;
		m_Logger.error(
			"Sensor timeout I2C Address 0x%02x delaytime: %d ms",
			addr,
			now - m_lastRotationUpdateMillis
		);
		networkConnection.sendSensorError(
			this->sensorId,
			static_cast<uint8_t>(PacketErrorCode::WATCHDOG_TIMEOUT)
		);
	}

	void motionLoop() final {
		calibrator.tick();

		// read fifo updating fusion
		uint32_t now = micros();

		if constexpr (Consts::DirectTempReadOnly) {
			uint32_t tempElapsed = now - lastTempPollTime;
			if (tempElapsed >= Consts::DirectTempReadTs * 1e6) {
				lastTempPollTime
					= now
					- (tempElapsed
					   - static_cast<uint32_t>(Consts::DirectTempReadTs * 1e6));
				lastReadTemperature = m_sensor.getDirectTemp();

				calibrator.provideTempSample(lastReadTemperature);

				if (toggles.getToggle(SensorToggles::TempGradientCalibrationEnabled)) {
					tempGradientCalculator.feedSample(
						lastReadTemperature,
						Consts::DirectTempReadTs
					);
				}
			}
		}

		if (toggles.getToggle(SensorToggles::TempGradientCalibrationEnabled)) {
			tempGradientCalculator.tick();
		}

		constexpr uint32_t targetPollIntervalMicros = 6000;
		uint32_t elapsed = now - m_lastPollTime;
		if (elapsed >= targetPollIntervalMicros) {
			m_lastPollTime = now - (elapsed - targetPollIntervalMicros);
		}

		// send new fusion values when time is up
		now = micros();
		constexpr float maxSendRateHz = 100.0f;
		constexpr uint32_t sendInterval = 1.0f / maxSendRateHz * 1e6f;
		elapsed = now - m_lastRotationPacketSent;
		if (elapsed >= sendInterval) {
			m_sensor.bulkRead({
				[&](const auto sample[3], float AccTs) {
					processAccelSample(sample, AccTs);
				},
				[&](const auto sample[3], float GyrTs) {
					processGyroSample(sample, GyrTs);
				},
				[&](int16_t sample, float TempTs) {
					processTempSample(sample, TempTs);
				},
			});
			if (!m_fusion.isUpdated()) {
				checkSensorTimeout();
				return;
			}
			hadData = true;
			m_lastRotationUpdateMillis = millis();
			m_fusion.clearUpdated();

			m_lastRotationPacketSent = now - (elapsed - sendInterval);

			setFusedRotation(m_fusion.getQuaternionQuat());
			setAcceleration(m_fusion.getLinearAccVec());
			optimistic_yield(100);
		}

		if (calibrationDetector.update(m_fusion)) {
			markRestCalibrationComplete();
		}
	}

	void motionSetup() final {
		if (!detected()) {
			m_status = SensorStatus::SENSOR_ERROR;
			return;
		}

		SlimeVR::Configuration::SensorConfig sensorCalibration
			= configuration.getSensor(sensorId);

		toggles = configuration.getSensorToggles(sensorId);

		// If no compatible calibration data is found, the calibration data will just be
		// zero-ed out
		if (calibrator.calibrationMatches(sensorCalibration)) {
			calibrator.assignCalibration(sensorCalibration);
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

		calibrator.begin();

		bool initResult = false;

		if constexpr (Calib::HasMotionlessCalib) {
			typename SensorType::MotionlessCalibrationData calibData;
			std::memcpy(
				&calibData,
				calibrator.getMotionlessCalibrationData(),
				sizeof(calibData)
			);
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

		calibrator.checkStartupCalibration();

		if constexpr (Consts::SupportsMags) {
			magDriver.init(
				SoftFusion::MagInterface{
					.readByte
					= [&](uint8_t address) { return m_sensor.readAux(address); },
					.writeByte = [&](uint8_t address, uint8_t value) {},
					.setDeviceId
					= [&](uint8_t deviceId) { m_sensor.setAuxId(deviceId); },
					.startPolling
					= [&](uint8_t dataReg, SoftFusion::MagDataWidth dataWidth
					  ) { m_sensor.startAuxPolling(dataReg, dataWidth); },
					.stopPolling = [&]() { m_sensor.stopAuxPolling(); },
				},
				Consts::Supports9ByteMag
			);

			if (toggles.getToggle(SensorToggles::MagEnabled)) {
				magDriver.startPolling();
			}
		}

		toggles.onToggleChange([&](SensorToggles toggle, bool value) {
			if (toggle == SensorToggles::MagEnabled) {
				if (value) {
					magDriver.startPolling();
				} else {
					magDriver.stopPolling();
				}
			}
		});
	}

	void startCalibration(int calibrationType) final {
		calibrator.startCalibration(calibrationType);
	}

	[[nodiscard]] bool isFlagSupported(SensorToggles toggle) const final {
		return toggle == SensorToggles::CalibrationEnabled
			|| toggle == SensorToggles::TempGradientCalibrationEnabled;
	}

	SensorStatus getSensorState() final { return m_status; }

	SensorFusion m_fusion;
	SensorType m_sensor;
	Calib calibrator{m_fusion, m_sensor, sensorId, m_Logger, toggles};

	SensorStatus m_status = SensorStatus::SENSOR_OFFLINE;
	uint32_t m_lastPollTime = micros();
	uint32_t m_lastRotationUpdateMillis = 0;
	uint32_t m_lastRotationPacketSent = 0;
	uint32_t m_lastTemperaturePacketSent = 0;

	RestCalibrationDetector calibrationDetector;

	SoftFusion::MagDriver magDriver;

	static bool checkPresent(const RegisterInterface& imuInterface) {
		I2Cdev::readTimeout = 100;
		auto value = imuInterface.readReg(SensorType::Regs::WhoAmI::reg);
		I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;
		if constexpr (requires { SensorType::Regs::WhoAmI::values.size(); }) {
			for (auto possible : SensorType::Regs::WhoAmI::values) {
				if (value == possible) {
					return true;
				}
			}
			return false;
		} else {
			if (value == SensorType::Regs::WhoAmI::value) {
				return true;
			}
			return false;
		}
	}

	const char* getAttachedMagnetometer() const final {
		return magDriver.getAttachedMagName();
	}
};

}  // namespace SlimeVR::Sensors
