/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Eiren Rain & SlimeVR Contributors

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

#include <map>
#include <memory>
#include <optional>
#include <type_traits>

#include "EmptySensor.h"
#include "ErroneousSensor.h"
#include "PinInterface.h"
#include "SensorManager.h"
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "consts.h"
#include "globals.h"
#include "icm20948sensor.h"
#include "logging/Logger.h"
#include "mpu6050sensor.h"
#include "mpu9250sensor.h"
#include "sensor.h"
#include "sensorinterface/DirectPinInterface.h"
#include "sensorinterface/DirectSPIInterface.h"
#include "sensorinterface/I2CPCAInterface.h"
#include "sensorinterface/I2CWireSensorInterface.h"
#include "sensorinterface/MCP23X17PinInterface.h"
#include "sensorinterface/RegisterInterface.h"
#include "sensorinterface/SPIImpl.h"
#include "sensorinterface/SensorInterface.h"
#include "sensorinterface/SensorInterfaceManager.h"
#include "sensorinterface/i2cimpl.h"
#include "softfusion/drivers/bmi160.h"
#include "softfusion/drivers/bmi270.h"
#include "softfusion/drivers/icm42688.h"
#include "softfusion/drivers/icm45605.h"
#include "softfusion/drivers/icm45686.h"
#include "softfusion/drivers/lsm6ds3trc.h"
#include "softfusion/drivers/lsm6dso.h"
#include "softfusion/drivers/lsm6dsr.h"
#include "softfusion/drivers/lsm6dsv.h"
#include "softfusion/drivers/mpu6050.h"
#include "softfusion/softfusionsensor.h"

#ifndef PRIMARY_IMU_ADDRESS_ONE
#define PRIMARY_IMU_ADDRESS_ONE false
#endif

#ifndef SECONDARY_IMU_ADDRESS_TWO
#define SECONDARY_IMU_ADDRESS_TWO true
#endif

#if USE_RUNTIME_CALIBRATION
#include "sensors/softfusion/runtimecalibration/RuntimeCalibration.h"
#define SFCALIBRATOR SlimeVR::Sensors::RuntimeCalibration::RuntimeCalibrator
#else
#include "sensors/softfusion/SoftfusionCalibration.h"
#define SFCALIBRATOR SlimeVR::Sensor::SoftfusionCalibrator
#endif

#if ESP32
#include "driver/i2c.h"
#endif

namespace SlimeVR::Sensors {
using SoftFusionLSM6DS3TRC
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DS3TRC, SFCALIBRATOR>;
using SoftFusionICM42688
	= SoftFusionSensor<SoftFusion::Drivers::ICM42688, SFCALIBRATOR>;
using SoftFusionBMI270 = SoftFusionSensor<SoftFusion::Drivers::BMI270, SFCALIBRATOR>;
using SoftFusionLSM6DSV = SoftFusionSensor<SoftFusion::Drivers::LSM6DSV, SFCALIBRATOR>;
using SoftFusionLSM6DSO = SoftFusionSensor<SoftFusion::Drivers::LSM6DSO, SFCALIBRATOR>;
using SoftFusionLSM6DSR = SoftFusionSensor<SoftFusion::Drivers::LSM6DSR, SFCALIBRATOR>;
using SoftFusionMPU6050 = SoftFusionSensor<SoftFusion::Drivers::MPU6050, SFCALIBRATOR>;
using SoftFusionICM45686
	= SoftFusionSensor<SoftFusion::Drivers::ICM45686, SFCALIBRATOR>;
using SoftFusionICM45605
	= SoftFusionSensor<SoftFusion::Drivers::ICM45605, SFCALIBRATOR>;
using SoftFusionBMI160 = SoftFusionSensor<SoftFusion::Drivers::BMI160, SFCALIBRATOR>;
class SensorAuto {};

struct SensorBuilder {
private:
	struct SensorDefinition {
		uint8_t sensorID;
		RegisterInterface& imuInterface;
		float rotation;
		SensorInterface* sensorInterface;
		bool optional;
		PinInterface* intPin;
		int extraParam;
	};

public:
	SensorManager* m_Manager;
	explicit SensorBuilder(SensorManager* sensorManager);

	uint8_t buildAllSensors();

	std::unique_ptr<::Sensor>
	buildSensorDynamically(SensorTypeID type, SensorDefinition sensorDef);

	template <typename Sensor, typename AccessInterface>
	std::optional<std::pair<SensorTypeID, RegisterInterface*>> checkSensorPresent(
		uint8_t sensorId,
		SensorInterface* sensorInterface,
		AccessInterface accessInterface
	) {
		auto* registerInterface
			= getRegisterInterface<Sensor>(sensorId, sensorInterface, accessInterface);

		if (!registerInterface->hasSensorOnBus()) {
			return {};
		}

		if constexpr (requires {
						  {
							  Sensor::checkPresent(*registerInterface)
						  } -> std::same_as<SensorTypeID>;
					  }) {
			auto type = Sensor::checkPresent(*registerInterface);

			if (type == SensorTypeID::Unknown) {
				return {};
			}

			return std::make_pair(type, registerInterface);
		} else {
			if (!Sensor::checkPresent(*registerInterface)) {
				return {};
			}
		}

		return std::make_pair(Sensor::TypeID, registerInterface);
	}

	template <typename AccessInterface>
	inline std::optional<std::pair<SensorTypeID, RegisterInterface*>>
	checkSensorsPresent(uint8_t, SensorInterface*, AccessInterface) {
		return std::nullopt;
	}

	template <typename AccessInterface, typename Sensor, typename... Rest>
	inline std::optional<std::pair<SensorTypeID, RegisterInterface*>>
	checkSensorsPresent(
		uint8_t sensorId,
		SensorInterface* sensorInterface,
		AccessInterface accessInterface
	) {
		auto result
			= checkSensorPresent<Sensor>(sensorId, sensorInterface, accessInterface);
		if (result) {
			return result;
		}

		return checkSensorsPresent<AccessInterface, Rest...>(
			sensorId,
			sensorInterface,
			accessInterface
		);
	}

	template <typename Sensor, typename AccessInterface>
	RegisterInterface* getRegisterInterface(
		uint8_t sensorId,
		SensorInterface* interface,
		AccessInterface access
	) {
		if constexpr (std::is_base_of_v<
						  PinInterface,
						  std::remove_pointer_t<AccessInterface>>) {
			return interfaceManager.spiImpl().get(
				static_cast<DirectSPIInterface*>(interface),
				access
			);
		} else if constexpr (std::is_same_v<AccessInterface, bool>) {
			uint8_t addressIncrement = access ? 1 : 0;
			return interfaceManager.i2cImpl().get(Sensor::Address + addressIncrement);
		} else if constexpr (std::is_integral_v<AccessInterface>) {
			return interfaceManager.i2cImpl().get(access);
		}

		return &EmptyRegisterInterface::instance;
	}

	template <typename AccessInterface>
	std::optional<std::pair<SensorTypeID, RegisterInterface*>> findSensorType(
		uint8_t sensorID,
		SensorInterface* sensorInterface,
		AccessInterface accessInterface
	) {
		sensorInterface->init();
		sensorInterface->swapIn();

		return checkSensorsPresent<
			AccessInterface,
			// SoftFusionLSM6DS3TRC,
			// SoftFusionICM42688,
			SoftFusionBMI270,
			SoftFusionLSM6DSV,
			SoftFusionLSM6DSO,
			SoftFusionLSM6DSR,
			// SoftFusionMPU6050,
			SoftFusionICM45686,
			// SoftFusionICM45605
			BNO085Sensor>(sensorID, sensorInterface, accessInterface);
	}

	template <typename SensorType, typename AccessInterface>
	bool sensorDescEntry(
		uint8_t sensorID,
		AccessInterface accessInterface,
		float rotation,
		SensorInterface* sensorInterface,
		bool optional = false,
		PinInterface* intPin = nullptr,
		int extraParam = 0
	) {
		std::unique_ptr<::Sensor> sensor;
		if constexpr (std::is_same<SensorType, SensorAuto>::value) {
			auto result = findSensorType(sensorID, sensorInterface, accessInterface);

			if (!result) {
				m_Manager->m_Logger.error(
					"Can't find sensor type for sensor %d",
					sensorID
				);
				return false;
			}

			auto sensorType = result->first;
			auto& regInterface = *(result->second);

			m_Manager->m_Logger.info(
				"Sensor %d automatically detected with %s",
				sensorID,
				getIMUNameByType(sensorType)
			);
			sensor = buildSensorDynamically(
				sensorType,
				{
					sensorID,
					regInterface,
					rotation,
					sensorInterface,
					optional,
					intPin,
					extraParam,
				}
			);
		} else {
			auto& regInterface = getRegisterInterface<SensorType>(
				sensorID,
				sensorInterface,
				accessInterface
			);

			sensor = buildSensor<SensorType>({
				sensorID,
				regInterface,
				rotation,
				sensorInterface,
				optional,
				intPin,
				extraParam,
			});
		}
		if (sensor->isWorking()) {
			m_Manager->m_Logger.info("Sensor %d configured", sensorID);
		}
		m_Manager->m_Sensors.push_back(std::move(sensor));

		return true;
	}

	template <typename ImuType>
	std::unique_ptr<::Sensor> buildSensor(SensorDefinition sensorDef) {
		m_Manager->m_Logger.trace(
			"Building IMU with: id=%d,\n\
						address=%s, rotation=%f,\n\
						interface=%s, int=%s, extraParam=%d, optional=%d",
			sensorDef.sensorID,
			sensorDef.imuInterface.toString().c_str(),
			sensorDef.rotation,
			sensorDef.sensorInterface->toString().c_str(),
			sensorDef.intPin->toString().c_str(),
			sensorDef.extraParam,
			sensorDef.optional
		);

		// Now start detecting and building the IMU
		std::unique_ptr<::Sensor> sensor;

		// Init I2C bus for each sensor upon startup
		sensorDef.sensorInterface->init();
		sensorDef.sensorInterface->swapIn();

		if (!sensorDef.imuInterface.hasSensorOnBus()) {
			if (!sensorDef.optional) {
				m_Manager->m_Logger.error(
					"Mandatory sensor %d not found at address %s",
					sensorDef.sensorID + 1,
					sensorDef.imuInterface.toString().c_str()
				);
				return std::make_unique<ErroneousSensor>(
					sensorDef.sensorID,
					ImuType::TypeID
				);
			} else {
				m_Manager->m_Logger.debug(
					"Optional sensor %d not found at address %s",
					sensorDef.sensorID + 1,
					sensorDef.imuInterface.toString().c_str()
				);
				return std::make_unique<EmptySensor>(sensorDef.sensorID);
			}
		}

		m_Manager->m_Logger.trace(
			"Sensor %d found at address %s",
			sensorDef.sensorID + 1,
			sensorDef.imuInterface.toString().c_str()
		);

		sensor = std::make_unique<ImuType>(
			sensorDef.sensorID,
			sensorDef.imuInterface,
			sensorDef.rotation,
			sensorDef.sensorInterface,
			sensorDef.intPin,
			sensorDef.extraParam
		);

		sensor->motionSetup();
		return sensor;
	}

private:
	SensorInterfaceManager interfaceManager;
};

}  // namespace SlimeVR::Sensors
