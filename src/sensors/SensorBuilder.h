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
#include "SensorManager.h"
#include "bmi160sensor.h"
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "globals.h"
#include "icm20948sensor.h"
#include "logging/Logger.h"
#include "mpu6050sensor.h"
#include "mpu9250sensor.h"
#include "sensor.h"
#include "sensorinterface/DirectPinInterface.h"
#include "sensorinterface/I2CPCAInterface.h"
#include "sensorinterface/I2CWireSensorInterface.h"
#include "sensorinterface/MCP23X17PinInterface.h"
#include "sensorinterface/RegisterInterface.h"
#include "sensorinterface/SPIImpl.h"
#include "sensorinterface/SensorInterfaceManager.h"
#include "sensorinterface/i2cimpl.h"
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
#define PRIMARY_IMU_ADDRESS_ONE 0
#endif

#ifndef SECONDARY_IMU_ADDRESS_TWO
#define SECONDARY_IMU_ADDRESS_TWO 0
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

	SensorTypeID findSensorType(SensorDefinition sensorDef);

	template <typename SensorType, typename AccessInterface>
	bool sensorDescEntry(
		uint8_t& sensorID,
		AccessInterface&& accessInterface,
		float rotation,
		SensorInterface* sensorInterface,
		bool optional = false,
		PinInterface* intPin = nullptr,
		int extraParam = 0
	) {
		RegisterInterface& regInterface = [&]() constexpr -> auto& {
			if constexpr (std::is_convertible_v<AccessInterface, RegisterInterface&>) {
				return accessInterface;
			} else {
				return *interfaceManager.i2cImpl().get(accessInterface);
			}
		}();

		std::unique_ptr<::Sensor> sensor;
		if constexpr (std::is_same<SensorType, SensorAuto>::value) {
			auto sensorType = findSensorType({
				sensorID,
				regInterface,
				rotation,
				sensorInterface,
				optional,
				intPin,
				extraParam,
			});

			if (sensorType == SensorTypeID::Unknown) {
				m_Manager->m_Logger.error(
					"Can't find sensor type for sensor %d",
					sensorID
				);
				return false;
			}

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
			sensorDef.imuInterface.toString(),
			sensorDef.rotation,
			sensorDef.sensorInterface,
			sensorDef.intPin,
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
					sensorDef.imuInterface.toString()
				);
				return std::make_unique<ErroneousSensor>(
					sensorDef.sensorID,
					ImuType::TypeID
				);
			} else {
				m_Manager->m_Logger.debug(
					"Optional sensor %d not found at address %s",
					sensorDef.sensorID + 1,
					sensorDef.imuInterface.toString()
				);
				return std::make_unique<EmptySensor>(sensorDef.sensorID);
			}
		}

		m_Manager->m_Logger.trace(
			"Sensor %d found at address %s",
			sensorDef.sensorID + 1,
			sensorDef.imuInterface.toString()
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
