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
using SoftFusionBMI160 = SoftFusionSensor<SoftFusion::Drivers::BMI160, SFCALIBRATOR>;
class SensorAuto {};

struct SensorBuilder {
public:
	SensorManager* m_Manager;
	SensorBuilder(SensorManager* sensorManager);

	uint8_t buildAllSensors();

	std::unique_ptr<::Sensor> buildSensorDynamically(
		SensorTypeID type,
		uint8_t sensorID,
		RegisterInterface& imuInterface,
		float rotation,
		SensorInterface* sensorInterface,
		bool optional,
		PinInterface* intPin,
		int extraParam = 0
	);

	std::unique_ptr<::Sensor> buildSensorDynamically(
		SensorTypeID type,
		uint8_t sensorID,
		uint8_t imuInterface,
		float rotation,
		SensorInterface* sensorInterface,
		bool optional,
		PinInterface* intPin,
		int extraParam = 0
	);

	SensorTypeID findSensorType(
		uint8_t sensorID,
		uint8_t imuAddress,
		float rotation,
		SensorInterface* sensorInterface,
		bool optional,
		PinInterface* intPin,
		int extraParam = 0
	);

	SensorTypeID findSensorType(
		uint8_t sensorID,
		RegisterInterface& imuInterface,
		float rotation,
		SensorInterface* sensorInterface,
		bool optional,
		PinInterface* intPin,
		int extraParam = 0
	);

	template <typename ImuType>
	std::unique_ptr<::Sensor> buildSensor(
		uint8_t sensorID,
		RegisterInterface& imuInterface,
		float rotation,
		SensorInterface* sensorInterface,
		bool optional = false,
		PinInterface* intPin = nullptr,
		int extraParam = 0
	) {
		m_Manager->m_Logger.trace(
			"Building IMU with: id=%d,\n\
						address=%s, rotation=%f,\n\
						interface=%s, int=%s, extraParam=%d, optional=%d",
			sensorID,
			imuInterface.toString().c_str(),
			rotation,
			sensorInterface->toString().c_str(),
			intPin->toString().c_str(),
			extraParam,
			optional
		);

		// Now start detecting and building the IMU
		std::unique_ptr<::Sensor> sensor;

		// Init I2C bus for each sensor upon startup
		sensorInterface->init();
		sensorInterface->swapIn();

		if (!imuInterface.hasSensorOnBus()) {
			if (!optional) {
				m_Manager->m_Logger.error(
					"Mandatory sensor %d not found at address %s",
					sensorID + 1,
					imuInterface.toString().c_str()
				);
				return std::make_unique<ErroneousSensor>(sensorID, ImuType::TypeID);
			} else {
				m_Manager->m_Logger.debug(
					"Optional sensor %d not found at address %s",
					sensorID + 1,
					imuInterface.toString().c_str()
				);
				return std::make_unique<EmptySensor>(sensorID);
			}
		}

		m_Manager->m_Logger.trace(
			"Sensor %d found at address %s",
			sensorID + 1,
			imuInterface.toString().c_str()
		);

		sensor = std::make_unique<ImuType>(
			sensorID,
			imuInterface,
			rotation,
			sensorInterface,
			intPin,
			extraParam
		);

		sensor->motionSetup();
		return sensor;
	}

	template <typename ImuType>
	std::unique_ptr<::Sensor> buildSensor(
		uint8_t sensorID,
		uint8_t imuAddress,
		float rotation,
		SensorInterface* sensorInterface,
		bool optional = false,
		PinInterface* intPin = nullptr,
		int extraParam = 0
	) {
		uint8_t address = imuAddress > 0 ? imuAddress : ImuType::Address + sensorID;
		return buildSensor<ImuType>(
			sensorID,
			*(new I2CImpl(address)),
			rotation,
			sensorInterface,
			optional,
			intPin,
			extraParam
		);
	}
};
}  // namespace SlimeVR::Sensors
