/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Eiren Rain, Gorbit99 & SlimeVR Contributors

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

#include "SensorBuilder.h"

namespace SlimeVR::Sensors {

SensorBuilder::SensorBuilder(SensorManager* sensorManager)
	: m_Manager(sensorManager) {}

#define SENSOR_DESC_ENTRY(ImuType, ...)                                           \
	activeSensorCount += sensorDescEntry<ImuType>(sensorID, __VA_ARGS__) ? 1 : 0; \
	sensorID++;

uint8_t SensorBuilder::buildAllSensors() {
	uint8_t sensorID = 0;
	uint8_t activeSensorCount = 0;

	[[maybe_unused]] const auto NO_PIN = nullptr;
	[[maybe_unused]] const auto NO_WIRE = &EmptySensorInterface::instance;
	[[maybe_unused]] const auto DIRECT_PIN
		= [&](uint8_t pin) { return interfaceManager.directPinInterface().get(pin); };
	[[maybe_unused]] const auto DIRECT_WIRE = [&](uint8_t scl, uint8_t sda) {
		return interfaceManager.i2cWireInterface().get(scl, sda);
	};
	[[maybe_unused]] const auto MCP_PIN = [&](uint8_t pin) {
		return interfaceManager.mcpPinInterface().get(&m_Manager->m_MCP, pin);
	};
	[[maybe_unused]] const auto PCA_WIRE
		= [&](uint8_t scl, uint8_t sda, uint8_t addr, uint8_t ch) {
			  return interfaceManager.pcaWireInterface().get(scl, sda, addr, ch);
		  };
	[[maybe_unused]] const auto DIRECT_SPI
		= [&](uint32_t clockFreq, uint8_t bitOrder, uint8_t dataMode) {
			  return interfaceManager.directSPIInterface().get(
				  SPI,
				  SPISettings(clockFreq, bitOrder, dataMode)
			  );
		  };

	// Apply descriptor list and expand to entries
	SENSOR_DESC_LIST;

	[[maybe_unused]] const auto SENSOR_INFO_ENTRY
		= [&](uint8_t ImuID, SensorPosition position) {
			  m_Manager->m_Sensors[ImuID]->setSensorInfo(position);
		  };

	// Apply sensor info list
	SENSOR_INFO_LIST;

	return activeSensorCount;
}

std::unique_ptr<::Sensor>
SensorBuilder::buildSensorDynamically(SensorTypeID type, SensorDefinition sensorDef) {
	switch (type) {
		// case SensorTypeID::MPU9250:
		//	return buildSensor<MPU9250Sensor>(sensorDef);
		// case SensorTypeID::BNO080:
		//	return buildSensor<BNO080Sensor>(sensorDef);
		case SensorTypeID::BNO085:
			return buildSensor<BNO085Sensor>(sensorDef);
		// case SensorTypeID::BNO055:
		//	return buildSensor<BNO055Sensor>(sensorDef);
		// case SensorTypeID::MPU6050:
		//	return buildSensor<SoftFusionMPU6050>(
		//		sensorDef
		//	);
		// case SensorTypeID::BNO086:
		//	return buildSensor<BNO086Sensor>(sensorDef);
		// case SensorTypeID::BMI160:
		// 	return buildSensor<BMI160Sensor>(sensorDef);
		// case SensorTypeID::ICM20948:
		//	return buildSensor<ICM20948Sensor>(sensorDef);
		// case SensorTypeID::ICM42688:
		//	return buildSensor<SoftFusionICM42688>(
		//		sensorDef
		//	);
		case SensorTypeID::BMI270:
			return buildSensor<SoftFusionBMI270>(sensorDef);
		// case SensorTypeID::LSM6DS3TRC:
		//	return buildSensor<SoftFusionLSM6DS3TRC>(
		//		sensorDef
		//	);
		case SensorTypeID::LSM6DSV:
			return buildSensor<SoftFusionLSM6DSV>(sensorDef);
		case SensorTypeID::LSM6DSO:
			return buildSensor<SoftFusionLSM6DSO>(sensorDef);
		case SensorTypeID::LSM6DSR:
			return buildSensor<SoftFusionLSM6DSR>(sensorDef);
		case SensorTypeID::ICM45686:
			return buildSensor<SoftFusionICM45686>(sensorDef);
		// case SensorTypeID::ICM45605:
		//	return buildSensor<SoftFusionICM45605>(
		//		sensorDef
		//	);
		default:
			m_Manager->m_Logger.error(
				"Unable to create sensor with type %s (%d)",
				getIMUNameByType(type),
				static_cast<int>(type)
			);
	}
	return std::make_unique<EmptySensor>(sensorDef.sensorID);
}

}  // namespace SlimeVR::Sensors
