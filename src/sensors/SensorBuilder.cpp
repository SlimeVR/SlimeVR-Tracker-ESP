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

uint8_t SensorBuilder::buildAllSensors() {
	SensorInterfaceManager interfaceManager;

	uint8_t sensorID = 0;
	uint8_t activeSensorCount = 0;

#define NO_PIN nullptr
#define NO_WIRE new EmptySensorInterface
#define DIRECT_PIN(pin) interfaceManager.directPinInterface().get(pin)
#define DIRECT_WIRE(scl, sda) interfaceManager.i2cWireInterface().get(scl, sda)
#define MCP_PIN(pin) interfaceManager.mcpPinInterface().get(&m_Manager->m_MCP, pin)
#define PCA_WIRE(scl, sda, addr, ch) \
	interfaceManager.pcaWireInterface().get(scl, sda, addr, ch)
#define DIRECT_SPI(clockfreq, bitorder, datamode)  \
	interfaceManager.directSPIInterface().get(     \
		SPI,                                       \
		SPISettings(clockfreq, bitorder, datamode) \
	)

#define SENSOR_DESC_ENTRY(                                                           \
	ImuType,                                                                         \
	access,                                                                          \
	rotation,                                                                        \
	interface,                                                                       \
	optional,                                                                        \
	intPin,                                                                          \
	extraParam                                                                       \
)                                                                                    \
	{                                                                                \
		do {                                                                         \
			std::unique_ptr<::Sensor> sensor;                                        \
			if constexpr (std::is_same<ImuType, SensorAuto>::value) {                \
				auto sensorType = findSensorType(sensorID, __VA_ARGS__);             \
				if (sensorType == SensorTypeID::Unknown) {                           \
					m_Manager->m_Logger.error(                                       \
						"Can't find sensor type for sensor %d",                      \
						sensorID                                                     \
					);                                                               \
					break;                                                           \
				} else {                                                             \
					m_Manager->m_Logger.info(                                        \
						"Sensor %d automatically detected with %s",                  \
						sensorID,                                                    \
						getIMUNameByType(sensorType)                                 \
					);                                                               \
					sensor                                                           \
						= buildSensorDynamically(sensorType, sensorID, __VA_ARGS__); \
				}                                                                    \
			} else {                                                                 \
				sensor = buildSensor<ImuType>(sensorID, __VA_ARGS__);                \
			}                                                                        \
			if (sensor->isWorking()) {                                               \
				m_Manager->m_Logger.info("Sensor %d configured", sensorID);          \
				activeSensorCount++;                                                 \
			}                                                                        \
			m_Manager->m_Sensors.push_back(std::move(sensor));                       \
		} while (false);                                                             \
		sensorID++;                                                                  \
	}

	// Apply descriptor list and expand to entries
	SENSOR_DESC_LIST;

#define SENSOR_INFO_ENTRY(ImuID, ...) \
	{ m_Manager->m_Sensors[ImuID]->setSensorInfo(__VA_ARGS__); }

	// Apply sensor info list
	SENSOR_INFO_LIST;

	return activeSensorCount;
}

#define BUILD_SENSOR_ARGS \
	sensorID, imuInterface, rotation, sensorInterface, optional, intPin, extraParam

std::unique_ptr<::Sensor> SensorBuilder::buildSensorDynamically(
	SensorTypeID type,
	uint8_t sensorID,
	RegisterInterface& imuInterface,
	float rotation,
	SensorInterface* sensorInterface,
	bool optional,
	PinInterface* intPin,
	int extraParam
) {
	switch (type) {
		// case SensorTypeID::MPU9250:
		//	return buildSensor<MPU9250Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::BNO080:
		//	return buildSensor<BNO080Sensor>(BUILD_SENSOR_ARGS);
		case SensorTypeID::BNO085:
			return buildSensor<BNO085Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::BNO055:
		//	return buildSensor<BNO055Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::MPU6050:
		//	return buildSensor<SoftFusionMPU6050>(
		//		BUILD_SENSOR_ARGS
		//	);
		// case SensorTypeID::BNO086:
		//	return buildSensor<BNO086Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::BMI160:
		//	return buildSensor<BMI160Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::ICM20948:
		//	return buildSensor<ICM20948Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::ICM42688:
		//	return buildSensor<SoftFusionICM42688>(
		//		BUILD_SENSOR_ARGS
		//	);
		case SensorTypeID::BMI270:
			return buildSensor<SoftFusionBMI270>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::LSM6DS3TRC:
		//	return buildSensor<SoftFusionLSM6DS3TRC>(
		//		BUILD_SENSOR_ARGS
		//	);
		case SensorTypeID::LSM6DSV:
			return buildSensor<SoftFusionLSM6DSV>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::LSM6DSO:
		//	return buildSensor<SoftFusionLSM6DSO>(
		//		BUILD_SENSOR_ARGS
		//	);
		// case SensorTypeID::LSM6DSR:
		//	return buildSensor<SoftFusionLSM6DSR>(
		//		BUILD_SENSOR_ARGS
		//	);
		case SensorTypeID::ICM45686:
			return buildSensor<SoftFusionICM45686>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::ICM45605:
		//	return buildSensor<SoftFusionICM45605>(
		//		BUILD_SENSOR_ARGS
		//	);
		default:
			m_Manager->m_Logger.error(
				"Unable to create sensor with type %s (%d)",
				getIMUNameByType(type),
				type
			);
	}
	return std::make_unique<EmptySensor>(sensorID);
}

std::unique_ptr<::Sensor> SensorBuilder::buildSensorDynamically(
	SensorTypeID type,
	uint8_t sensorID,
	uint8_t imuInterface,
	float rotation,
	SensorInterface* sensorInterface,
	bool optional,
	PinInterface* intPin,
	int extraParam
) {
	switch (type) {
		// case SensorTypeID::MPU9250:
		//	return buildSensor<MPU9250Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::BNO080:
		//	return buildSensor<BNO080Sensor>(BUILD_SENSOR_ARGS);
		case SensorTypeID::BNO085:
			return buildSensor<BNO085Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::BNO055:
		//	return buildSensor<BNO055Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::MPU6050:
		//	return buildSensor<SoftFusionMPU6050>(
		//		BUILD_SENSOR_ARGS
		//	);
		// case SensorTypeID::BNO086:
		//	return buildSensor<BNO086Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::BMI160:
		//	return buildSensor<BMI160Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::ICM20948:
		//	return buildSensor<ICM20948Sensor>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::ICM42688:
		//	return buildSensor<SoftFusionICM42688>(
		//		BUILD_SENSOR_ARGS
		//	);
		case SensorTypeID::BMI270:
			return buildSensor<SoftFusionBMI270>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::LSM6DS3TRC:
		//	return buildSensor<SoftFusionLSM6DS3TRC>(
		//		BUILD_SENSOR_ARGS
		//	);
		case SensorTypeID::LSM6DSV:
			return buildSensor<SoftFusionLSM6DSV>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::LSM6DSO:
		//	return buildSensor<SoftFusionLSM6DSO>(
		//		BUILD_SENSOR_ARGS
		//	);
		// case SensorTypeID::LSM6DSR:
		//	return buildSensor<SoftFusionLSM6DSR>(
		//		BUILD_SENSOR_ARGS
		//	);
		case SensorTypeID::ICM45686:
			return buildSensor<SoftFusionICM45686>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::ICM45605:
		//	return buildSensor<SoftFusionICM45605>(
		//		BUILD_SENSOR_ARGS
		//	);
		default:
			m_Manager->m_Logger.error(
				"Unable to create sensor with type %s (%d)",
				getIMUNameByType(type),
				type
			);
	}
	return std::make_unique<EmptySensor>(sensorID);
}

SensorTypeID SensorBuilder::findSensorType(
	uint8_t sensorID,
	uint8_t imuAddress,
	float rotation,
	SensorInterface* sensorInterface,
	bool optional,
	PinInterface* intPin,
	int extraParam
) {
	sensorInterface->init();
	sensorInterface->swapIn();
	// if (SoftFusionLSM6DS3TRC::checkPresent(sensorID, imuAddress))
	// { 	return SensorTypeID::LSM6DS3TRC;
	// }
	// if (SoftFusionICM42688::checkPresent(sensorID, imuAddress)) {
	//	return SensorTypeID::ICM42688;
	//}
	if (SoftFusionBMI270::checkPresent(sensorID, imuAddress)) {
		return SensorTypeID::BMI270;
	}
	if (SoftFusionLSM6DSV::checkPresent(sensorID, imuAddress)) {
		return SensorTypeID::LSM6DSV;
	}
	// if (SoftFusionLSM6DSO::checkPresent(sensorID, imuAddress)) {
	//	return SensorTypeID::LSM6DSO;
	// }
	// if (SoftFusionLSM6DSR::checkPresent(sensorID, imuAddress)) {
	//	return SensorTypeID::LSM6DSR;
	// }
	// if (SoftFusionMPU6050::checkPresent(sensorID, imuAddress)) {
	//	return SensorTypeID::MPU6050;
	// }
	if (SoftFusionICM45686::checkPresent(sensorID, imuAddress)) {
		return SensorTypeID::ICM45686;
	}
	return BNO080Sensor::checkPresent(sensorID, imuAddress, intPin);
	// if (SoftFusionICM45605::checkPresent(sensorID, imuAddress)) {
	//	return SensorTypeID::ICM45605;
	// }

	return SensorTypeID::Unknown;
}

SensorTypeID SensorBuilder::findSensorType(
	uint8_t sensorID,
	RegisterInterface& imuInterface,
	float rotation,
	SensorInterface* sensorInterface,
	bool optional,
	PinInterface* intPin,
	int extraParam
) {
	sensorInterface->init();
	sensorInterface->swapIn();
	// if (SoftFusionLSM6DS3TRC::checkPresent(sensorID, imuInterface))
	// { 	return SensorTypeID::LSM6DS3TRC;
	// }
	// if (SoftFusionICM42688::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::ICM42688;
	//}
	if (SoftFusionBMI270::checkPresent(sensorID, imuInterface)) {
		return SensorTypeID::BMI270;
	}
	if (SoftFusionLSM6DSV::checkPresent(sensorID, imuInterface)) {
		return SensorTypeID::LSM6DSV;
	}
	// if (SoftFusionLSM6DSO::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::LSM6DSO;
	// }
	// if (SoftFusionLSM6DSR::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::LSM6DSR;
	// }
	// if (SoftFusionMPU6050::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::MPU6050;
	// }
	if (SoftFusionICM45686::checkPresent(sensorID, imuInterface)) {
		return SensorTypeID::ICM45686;
	}
	return BNO080Sensor::checkPresent(sensorID, sensorInterface, intPin);
	// if (SoftFusionICM45605::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::ICM45605;
	// }

	// return SensorTypeID::Unknown;
}

}  // namespace SlimeVR::Sensors
