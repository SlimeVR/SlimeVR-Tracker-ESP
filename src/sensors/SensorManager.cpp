/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 TheDevMinerTV

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

#include "SensorManager.h"

#include <map>
#include <type_traits>

#include "bmi160sensor.h"
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "icm20948sensor.h"
#include "mpu6050sensor.h"
#include "mpu9250sensor.h"
#include "sensor.h"
#include "sensorinterface/SPIImpl.h"
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
#define PRIMARY_IMU_ADDRESS_ONE std::nullopt
#endif

#ifndef SECONDARY_IMU_ADDRESS_TWO
#define SECONDARY_IMU_ADDRESS_TWO std::nullopt
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

namespace SlimeVR {
namespace Sensors {

template <typename RegInterface>
using SoftFusionLSM6DS3TRC
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DS3TRC, RegInterface, SFCALIBRATOR>;
template <typename RegInterface>
using SoftFusionICM42688
	= SoftFusionSensor<SoftFusion::Drivers::ICM42688, RegInterface, SFCALIBRATOR>;
template <typename RegInterface>
using SoftFusionBMI270
	= SoftFusionSensor<SoftFusion::Drivers::BMI270, RegInterface, SFCALIBRATOR>;
template <typename RegInterface>
using SoftFusionLSM6DSV
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DSV, RegInterface, SFCALIBRATOR>;
template <typename RegInterface>
using SoftFusionLSM6DSO
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DSO, RegInterface, SFCALIBRATOR>;
template <typename RegInterface>
using SoftFusionLSM6DSR
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DSR, RegInterface, SFCALIBRATOR>;
template <typename RegInterface>
using SoftFusionMPU6050
	= SoftFusionSensor<SoftFusion::Drivers::MPU6050, RegInterface, SFCALIBRATOR>;
template <typename RegInterface>
using SoftFusionICM45686
	= SoftFusionSensor<SoftFusion::Drivers::ICM45686, RegInterface, SFCALIBRATOR>;
template <typename RegInterface>
using SoftFusionICM45605
	= SoftFusionSensor<SoftFusion::Drivers::ICM45605, RegInterface, SFCALIBRATOR>;
template <typename RegInterface>
class SensorAuto {};

void SensorManager::setup() {
	std::map<int, DirectPinInterface*> directPinInterfaces;
	std::map<int, MCP23X17PinInterface*> mcpPinInterfaces;
	std::map<std::tuple<int, int>, I2CWireSensorInterface*> i2cWireInterfaces;
	std::map<std::tuple<int, int, int, int>, I2CPCASensorInterface*> pcaWireInterfaces;

	auto directPin = [&](int pin) {
		if (!directPinInterfaces.contains(pin)) {
			auto ptr = new DirectPinInterface(pin);
			directPinInterfaces[pin] = ptr;
		}
		return directPinInterfaces[pin];
	};

	auto mcpPin = [&](int pin) {
		if (!mcpPinInterfaces.contains(pin)) {
			auto ptr = new MCP23X17PinInterface(&m_MCP, pin);
			mcpPinInterfaces[pin] = ptr;
		}
		return mcpPinInterfaces[pin];
	};

	auto directWire = [&](int scl, int sda) {
		auto pair = std::make_tuple(scl, sda);
		if (!i2cWireInterfaces.contains(pair)) {
			auto ptr = new I2CWireSensorInterface(scl, sda);
			i2cWireInterfaces[pair] = ptr;
		}
		return i2cWireInterfaces[pair];
	};

	auto pcaWire = [&](int scl, int sda, int addr, int ch) {
		auto pair = std::make_tuple(scl, sda, addr, ch);
		if (!pcaWireInterfaces.contains(pair)) {
			auto ptr = new I2CPCASensorInterface(scl, sda, addr, ch);
			pcaWireInterfaces[pair] = ptr;
		}
		return pcaWireInterfaces[pair];
	};
	uint8_t sensorID = 0;
	uint8_t activeSensorCount = 0;
	if (m_MCP.begin_I2C()) {
		m_Logger.info("MCP initialized");
	}

#define NO_PIN nullptr
#define NO_WIRE new EmptySensorInterface
#define DIRECT_PIN(pin) directPin(pin)
#define DIRECT_WIRE(scl, sda) directWire(scl, sda)
#define MCP_PIN(pin) mcpPin(pin)
#define PCA_WIRE(scl, sda, addr, ch) pcaWire(scl, sda, addr, ch)
#define DIRECT_SPI(clockfreq, bitorder, datamode, CS_PIN) \
	SoftFusion::SPIImpl(SPI, SPISettings(clockfreq, bitorder, datamode), CS_PIN)

#define SENSOR_DESC_ENTRY(ImuType, ...)                                              \
	{                                                                                \
		std::unique_ptr<::Sensor> sensor;                                            \
		if constexpr (std::is_same<                                                  \
						  ImuType<SoftFusion::I2CImpl>,                              \
						  SensorAuto<SoftFusion::I2CImpl>>::value) {                 \
			auto sensorType                                                          \
				= findSensorType<SoftFusion::I2CImpl>(sensorID, __VA_ARGS__);        \
			if (sensorType == SensorTypeID::Unknown) {                               \
				m_Logger.error("Can't find sensor type for sensor %d", sensorID);    \
			} else {                                                                 \
				m_Logger.info(                                                       \
					"Sensor %d automatically detected with %s",                      \
					sensorID,                                                        \
					getIMUNameByType(sensorType)                                     \
				);                                                                   \
				sensor = buildSensorDynamically<SoftFusion::I2CImpl>(                \
					sensorType,                                                      \
					sensorID,                                                        \
					__VA_ARGS__                                                      \
				);                                                                   \
			}                                                                        \
		} else {                                                                     \
			sensor = buildSensor<ImuType<SoftFusion::I2CImpl>, SoftFusion::I2CImpl>( \
				sensorID,                                                            \
				__VA_ARGS__                                                          \
			);                                                                       \
		}                                                                            \
		if (sensor->isWorking()) {                                                   \
			m_Logger.info("Sensor %d configured", sensorID + 1);                     \
			activeSensorCount++;                                                     \
		}                                                                            \
		m_Sensors.push_back(std::move(sensor));                                      \
		sensorID++;                                                                  \
	}

#define SENSOR_DESC_ENTRY_SPI(ImuType, ...)                                          \
	{                                                                                \
		std::unique_ptr<::Sensor> sensor;                                            \
		if constexpr (std::is_same<                                                  \
						  ImuType<SoftFusion::SPIImpl>,                              \
						  SensorAuto<SoftFusion::SPIImpl>>::value) {                 \
			auto sensorType                                                          \
				= findSensorType<SoftFusion::SPIImpl>(sensorID, __VA_ARGS__);        \
			if (sensorType == SensorTypeID::Unknown) {                               \
				m_Logger.error("Can't find sensor type for sensor %d", sensorID);    \
			} else {                                                                 \
				m_Logger.info(                                                       \
					"Sensor %d automatically detected with %s",                      \
					sensorID,                                                        \
					getIMUNameByType(sensorType)                                     \
				);                                                                   \
				sensor = buildSensorDynamically<SoftFusion::SPIImpl>(                \
					sensorType,                                                      \
					sensorID,                                                        \
					__VA_ARGS__                                                      \
				);                                                                   \
			}                                                                        \
		} else {                                                                     \
			sensor = buildSensor<ImuType<SoftFusion::SPIImpl>, SoftFusion::SPIImpl>( \
				sensorID,                                                            \
				__VA_ARGS__                                                          \
			);                                                                       \
		}                                                                            \
		if (sensor->isWorking()) {                                                   \
			m_Logger.info("Sensor %d configured", sensorID + 1);                     \
			activeSensorCount++;                                                     \
		}                                                                            \
		m_Sensors.push_back(std::move(sensor));                                      \
		sensorID++;                                                                  \
	}

	// Apply descriptor list and expand to entries
	SENSOR_DESC_LIST;

#define SENSOR_INFO_ENTRY(ImuID, ...) \
	{ m_Sensors[SensorTypeID]->setSensorInfo(__VA_ARGS__); }
	SENSOR_INFO_LIST;

	m_Logger.info("%d sensor(s) configured", activeSensorCount);
	// Check and scan i2c if no sensors active
	if (activeSensorCount == 0) {
		m_Logger.error(
			"Can't find I2C device on provided addresses, scanning for all I2C devices "
			"in the background"
		);
		I2CSCAN::scani2cports();
	}
}

void SensorManager::postSetup() {
	for (auto& sensor : m_Sensors) {
		if (sensor->isWorking()) {
			if (sensor->m_hwInterface != nullptr) {
				sensor->m_hwInterface->swapIn();
			}
			sensor->postSetup();
		}
	}
}

void SensorManager::update() {
	// Gather IMU data
	bool allIMUGood = true;
	for (auto& sensor : m_Sensors) {
		if (sensor->isWorking()) {
			if (sensor->m_hwInterface != nullptr) {
				sensor->m_hwInterface->swapIn();
			}
			sensor->motionLoop();
		}
		if (sensor->getSensorState() == SensorStatus::SENSOR_ERROR) {
			allIMUGood = false;
		}
	}

	statusManager.setStatus(SlimeVR::Status::IMU_ERROR, !allIMUGood);

	if (!networkConnection.isConnected()) {
		return;
	}

#ifndef PACKET_BUNDLING
	static_assert(false, "PACKET_BUNDLING not set");
#endif
#if PACKET_BUNDLING == PACKET_BUNDLING_BUFFERED
	uint32_t now = micros();
	bool shouldSend = false;
	bool allSensorsReady = true;
	for (auto& sensor : m_Sensors) {
		if (!sensor->isWorking()) {
			continue;
		}
		if (sensor->hasNewDataToSend()) {
			shouldSend = true;
		}
		allSensorsReady &= sensor->hasNewDataToSend();
	}

	if (now - m_LastBundleSentAtMicros < PACKET_BUNDLING_BUFFER_SIZE_MICROS) {
		shouldSend &= allSensorsReady;
	}

	if (!shouldSend) {
		return;
	}

	m_LastBundleSentAtMicros = now;
#endif

#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
	networkConnection.beginBundle();
#endif

	for (auto& sensor : m_Sensors) {
		if (sensor->isWorking()) {
			sensor->sendData();
		}
	}

#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
	networkConnection.endBundle();
#endif
}

#define BUILD_SENSOR_ARGS \
	sensorID, imuInterface, rotation, sensorInterface, optional, intPin, extraParam

template <typename RegInterface>
std::unique_ptr<::Sensor> SensorManager::buildSensorDynamically(
	SensorTypeID type,
	uint8_t sensorID,
	std::optional<RegInterface> imuInterface,
	float rotation,
	SensorInterface* sensorInterface,
	bool optional,
	PinInterface* intPin,
	int extraParam
) {
	switch (type) {
		// case SensorTypeID::MPU9250:
		//	return buildSensor<MPU9250Sensor, RegInterface>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::BNO080:
		//	return buildSensor<BNO080Sensor, RegInterface>(BUILD_SENSOR_ARGS);
		case SensorTypeID::BNO085:
			return buildSensor<BNO085Sensor<RegInterface>, RegInterface>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::BNO055:
		//	return buildSensor<BNO055Sensor, RegInterface>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::MPU6050:
		//	return buildSensor<SoftFusionMPU6050<RegInterface>, RegInterface>(
		//		BUILD_SENSOR_ARGS
		//	);
		// case SensorTypeID::BNO086:
		//	return buildSensor<BNO086Sensor, RegInterface>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::BMI160:
		//	return buildSensor<BMI160Sensor, RegInterface>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::ICM20948:
		//	return buildSensor<ICM20948Sensor, RegInterface>(BUILD_SENSOR_ARGS);
		// case SensorTypeID::ICM42688:
		//	return buildSensor<SoftFusionICM42688<RegInterface>, RegInterface>(
		//		BUILD_SENSOR_ARGS
		//	);
		case SensorTypeID::BMI270:
			return buildSensor<SoftFusionBMI270<RegInterface>, RegInterface>(
				BUILD_SENSOR_ARGS
			);
		// case SensorTypeID::LSM6DS3TRC:
		//	return buildSensor<SoftFusionLSM6DS3TRC<RegInterface>, RegInterface>(
		//		BUILD_SENSOR_ARGS
		//	);
		case SensorTypeID::LSM6DSV:
			return buildSensor<SoftFusionLSM6DSV<RegInterface>, RegInterface>(
				BUILD_SENSOR_ARGS
			);
		// case SensorTypeID::LSM6DSO:
		//	return buildSensor<SoftFusionLSM6DSO<RegInterface>, RegInterface>(
		//		BUILD_SENSOR_ARGS
		//	);
		// case SensorTypeID::LSM6DSR:
		//	return buildSensor<SoftFusionLSM6DSR<RegInterface>, RegInterface>(
		//		BUILD_SENSOR_ARGS
		//	);
		case SensorTypeID::ICM45686:
			return buildSensor<SoftFusionICM45686<RegInterface>, RegInterface>(
				BUILD_SENSOR_ARGS
			);
		// case SensorTypeID::ICM45605:
		//	return buildSensor<SoftFusionICM45605<RegInterface>, RegInterface>(
		//		BUILD_SENSOR_ARGS
		//	);
		default:
			m_Logger.error(
				"Unable to create sensor with type %s (%d)",
				getIMUNameByType(type),
				type
			);
	}
	return std::make_unique<EmptySensor>(sensorID);
}

template <typename RegInterface>
SensorTypeID SensorManager::findSensorType(
	uint8_t sensorID,
	std::optional<RegInterface> imuInterface,
	float rotation,
	SensorInterface* sensorInterface,
	bool optional,
	PinInterface* intPin,
	int extraParam
) {
	sensorInterface->init();
	sensorInterface->swapIn();
	// if (SoftFusionLSM6DS3TRC<RegInterface>::checkPresent(sensorID, imuInterface))
	// { 	return SensorTypeID::LSM6DS3TRC;
	// }
	// if (SoftFusionICM42688<RegInterface>::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::ICM42688;
	//}
	if (SoftFusionBMI270<RegInterface>::checkPresent(sensorID, imuInterface)) {
		return SensorTypeID::BMI270;
	}
	if (SoftFusionLSM6DSV<RegInterface>::checkPresent(sensorID, imuInterface)) {
		return SensorTypeID::LSM6DSV;
	}
	// if (SoftFusionLSM6DSO<RegInterface>::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::LSM6DSO;
	// }
	// if (SoftFusionLSM6DSR<RegInterface>::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::LSM6DSR;
	// }
	// if (SoftFusionMPU6050<RegInterface>::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::MPU6050;
	// }
	if (SoftFusionICM45686<RegInterface>::checkPresent(sensorID, imuInterface)) {
		return SensorTypeID::ICM45686;
	}
	return BNO080Sensor::checkIfPresent(sensorID, sensorInterface, intPin);
	// if (SoftFusionICM45605<RegInterface>::checkPresent(sensorID, imuInterface)) {
	//	return SensorTypeID::ICM45605;
	// }

	return SensorTypeID::Unknown;
}
}  // namespace Sensors
}  // namespace SlimeVR
