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

#include <initializer_list>
#include <map>
#include <type_traits>

#include "../sensorinterface/ADS111xInterface.h"
#include "../sensorinterface/ADS111xPin.h"
#include "ADCResistanceSensor.h"
#include "PinInterface.h"
#include "bmi160sensor.h"
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "icm20948sensor.h"
#include "mpu6050sensor.h"
#include "mpu9250sensor.h"
#include "sensorinterface/SensorInterface.h"
#include "sensorinterface/SensorInterfaceManager.h"
#include "sensors/softfusion/SoftfusionCalibration.h"
#include "sensors/softfusion/runtimecalibration/RuntimeCalibration.h"
#include "softfusion/drivers/bmi270.h"
#include "softfusion/drivers/icm42688.h"
#include "softfusion/drivers/icm45605.h"
#include "softfusion/drivers/icm45686.h"
#include "softfusion/drivers/lsm6ds3trc.h"
#include "softfusion/drivers/lsm6dso.h"
#include "softfusion/drivers/lsm6dsr.h"
#include "softfusion/drivers/lsm6dsv.h"
#include "softfusion/drivers/mpu6050.h"
#include "softfusion/i2cimpl.h"
#include "softfusion/softfusionsensor.h"

#if ESP32
#include "driver/i2c.h"
#endif

#if USE_RUNTIME_CALIBRATION
#define SFCALIBRATOR SlimeVR::Sensors::RuntimeCalibration::RuntimeCalibrator
#else
#define SFCALIBRATOR SlimeVR::Sensor::SoftfusionCalibrator
#endif

namespace SlimeVR {
namespace Sensors {
using SoftFusionLSM6DS3TRC = SoftFusionSensor<
	SoftFusion::Drivers::LSM6DS3TRC,
	SoftFusion::I2CImpl,
	SFCALIBRATOR>;
using SoftFusionICM42688 = SoftFusionSensor<
	SoftFusion::Drivers::ICM42688,
	SoftFusion::I2CImpl,
	SFCALIBRATOR>;
using SoftFusionBMI270
	= SoftFusionSensor<SoftFusion::Drivers::BMI270, SoftFusion::I2CImpl, SFCALIBRATOR>;
using SoftFusionLSM6DSV
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DSV, SoftFusion::I2CImpl, SFCALIBRATOR>;
using SoftFusionLSM6DSO
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DSO, SoftFusion::I2CImpl, SFCALIBRATOR>;
using SoftFusionLSM6DSR
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DSR, SoftFusion::I2CImpl, SFCALIBRATOR>;
using SoftFusionMPU6050
	= SoftFusionSensor<SoftFusion::Drivers::MPU6050, SoftFusion::I2CImpl, SFCALIBRATOR>;
using SoftFusionICM45686 = SoftFusionSensor<
	SoftFusion::Drivers::ICM45686,
	SoftFusion::I2CImpl,
	SFCALIBRATOR>;
using SoftFusionICM45605 = SoftFusionSensor<
	SoftFusion::Drivers::ICM45605,
	SoftFusion::I2CImpl,
	SFCALIBRATOR>;

void SensorManager::setup() {
	SensorInterfaceManager interfaceManager;

	uint8_t sensorID = 0;
	uint8_t activeSensorCount = 0;
	if (m_MCP.begin_I2C()) {
		m_Logger.info("MCP initialized");
	}

	[[maybe_unused]] static constexpr auto* NO_PIN
		= static_cast<PinInterface*>(nullptr);
	[[maybe_unused]] static auto DIRECT_PIN = [&](uint8_t pin) constexpr {
		return interfaceManager.directPinInterface().get(pin);
	};
	[[maybe_unused]] static auto DIRECT_WIRE = [&](uint8_t scl, uint8_t sda) constexpr {
		return interfaceManager.i2cWireInterface().get(scl, sda);
	};
	[[maybe_unused]] static auto MCP_PIN = [&](uint8_t pin) constexpr {
		return interfaceManager.mcpPinInterface().get(&m_MCP, pin);
	};
	[[maybe_unused]] static auto PCA_WIRE
		= [&](uint8_t scl, uint8_t sda, uint8_t addr, uint8_t ch) constexpr {
			  return interfaceManager.pcaWireInterface().get(scl, sda, addr, ch);
		  };
	[[maybe_unused]] static auto ADS_PIN
		= [&](SensorInterface* interface, PinInterface* drdy, uint8_t addr, uint8_t ch
		  ) constexpr {
			  return interfaceManager.adsPinInterface().get(
				  interfaceManager.adsInterface().get(interface, drdy, addr),
				  ch
			  );
		  };
	[[maybe_unused]] static auto MUX_PIN
		= [&](PinInterface* data,
			  std::vector<PinInterface*>&& addressPins,
			  uint8_t channel,
			  PinInterface* enablePin = nullptr,
			  bool enableActiveLevel = false,
			  bool addressActiveLevel = true) constexpr {
			  return interfaceManager.parallelMuxPinInterface().get(
				  interfaceManager.parallelMuxInterface().get(
					  data,
					  addressPins,
					  enablePin,
					  enableActiveLevel,
					  addressActiveLevel
				  ),
				  channel
			  );
		  };

#define SENSOR_DESC_ENTRY(ImuType, ...)                            \
	{                                                              \
		auto sensor = buildSensor<ImuType>(sensorID, __VA_ARGS__); \
		if (sensor->isWorking()) {                                 \
			m_Logger.info("Sensor %d configured", sensorID + 1);   \
			activeSensorCount++;                                   \
		}                                                          \
		m_Sensors.push_back(std::move(sensor));                    \
		sensorID++;                                                \
	}

	// Apply descriptor list and expand to entries
	SENSOR_DESC_LIST;

#define SENSOR_INFO_ENTRY(SensorTypeID, ...) \
	{ m_Sensors[SensorTypeID]->setSensorInfo(__VA_ARGS__); }
	SENSOR_INFO_LIST;

#undef SENSOR_DESC_ENTRY
#undef NO_PIN
#undef DIRECT_PIN
#undef DIRECT_WIRE
#undef MCP_PIN
#undef PCA_WIRE
#undef ADS_PIN

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
}  // namespace Sensors
}  // namespace SlimeVR
