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

#include "bmi160sensor.h"
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "icm20948sensor.h"
#include "mpu6050sensor.h"
#include "mpu9250sensor.h"
#include "softfusion/drivers/bmi270.h"
#include "softfusion/drivers/icm42688.h"
#include "softfusion/drivers/icm45605.h"
#include "softfusion/drivers/icm45686.h"
#include "softfusion/drivers/lsm6ds3trc.h"
#include "softfusion/drivers/lsm6dso.h"
#include "softfusion/drivers/lsm6dsr.h"
#include "softfusion/drivers/lsm6dsv.h"
#include "softfusion/drivers/mpu6050.h"
#include "softfusion/drivers/bmi323.h"
#include "softfusion/i2cimpl.h"
#include "softfusion/softfusionsensor.h"

#if ESP32
#include "driver/i2c.h"
#endif

namespace SlimeVR {
namespace Sensors {
using SoftFusionLSM6DS3TRC
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DS3TRC, SoftFusion::I2CImpl>;
using SoftFusionICM42688
	= SoftFusionSensor<SoftFusion::Drivers::ICM42688, SoftFusion::I2CImpl>;
using SoftFusionBMI270
	= SoftFusionSensor<SoftFusion::Drivers::BMI270, SoftFusion::I2CImpl>;
using SoftFusionLSM6DSV
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DSV, SoftFusion::I2CImpl>;
using SoftFusionLSM6DSO
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DSO, SoftFusion::I2CImpl>;
using SoftFusionLSM6DSR
	= SoftFusionSensor<SoftFusion::Drivers::LSM6DSR, SoftFusion::I2CImpl>;
using SoftFusionMPU6050
	= SoftFusionSensor<SoftFusion::Drivers::MPU6050, SoftFusion::I2CImpl>;
using SoftFusionICM45686
	= SoftFusionSensor<SoftFusion::Drivers::ICM45686, SoftFusion::I2CImpl>;
using SoftFusionICM45605
	= SoftFusionSensor<SoftFusion::Drivers::ICM45605, SoftFusion::I2CImpl>;
using SoftFusionBMI323
	= SoftFusionSensor<SoftFusion::Drivers::BMI323, SoftFusion::I2CImpl>;

// TODO Make it more generic in the future and move another place (abstract sensor
// interface)
void SensorManager::swapI2C(uint8_t sclPin, uint8_t sdaPin) {
	if (sclPin != activeSCL || sdaPin != activeSDA || !running) {
		Wire.flush();
#if ESP32
		if (running) {
		} else {
			// Reset HWI2C to avoid being affected by I2CBUS reset
			Wire.end();
		}
		// Disconnect pins from HWI2C
		gpio_set_direction((gpio_num_t)activeSCL, GPIO_MODE_INPUT);
		gpio_set_direction((gpio_num_t)activeSDA, GPIO_MODE_INPUT);

		if (running) {
			i2c_set_pin(I2C_NUM_0, sdaPin, sclPin, false, false, I2C_MODE_MASTER);
		} else {
			Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin), I2C_SPEED);
			Wire.setTimeOut(150);
		}
#else
		Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin));
#endif

		activeSCL = sclPin;
		activeSDA = sdaPin;
	}
}

void SensorManager::setup() {
	running = false;
	activeSCL = PIN_IMU_SCL;
	activeSDA = PIN_IMU_SDA;

	uint8_t sensorID = 0;
	uint8_t activeSensorCount = 0;
#define IMU_DESC_ENTRY(ImuType, ...)                               \
	{                                                              \
		auto sensor = buildSensor<ImuType>(sensorID, __VA_ARGS__); \
		if (sensor->isWorking()) {                                 \
			m_Logger.info("Sensor %d configured", sensorID + 1);   \
			activeSensorCount++;                                   \
		}                                                          \
		m_Sensors.push_back(std::move(sensor));                    \
		sensorID++;                                                \
	}
	// Apply descriptor list and expand to entrys
	IMU_DESC_LIST;

#undef IMU_DESC_ENTRY
	m_Logger.info("%d sensor(s) configured", activeSensorCount);
	// Check and scan i2c if no sensors active
	if (activeSensorCount == 0) {
		m_Logger.error(
			"Can't find I2C device on provided addresses, scanning for all I2C devices "
			"and returning"
		);
		I2CSCAN::scani2cports();
	}
}

void SensorManager::postSetup() {
	running = true;
	for (auto& sensor : m_Sensors) {
		if (sensor->isWorking()) {
			swapI2C(sensor->sclPin, sensor->sdaPin);
			sensor->postSetup();
		}
	}
}

void SensorManager::update() {
	// Gather IMU data
	bool allIMUGood = true;
	for (auto& sensor : m_Sensors) {
		if (sensor->isWorking()) {
			swapI2C(sensor->sclPin, sensor->sdaPin);
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
