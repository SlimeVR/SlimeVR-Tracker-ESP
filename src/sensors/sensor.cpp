/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#include "sensor.h"

#include <i2cscan.h>

#include "GlobalVars.h"
#include "calibration.h"

SensorStatus Sensor::getSensorState() {
	return isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_ERROR;
}

void Sensor::setAcceleration(Vector3 a) {
	acceleration = a;
	newAcceleration = true;
}

void Sensor::setFusedRotation(Quat r) {
	fusedRotation = r * sensorOffset;
	bool changed = OPTIMIZE_UPDATES
					 ? !lastFusedRotationSent.equalsWithEpsilon(fusedRotation)
					 : true;
	if (ENABLE_INSPECTION || changed) {
		newFusedRotation = true;
		lastFusedRotationSent = fusedRotation;
	}
}

void Sensor::sendData() {
	if (newFusedRotation) {
		newFusedRotation = false;
		networkConnection.sendRotationData(
			sensorId,
			&fusedRotation,
			DATA_TYPE_NORMAL,
			calibrationAccuracy
		);

#ifdef DEBUG_SENSOR
		m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(fusedRotation));
#endif

#if SEND_ACCELERATION
		if (newAcceleration) {
			newAcceleration = false;
			networkConnection.sendSensorAcceleration(sensorId, acceleration);
		}
#endif
	}
}

void Sensor::printTemperatureCalibrationUnsupported() {
	m_Logger.error(
		"Temperature calibration not supported for IMU %s",
		getIMUNameByType(sensorType)
	);
}
void Sensor::printTemperatureCalibrationState() {
	printTemperatureCalibrationUnsupported();
};
void Sensor::printDebugTemperatureCalibrationState() {
	printTemperatureCalibrationUnsupported();
};
void Sensor::saveTemperatureCalibration() { printTemperatureCalibrationUnsupported(); };
void Sensor::resetTemperatureCalibrationState() {
	printTemperatureCalibrationUnsupported();
};

uint16_t Sensor::getSensorConfigData() {
	SlimeVR::Configuration::SensorConfig sensorConfig
		= configuration.getSensor(sensorId);
	return SlimeVR::Configuration::configDataToNumber(
		&sensorConfig,
		magStatus != MagnetometerStatus::MAG_NOT_SUPPORTED
	);
}

const char* getIMUNameByType(ImuID imuType) {
	switch (imuType) {
		case ImuID::MPU9250:
			return "MPU9250";
		case ImuID::MPU6500:
			return "MPU6500";
		case ImuID::BNO080:
			return "BNO080";
		case ImuID::BNO085:
			return "BNO085";
		case ImuID::BNO055:
			return "BNO055";
		case ImuID::MPU6050:
			return "MPU6050";
		case ImuID::BNO086:
			return "BNO086";
		case ImuID::BMI160:
			return "BMI160";
		case ImuID::ICM20948:
			return "ICM20948";
		case ImuID::ICM42688:
			return "ICM42688";
		case ImuID::BMI270:
			return "BMI270";
		case ImuID::LSM6DS3TRC:
			return "LSM6DS3TRC";
		case ImuID::LSM6DSV:
			return "LSM6DSV";
		case ImuID::LSM6DSO:
			return "LSM6DSO";
		case ImuID::LSM6DSR:
			return "LSM6DSR";
		case ImuID::ICM45686:
			return "ICM45686";
		case ImuID::ICM45605:
			return "ICM45605";
		case ImuID::Unknown:
		case ImuID::Empty:
			return "UNKNOWN";
	}
	return "Unknown";
}
