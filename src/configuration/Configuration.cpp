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

#include "Configuration.h"

#include <LittleFS.h>

#include "../FSHelper.h"
#include "consts.h"
#include "utils.h"

#define DIR_CALIBRATIONS "/calibrations"
#define DIR_TEMPERATURE_CALIBRATIONS "/tempcalibrations"

namespace SlimeVR {
namespace Configuration {
void Configuration::setup() {
	if (m_Loaded) {
		return;
	}

	bool status = LittleFS.begin();
	if (!status) {
		this->m_Logger.warn("Could not mount LittleFS, formatting");

		status = LittleFS.format();
		if (!status) {
			this->m_Logger.warn("Could not format LittleFS, aborting");
			return;
		}

		status = LittleFS.begin();
		if (!status) {
			this->m_Logger.error("Could not mount LittleFS, aborting");
			return;
		}
	}

	if (LittleFS.exists("/config.bin")) {
		m_Logger.trace("Found configuration file");

		auto file = LittleFS.open("/config.bin", "r");

		file.read((uint8_t*)&m_Config.version, sizeof(int32_t));

		if (m_Config.version < CURRENT_CONFIGURATION_VERSION) {
			m_Logger.debug(
				"Configuration is outdated: v%d < v%d",
				m_Config.version,
				CURRENT_CONFIGURATION_VERSION
			);

			if (!runMigrations(m_Config.version)) {
				m_Logger.error(
					"Failed to migrate configuration from v%d to v%d",
					m_Config.version,
					CURRENT_CONFIGURATION_VERSION
				);
				return;
			}
		} else {
			m_Logger.info("Found up-to-date configuration v%d", m_Config.version);
		}

		file.seek(0);
		file.read((uint8_t*)&m_Config, sizeof(DeviceConfig));
		file.close();
	} else {
		m_Logger.info("No configuration file found, creating new one");
		m_Config.version = CURRENT_CONFIGURATION_VERSION;
		save();
	}

	loadSensors();

	m_Loaded = true;

	m_Logger.info("Loaded configuration");

#ifdef DEBUG_CONFIGURATION
	print();
#endif
}

void Configuration::save() {
	for (size_t i = 0; i < m_Sensors.size(); i++) {
		SensorConfig config = m_Sensors[i];
		if (config.type == SensorConfigType::NONE) {
			continue;
		}

		char path[17];
		sprintf(path, DIR_CALIBRATIONS "/%d", i);

		m_Logger.trace("Saving sensor config data for %d", i);

		File file = LittleFS.open(path, "w");
		file.write((uint8_t*)&config, sizeof(SensorConfig));
		file.close();
	}

	{
		File file = LittleFS.open("/config.bin", "w");
		file.write((uint8_t*)&m_Config, sizeof(DeviceConfig));
		file.close();
	}

	m_Logger.debug("Saved configuration");
}

void Configuration::reset() {
	LittleFS.format();

	m_Sensors.clear();
	m_Config.version = 1;
	save();

	m_Logger.debug("Reset configuration");
}

int32_t Configuration::getVersion() const { return m_Config.version; }

size_t Configuration::getSensorCount() const { return m_Sensors.size(); }

SensorConfig Configuration::getSensor(size_t sensorID) const {
	if (sensorID >= m_Sensors.size()) {
		return {};
	}

	return m_Sensors.at(sensorID);
}

void Configuration::setSensor(size_t sensorID, const SensorConfig& config) {
	size_t currentSensors = m_Sensors.size();

	if (sensorID >= currentSensors) {
		m_Sensors.resize(sensorID + 1);
	}

	m_Sensors[sensorID] = config;
}

void Configuration::loadSensors() {
	SlimeVR::Utils::forEachFile(DIR_CALIBRATIONS, [&](SlimeVR::Utils::File f) {
		SensorConfig sensorConfig;
		f.read((uint8_t*)&sensorConfig, sizeof(SensorConfig));

		uint8_t sensorId = strtoul(f.name(), nullptr, 10);
		m_Logger.debug(
			"Found sensor calibration for %s at index %d",
			calibrationConfigTypeToString(sensorConfig.type),
			sensorId
		);

		setSensor(sensorId, sensorConfig);
	});
}

bool Configuration::loadTemperatureCalibration(
	uint8_t sensorId,
	GyroTemperatureCalibrationConfig& config
) {
	if (!SlimeVR::Utils::ensureDirectory(DIR_TEMPERATURE_CALIBRATIONS)) {
		return false;
	}

	char path[32];
	sprintf(path, DIR_TEMPERATURE_CALIBRATIONS "/%d", sensorId);

	if (!LittleFS.exists(path)) {
		return false;
	}

	auto f = SlimeVR::Utils::openFile(path, "r");
	if (f.isDirectory()) {
		return false;
	}

	if (f.size() != sizeof(GyroTemperatureCalibrationConfig)) {
		m_Logger.debug(
			"Found incompatible sensor temperature calibration (size mismatch) "
			"sensorId:%d, skipping",
			sensorId
		);
		return false;
	}

	SensorConfigType storedConfigType;
	f.read((uint8_t*)&storedConfigType, sizeof(SensorConfigType));

	if (storedConfigType != config.type) {
		m_Logger.debug(
			"Found incompatible sensor temperature calibration (expected %s, "
			"found %s) sensorId:%d, skipping",
			calibrationConfigTypeToString(config.type),
			calibrationConfigTypeToString(storedConfigType),
			sensorId
		);
		return false;
	}

	f.seek(0);
	f.read((uint8_t*)&config, sizeof(GyroTemperatureCalibrationConfig));
	m_Logger.debug(
		"Found sensor temperature calibration for %s sensorId:%d",
		calibrationConfigTypeToString(config.type),
		sensorId
	);
	return true;
}

bool Configuration::saveTemperatureCalibration(
	uint8_t sensorId,
	const GyroTemperatureCalibrationConfig& config
) {
	if (config.type == SensorConfigType::NONE) {
		return false;
	}

	char path[32];
	sprintf(path, DIR_TEMPERATURE_CALIBRATIONS "/%d", sensorId);

	m_Logger.trace("Saving temperature calibration data for sensorId:%d", sensorId);

	File file = LittleFS.open(path, "w");
	file.write((uint8_t*)&config, sizeof(GyroTemperatureCalibrationConfig));
	file.close();

	m_Logger.debug("Saved temperature calibration data for sensorId:%i", sensorId);
	return true;
}

bool Configuration::runMigrations(int32_t version) { return true; }

void Configuration::print() {
	m_Logger.info("Configuration:");
	m_Logger.info("  Version: %d", m_Config.version);
	m_Logger.info("  %d Sensors:", m_Sensors.size());

	for (size_t i = 0; i < m_Sensors.size(); i++) {
		const SensorConfig& c = m_Sensors[i];
		m_Logger.info("    - [%3d] %s", i, calibrationConfigTypeToString(c.type));

		switch (c.type) {
			case SensorConfigType::NONE:
				break;

			case SensorConfigType::BMI160:
				m_Logger.info(
					"            A_B        : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.bmi160.A_B)
				);

				m_Logger.info("            A_Ainv     :");
				for (uint8_t i = 0; i < 3; i++) {
					m_Logger.info(
						"                         %f, %f, %f",
						UNPACK_VECTOR_ARRAY(c.data.bmi160.A_Ainv[i])
					);
				}

				m_Logger.info(
					"            G_off      : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.bmi160.G_off)
				);
				m_Logger.info("            Temperature: %f", c.data.bmi160.temperature);

				break;

			case SensorConfigType::SFUSION:
				m_Logger.info(
					"            A_B        : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.sfusion.A_B)
				);

				m_Logger.info("            A_Ainv     :");
				for (uint8_t i = 0; i < 3; i++) {
					m_Logger.info(
						"                         %f, %f, %f",
						UNPACK_VECTOR_ARRAY(c.data.sfusion.A_Ainv[i])
					);
				}

				m_Logger.info(
					"            G_off      : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.sfusion.G_off)
				);
				m_Logger.info(
					"            Temperature: %f",
					c.data.sfusion.temperature
				);
				break;

			case SensorConfigType::ICM20948:
				m_Logger.info(
					"            G: %d, %d, %d",
					UNPACK_VECTOR_ARRAY(c.data.icm20948.G)
				);
				m_Logger.info(
					"            A: %d, %d, %d",
					UNPACK_VECTOR_ARRAY(c.data.icm20948.A)
				);
				m_Logger.info(
					"            C: %d, %d, %d",
					UNPACK_VECTOR_ARRAY(c.data.icm20948.C)
				);

				break;

			case SensorConfigType::MPU9250:
				m_Logger.info(
					"            A_B   : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu9250.A_B)
				);

				m_Logger.info("            A_Ainv:");
				for (uint8_t i = 0; i < 3; i++) {
					m_Logger.info(
						"                    %f, %f, %f",
						UNPACK_VECTOR_ARRAY(c.data.mpu9250.A_Ainv[i])
					);
				}

				m_Logger.info(
					"            M_B   : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu9250.M_B)
				);

				m_Logger.info("            M_Ainv:");
				for (uint8_t i = 0; i < 3; i++) {
					m_Logger.info(
						"                    %f, %f, %f",
						UNPACK_VECTOR_ARRAY(c.data.mpu9250.M_Ainv[i])
					);
				}

				m_Logger.info(
					"            G_off  : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu9250.G_off)
				);

				break;

			case SensorConfigType::MPU6050:
				m_Logger.info(
					"            A_B  : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu6050.A_B)
				);
				m_Logger.info(
					"            G_off: %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu6050.G_off)
				);

				break;

			case SensorConfigType::BNO0XX:
				m_Logger.info("            magEnabled: %d", c.data.bno0XX.magEnabled);

				break;
		}
	}
}
}  // namespace Configuration
}  // namespace SlimeVR
