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

#include <cstdint>
#include <cstring>

#include "../FSHelper.h"
#include "GlobalVars.h"
#include "consts.h"
#include "sensors/SensorToggles.h"
#include "utils.h"

#ifdef ESP32
#include "nvs_flash.h"
#endif

#define DIR_CALIBRATIONS "/calibrations"
#define DIR_TEMPERATURE_CALIBRATIONS "/tempcalibrations"
#define DIR_TOGGLES_OLD "/toggles"
#define DIR_TOGGLES "/sensortoggles"

extern bool initFullreset;

namespace SlimeVR::Configuration {
void Configuration::setup() {
	if (m_Loaded) {
		return;
	}

	if (initFullreset) {
		this->m_Logger.info(
			PSTR("Request for Factory reset from Recovery Mode received.")
		);
		if (LittleFS.begin()) {
			this->factoryReset();
		} else {
			this->m_Logger.error(PSTR("Could not mount LittleFS try to format it"));
			LittleFS.format();
			this->factoryReset();
		}
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
		m_ConfigChanged = true;
		saveNeeded = true;
	}

	loadSensors();

	m_Loaded = true;

	m_Logger.info("Loaded configuration");

	cleanupMigration();

#ifdef DEBUG_CONFIGURATION
	print();
#endif
}

void Configuration::factoryReset() {
	this->reset();
	this->wifiReset();
	this->m_Logger.info("Rebooting...");
	delay(3000);
	ESP.restart();
}

void Configuration::wifiReset() {
	WiFi.disconnect(true);  // Clear WiFi credentials
#if ESP8266
	ESP.eraseConfig();  // Clear ESP config
#elif defined(ESP32)
	nvs_flash_erase();
#else
#warning SERIAL COMMAND FACTORY RESET NOT SUPPORTED
	this->m_Logger.info(PSTR("FACTORY RESET NOT SUPPORTED"));
	return;
#endif
#if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
#warning FACTORY RESET does not clear your hardcoded WiFi credentials!
	this->m_Logger.warn(
		PSTR("FACTORY RESET does not clear your hardcoded WiFi credentials!")
	);
#endif
}

// Constant called function
// Should do the following:
//  - Check if a config is flagged to be saved.
//  - after processing that one flag, check if there is a other config to save.
//  - make a minimal delay in between saves of files, to limit the impact of multi
//    config file saves
void Configuration::tick() {
	if (saveNeeded && ((lastsave + delaysave) < millis())) {
		bool saved = false;

		// Handle SensorConfig and saveNeeded
		for (size_t i = 0; i < m_SensorsChanged.size(); i++) {
			if (m_SensorsChanged[i] && !saved) {
				if (saveSensorConfig(i)) {
					m_SensorsChanged[i] = false;
					saved = true;
				}
			}
			if (m_SensorsChanged[i]) {
				saveNeeded = true;
			}
		}

		// Handle SensorToggle and saveNeeded
		for (size_t i = 0; i < m_SensorTogglesChanged.size(); i++) {
			if (m_SensorTogglesChanged[i] && !saved) {
				if (saveSensorToggle(i)) {
					m_SensorTogglesChanged[i] = false;
					saved = true;
				}
			}
			if (m_SensorTogglesChanged[i]) {
				saveNeeded = true;
			}
		}

		// Handle sensor config
		if (m_ConfigChanged && !saved) {
			File file = LittleFS.open("/config.bin", "w");
			file.write((uint8_t*)&m_Config, sizeof(DeviceConfig));
			file.close();
			m_ConfigChanged = false;
		}
		if (m_ConfigChanged) {
			saveNeeded = true;
		}
	}
}

bool Configuration::saveSensorConfig(size_t sensorId) {
	SensorConfig config = m_Sensors[sensorId];
	if (config.type == SensorConfigType::NONE) {
		return true;
	}

	char path[17];
	sprintf(path, DIR_CALIBRATIONS "/%zu", sensorId);

	m_Logger.trace("Saving sensor config data for %zu", sensorId);

	File file = LittleFS.open(path, "w");
	bool ret = true;
	if (file.write((uint8_t*)&config, sizeof(config)) < sizeof(config)) {
		m_Logger
			.error("Failed to save sensor config: %s SensorID: %zu", path, sensorId);
		ret = false;
	}
	file.close();
	return ret;
}

bool Configuration::saveSensorToggle(size_t sensorId) {
	char path[17];
	if (sensorId < m_SensorToggles.size()) {
		sprintf(path, DIR_TOGGLES "/%zu", sensorId);

		m_Logger.trace("Saving sensor toggle state for %d", sensorId);

		File file = LittleFS.open(path, "w");
		auto toggleValues = m_SensorToggles[sensorId].getValues();
		bool ret = true;
		if (file.write((uint8_t*)&toggleValues, sizeof(toggleValues))
			< sizeof(toggleValues)) {
			m_Logger.error(
				"Failed to save sensor toggle state: %s SensorID: %zu",
				path,
				sensorId
			);
			ret = false;
		};
		file.close();
		return ret;
	} else {
		m_Logger.trace(
			"Skipping saving toggles for sensor %d, no toggles present",
			sensorId
		);
		return true;
	}
}

void Configuration::cleanupMigration() {
	// Clean up old toggles directory
	if (LittleFS.exists(DIR_TOGGLES_OLD)) {
		char path[17] = DIR_TOGGLES_OLD;
		char* end = path + strlen(DIR_TOGGLES_OLD);
		Utils::forEachFile(DIR_TOGGLES_OLD, [&](SlimeVR::Utils::File file) {
			sprintf(end, "/%s", file.name());
			LittleFS.remove(path);
			file.close();
		});
		LittleFS.rmdir(DIR_TOGGLES_OLD);
		m_Logger.debug("Cleanup Migration done");
	}
}

void Configuration::reset() {
	LittleFS.format();

	m_Sensors.clear();
	m_SensorsChanged.clear();
	m_SensorToggles.clear();
	m_SensorTogglesChanged.clear();
	m_Config.version = CURRENT_CONFIGURATION_VERSION;
	m_ConfigChanged = true;
	// Todo:
	// - we don't want to save the old tracker configuration. As it would defeat the
	// purpose of the reset. Also not clear if we really need to save the basic
	// configuration, until a command changes it again. But this also means, after
	// a reset we need to restart a tracker to activate the new configuration on the
	// sensor. The sensorconfig is not cleared in the running class.
	saveNeeded = true;

	m_Logger.debug("Reset configuration");
}

int32_t Configuration::getVersion() const { return m_Config.version; }

size_t Configuration::getSensorCount() const { return m_Sensors.size(); }

SensorConfig Configuration::getSensor(size_t sensorId) const {
	if (sensorId >= m_Sensors.size()) {
		return {};
	}

	return m_Sensors.at(sensorId);
}

void Configuration::setSensor(
	size_t sensorId,
	const SensorConfig& config,
	bool nosave
) {
	size_t currentSensors = m_Sensors.size();
	m_Logger.trace("setSensor ID %d", sensorId);
	if (sensorId >= currentSensors) {
		m_Sensors.resize(sensorId + 1);
	}
	if ((m_Sensors[sensorId] != config) && !nosave) {
		m_Logger.trace("setSensor ID %d, saving", sensorId);
		m_SensorsChanged.resize(sensorId + 1);
		m_SensorsChanged[sensorId] = true;
		saveNeeded = true;
	}
	m_Sensors[sensorId] = config;
}

SensorToggleState Configuration::getSensorToggles(size_t sensorId) const {
	if (sensorId >= m_SensorToggles.size()) {
		return {};
	}

	return m_SensorToggles.at(sensorId);
}

void Configuration::setSensorToggles(
	size_t sensorId,
	SensorToggleState state,
	bool nosave
) {
	size_t currentSensors = m_SensorToggles.size();
	m_Logger.trace("setSensorToggles ID %d", sensorId);
	if (sensorId >= currentSensors) {
		m_SensorToggles.resize(sensorId + 1);
	}
	if (sensorId >= m_SensorTogglesChanged.size()) {
		m_SensorTogglesChanged.resize(sensorId + 1);
	}
	if ((m_SensorToggles[sensorId].getValues() != state.getValues()) && !nosave) {
		m_Logger.trace("setSensorToggles ID %d, saving", sensorId);
		m_SensorTogglesChanged[sensorId] = true;
		saveNeeded = true;
	}
	m_SensorToggles[sensorId] = state;
}

// eraseSensors()
// Removes the Configuration:
// m_Sensors from Configuration, but it leaves an active copy on
// the Sensors itself. To make a change after eraseSensors the tracker
// needs to be rebooted. Only then the Sensors are reinitialized
void Configuration::eraseSensors() {
	m_Sensors.clear();
	m_SensorsChanged.clear();

	SlimeVR::Utils::forEachFile(DIR_CALIBRATIONS, [&](SlimeVR::Utils::File f) {
		char path[17];
		sprintf(path, DIR_CALIBRATIONS "/%s", f.name());

		f.close();

		LittleFS.remove(path);
	});
	saveNeeded = true;
}

void Configuration::loadSensors() {
	SlimeVR::Utils::forEachFile(DIR_CALIBRATIONS, [&](SlimeVR::Utils::File f) {
		uint8_t sensorId = strtoul(f.name(), nullptr, 10);

		if (f.size() != sizeof(SensorConfig)) {
			m_Logger.warn(
				"Skipping incompatible sensor calibration file index %d (size=%u "
				"expected=%u)",
				sensorId,
				static_cast<unsigned>(f.size()),
				static_cast<unsigned>(sizeof(SensorConfig))
			);
			return;
		}

		SensorConfig sensorConfig{};
		auto bytesRead = f.read((uint8_t*)&sensorConfig, sizeof(SensorConfig));
		if (bytesRead != sizeof(SensorConfig)) {
			m_Logger.warn(
				"Skipping unreadable sensor calibration file index %d (read=%u "
				"expected=%u)",
				sensorId,
				static_cast<unsigned>(bytesRead),
				static_cast<unsigned>(sizeof(SensorConfig))
			);
			return;
		}

		if (sensorConfig.type > SensorConfigType::RUNTIME_CALIBRATION) {
			m_Logger.warn(
				"Skipping sensor calibration file index %d with invalid type=%d",
				sensorId,
				static_cast<int>(sensorConfig.type)
			);
			return;
		}

		m_Logger.debug(
			"Found sensor calibration for %s at index %d",
			calibrationConfigTypeToString(sensorConfig.type),
			sensorId
		);

		if (sensorConfig.type == SensorConfigType::BNO0XX) {
			SensorToggleState toggles;
			toggles.setToggle(
				SensorToggles::MagEnabled,
				sensorConfig.data.bno0XX.magEnabled
			);
			setSensorToggles(sensorId, toggles, true);
		}

		setSensor(sensorId, sensorConfig, true);
	});

	if (LittleFS.exists(DIR_TOGGLES_OLD)) {
		SlimeVR::Utils::forEachFile(DIR_TOGGLES_OLD, [&](SlimeVR::Utils::File f) {
			SensorToggleValues values;
			// Migration for pre 0.7.0 togglestate, the values started at offset 20 and
			// there were 3 of them
			f.seek(20);
			f.read(reinterpret_cast<uint8_t*>(&values), 3);

			uint8_t sensorId = strtoul(f.name(), nullptr, 10);
			m_Logger.debug("Found sensor toggle state at index %d", sensorId);

			setSensorToggles(sensorId, SensorToggleState{values});
		});
		cleanupMigration();
	}

	SlimeVR::Utils::forEachFile(DIR_TOGGLES, [&](SlimeVR::Utils::File f) {
		if (f.size() > sizeof(SensorToggleValues)) {
			return;
		}
		SensorToggleValues values;
		// With the magic of C++ default initialization, the rest of the values should
		// be their default after reading
		f.read(reinterpret_cast<uint8_t*>(&values), f.size());

		uint8_t sensorId = strtoul(f.name(), nullptr, 10);
		m_Logger.debug("Found sensor toggle state at index %d", sensorId);

		setSensorToggles(sensorId, SensorToggleState{values});
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
			case SensorConfigType::RUNTIME_CALIBRATION:
				m_Logger.info(
					"            A_Ts       : %f",
					c.data.runtimeCalibration.A_Ts
				);
				m_Logger.info(
					"            G_Ts       : %f",
					c.data.runtimeCalibration.G_Ts
				);
				m_Logger.info(
					"            M_Ts       : %f",
					c.data.runtimeCalibration.M_Ts
				);
				m_Logger.info(
					"            T_Ts       : %f",
					c.data.runtimeCalibration.T_Ts
				);

				m_Logger.info(
					"    TimestepsCalibrated: %d",
					c.data.runtimeCalibration.sensorTimestepsCalibrated
				);

				m_Logger.info(
					"        accelCalibrated: %d, %d, %d",
					UNPACK_VECTOR_ARRAY(c.data.runtimeCalibration.accelCalibrated)
				);
				m_Logger.info(
					"            A_off      : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.runtimeCalibration.A_off)
				);

				m_Logger.info(
					"   gyroPointsCalibrated: %d",
					c.data.runtimeCalibration.gyroPointsCalibrated
				);

				m_Logger.info(
					"            G_OffTemp1 : %f",
					c.data.runtimeCalibration.gyroMeasurementTemperature1
				);
				m_Logger.info(
					"            G_off1     : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.runtimeCalibration.G_off1)
				);

				m_Logger.info(
					"            G_OffTemp2 : %f",
					c.data.runtimeCalibration.gyroMeasurementTemperature2
				);
				m_Logger.info(
					"            G_off2     : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.runtimeCalibration.G_off2)
				);

				m_Logger.info(
					"      MotionlessDataLen: %d",
					c.data.runtimeCalibration.MotionlessDataLen
				);

				m_Logger.info("         MotionlessData:");
				m_Logger.loghex(
					SlimeVR::Logging::INFO,
					c.data.runtimeCalibration.MotionlessData,
					sizeof(c.data.runtimeCalibration.MotionlessData)
				);

				break;
			default:
				m_Logger.info(
					"            Not defined to Print %s",
					calibrationConfigTypeToString(c.type)
				);
				break;
		}

		if (i < m_SensorToggles.size()) {
			m_Logger.info("      SensorToggles:");
			m_Logger.info(
				"             MagEnabled: %d",
				m_SensorToggles[i].getToggle(SensorToggles::MagEnabled)
			);
			m_Logger.info(
				"     CalibrationEnabled: %d",
				m_SensorToggles[i].getToggle(SensorToggles::CalibrationEnabled)
			);
			m_Logger.info(
				" TempCalibrationEnabled: %d",
				m_SensorToggles[i].getToggle(
					SensorToggles::TempGradientCalibrationEnabled
				)
			);
		}
	}
}
}  // namespace SlimeVR::Configuration
