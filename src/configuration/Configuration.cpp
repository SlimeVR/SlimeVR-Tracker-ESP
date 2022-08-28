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

#include <LittleFS.h>

#include "Configuration.h"
#include "consts.h"
#include "utils.h"

namespace SlimeVR {
    namespace Configuration {
        CalibrationConfig Configuration::m_EmptyCalibration = {NONE};

        void Configuration::setup() {
            if (m_Loaded) {
                return;
            }

            bool status = LittleFS.begin();
            if (!status) {
                m_Logger.warn("Could not mount LittleFS, formatting");

                status = LittleFS.format();
                if (!status) {
                    m_Logger.warn("Could not format LittleFS, aborting");
                    return;
                }

                status = LittleFS.begin();
                if (!status) {
                    m_Logger.error("Could not mount LittleFS, aborting");
                    return;
                }
            }

            if (LittleFS.exists("/config.bin")) {
                m_Logger.trace("Found configuration file");

                File file = LittleFS.open("/config.bin", "r");

                file.read((uint8_t*)&m_Config.version, sizeof(int32_t));

                if (m_Config.version < CURRENT_CONFIGURATION_VERSION) {
                    m_Logger.debug("Configuration is outdated: v%d < v%d", m_Config.version, CURRENT_CONFIGURATION_VERSION);

                    if (!runMigrations(m_Config.version)) {
                        m_Logger.error("Failed to migrate configuration from v%d to v%d", m_Config.version, CURRENT_CONFIGURATION_VERSION);
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

            loadCalibrations();
            loadWiFiCredentials();

            m_Loaded = true;

            m_Logger.info("Loaded configuration");

#ifdef DEBUG_CONFIGURATION
            print();
#endif
        }

        void Configuration::save() {
// Todo:
// - Delete files if no longer needed?
// - Only save files that have actualy changed
            for (size_t i = 0; i < m_Calibrations.size(); i++) {
                CalibrationConfig config = m_Calibrations[i];
                if (config.type == CalibrationConfigType::NONE) {
                    continue;
                }

                char path[17];
                sprintf(path, "/calibrations/%d", i);

                m_Logger.trace("Saving calibration data for %d", i);

                File file = LittleFS.open(path, "w");
                file.write((uint8_t*)&config, sizeof(CalibrationConfig));
                file.close();
            }
// Todo:
// - Delete files if no longer needed
// - Only save files that have actualy changed
            for (const auto& c : m_WiFiCredentials) {
                const WiFiCredential credential = c.second;

                // 51 = 18 (for the path) + 32 (for the max SSID length) + 1 (for the null terminator)
                char path[51];
                sprintf(path, "/wifi_credentials/%s", credential.ssid.c_str());

                m_Logger.trace("Saving WiFi credentials data for %s", credential.ssid);

                File file = LittleFS.open(path, "w");
                file.write(reinterpret_cast<const uint8_t*>(&credential), sizeof(WiFiCredential));
                file.close();
            }
// Todo:
// - Only save files that have actualy changed
            {
                File file = LittleFS.open("/config.bin", "w");
                file.write((uint8_t*)&m_Config, sizeof(DeviceConfig));
                file.close();
            }

            m_Logger.debug("Saved configuration");
        }

        void Configuration::reset() {
            LittleFS.format();

            m_Calibrations.clear();
            m_WiFiCredentials.clear();
            m_Config.version = 1;
            save();

            m_Logger.debug("Reset configuration");
        }

        int32_t Configuration::getVersion() const {
            return m_Config.version;
        }

        size_t Configuration::getCalibrationCount() const {
            return m_Calibrations.size();
        }

        CalibrationConfig Configuration::getCalibration(size_t sensorID) const {
            if (sensorID >= m_Calibrations.size()) {
                return m_EmptyCalibration;
            }

            return m_Calibrations.at(sensorID);
        }

        void Configuration::setCalibration(size_t sensorID, const CalibrationConfig& config) {
            size_t currentCalibrations = m_Calibrations.size();

            if (sensorID >= currentCalibrations) {
                m_Calibrations.resize(sensorID + 1, m_EmptyCalibration);
            }

            m_Calibrations[sensorID] = config;
        }

        size_t Configuration::getWiFiCredentialCount() const {
            return m_WiFiCredentials.size();
        }

        const std::unordered_map<std::string, WiFiCredential>* const Configuration::getWiFiCredentials() const {
            return &m_WiFiCredentials;
        }

        void Configuration::setWiFiCredential(const WiFiCredential& credential) {
            m_WiFiCredentials[credential.ssid] = credential;
        }

        void Configuration::removeWiFiCredential(std::string ssid) {
            m_WiFiCredentials.erase(ssid);
        }

        void Configuration::loadCalibrations() {
#ifdef ESP32
            {
                File calibrations = LittleFS.open("/calibrations");
                if (!calibrations) {
                    m_Logger.warn("No calibration data found, creating new directory...");

                    if (!LittleFS.mkdir("/calibrations")) {
                        m_Logger.error("Failed to create directory: /calibrations");
                        return;
                    }

                    calibrations = LittleFS.open("/calibrations");
                }

                if (!calibrations.isDirectory()) {
                    calibrations.close();

                    m_Logger.warn("Found file instead of directory: /calibrations");

                    if (!LittleFS.remove("/calibrations")) {
                        m_Logger.error("Failed to remove directory: /calibrations");
                        return;
                    }

                    if (!LittleFS.mkdir("/calibrations")) {
                        m_Logger.error("Failed to create directory: /calibrations");
                        return;
                    }

                    calibrations = LittleFS.open("/calibrations");
                }

                m_Logger.debug("Found calibration data directory");

                while (File f = calibrations.openNextFile()) {
                    if (f.isDirectory()) {
                        continue;
                    }
                    uint8_t sensorId = strtoul(f.name(), nullptr, 10);
                    m_Logger.trace("Found calibration data file: %s", f.name());

                    CalibrationConfig calibrationConfig;
                    f.read((uint8_t*)&calibrationConfig, sizeof(CalibrationConfig));
                    f.close();

                    m_Logger.debug("Found sensor calibration for %s at index %d", calibrationConfigTypeToString(calibrationConfig.type), sensorId);

                    setCalibration(sensorId, calibrationConfig);
                }

                calibrations.close();
            }
#else
            {
                if (!LittleFS.exists("/calibrations")) {
                    m_Logger.warn("No calibration data found, creating new directory...");

                    if (!LittleFS.mkdir("/calibrations")) {
                        m_Logger.error("Failed to create directory: /calibrations");
                        return;
                    }

                    // There's no calibrations here, so we're done
                    return;
                }

                Dir calibrations = LittleFS.openDir("/calibrations");
                while (calibrations.next()) {
                    File f = calibrations.openFile("r");
                    if (!f.isFile()) {
                        continue;
                    }

                    m_Logger.trace("Found calibration data file: %s", f.name());

                    CalibrationConfig calibrationConfig;
                    f.read((uint8_t*)&calibrationConfig, sizeof(CalibrationConfig));
                    f.close();

                    uint8_t sensorId = strtoul(calibrations.fileName().c_str(), nullptr, 10);
                    m_Logger.debug("Found sensor calibration for %s at index %d", calibrationConfigTypeToString(calibrationConfig.type), sensorId);

                    setCalibration(sensorId, calibrationConfig);
                }
            }
#endif
        }

        void Configuration::loadWiFiCredentials() {
#ifdef ESP32
            {
                File calibrations = LittleFS.open("/wifi_credentials");
                if (!calibrations) {
                    m_Logger.warn("No WiFi credentials found, creating new directory...");

                    if (!LittleFS.mkdir("/wifi_credentials")) {
                        m_Logger.error("Failed to create directory: /wifi_credentials");
                        return;
                    }

                    calibrations = LittleFS.open("/wifi_credentials");
                }

                if (!calibrations.isDirectory()) {
                    calibrations.close();

                    m_Logger.warn("Found file instead of directory: /wifi_credentials");

                    if (!LittleFS.remove("/wifi_credentials")) {
                        m_Logger.error("Failed to remove directory: /wifi_credentials");
                        return;
                    }

                    if (!LittleFS.mkdir("/wifi_credentials")) {
                        m_Logger.error("Failed to create directory: /wifi_credentials");
                        return;
                    }

                    calibrations = LittleFS.open("/wifi_credentials");
                }

                m_Logger.debug("Found WiFi credential directory");

                while (File f = calibrations.openNextFile()) {
                    if (f.isDirectory()) {
                        continue;
                    }

                    m_Logger.trace("Found WiFi credential file: %s", f.name());

                    WiFiCredential wifiCredential;

                    f.read(reinterpret_cast<uint8_t*>(&wifiCredential.version), sizeof(int32_t));
                    if (wifiCredential.version < CURRENT_WIFICREDENTIAL_VERSION) {
                        m_Logger.debug("WiFi Credential version is outdated: v%d < v%d", wifiCredential.version, CURRENT_WIFICREDENTIAL_VERSION);

                        if (!runWiFiCredentialMigrations(wifiCredential)) {
                            m_Logger.error("Failed to migrate WiFi credential from v%d to v%d", wifiCredential.version, CURRENT_WIFICREDENTIAL_VERSION);
                            continue;
                        }
                    } else {
                        m_Logger.info("Found up-to-date WiFi credential v%d", m_Config.version);
                    }

                    f.seek(0);
                    f.read(reinterpret_cast<uint8_t*>(&wifiCredential), sizeof(WiFiCredential));
                    f.close();

                    setWiFiCredential(wifiCredential);
                }

                calibrations.close();
            }
#else
            {
                if (!LittleFS.exists("/wifi_credentials")) {
                    m_Logger.warn("No WiFi credentials found, creating new directory...");

                    if (!LittleFS.mkdir("/wifi_credentials")) {
                        m_Logger.error("Failed to create directory: /wifi_credentials");
                        return;
                    }

                    // There's no credentials here, so we're done
                    return;
                }

                Dir calibrations = LittleFS.openDir("/wifi_credentials");
                while (calibrations.next()) {
                    File f = calibrations.openFile("r");
                    if (!f.isFile()) {
                        continue;
                    }

                    m_Logger.trace("Found WiFi credential file: %s", f.name());

                    WiFiCredential wifiCredential;

                    f.read(reinterpret_cast<uint8_t*>(&wifiCredential.version), sizeof(int32_t));
                    if (wifiCredential.version < CURRENT_WIFICREDENTIAL_VERSION) {
                        m_Logger.debug("WiFi Credential version is outdated: v%d < v%d", wifiCredential.version, CURRENT_WIFICREDENTIAL_VERSION);

                        if (!runWiFiCredentialMigrations(wifiCredential)) {
                            m_Logger.error("Failed to migrate WiFi credential from v%d to v%d", wifiCredential.version, CURRENT_WIFICREDENTIAL_VERSION);
                            continue;
                        }
                    } else {
                        m_Logger.info("Found up-to-date WiFi credential v%d", m_Config.version);
                    }

                    f.seek(0);
                    f.read(reinterpret_cast<uint8_t*>(&wifiCredential), sizeof(WiFiCredential));
                    f.close();

                    m_Logger.trace("Found WiFi credential file for SSID \"%s\"", wifiCredential.ssid);

                    setWiFiCredential(wifiCredential);
                }
            }
#endif
#if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
            m_Logger.info("Found hardcoded WiFi credentials loading");
            WiFiCredential wifiCredentialHC = {CURRENT_WIFICREDENTIAL_VERSION, WIFI_CREDS_SSID, WIFI_CREDS_PASSWD};
            setWiFiCredential(wifiCredentialHC);
#endif
        }

        bool Configuration::runMigrations(int32_t version) {
            return true;
        }

        bool Configuration::runWiFiCredentialMigrations(WiFiCredential& wifiCredential) {
            return true;
        }

        void Configuration::print() {
            m_Logger.info("Configuration:");
            m_Logger.info("  Version: %d", m_Config.version);
            m_Logger.info("  %d Calibrations:", m_Calibrations.size());

            for (size_t i = 0; i < m_Calibrations.size(); i++) {
                const CalibrationConfig& c = m_Calibrations[i];
                m_Logger.info("    - [%3d] %s", i, calibrationConfigTypeToString(c.type));

                switch (c.type) {
                case CalibrationConfigType::NONE:
                    break;

                case CalibrationConfigType::BMI160:
                    m_Logger.info("            A_B        : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.bmi160.A_B));

                    m_Logger.info("            A_Ainv     :");
                    for (uint8_t i = 0; i < 3; i++) {
                        m_Logger.info("                         %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.bmi160.A_Ainv[i]));
                    }

                    m_Logger.info("            G_off      : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.bmi160.G_off));
                    m_Logger.info("            Temperature: %f", c.data.bmi160.temperature);

                    break;

                case CalibrationConfigType::ICM20948:
                    m_Logger.info("            G: %d, %d, %d", UNPACK_VECTOR_ARRAY(c.data.icm20948.G));
                    m_Logger.info("            A: %d, %d, %d", UNPACK_VECTOR_ARRAY(c.data.icm20948.A));
                    m_Logger.info("            C: %d, %d, %d", UNPACK_VECTOR_ARRAY(c.data.icm20948.C));

                    break;

                case CalibrationConfigType::MPU9250:
                    m_Logger.info("            A_B   : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.A_B));

                    m_Logger.info("            A_Ainv:");
                    for (uint8_t i = 0; i < 3; i++) {
                        m_Logger.info("                    %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.A_Ainv[i]));
                    }

                    m_Logger.info("            M_B   : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.M_B));

                    m_Logger.info("            M_Ainv:");
                    for (uint8_t i = 0; i < 3; i++) {
                        m_Logger.info("                    %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.M_Ainv[i]));
                    }

                    m_Logger.info("            G_off  : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.G_off));

                    break;

                case CalibrationConfigType::MPU6050:
                    m_Logger.info("            A_B  : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu6050.A_B));
                    m_Logger.info("            G_off: %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu6050.G_off));

                    break;
                }
            }

            m_Logger.info("  %d WiFi credentials:", m_WiFiCredentials.size());

            for (const auto& c : m_WiFiCredentials) {
                m_Logger.info("    - %s", c.second.ssid);
            }
        }
    }
}
