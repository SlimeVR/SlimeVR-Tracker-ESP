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

#ifdef ESP32
#include <LITTLEFS.h>
#define LittleFS LITTLEFS
#else
#include <LittleFS.h>
#endif

#include "Configuration.h"
#include "consts.h"
#include "utils.h"

namespace SlimeVR
{
    namespace Configuration
    {
        CalibrationConfig Configuration::m_EmptyCalibration = {NONE};

        void Configuration::setup()
        {
            if (m_Loaded)
            {
                return;
            }

            LittleFS.begin();

            if (LittleFS.exists("/config.bin"))
            {
                File file = LittleFS.open("/config.bin", "r");

                file.read((uint8_t *)&m_Config.version, sizeof(int32_t));

                if (m_Config.version < CURRENT_CONFIGURATION_VERSION)
                {
                    m_Logger.debug("Found outdated configuration: %d < %d", m_Config.version, CURRENT_CONFIGURATION_VERSION);

                    if (!runMigrations(m_Config.version))
                    {
                        m_Logger.error("Failed to migrate configuration from %d to %d", m_Config.version, CURRENT_CONFIGURATION_VERSION);
                        return;
                    }
                }
                else
                {
                    m_Logger.info("Found up-to-date configuration v%d", m_Config.version);
                }

                file.seek(0);
                file.read((uint8_t *)&m_Config, sizeof(DeviceConfig));
                file.close();
            }
            else
            {
                m_Config.version = 1;
                save();
            }

            loadCalibrations();

            m_Loaded = true;

            m_Logger.info("Loaded configuration");

#ifdef FULL_DEBUG
            print();
#endif
        }

        void Configuration::save()
        {
            for (size_t i = 0; i < m_Calibrations.size(); i++)
            {
                CalibrationConfig config = m_Calibrations[i];
                if (config.type == CalibrationConfigType::NONE)
                {
                    continue;
                }

                char path[17];
                sprintf(path, "/calibrations/%d", i);

                m_Logger.trace("Saving calibration data for %d", i);

                File file = LittleFS.open(path, "w");
                file.write((uint8_t *)&config, sizeof(CalibrationConfig));
                file.close();
            }

            File file = LittleFS.open("/config.bin", "w");
            file.write((uint8_t *)&m_Config, sizeof(DeviceConfig));
            file.close();

            m_Logger.debug("Saved configuration");
        }

        void Configuration::reset()
        {
            LittleFS.remove("/config.bin");

#ifdef ESP32
            File calibrations = LittleFS.open("/calibrations");
            if (calibrations.isDirectory())
            {
                File f = f.openNextFile();
                while (f)
                {
                    m_Logger.trace("Removing calibration data for %s", calibrations.name());

                    LittleFS.remove(f.name());
                }
            }
#else
            Dir calibrations = LittleFS.openDir("/calibrations");
            while (calibrations.next())
            {
                char path[25];
                sprintf(path, "/calibrations/%s", calibrations.fileName().c_str());

                m_Logger.trace("Removing calibration data for %s", calibrations.fileName().c_str());

                LittleFS.remove(path);
            }
#endif

            m_Calibrations.clear();
            m_Config.version = 1;

            m_Logger.debug("Reset configuration");
        }

        int32_t Configuration::getVersion() const
        {
            return m_Config.version;
        }

        CalibrationConfig Configuration::getCalibration(uint8_t sensorID) const
        {
            if (sensorID >= m_Calibrations.size())
            {
                return m_EmptyCalibration;
            }

            return m_Calibrations.at(sensorID);
        }

        void Configuration::setCalibration(uint8_t sensorID, const CalibrationConfig &config)
        {
            size_t currentCalibrations = m_Calibrations.size();

            if (sensorID >= currentCalibrations)
            {
                m_Calibrations.resize(sensorID + 1, m_EmptyCalibration);
            }

            m_Calibrations[sensorID] = config;
        }

        void Configuration::loadCalibrations()
        {
#ifdef ESP32
            File calibrations = LittleFS.open("/calibrations");
            if (calibrations.isDirectory())
            {
                File f = f.openNextFile();
                while (f)
                {
                    if (f.isDirectory())
                    {
                        continue;
                    }

                    CalibrationConfig calibrationConfig;
                    f.read((uint8_t *)&calibrationConfig, sizeof(CalibrationConfig));

                    uint8_t sensorId = strtoul(calibrations.name(), nullptr, 10);
                    m_Logger.debug("Found sensor calibration for %s at index %d", calibrationConfigTypeToString(calibrationConfig.type), sensorId);

                    setCalibration(sensorId, calibrationConfig);
                }
            }
#else
            Dir calibrations = LittleFS.openDir("/calibrations");
            while (calibrations.next())
            {
                File f = calibrations.openFile("r");
                if (!f.isFile())
                {
                    continue;
                }

                CalibrationConfig calibrationConfig;
                f.read((uint8_t *)&calibrationConfig, sizeof(CalibrationConfig));

                uint8_t sensorId = strtoul(calibrations.fileName().c_str(), nullptr, 10);
                m_Logger.debug("Found sensor calibration for %s at index %d", calibrationConfigTypeToString(calibrationConfig.type), sensorId);

                setCalibration(sensorId, calibrationConfig);
            }
#endif
        }

        bool Configuration::runMigrations(int32_t version)
        {
            return true;
        }

        void Configuration::print()
        {
            m_Logger.info("Configuration:");
            m_Logger.info("  Version: %d", m_Config.version);
            m_Logger.info("  Calibrations:");

            for (size_t i = 0; i < m_Calibrations.size(); i++)
            {
                const CalibrationConfig &c = m_Calibrations[i];
                m_Logger.info("    - [%3d] %s", i, calibrationConfigTypeToString(c.type));

                switch (c.type)
                {
                case CalibrationConfigType::NONE:
                    break;

                case CalibrationConfigType::BMI160:
                    m_Logger.info("            A_B        : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.bmi160.A_B));

                    m_Logger.info("            A_Ainv     :");
                    for (uint8_t i = 0; i < 3; i++)
                    {
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
                    for (uint8_t i = 0; i < 3; i++)
                    {
                        m_Logger.info("                    %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.A_Ainv[i]));
                    }

                    m_Logger.info("            M_B   : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.M_B));

                    m_Logger.info("            M_Ainv:");
                    for (uint8_t i = 0; i < 3; i++)
                    {
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
        }
    }
}
