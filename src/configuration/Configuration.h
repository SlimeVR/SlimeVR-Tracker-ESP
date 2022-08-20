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

#ifndef SLIMEVR_CONFIGURATION_CONFIGURATION_H
#define SLIMEVR_CONFIGURATION_CONFIGURATION_H

#include <unordered_map>
#include <vector>

#include "DeviceConfig.h"
#include "WiFiCredential.h"
#include "logging/Logger.h"

namespace SlimeVR {
    namespace Configuration {
        class Configuration {
        public:
            void setup();

            void save();
            void reset();

            void print();

            int32_t getVersion() const;

            size_t getCalibrationCount() const;
            CalibrationConfig getCalibration(size_t sensorID) const;
            void setCalibration(size_t sensorID, const CalibrationConfig& config);

            size_t getWiFiCredentialCount() const;
            const std::unordered_map<std::string, WiFiCredential>* const getWiFiCredentials() const;
            void setWiFiCredential(const WiFiCredential& config);
            void removeWiFiCredential(std::string ssid);

        private:
            void loadCalibrations();
            void loadWiFiCredentials();

            bool runMigrations(int32_t version);
            bool runWiFiCredentialMigrations(WiFiCredential& config);

            bool m_Loaded = false;

            DeviceConfig m_Config{};
            std::vector<CalibrationConfig> m_Calibrations;
            std::unordered_map<std::string, WiFiCredential> m_WiFiCredentials;

            Logging::Logger m_Logger = Logging::Logger("Configuration");

            static CalibrationConfig m_EmptyCalibration;
        };
    } // namespace Configuration
} // namespace SlimeVR

#endif
