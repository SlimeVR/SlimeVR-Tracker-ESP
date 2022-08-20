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

#include "WiFiManager.h"

#include "GlobalVars.h"
#include "globals.h"
#include "logging/Logger.h"
#include "network.h"

#if ESP32
    #include "esp_wifi.h"
#endif

namespace SlimeVR {
    namespace Network {
        const bool WiFiManager::isConnected() const {
            return this->connected;
        }

        const bool WiFiManager::isProvisioning() const {
            return this->provisioning && !WiFi.smartConfigDone();
        }

        const IPAddress WiFiManager::getIP() const {
            return WiFi.localIP();
        }

        void WiFiManager::setup() {
            this->logger.info("Setting up WiFi...");

            statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, true);

            WiFi.persistent(false);
            WiFi.mode(WIFI_STA);
            WiFi.hostname(TRACKER_HOSTNAME);

#if ESP8266
    #if POWERSAVING_MODE == POWER_SAVING_NONE
            WiFi.setSleepMode(WIFI_NONE_SLEEP);
    #elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
            WiFi.setSleepMode(WIFI_MODEM_SLEEP);
    #elif POWERSAVING_MODE == POWER_SAVING_MODERATE
            WiFi.setSleepMode(WIFI_MODEM_SLEEP, 10);
    #elif POWERSAVING_MODE == POWER_SAVING_MAXIMUM
        #error "MAX POWER SAVING NOT WORKING YET, please disable!"
    #endif
#else
    #if POWERSAVING_MODE == POWER_SAVING_NONE
            WiFi.setSleep(WIFI_PS_NONE);
    #elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
            WiFi.setSleep(WIFI_PS_MIN_MODEM);
    #elif POWERSAVING_MODE == POWER_SAVING_MODERATE || POWERSAVING_MODE == POWER_SAVING_MAXIMUM
            {
                wifi_config_t conf;
                if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK) {
                    conf.sta.listen_interval = 10;
                    esp_wifi_set_config(WIFI_IF_STA, &conf);
                    WiFi.setSleep(WIFI_PS_MAX_MODEM);
                } else {
                    wifiManagerLogger.error("Unable to get WiFi config, power saving not enabled!");
                }
            }
    #endif
#endif

            this->loadCredentials();

            this->millisSinceConnectionStart = millis();
        }

        void WiFiManager::loadCredentials() {
            auto wifiCredentials = configuration.getWiFiCredentials();
            for (const auto& wifiCredential : *wifiCredentials) {
                this->addCredential(wifiCredential.second);
            }

#if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
            Configuration::WiFiCredential wifiCredential = {CURRENT_WIFICREDENTIAL_VERSION,
                                                            WIFI_CREDS_SSID, WIFI_CREDS_PASSWD};
            this->addCredential(wifiCredential);
#endif
        }

        void WiFiManager::addCredential(const Configuration::WiFiCredential& credential) {
            if (credential.ssid.length() >= 32) {
                this->logger.error("SSID too long!");
                return;
            }

            if (credential.password.length() >= 64) {
                this->logger.error("Password too long!");
                return;
            }

            this->logger.trace("Adding WiFi credential: %s", credential.ssid);

            if (this->wifi.addAP(credential.ssid.c_str(), credential.password.c_str())) {
                this->logger.info("Loaded credentials for \"%s\"", credential.ssid.c_str());
            }
        }

        void WiFiManager::reload() {
            this->logger.info("Reloading WiFi credentials...");

            this->wifi = WiFiMulti();

            this->loadCredentials();

            this->millisSinceConnectionStart = millis();
        }

        void WiFiManager::onConnect() {
            this->logger.info("Connected to \"%s\": %s", WiFi.SSID().c_str(),
                              WiFi.localIP().toString().c_str());

            statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTED, true);
            statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, false);

            this->connected = true;
            this->hadWiFi = true;

            this->stopProvisioning();
        }

        void WiFiManager::onDisconnect() {
            this->logger.warn("Connection to WiFi lost, reconnecting...");

            statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTED, false);
            statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, true);

            this->connected = false;
            this->millisSinceConnectionStart = millis();
        }

        void WiFiManager::update() {
            wl_status_t status = static_cast<wl_status_t>(this->wifi.run(10000));

            if (status == WL_CONNECTED) {
                if (!this->connected) {
                    this->onConnect();
                }

                if (this->lastRSSISampleTime + 2000 < millis()) {
                    this->lastRSSISampleTime = millis();
                    uint8_t rssi = WiFi.RSSI();
                    ::Network::sendSignalStrength(rssi);
                }

                return;
            }

            // Call our `onDisconnect` handler when we were connected previously and now aren't
            // anymore
            if (this->connected) {
                this->onDisconnect();
            }

            if (this->millisSinceConnectionStart + 5000 < millis() && !this->hadWiFi &&
                !this->isProvisioning()) {
                if (status == WL_IDLE_STATUS) {
                    this->logger.error("Can't connect from any credentials, status: %d.",
                                       WiFi.status());
                }

                this->startProvisioning();
            }
        }

        void WiFiManager::updateProvisioning() {
            // Called even when not provisioning to do things like provide neighbours or other
            // upkeep
        }

        void WiFiManager::startProvisioning() {
            if (!WiFi.beginSmartConfig()) {
                this->logger.error("Could not start SmartConfig!");

                return;
            }

            this->provisioning = true;
            this->logger.info("SmartConfig started");
        }

        void WiFiManager::stopProvisioning() {
            WiFi.stopSmartConfig();
            this->provisioning = false;
        }

        // TODO: SmartConfig can't do this
        // This is just here for future reference
        // void WiFiManager::provideNeighbours() {
        // }
    } // namespace Network
} // namespace SlimeVR
