/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

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
#include "globals.h"
#include "network.h"
#include "ledmgr.h"
#if !ESP8266
#include "esp_wifi.h"
#endif

unsigned long lastWifiReportTime = 0;
unsigned long wifiConnectionTimeout = millis();
bool isWifiConnected = false;
uint8_t wifiState = 0;
bool hadWifi = false;
unsigned long last_rssi_sample = 0;

void reportWifiError() {
    if(lastWifiReportTime + 1000 < millis()) {
        lastWifiReportTime = millis();
        Serial.print(".");
    }
}

bool WiFiNetwork::isConnected() {
    return isWifiConnected;
}

void WiFiNetwork::setWiFiCredentials(const char * SSID, const char * pass) {
    stopProvisioning();
    WiFi.begin(SSID, pass);
    // Reset state, will get back into provisioning if can't connect
    hadWifi = false;
    wifiState = 2;
    wifiConnectionTimeout = millis();
}

IPAddress WiFiNetwork::getAddress() {
    return WiFi.localIP();
}

void WiFiNetwork::setUp() {
    Serial.println("[NOTICE] WiFi: Setting up WiFi");
    WiFi.persistent(true);
    WiFi.mode(WIFI_STA);
    WiFi.hostname("SlimeVR FBT Tracker");
    Serial.printf("[NOTICE] WiFi: Loaded credentials for SSID %s and pass length %d\n", WiFi.SSID().c_str(), WiFi.psk().length());
    wl_status_t status = WiFi.begin(); // Should connect to last used access point, see https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/station-class.html#begin
    Serial.printf("[NOTICE] Status: %d", status);
    wifiState = 1;
    wifiConnectionTimeout = millis();
    
#if ESP8266
#if POWERSAVING_MODE == POWER_SAVING_NONE
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
#elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
    WiFi.setSleepMode(WIFI_MODEM_SLEEP);
#elif POWERSAVING_MODE == POWER_SAVING_MODERATE
    WiFi.setSleepMode(WIFI_MODEM_SLEEP, 10);
#elif POWERSAVING_MODE == POWER_SAVING_MAXIMUM
    WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 10);
#error "MAX POWER SAVING NOT WORKING YET, please disable!"
#endif
#else
#if POWERSAVING_MODE == POWER_SAVING_NONE
    WiFi.setSleep(WIFI_PS_NONE);
#elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
#elif POWERSAVING_MODE == POWER_SAVING_MODERATE || POWERSAVING_MODE == POWER_SAVING_MAXIMUM
    wifi_config_t conf;
    if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK)
    {
        conf.sta.listen_interval = 10;
        esp_wifi_set_config(WIFI_IF_STA, &conf);
        WiFi.setSleep(WIFI_PS_MAX_MODEM);
    }
    else
    {
        Serial.println("[ERR] Unable to get wifi config, power saving not enabled!");
    }
#endif
#endif
}

void onConnected() {
    WiFiNetwork::stopProvisioning();
    LEDManager::unsetLedStatus(LED_STATUS_WIFI_CONNECTING);
    isWifiConnected = true;
    hadWifi = true;
    Serial.printf("[NOTICE] WiFi: Connected successfully to SSID '%s', ip address %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
}

void WiFiNetwork::upkeep() {
    upkeepProvisioning();
    if(WiFi.status() != WL_CONNECTED) {
        if(isWifiConnected) {
            Serial.printf("[NOTICE] WiFi: Connection to WiFi lost, reconnecting...");
            isWifiConnected = false;
        }
        LEDManager::setLedStatus(LED_STATUS_WIFI_CONNECTING);
        reportWifiError();
        if(wifiConnectionTimeout + 11000 < millis()) {
            switch(wifiState) {
                case 0: // Wasn't set up
                return;
                case 1: // Couldn't connect with first set of credentials
                    #if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
                        // Try hardcoded credentials now
                        WiFi.begin(WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
                        wifiConnectionTimeout = millis();
                        Serial.printf("[NOTICE] WiFi: Can't connect from saved credentials, status: %d.\n", WiFi.status());
                        Serial.println("[NOTICE] WiFi: Trying hardcoded credentials...");
                    #endif
                    wifiState = 2;
                return;
                case 2: // Couldn't connect with second set of credentials
                    // Start smart config
                    if(!hadWifi && !WiFi.smartConfigDone() && wifiConnectionTimeout + 11000 < millis()) {
                        if(WiFi.status() != WL_IDLE_STATUS) {
                            Serial.printf("[NOTICE] WiFi: Can't connect from any credentials, status: %d.\n", WiFi.status());
                        }
                        startProvisioning();
                    }
                return;
            }
        }
        return;
    }
    if(!isWifiConnected) {
        onConnected();
        return;
    } else {
        if(millis() - last_rssi_sample >= 2000) {
            last_rssi_sample = millis();
            uint8_t signalStrength = WiFi.RSSI();
            Network::sendSignalStrength(signalStrength);
        }
    }
    return;
}