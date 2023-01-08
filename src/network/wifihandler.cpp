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
#include "globals.h"
#include "network.h"
#include "logging/Logger.h"
#include "GlobalVars.h"
#if !ESP8266
#include "esp_wifi.h"
#endif

unsigned long lastWifiReportTime = 0;
unsigned long wifiConnectionTimeout = millis();
bool isWifiConnected = false;
uint8_t wifiState = SLIME_WIFI_NOT_SETUP;
bool hadWifi = false;
unsigned long last_rssi_sample = 0;

// TODO: Cleanup with proper classes
SlimeVR::Logging::Logger wifiHandlerLogger("WiFiHandler");

void reportWifiError() {
    if(lastWifiReportTime + 1000 < millis()) {
        lastWifiReportTime = millis();
        Serial.print(".");
    }
}

void setStaticIPIfDefined() {
    #ifdef WIFI_USE_STATICIP
    const IPAddress ip(WIFI_STATIC_IP);
    const IPAddress gateway(WIFI_STATIC_GATEWAY);
    const IPAddress subnet(WIFI_STATIC_SUBNET);
    WiFi.config(ip, gateway, subnet);
    #endif
}

bool WiFiNetwork::isConnected() {
    return isWifiConnected;
}

void WiFiNetwork::setWiFiCredentials(const char * SSID, const char * pass) {
    stopProvisioning();
    setStaticIPIfDefined();
    WiFi.begin(SSID, pass);
    // Reset state, will get back into provisioning if can't connect
    hadWifi = false;
    wifiState = SLIME_WIFI_SERVER_CRED_ATTEMPT;
    wifiConnectionTimeout = millis();
}

IPAddress WiFiNetwork::getAddress() {
    return WiFi.localIP();
}

void WiFiNetwork::setUp() {
    wifiHandlerLogger.info("Setting up WiFi");
    WiFi.persistent(true);
    WiFi.mode(WIFI_STA);
    #if ESP8266
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
    #endif
    WiFi.hostname("SlimeVR FBT Tracker");
    wifiHandlerLogger.info("Loaded credentials for SSID %s and pass length %d", WiFi.SSID().c_str(), WiFi.psk().length());
    setStaticIPIfDefined();
    wl_status_t status = WiFi.begin(); // Should connect to last used access point, see https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/station-class.html#begin
    wifiHandlerLogger.debug("Status: %d", status);
    wifiState = SLIME_WIFI_SAVED_ATTEMPT;
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
        wifiHandlerLogger.error("Unable to get WiFi config, power saving not enabled!");
    }
#endif
#endif
}

void onConnected() {
    WiFiNetwork::stopProvisioning();
    statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, false);
    isWifiConnected = true;
    hadWifi = true;
    wifiHandlerLogger.info("Connected successfully to SSID '%s', ip address %s", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
}

uint8_t WiFiNetwork::getWiFiState() {
    return wifiState;
}

void WiFiNetwork::upkeep() {
    upkeepProvisioning();
    if(WiFi.status() != WL_CONNECTED) {
        if(isWifiConnected) {
            wifiHandlerLogger.warn("Connection to WiFi lost, reconnecting...");
            isWifiConnected = false;
        }
        statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, true);
        reportWifiError();
        if(wifiConnectionTimeout + 11000 < millis()) {
            switch(wifiState) {
                case SLIME_WIFI_NOT_SETUP: // Wasn't set up
                return;
                case SLIME_WIFI_SAVED_ATTEMPT: // Couldn't connect with first set of credentials
                    #if ESP8266
                        // Try again but with 11G
                        // But only if there are credentials, otherwise we just waste time before
                        // switching to hardcoded credentials.
                        if (WiFi.SSID().length() > 0) {
                            WiFi.setPhyMode(WIFI_PHY_MODE_11G);
                            setStaticIPIfDefined();
                            WiFi.begin();
                            wifiConnectionTimeout = millis();
                            wifiHandlerLogger.error("Can't connect from saved credentials, status: %d.", WiFi.status());
                            wifiHandlerLogger.debug("Trying saved credentials with PHY Mode G...");
                        } else {
                            wifiHandlerLogger.debug("Skipping PHY Mode G attempt on 0-length SSID...");
                        }
                    #endif
                    wifiState = SLIME_WIFI_SAVED_G_ATTEMPT;
                return;
                case SLIME_WIFI_SAVED_G_ATTEMPT: // Couldn't connect with first set of credentials with PHY Mode G
                    #if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
                        // Try hardcoded credentials now
                        #if ESP8266
                            WiFi.setPhyMode(WIFI_PHY_MODE_11N);
                        #endif
                        setStaticIPIfDefined();
                        WiFi.begin(WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
                        wifiConnectionTimeout = millis();
                        wifiHandlerLogger.error("Can't connect from saved credentials, status: %d.", WiFi.status());
                        wifiHandlerLogger.debug("Trying hardcoded credentials...");
                    #endif
                    wifiState = SLIME_WIFI_HARDCODE_ATTEMPT;
                return;
                case SLIME_WIFI_HARDCODE_ATTEMPT: // Couldn't connect with second set of credentials
                    #if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD) && ESP8266
                        // Try hardcoded credentials again, but with PHY Mode G
                        WiFi.setPhyMode(WIFI_PHY_MODE_11G);
                        setStaticIPIfDefined();
                        WiFi.begin(WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
                        wifiConnectionTimeout = millis();
                        wifiHandlerLogger.error("Can't connect from saved credentials, status: %d.", WiFi.status());
                        wifiHandlerLogger.debug("Trying hardcoded credentials with WiFi PHY Mode G...");
                    #endif
                    wifiState = SLIME_WIFI_HARDCODE_G_ATTEMPT;
                return;
                case SLIME_WIFI_SERVER_CRED_ATTEMPT: // Couldn't connect with server-sent credentials. 
                    #if ESP8266
                        // Try again silently but with 11G
                        WiFi.setPhyMode(WIFI_PHY_MODE_11G);
                        setStaticIPIfDefined();
                        WiFi.begin();
                        wifiConnectionTimeout = millis();
                        wifiState = SLIME_WIFI_SERVER_CRED_G_ATTEMPT;
                    #endif
                return;                
                case SLIME_WIFI_HARDCODE_G_ATTEMPT: // Couldn't connect with second set of credentials with PHY Mode G.
                case SLIME_WIFI_SERVER_CRED_G_ATTEMPT: // Or if couldn't connect with server-sent credentials
                    // Start smart config
                    if(!hadWifi && !WiFi.smartConfigDone() && wifiConnectionTimeout + 11000 < millis()) {
                        if(WiFi.status() != WL_IDLE_STATUS) {
                            wifiHandlerLogger.error("Can't connect from any credentials, status: %d.", WiFi.status());
                            wifiConnectionTimeout = millis();
                        }
                        // Return to the default PHY Mode N.
                        #if ESP8266
                            WiFi.setPhyMode(WIFI_PHY_MODE_11N);
                        #endif
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
