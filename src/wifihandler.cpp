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

#include "wifihandler.h"
#include "udpclient.h"
#include "defines.h"
#include "ledstatus.h"

unsigned long lastWifiReportTime = 0;
unsigned long wifiConnectionTimeout = millis();
bool isWifiConnected = false;
uint8_t wifiState = 0;
bool hadWifi = false;

namespace {
    void reportWifiError() {
        if(lastWifiReportTime + 1000 < millis()) {
            lastWifiReportTime = millis();
            Serial.print(".");
        }
    }
}

bool isWiFiConnected() {
    return isWifiConnected;
}

void setWiFiCredentials(const char * SSID, const char * pass) {
    WiFi.stopSmartConfig();
    WiFi.begin(SSID, pass);
    wifiState = 2;
    wifiConnectionTimeout = millis();
}

void setUpWiFi() {
    Serial.println("[NOTICE] WiFi: Setting up WiFi");
    WiFi.persistent(true);
    WiFi.mode(WIFI_STA);
    WiFi.hostname("SlimeVR FBT Tracker");
    Serial.printf("[NOTICE] WiFi: Loaded credentials for SSID %s and pass length %d\n", WiFi.SSID().c_str(), WiFi.psk().length());
    wl_status_t status = WiFi.begin(); // Should connect to last used access point, see https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/station-class.html#begin
    Serial.printf("[NOTICE] Status: %d", status);
    wifiState = 1;
    wifiConnectionTimeout = millis();
}

void onConnected() {
    unsetLedStatus(LED_STATUS_WIFI_CONNECTING);
    isWifiConnected = true;
    hadWifi = true;
    Serial.printf("[NOTICE] WiFi: Connected successfully to SSID '%s', ip address %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    onWiFiConnected();
}

void wifiUpkeep() {
    if(WiFi.status() != WL_CONNECTED) {
        if(isWifiConnected) {
            Serial.printf("[NOTICE] WiFi: Connection to WiFi lost, reconnecting...");
            isWifiConnected = false;
        }
        setLedStatus(LED_STATUS_WIFI_CONNECTING);
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
                        if (WiFi.status() != WL_IDLE_STATUS) {
                            Serial.printf("[NOTICE] WiFi: Can't connect from any credentials, status: %d.\n", WiFi.status());
                        }
                        if(WiFi.beginSmartConfig()) {
                            Serial.println("[NOTICE] WiFi: SmartConfig started");
                        }
                    }
                return;
            }
        }
        return;
    }
    if(!isWifiConnected) {
        onConnected();
        return;
    }
    return;
}