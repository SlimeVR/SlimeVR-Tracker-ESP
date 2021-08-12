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

unsigned long lastWifiReportTime = 0;
unsigned long wifiConnectionTimeout = millis();
bool isWifiConnected = false;
uint8_t wifiState = 0;

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
    WiFi.mode(WIFI_STA);
    WiFi.hostname("SlimeVR FBT Tracker");
#ifdef ESP32
    WiFi.begin();
#endif
#if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
    // Try saved credentials first, even if we have hardcoded credentials
    if(WiFi.SSID().length() == 0) {
        WiFi.begin(WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
        wifiState = 2;
    } else {
        WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
        wifiState = 1;
    }
#else
    WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
    wifiState = 1;
#endif
    wifiConnectionTimeout = millis();
}

void onConnected() {
    isWifiConnected = true;
    Serial.printf("[NOTICE] WiFi: Connected successfully to SSID '%s', ip address %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    onWiFiConnected();
}

void wifiUpkeep() {
    if(WiFi.status() != WL_CONNECTED) {
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
                        Serial.println("[NOTICE] WiFi: Trying hardcoded credentials...");
                    #endif
                    wifiState = 2;
                return;
                case 2: // Couldn't connect with second set of credentials
                    // Start smart config
                    if(!WiFi.smartConfigDone() && wifiConnectionTimeout + 11000 < millis()) {
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