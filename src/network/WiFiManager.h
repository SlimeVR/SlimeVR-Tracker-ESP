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

#ifndef NETWORK_WIFIMANAGER_H
#define NETWORK_WIFIMANAGER_H

#include "configuration/WiFiCredential.h"
#include "logging/Logger.h"

#ifdef ESP8266
    #include <ESP8266WiFi.h>
    #include <ESP8266WiFiMulti.h>
    #define WiFiMulti ESP8266WiFiMulti
#else
    #include <WiFi.h>
    #include <WiFiMulti.h>
#endif

namespace SlimeVR {
    namespace Network {
        class WiFiManager {
        public:
            /* `WiFiManager#setup()` initialized the WiFi instance and loads all credentials. */
            void setup();

            /* `WiFiManager#update()` runs the WiFi instance and performs connectivity checks. */
            void update();

            /* `WiFiManager#reload()` reloads all credentials and restarts the WiFi instance. */
            void reload();

            /* `WiFiManager#addCredential(WiFiCredential)` adds a new credential to the WiFi instance. */
            void addCredential(const Configuration::WiFiCredential& credential);

            const bool isConnected() const;
            const bool isProvisioning() const;

            const IPAddress getIP() const;

        private:
            void onConnect();
            void onDisconnect();

            void loadCredentials();

            void updateProvisioning();
            void startProvisioning();
            void stopProvisioning();

            // TODO: SmartConfig can't do this
            // This is just here for the future
            // void provideNeighbours();

            WiFiMulti wifi;

            bool connected = false;
            bool hadWiFi = false;
            bool provisioning = false;

            unsigned int attempts = 0;
            unsigned long lastRSSISampleTime = 0;
            unsigned long millisSinceConnectionStart = 0;

            Logging::Logger logger = Logging::Logger("WiFiManager");
        };
    } // namespace Network
} // namespace SlimeVR

#endif // NETWORK_WIFIMANAGER_H
