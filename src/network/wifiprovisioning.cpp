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
#include "network.h"
#include "logging/Logger.h"

// TODO Currently provisioning implemented via SmartConfig
// it sucks.
// TODO: New implementation: https://github.com/SlimeVR/SlimeVR-Tracker-ESP/issues/71

// TODO: Cleanup with proper classes
SlimeVR::Logging::Logger wifiProvisioningLogger("WiFiProvisioning");
bool provisioning = false;

void WiFiNetwork::upkeepProvisioning() {
    // Called even when not provisioning to do things like provide neighbours or other upkeep
}

void WiFiNetwork::startProvisioning() {
    if(WiFi.beginSmartConfig()) {
        provisioning = true;
        wifiProvisioningLogger.info("SmartConfig started");
    }
}

void WiFiNetwork::stopProvisioning() {
    WiFi.stopSmartConfig();
    provisioning = false;
}

void WiFiNetwork::provideNeighbours() {
    // TODO: SmartConfig can't do this, created for future
}

bool WiFiNetwork::isProvisioning() {
    return provisioning && !WiFi.smartConfigDone();
}