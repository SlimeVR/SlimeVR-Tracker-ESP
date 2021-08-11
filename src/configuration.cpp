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

#include <EEPROM.h>
#include "configuration.h"

DeviceConfig config{};
bool configLoaded;

void initializeConfig() {
    EEPROM.begin(sizeof(DeviceConfig) + 1);
}

bool hasConfigStored() {
    bool hasConfigStored = false;
    EEPROM.get(0, hasConfigStored);
    return hasConfigStored;
}

DeviceConfig *const getConfigPtr()
{
    if (!configLoaded)
    {
        initializeConfig();
        if (hasConfigStored())
        {
            EEPROM.get(1, config);
        }
        configLoaded = true;
    }
    return &config;
}

void setConfig(const DeviceConfig & newConfig) {
    config = newConfig;
    saveConfig();
}

void saveConfig() {
    EEPROM.put(0, true);
    EEPROM.put(1, config);
    EEPROM.commit();
}