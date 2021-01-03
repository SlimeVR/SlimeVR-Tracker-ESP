#include <EEPROM.h>
#include "configuration.h"

void initializeConfig() {
    EEPROM.begin(sizeof(DeviceConfig) + 1);
}

bool hasConfigStored() {
    bool hasConfigStored = false;
    EEPROM.get(0, hasConfigStored);
    return hasConfigStored;
}

void loadConfig(DeviceConfig * cfg) {
    EEPROM.get(1, cfg);
}

void saveConfig(DeviceConfig * const cfg) {
    EEPROM.put(0, true);
    EEPROM.put(1, cfg);
    EEPROM.commit();
}