#ifndef _I2CSCAN_H_
#define _I2CSCAN_H_ 1

#include <Arduino.h>
#include <Wire.h>

namespace I2CSCAN {
    void scani2cports();
    void update();
    bool checkI2C(uint8_t i, uint8_t j);
    bool hasDevOnBus(uint8_t addr);
    uint8_t pickDevice(uint8_t addr1, uint8_t addr2, bool scanIfNotFound);
    int clearBus(uint8_t SDA, uint8_t SCL);
    boolean inArray(uint8_t value, uint8_t* arr, size_t arrSize);
}

#endif // _I2CSCAN_H_