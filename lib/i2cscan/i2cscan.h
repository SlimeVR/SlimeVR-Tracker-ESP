#ifndef _I2CSCAN_H_
#define _I2CSCAN_H_ 1

#include <Arduino.h>
#include <Wire.h>

namespace I2CSCAN {
    void scani2cports();
    bool checkI2C(uint8_t i, uint8_t j);
    bool isI2CExist(uint8_t addr);
}

#endif // _I2CSCAN_H_