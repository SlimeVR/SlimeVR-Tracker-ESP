#pragma once

#include <cstdint>
#include "I2Cdev.h"


namespace SlimeVR::Sensors::SoftFusion
{

template <uint8_t DevAddr>
struct I2CImpl
{
    static uint8_t readReg(uint8_t regAddr) {
        uint8_t buffer = 0;
        I2Cdev::readByte(DevAddr, regAddr, &buffer);
        return buffer;
    }

    static uint16_t readReg16(uint8_t regAddr) {
        uint16_t buffer = 0;
        I2Cdev::readBytes(DevAddr, regAddr, sizeof(buffer), reinterpret_cast<uint8_t*>(&buffer));
        return buffer;
    }

    static void writeReg(uint8_t regAddr, uint8_t value)
    {
        I2Cdev::writeByte(DevAddr, regAddr, value);
    }

    static void readBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) {
        I2Cdev::readBytes(DevAddr, regAddr, size, buffer);
    }
};

}
