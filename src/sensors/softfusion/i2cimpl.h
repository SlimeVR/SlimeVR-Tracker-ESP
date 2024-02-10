#pragma once

#include <cstdint>
#include "I2Cdev.h"


namespace SlimeVR::Sensors::SoftFusion
{

struct I2CImpl
{
    I2CImpl(uint8_t devAddr)
        : m_devAddr(devAddr) {}

    uint8_t readReg(uint8_t regAddr) const {
        uint8_t buffer = 0;
        I2Cdev::readByte(m_devAddr, regAddr, &buffer);
        return buffer;
    }

    uint16_t readReg16(uint8_t regAddr) const {
        uint16_t buffer = 0;
        I2Cdev::readBytes(m_devAddr, regAddr, sizeof(buffer), reinterpret_cast<uint8_t*>(&buffer));
        return buffer;
    }

    void writeReg(uint8_t regAddr, uint8_t value) const {
        I2Cdev::writeByte(m_devAddr, regAddr, value);
    }

    void readBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const {
        I2Cdev::readBytes(m_devAddr, regAddr, size, buffer);
    }

    private:
        uint8_t m_devAddr;
};

}
