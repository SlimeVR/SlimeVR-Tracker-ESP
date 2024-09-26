/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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

#pragma once

#include <cstdint>
#include <span>
#include <array>
#include <algorithm>

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

template <typename I2CImpl, typename Mag>
struct LSM6DSOutputHandler
{
    struct Regs {
        struct FuncCfgAccess {
            static constexpr uint8_t reg = 0x01;
            static constexpr uint8_t defaultValue = 0x00;
            static constexpr uint8_t sensorHub = 0x40;
        };
        struct MasterConfig {
            static constexpr uint8_t reg = 0x14;
            static constexpr uint8_t reset = (0b10000000);
            static constexpr uint8_t disable = (0b00000000);
            static constexpr uint8_t enable = (0b01000100); // Write Once, Master On
        };
        struct Slave0Config {
            static constexpr uint8_t reg = 0x17;
            static constexpr uint8_t value = (0b10001000);  // ODR 26, FIFO enabled
        };
        
        static constexpr uint8_t Slave0Address = 0x15;
        static constexpr uint8_t Slave0Register = 0x16;
        static constexpr uint8_t DataWriteSlave0 = 0x21;
        static constexpr uint8_t StatusMaster = 0x22;
    };

    LSM6DSOutputHandler(I2CImpl i2c, SlimeVR::Logging::Logger &logger) 
        : i2c(i2c), logger(logger)
    {}

    Mag mag;
    I2CImpl i2c;
    SlimeVR::Logging::Logger &logger;

    void initializeMag() {
        i2c.writeReg(Regs::FuncCfgAccess::reg, Regs::FuncCfgAccess::sensorHub);
        i2c.writeReg(Regs::MasterConfig::reg, Regs::MasterConfig::reset);
        auto success = mag.initialize(
            [&](uint8_t addr, uint8_t reg, uint8_t value) {
                logger.debug("Writing mag@0x%02X, register 0x%02X - 0x%02X", addr, reg, value);
                i2c.writeReg(Regs::MasterConfig::reg, Regs::MasterConfig::disable);
                i2c.writeReg(Regs::Slave0Address, addr << 1);
                i2c.writeReg(Regs::Slave0Register, reg);
                i2c.writeReg(Regs::DataWriteSlave0, value);
                i2c.writeReg(Regs::MasterConfig::reg, Regs::MasterConfig::enable);
                delay(10);
            }
        );
        i2c.writeReg(Regs::MasterConfig::reg, Regs::MasterConfig::disable);

        if (success) {
            logger.debug("Successfully initialized mag");
            i2c.writeReg(Regs::Slave0Address, (mag.getAddress() << 1) | 1);
            i2c.writeReg(Regs::Slave0Register, mag.getStartReg());
            auto count = mag.getReadCount();
            i2c.writeReg(Regs::Slave0Config::reg, Regs::Slave0Config::value | count);
            i2c.writeReg(Regs::MasterConfig::reg, Regs::MasterConfig::enable);
        }
        i2c.writeReg(Regs::FuncCfgAccess::reg, Regs::FuncCfgAccess::defaultValue);
    }

    template<typename Regs>
    float getDirectTemp() const
    {
        const auto value = static_cast<int16_t>(i2c.readReg16(Regs::OutTemp));
        float result = ((float)value / 256.0f) + 25.0f;

        return result;
    }

    #pragma pack(push, 1)
    struct FifoEntryAligned {
        union {
            int16_t xyz[3];
            uint8_t raw[6];
        };
    };
    #pragma pack(pop)

    static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

    template <typename AccelCall, typename GyroCall, typename MagCall, typename Regs>
    void bulkRead(AccelCall &processAccelSample, GyroCall &processGyroSample, MagCall &processMagSample, float GyrTs, float AccTs, float MagTs) {
        constexpr auto FIFO_SAMPLES_MASK = 0x3ff;
        constexpr auto FIFO_OVERRUN_LATCHED_MASK = 0x800;
        
        const auto fifo_status = i2c.readReg16(Regs::FifoStatus);
        const auto available_axes = fifo_status & FIFO_SAMPLES_MASK;
        const auto fifo_bytes = available_axes * FullFifoEntrySize;
        if (fifo_status & FIFO_OVERRUN_LATCHED_MASK) {
            // FIFO overrun is expected to happen during startup and calibration
            logger.error("FIFO OVERRUN! This occuring during normal usage is an issue.");
        }
        
        std::array<uint8_t, FullFifoEntrySize * 8> read_buffer; // max 8 readings
        const auto bytes_to_read = std::min(static_cast<size_t>(read_buffer.size()),
            static_cast<size_t>(fifo_bytes)) / FullFifoEntrySize * FullFifoEntrySize;
        i2c.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());
        for (auto i=0u; i<bytes_to_read; i+=FullFifoEntrySize) {
            FifoEntryAligned entry;
            uint8_t tag = read_buffer[i] >> 3;
            memcpy(entry.raw, &read_buffer[i+0x1], sizeof(FifoEntryAligned)); // skip fifo header

            switch (tag) {
                case 0x01: // Gyro NC
                    processGyroSample(entry.xyz, GyrTs);
                    break;
                case 0x02: // Accel NC
                    processAccelSample(entry.xyz, AccTs);
                    break;
                case 0x0E:
                    mag.unpackSample(
                        std::span((uint8_t*)&entry.raw, sizeof(entry.raw)),
                        [&](int16_t x, int16_t y, int16_t z) {
                            int16_t xyz[] = {x, y, z};
                            processMagSample(xyz, MagTs);
                        }
                    );
                    break;
            }
        }      
    }


};

} // namespace