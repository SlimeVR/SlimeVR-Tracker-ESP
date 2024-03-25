#pragma once

#include <cstdint>
#include <array>
#include <algorithm>

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

template <typename I2CImpl>
struct LSM6DSOutputHandler
{
    LSM6DSOutputHandler(I2CImpl i2c, SlimeVR::Logging::Logger &logger) 
        : i2c(i2c), logger(logger)
    {}

    I2CImpl i2c;
    SlimeVR::Logging::Logger &logger;

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

    template <typename AccelCall, typename GyroCall, typename Regs>
    void bulkRead(AccelCall &processAccelSample, GyroCall &processGyroSample, float GyrTs, float AccTs) {
        const auto fifo_status = i2c.readReg16(Regs::FifoStatus);
        const auto available_axes = fifo_status & 0x3ff;
        const auto fifo_bytes = available_axes * 7;
        
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
            }
        }      
    }


};

} // namespace