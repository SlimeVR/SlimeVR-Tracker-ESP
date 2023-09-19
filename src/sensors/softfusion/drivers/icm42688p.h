#pragma once

#include <cstdint>
#include <array>
#include <algorithm>

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

// Driver uses acceleration range at 8g
// and gyroscope range at 1000dps

template <template<uint8_t> typename I2CImpl>
struct ICM42688P
{
    static constexpr uint8_t DevAddr = 0x68;
    static constexpr auto Name = "ICM-42688-P";
    static constexpr auto Type = 13;

    static constexpr float GyrTs=1.0/500;
    static constexpr float AccTs=1.0/100;
    static constexpr float MagTs=1.0/100;

    static constexpr float GyroSensitivity = 32.8f;
    static constexpr float AccelSensitivity = 4096.0f;

    using i2c = I2CImpl<DevAddr>;
 
    //uint32_t m_freqSamples = 1;
    //float m_freq = 425.0f;
    //unsigned long m_lastTimestamp = millis();


    struct Regs {
        struct WhoAmI {
            static constexpr uint8_t reg = 0x75;
            static constexpr uint8_t value = 0x47;
        };
        static constexpr uint8_t TempData = 0x1d;

        struct DeviceConfig {
            static constexpr uint8_t reg = 0x11;
            static constexpr uint8_t valueSwReset = 1;
        };
        struct IntfConfig0 {
            static constexpr uint8_t reg = 0x4c;
            static constexpr uint8_t value = (0 << 4) | (0 << 5) | (0 << 6); //fifo count in LE, sensor data in LE, fifo size in bytes
        };
        struct FifoConfig0 {
            static constexpr uint8_t reg = 0x16;
            static constexpr uint8_t value = (0b01 << 6); //stream to FIFO mode
        };
        struct FifoConfig1 {
            static constexpr uint8_t reg = 0x5f;
            static constexpr uint8_t value = 0b1 | (0b1 << 1) | (0b0 << 2); //fifo accel en=1, gyro=1, temp=0 todo: fsync, hires 
        };
        struct GyroConfig {
            static constexpr uint8_t reg = 0x4f;
            static constexpr uint8_t value = (0b001 << 5) | 0b1111; //1000dps, odr=500Hz
        };
        struct AccelConfig {
            static constexpr uint8_t reg = 0x50;
            static constexpr uint8_t value = (0b001 << 5) | 0b1000; //8g, odr = 100Hz
        };
        struct PwrMgmt {
            static constexpr uint8_t reg = 0x4e;
            static constexpr uint8_t value = 0b11 | (0b11 << 2); //accel in low noise mode, gyro in low noise
        };

        //GYRO_CONFIG1
        //GYRO_ACCEL_CONFIG0
        //ACCEL_CONFIG1
        //TMST_CONFIG check deltas!

        static constexpr uint8_t FifoCount = 0x2e;
        static constexpr uint8_t FifoData = 0x30;
    };

    bool initialize()
    {
        // perform initialization step
        i2c::writeReg(Regs::DeviceConfig::reg, Regs::DeviceConfig::valueSwReset);
        delay(20);
        i2c::writeReg(Regs::IntfConfig0::reg, Regs::IntfConfig0::value);
        i2c::writeReg(Regs::GyroConfig::reg, Regs::GyroConfig::value);
        i2c::writeReg(Regs::AccelConfig::reg, Regs::AccelConfig::value);
        i2c::writeReg(Regs::FifoConfig0::reg, Regs::FifoConfig0::value);
        i2c::writeReg(Regs::FifoConfig1::reg, Regs::FifoConfig1::value);
        i2c::writeReg(Regs::PwrMgmt::reg, Regs::PwrMgmt::value);
        delay(100);
        return true;
    }

    float getDirectTemp() const
    {
        const auto value = static_cast<int16_t>(i2c::readReg16(Regs::TempData));
        float result = ((float)value / 132.48f) + 25.0f;
        return result;
    }

    template <typename AccelCall, typename GyroCall>
    void bulkRead(AccelCall &&processAccelSample, GyroCall &&processGyroSample) {
        const auto fifo_bytes = i2c::readReg16(Regs::FifoCount);
        constexpr auto single_measurement_bytes = 16;
        
        std::array<uint8_t, single_measurement_bytes * 6> read_buffer; // max 8 readings
        const auto bytes_to_read = std::min(static_cast<size_t>(read_buffer.size()),
            static_cast<size_t>(fifo_bytes)) / single_measurement_bytes * single_measurement_bytes;
        i2c::readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());
        //static auto samples = 0;
        for (uint16_t i=0; i<bytes_to_read; i+=single_measurement_bytes) {
            //printf("\r%d/%d     ", i, bytes_to_read/*, read_buffer[3], read_buffer[4], read_buffer[5]*/);
            int16_t samples[3];
            memcpy(samples, &read_buffer[i+0x07], sizeof(samples));
            processGyroSample(samples, GyrTs);

            memcpy(samples, &read_buffer[i+0x01], sizeof(samples));
            if (samples[0] != -32768) {
                processAccelSample(samples, AccTs);
            }
            //samples++;
        }
        /*
        auto stop = millis();
        if (stop - m_lastTimestamp >= 1000) {
            float lastSamples =  (samples*1000.0) / (stop - m_lastTimestamp);
            printf("Samples %f mean %f diff %d\n", lastSamples, m_freq, stop - m_lastTimestamp);
            m_freq += (lastSamples - m_freq) / m_freqSamples;
            samples = 0;
            m_lastTimestamp += 1000;

            m_freqSamples++;
        }*/
        
    }


};

} // namespace