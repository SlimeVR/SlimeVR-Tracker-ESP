#pragma once

#include <cstdint>
#include <array>
#include <algorithm>
#include "bmi270fw.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

// Driver uses acceleration range at 16g
// and gyroscope range at 1000dps
// Gyroscope ODR = 400Hz, accel ODR = 100Hz
// Timestamps reading are not used

template <typename I2CImpl>
struct BMI270
{
    static constexpr uint8_t Address = 0x68;
    static constexpr auto Name = "BMI270";
    static constexpr auto Type = ImuID::BMI270;

    static constexpr float GyrTs=1.0/400.0;
    static constexpr float AccTs=1.0/100.0;

    static constexpr float MagTs=1.0/100;

    static constexpr float GyroSensitivity = 32.768f;
    static constexpr float AccelSensitivity = 2048.0f;

    I2CImpl i2c;
    BMI270(I2CImpl i2c)
    : i2c(i2c) {}

    struct Regs {
        struct WhoAmI {
            static constexpr uint8_t reg = 0x00;
            static constexpr uint8_t value = 0x24;
        };
        static constexpr uint8_t TempData = 0x22;

        struct Cmd {
            static constexpr uint8_t reg = 0x7e;
            static constexpr uint8_t valueSwReset = 0xb6;
            static constexpr uint8_t valueFifoFlush = 0xb0;
        };

        struct PwrConf {
            static constexpr uint8_t reg = 0x7c;
            static constexpr uint8_t valueNoPowerSaving = 0x0;
            static constexpr uint8_t valueFifoSelfWakeup = 0x2;            
        };

        struct PwrCtrl {
            static constexpr uint8_t reg = 0x7d;
            static constexpr uint8_t valueOff = 0x0;
            static constexpr uint8_t valueGyrAccTempOn = 0b1110; // aux off
            static constexpr uint8_t valueAccTempOn = 0b1100; // aux, gyr off
        };

        struct InitCtrl {
            static constexpr uint8_t reg = 0x59;
            static constexpr uint8_t valueStartInit = 0x00;
            static constexpr uint8_t valueEndInit = 0x01;
        };

        static constexpr uint8_t InitAddr = 0x5b;
        static constexpr uint8_t InitData = 0x5e;        

        struct InternalStatus {
            static constexpr uint8_t reg = 0x21;
            static constexpr uint8_t initializedBit = 0x01;
        };

        struct GyrConf {
            static constexpr uint8_t reg = 0x42;

            static constexpr uint8_t rate25Hz = 6;
            static constexpr uint8_t rate50Hz = 7;
            static constexpr uint8_t rate100Hz = 8;
            static constexpr uint8_t rate200Hz = 9;
            static constexpr uint8_t rate400Hz = 10;
            static constexpr uint8_t rate800Hz = 11;
            static constexpr uint8_t rate1600Hz = 12;
            static constexpr uint8_t rate3200Hz = 13;

            static constexpr uint8_t DLPFModeOsr4 = 0 << 4;
            static constexpr uint8_t DLPFModeOsr2 = 1 << 4;
            static constexpr uint8_t DLPFModeNorm = 2 << 4;

            static constexpr uint8_t noisePerfMode = 1 << 6;
            static constexpr uint8_t filterHighPerfMode = 1 << 7;

            static constexpr uint8_t value = rate400Hz | DLPFModeNorm | noisePerfMode | filterHighPerfMode;
        };

        struct GyrRange {
            static constexpr uint8_t reg = 0x43;

            static constexpr uint8_t range125dps = 4;
            static constexpr uint8_t range250dps = 3;
            static constexpr uint8_t range500dps = 2;
            static constexpr uint8_t range1000dps = 1;
            static constexpr uint8_t range2000dps = 0;

            static constexpr uint8_t value = range1000dps;
        };

        struct AccConf {
            static constexpr uint8_t reg = 0x40;

            static constexpr uint8_t rate0_78Hz = 1;
            static constexpr uint8_t rate1_5Hz = 2;
            static constexpr uint8_t rate3_1Hz = 3;
            static constexpr uint8_t rate6_25Hz = 4;
            static constexpr uint8_t rate12_5Hz = 5;
            static constexpr uint8_t rate25Hz = 6;
            static constexpr uint8_t rate50Hz = 7;
            static constexpr uint8_t rate100Hz = 8;
            static constexpr uint8_t rate200Hz = 9;
            static constexpr uint8_t rate400Hz = 10;
            static constexpr uint8_t rate800Hz = 11;
            static constexpr uint8_t rate1600Hz = 12;

            static constexpr uint8_t DLPFModeAvg1 = 0 << 4;
            static constexpr uint8_t DLPFModeAvg2 = 1 << 4;
            static constexpr uint8_t DLPFModeAvg4 = 2 << 4;
            static constexpr uint8_t DLPFModeAvg8 = 3 << 4;

            static constexpr uint8_t filterHighPerfMode = 1 << 7;

            static constexpr uint8_t value = rate100Hz | DLPFModeAvg4 | filterHighPerfMode;
        };

        struct AccRange {
            static constexpr uint8_t reg = 0x41;

            static constexpr uint8_t range2G = 0;
            static constexpr uint8_t range4G = 1;
            static constexpr uint8_t range8G = 2;
            static constexpr uint8_t range16G = 3;

            static constexpr uint8_t value = range16G;
        };

        struct FifoConfig0 {
            static constexpr uint8_t reg = 0x48;
            static constexpr uint8_t value = 0x01; // fifo_stop_on_full=1, fifo_time_en=0
        };

        struct FifoConfig1 {
            static constexpr uint8_t reg = 0x49;
            static constexpr uint8_t value = (1 << 4) | (1 << 6) | (1 << 7); // header en, acc en, gyr en
        };

        static constexpr uint8_t FifoCount = 0x24;
        static constexpr uint8_t FifoData = 0x26;
    };

    struct Fifo {
        static constexpr uint8_t ModeMask = 0b11000000;
        static constexpr uint8_t SkipFrame = 0b01000000;
        static constexpr uint8_t DataFrame = 0b10000000;

        static constexpr uint8_t GyrDataBit = 0b00001000;
        static constexpr uint8_t AccelDataBit = 0b00000100;
    };

    bool initialize()
    {
        // perform initialization step
        i2c.writeReg(Regs::Cmd::reg, Regs::Cmd::valueSwReset);
        delay(12);
        // disable power saving
        i2c.writeReg(Regs::PwrConf::reg, Regs::PwrConf::valueNoPowerSaving);
        delay(1);
        
        // firmware upload
        i2c.writeReg(Regs::InitCtrl::reg, Regs::InitCtrl::valueStartInit);
        for (uint16_t pos=0; pos<sizeof(bmi270_firmware);)
        {
            // tell the device current position

            // this thing is little endian, but it requires address in bizzare form
            // LSB register is only 4 bits, while MSB register is 8bits
            // also value requested is in words (16bit) not in bytes (8bit)

            const uint16_t pos_words = pos >> 1; // convert current position to words
            const uint16_t position = (pos_words & 0x0F) | ((pos_words << 4) & 0xff00);
            i2c.writeReg16(Regs::InitAddr, position);
            // write actual payload chunk
            const uint16_t burstWrite = std::min(sizeof(bmi270_firmware) - pos, I2CImpl::MaxTransactionLength);
            i2c.writeBytes(Regs::InitData, burstWrite, const_cast<uint8_t*>(bmi270_firmware + pos));
            pos += burstWrite;
        }
        i2c.writeReg(Regs::InitCtrl::reg, Regs::InitCtrl::valueEndInit);
        delay(140);
        // leave fifo_self_wakeup enabled
        i2c.writeReg(Regs::PwrConf::reg, Regs::PwrConf::valueFifoSelfWakeup);
        // check if IMU initialized correctly
        if (!(i2c.readReg(Regs::InternalStatus::reg) & Regs::InternalStatus::initializedBit))
        {
            // firmware upload fail or sensor not initialized
            return false;
        }

        i2c.writeReg(Regs::GyrConf::reg, Regs::GyrConf::value);
        i2c.writeReg(Regs::GyrRange::reg, Regs::GyrRange::value);

        i2c.writeReg(Regs::AccConf::reg, Regs::AccConf::value);
        i2c.writeReg(Regs::AccRange::reg, Regs::AccRange::value);

        i2c.writeReg(Regs::PwrCtrl::reg, Regs::PwrCtrl::valueGyrAccTempOn);
        delay(100); // power up delay
        i2c.writeReg(Regs::FifoConfig0::reg, Regs::FifoConfig0::value);
        i2c.writeReg(Regs::FifoConfig1::reg, Regs::FifoConfig1::value);

        delay(4);
        i2c.writeReg(Regs::Cmd::reg, Regs::Cmd::valueFifoFlush);
        delay(2);

        // todo: handle errors

        return true;
    }

    float getDirectTemp() const
    {
        // middle value is 23 degrees C (0x0000)
        // temperature per step from -41 + 1/2^9 degrees C (0x8001) to 87 - 1/2^9 degrees C (0x7FFF)
        constexpr float TempStep = 128. / 65535;
        const auto value = static_cast<int16_t>(i2c.readReg16(Regs::TempData));
        return static_cast<float>(value) * TempStep + 23.0f;
    }

    using FifoBuffer = std::array<uint8_t, I2CImpl::MaxTransactionLength>;
    FifoBuffer read_buffer; 

    template<typename T>
    inline T getFromFifo(uint32_t &position, FifoBuffer& fifo) {
        T to_ret;
        memcpy(&to_ret, &fifo[position], sizeof(T));
        position += sizeof(T);
        return to_ret;
    }

    template <typename AccelCall, typename GyroCall>
    void bulkRead(AccelCall &&processAccelSample, GyroCall &&processGyroSample) {
        const auto fifo_bytes = i2c.readReg16(Regs::FifoCount);
        
        const auto bytes_to_read = std::min(static_cast<size_t>(read_buffer.size()),
            static_cast<size_t>(fifo_bytes));
        i2c.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());

        for (uint32_t i=0u; i<bytes_to_read;) {
            const uint8_t header = getFromFifo<uint8_t>(i, read_buffer);
            if ((header & Fifo::ModeMask) == Fifo::SkipFrame && (i - bytes_to_read) >= 1) {
                getFromFifo<uint8_t>(i, read_buffer); // skip 1 byte
            }
            else if ((header & Fifo::ModeMask) == Fifo::DataFrame) {
                const uint8_t required_length = 
                    (((header & Fifo::GyrDataBit) >> Fifo::GyrDataBit) +
                    ((header & Fifo::AccelDataBit) >> Fifo::AccelDataBit)) * 6;
                if (i - bytes_to_read < required_length) {
                    // incomplete frame, will be re-read next time
                    break;
                }
                if (header & Fifo::GyrDataBit) {
                    int16_t gyro[3];
                    gyro[0] = getFromFifo<uint16_t>(i, read_buffer);
                    gyro[1] = getFromFifo<uint16_t>(i, read_buffer);
                    gyro[2] = getFromFifo<uint16_t>(i, read_buffer);
                    processGyroSample(gyro, GyrTs);
                }

                if (header & Fifo::AccelDataBit) {
                    int16_t accel[3];
                    accel[0] = getFromFifo<uint16_t>(i, read_buffer);
                    accel[1] = getFromFifo<uint16_t>(i, read_buffer);
                    accel[2] = getFromFifo<uint16_t>(i, read_buffer);
                    processAccelSample(accel, AccTs);
                }
            }
        }
    }

};

} // namespace