#include <span>
#include <stdint.h>

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

struct QMC5883L {
    static constexpr uint8_t Address = 0x0D;

    struct Regs {
        static constexpr uint8_t Data = 0x00;   // 3 x 2 bytes
        static constexpr uint8_t Status = 0x06; // 1 byte
        static constexpr uint8_t Temp = 0x07;   // 2 bytes

        struct Control {
            static constexpr uint8_t reg = 0x09;
            static constexpr uint8_t value = (0b00011101); // 512 oversampling rate / 8 Gauss range / 200Hz ODR / Continuous mode
        };
        struct Reset {
            static constexpr uint8_t reg = 0x0B;
            static constexpr uint8_t value = 1;
        };
        static constexpr uint8_t Reset = 0x0B;
    };

    template <typename WriteRegCall>
    bool initialize(WriteRegCall&& writeReg) {
        delay(2);
        writeReg(Address, Regs::Reset::reg, Regs::Reset::value);
        delay(5);
        writeReg(Address, Regs::Control::reg, Regs::Control::value);
        return true;
    }

    uint8_t getAddress() { return Address; }
    uint8_t getStartReg() { return Regs::Data; }
    uint8_t getReadCount() { return 6; }

    template <typename MagCall>
    void unpackSample(std::span<uint8_t> sample, MagCall&& processMagSample) {
        auto x = (int16_t)((int)sample[0] | sample[1] << 8);
        auto y = (int16_t)((int)sample[2] | sample[3] << 8);
        auto z = (int16_t)((int)sample[4] | sample[5] << 8);
        processMagSample(x, y, z);
    }

};

}
