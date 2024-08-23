#pragma once

#include <span>
#include <stdint.h>

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

struct NoMag {
    template <typename WriteRegCall>
    bool initialize(WriteRegCall&& writeReg) {
        return false;
    }

    uint8_t getAddress() { return 0; }
    uint8_t getStartReg() { return 0; }
    uint8_t getReadCount() { return 0; }

    template <typename MagCall>
    void unpackSample(std::span<uint8_t> sample, MagCall&& processMagSample) { }
};

}
