#include "ADS111xPin.h"

#include <cassert>

namespace SlimeVR {

ADS111xPin::ADS111xPin(ADS111xInterface* interface, uint8_t channel)
	: ads111x{interface}
	, channel{channel} {
	assert(channel < 4);
}

int ADS111xPin::digitalRead() { return analogRead() >= 0.5f; }
void ADS111xPin::pinMode(uint8_t mode) {}
void ADS111xPin::digitalWrite(uint8_t val) {}

float ADS111xPin::analogRead() { return ads111x->read(channel); }

};  // namespace SlimeVR
