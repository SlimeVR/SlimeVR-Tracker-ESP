#include "ADS111xInterface.h"

#include <cstring>

namespace SlimeVR {

ADS111xInterface::ADS111xInterface(SensorInterface* interface, uint8_t address)
	: interface {
	interface
}, address{address} {
}

bool ADS111xInterface::init() {
	interface->swapIn();
	Wire.beginTransmission(address);
	Wire.write(static_cast<uint8_t>(Registers::Addresses::Config));
	Registers::Config config{
		.mode = 0b1,  // Single-shot
		.pga = 0b010,  // Gain (FSR): 2.048V
		.mux = 0b000,  // Doesn't matter
		.os = 0b0,  // Don't start read
		.compQue = 0b11,  // Disable comparator
		.compLat = 0b0,  // Doesn't matter
		.compPol = 0b0,  // Doesn't matter
		.compMode = 0b0,  // Doesn't matter
		.dr = 0b100,  // Doesn't matter
	};
	uint8_t* bytes = reinterpret_cast<uint8_t*>(&config);
	Wire.write(bytes[0]);
	Wire.write(bytes[1]);
	auto result = Wire.endTransmission();

	if (result != 0) {
		logger.error("Couldn't initialize ADS interface!\n");
		return false;
	}

	return true;
}

float ADS111xInterface::read(uint8_t channel) {
	interface->swapIn();
	Wire.beginTransmission(address);
	Wire.write(static_cast<uint8_t>(Registers::Addresses::Config));
	Registers::Config config{
		.mode = 0b1,  // Single-shot
		.pga = 0b010,  // Gain (FSR): 2.048V
		.mux = static_cast<uint8_t>(0b100 | channel),  // Current channel
		.os = 0b1,  // Start read
		.compQue = 0b11,  // Disable comparator
		.compLat = 0b0,  // Doesn't matter
		.compPol = 0b0,  // Doesn't matter
		.compMode = 0b0,  // Doesn't matter
		.dr = 0b111,  // 860 samples per second
	};

	auto* bytes = reinterpret_cast<uint8_t*>(&config);
	Wire.write(bytes[0]);
	Wire.write(bytes[1]);
	Wire.endTransmission();

	delayMicroseconds(1e6 / 860);

	Wire.beginTransmission(address);
	Wire.write(static_cast<uint8_t>(Registers::Addresses::Conversion));
	Wire.endTransmission();

	Wire.beginTransmission(address);
	Wire.requestFrom(address, 2);
	uint8_t msb = Wire.read();
	uint8_t lsb = Wire.read();
	Wire.endTransmission();

	uint16_t value = (msb << 8) | lsb;

	return static_cast<float>(value) / maxValue;
}

}  // namespace SlimeVR
