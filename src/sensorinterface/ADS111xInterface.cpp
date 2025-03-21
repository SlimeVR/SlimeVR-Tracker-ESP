#include "ADS111xInterface.h"

namespace SlimeVR {

ADS111xInterface::ADS111xInterface(
	SensorInterface* interface,
	PinInterface* drdy,
	uint8_t address,
	uint8_t channel
)
	: interface(interface)
	, address(address)
	, channel(channel)
	, drdy(drdy) {}

bool ADS111xInterface::init() {
	interface->swapIn();
	Wire.beginTransmission(address);
	Wire.write(Registers::Config::Addr);
	Registers::Config config{
		.os = 0b0,  // Don't start read
		.mux = 0b000,  // Doesn't matter
		.pga = 0b010,  // Gain (FSR): 2.048V
		.mode = 0b1,  // Single-shot
		.dr = 0b100,  // Data-rate: 128 samples per second
		.compMode = 0b0,  // Traditional comparator
		.compPol = 0b0,  // Active low drdy
		.compLat = 0b1,  // Latching drdy
		.compQue = 0b00,  // Assert drdy after every sample
	};
	uint8_t* bytes = reinterpret_cast<uint8_t*>(&config);
	Wire.write(bytes[1]);
	Wire.write(bytes[0]);
	Wire.endTransmission();
	drdy->pinMode(INPUT_PULLUP);
	return true;
}

int ADS111xInterface::digitalRead() { return analogRead() >= 0.5f; }

void ADS111xInterface::pinMode(uint8_t mode) {}

void ADS111xInterface::digitalWrite(uint8_t val) {}

float ADS111xInterface::analogRead() {
	interface->swapIn();
	Wire.beginTransmission(address);
	Wire.write(Registers::Config::Addr);
	Registers::Config config{
		.os = 0b1,  // Start read
		.mux = static_cast<uint8_t>(0b100 | channel),  // Current channel
		.pga = 0b010,  // Gain (FSR): 2.048V
		.mode = 0b1,  // Single-shot
		.dr = 0b100,  // Data-rate: 128 samples per second
		.compMode = 0b0,  // Traditional comparator
		.compPol = 0b0,  // Active low drdy
		.compLat = 0b0,  // Non-latching drdy
		.compQue = 0b11,  // Comparator disabled
	};
	auto* bytes = reinterpret_cast<uint8_t*>(&config);
	Wire.write(bytes[1]);
	Wire.write(bytes[0]);
	Wire.endTransmission();

	// Wait for drdy signal
	while (drdy->digitalRead())
		;

	Wire.beginTransmission(address);
	Wire.write(Registers::ConversionAddr);
	Wire.endTransmission();

	uint8_t msb = Wire.read();
	uint8_t lsb = Wire.read();

	uint16_t value = (msb << 8) | lsb;
	return static_cast<float>(value) / maxValue;
}

}  // namespace SlimeVR
