/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors

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
#include "ADS111xInterface.h"

#include <cstring>

namespace SlimeVR {

ADS111xInterface::ADS111xInterface(
	SensorInterface* interface,
	PinInterface* drdy,
	uint8_t address
)
	: interface{interface}
	, drdy{drdy}
	, address{address} {}

bool ADS111xInterface::init() {
	Registers::Config config{
		.compQue = 0b11,  // Disable comparator
		.compLat = 0b0,  // Doesn't matter
		.compPol = 0b0,  // Doesn't matter
		.compMode = 0b0,  // Doesn't matter
		.dr = 0b100,  // Doesn't matter
		.mode = 0b1,  // Single-shot
		.pga = 0b010,  // Gain (FSR): 2.048V
		.mux = 0b000,  // Doesn't matter
		.os = 0b0,  // Don't start read
	};

	if (!writeRegister(Registers::Addresses::Config, config)) {
		logger.error("Couldn't initialize ADS interface!\n");
		return false;
	}

	// Enable conversion ready functionality
	writeRegister(Registers::Addresses::HiThresh, static_cast<uint16_t>(0x8000));
	writeRegister(Registers::Addresses::LoThresh, static_cast<uint16_t>(0x0000));

	drdy->pinMode(INPUT);

	return true;
}

float ADS111xInterface::read(uint8_t channel) {
	if (__builtin_popcount(usedPins) == 1) {
		// Just read the last sample
		uint16_t value = readRegister(Registers::Addresses::Conversion);

		return static_cast<float>(value) / maxValue;
	}

	Registers::Config config{
		.compQue = 0b00,  // Alert after every sample
		.compLat = 0b1,  // Latch alert
		.compPol = 0b0,  // Doesn't matter
		.compMode = 0b0,  // Doesn't matter
		.dr = 0b111,  // 860 samples per second
		.mode = 0b1,  // Single-shot
		.pga = 0b010,  // Gain (FSR): 2.048V
		.mux = static_cast<uint8_t>(0b100 | channel),  // Current channel
		.os = 0b1,  // Start read
	};

	writeRegister(Registers::Addresses::Config, config);

	while (drdy->digitalRead())
		;

	uint16_t value = readRegister(Registers::Addresses::Conversion);

	return static_cast<float>(value) / maxValue;
}

uint16_t ADS111xInterface::readRegister(Registers::Addresses reg) {
	Wire.beginTransmission(address);
	Wire.write(static_cast<uint8_t>(Registers::Addresses::Conversion));
	Wire.endTransmission();

	Wire.beginTransmission(address);
	Wire.requestFrom(address, 2);
	uint8_t msb = Wire.read();
	uint8_t lsb = Wire.read();
	Wire.endTransmission();

	return (msb << 8) | lsb;
}

void ADS111xInterface::registerChannel(uint8_t channel) {
	usedPins |= 1 << channel;
	if (__builtin_popcount(usedPins) != 1) {
		return;
	}

	// If we have only one channel used, just set up continuous reads
	Registers::Config config{
		.compQue = 0b11,  // Disable comparator
		.compLat = 0b0,  // Doesn't matter
		.compPol = 0b0,  // Doesn't matter
		.compMode = 0b0,  // Doesn't matter
		.dr = 0b100,  // 128 samples per second
		.mode = 0b0,  // Continuous mode
		.pga = 0b010,  // Gain (FSR): 2.048V
		.mux = static_cast<uint8_t>(0b100 | channel),  // Use the channel
		.os = 0b1,  // Start reads
	};

	writeRegister(Registers::Addresses::Config, config);
}

}  // namespace SlimeVR
