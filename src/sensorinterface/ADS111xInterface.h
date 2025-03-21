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

#include <PinInterface.h>

#include <cstdint>

#include "DirectPinInterface.h"
#include "I2CWireSensorInterface.h"
#include "SensorInterface.h"

namespace SlimeVR {

class ADS111xInterface : public PinInterface {
public:
	explicit ADS111xInterface(
		SensorInterface* interface,
		PinInterface* drdy,
		uint8_t address,
		uint8_t channel
	);
	bool init();
	int digitalRead() override final;
	void pinMode(uint8_t mode) override final;
	void digitalWrite(uint8_t val);
	float analogRead();

private:
	static constexpr uint32_t maxValue = 0x7fff;

	struct Registers {
		static constexpr uint8_t ConversionAddr = 0b00;
		struct Config {
			static constexpr uint8_t Addr = 0b01;
			uint8_t os : 1;
			uint8_t mux : 3;
			uint8_t pga : 3;
			uint8_t mode : 1;
			uint8_t dr : 3;
			uint8_t compMode : 1;
			uint8_t compPol : 1;
			uint8_t compLat : 1;
			uint8_t compQue : 2;
		};
	};
	static_assert(sizeof(Registers::Config) == 2);

	SensorInterface* interface;
	uint8_t address;
	uint8_t channel;
	PinInterface* drdy;
};

}  // namespace SlimeVR
