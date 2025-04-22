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

#include "ParallelMuxInterface.h"

#include <Arduino.h>

#include <cassert>

namespace SlimeVR {

ParallelMuxInterface::ParallelMuxInterface(
	PinInterface* dataPin,
	std::vector<PinInterface*>& addressPins,
	PinInterface* enablePin,
	bool enableActiveLevel,
	bool addressActiveLevel
)
	: dataPin{dataPin}
	, addressPins{addressPins}
	, enablePin{enablePin}
	, enableActiveLevel{enableActiveLevel}
	, addressActiveLevel{addressActiveLevel} {
	assert(addressPins.size() <= 8);
}

bool ParallelMuxInterface::init() {
	if (enablePin != nullptr) {
		enablePin->pinMode(OUTPUT);
		enablePin->digitalWrite(enableActiveLevel);
	}

	for (auto* pin : addressPins) {
		pin->pinMode(OUTPUT);
		pin->digitalWrite(false);
	}

	return true;
}

void ParallelMuxInterface::pinMode(uint8_t mode) { dataPin->pinMode(mode); }

void ParallelMuxInterface::digitalWrite(uint8_t address, uint8_t value) {
	switchTo(address);
	dataPin->digitalWrite(value);
}

int ParallelMuxInterface::digitalRead(uint8_t address) {
	switchTo(address);
	return dataPin->digitalRead();
}

float ParallelMuxInterface::analogRead(uint8_t address) {
	switchTo(address);
	return dataPin->analogRead();
}

void ParallelMuxInterface::switchTo(uint8_t address) {
	assert(address < 1 << addressPins.size());

	if (address == currentAddress) {
		return;
	}

	if (enablePin != nullptr) {
		enablePin->digitalWrite(!enableActiveLevel);
	}

	for (auto* addressPin : addressPins) {
		bool value = address & 0x01;
		address >>= 1;

		addressPin->digitalWrite(value);
	}

	if (enablePin != nullptr) {
		enablePin->digitalWrite(enableActiveLevel);
	}

	currentAddress = address;
	delay(1);
}

}  // namespace SlimeVR
