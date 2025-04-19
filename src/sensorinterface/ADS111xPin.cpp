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
#include "ADS111xPin.h"

#include <cassert>

namespace SlimeVR {

ADS111xPin::ADS111xPin(ADS111xInterface* interface, uint8_t channel)
	: ads111x{interface}
	, channel{channel} {
	assert(channel < 4);

	interface->registerChannel(channel);
}

int ADS111xPin::digitalRead() { return analogRead() >= 0.5f; }
void ADS111xPin::pinMode(uint8_t mode) {}
void ADS111xPin::digitalWrite(uint8_t val) {}

float ADS111xPin::analogRead() { return ads111x->read(channel); }

};  // namespace SlimeVR
