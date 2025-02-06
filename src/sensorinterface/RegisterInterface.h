/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Tailsy13 & SlimeVR Contributors

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

#include <cstdint>

#include "I2Cdev.h"

namespace SlimeVR::Sensors::SoftFusion {

struct RegisterInterface {
	static constexpr size_t MaxTransactionLength = I2C_BUFFER_LENGTH - 2;

	virtual uint8_t readReg(uint8_t regAddr) const = 0;
	virtual uint16_t readReg16(uint8_t regAddr) const = 0;
	virtual void writeReg(uint8_t regAddr, uint8_t value) const = 0;
	virtual void writeReg16(uint8_t regAddr, uint16_t value) const = 0;
	virtual void readBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const = 0;
	virtual void writeBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const = 0;
	virtual uint8_t getAddress() const = 0;
	virtual bool hasSensorOnBus() = 0;
};

}  // namespace SlimeVR::Sensors::SoftFusion
