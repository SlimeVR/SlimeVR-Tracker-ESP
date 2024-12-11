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

struct I2CImpl {
	static constexpr size_t MaxTransactionLength = I2C_BUFFER_LENGTH - 2;

	I2CImpl(uint8_t devAddr)
		: m_devAddr(devAddr) {}

	uint8_t readReg(uint8_t regAddr) const {
		uint8_t buffer = 0;
		I2Cdev::readByte(m_devAddr, regAddr, &buffer);
		return buffer;
	}

	uint16_t readReg16(uint8_t regAddr) const {
		uint16_t buffer = 0;
		I2Cdev::readBytes(
			m_devAddr,
			regAddr,
			sizeof(buffer),
			reinterpret_cast<uint8_t*>(&buffer)
		);
		return buffer;
	}

	void writeReg(uint8_t regAddr, uint8_t value) const {
		I2Cdev::writeByte(m_devAddr, regAddr, value);
	}

	void writeReg16(uint8_t regAddr, uint16_t value) const {
		I2Cdev::writeBytes(
			m_devAddr,
			regAddr,
			sizeof(value),
			reinterpret_cast<uint8_t*>(&value)
		);
	}

	void readBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const {
		I2Cdev::readBytes(m_devAddr, regAddr, size, buffer);
	}

	void writeBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const {
		I2Cdev::writeBytes(m_devAddr, regAddr, size, buffer);
	}

private:
	uint8_t m_devAddr;
};

}  // namespace SlimeVR::Sensors::SoftFusion
