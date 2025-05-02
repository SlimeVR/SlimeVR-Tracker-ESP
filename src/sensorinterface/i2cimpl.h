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

#include <i2cscan.h>

#include <cstdint>

#include "I2Cdev.h"
#include "RegisterInterface.h"

namespace SlimeVR::Sensors {

struct I2CImpl : public RegisterInterface {
	I2CImpl(uint8_t devAddr)
		: m_devAddr(devAddr) {}

	uint8_t readReg(uint8_t regAddr) const override {
		uint8_t buffer = 0;
		I2Cdev::readByte(m_devAddr, regAddr, &buffer);
		return buffer;
	}

	uint16_t readReg16(uint8_t regAddr) const override {
		uint16_t buffer = 0;
		I2Cdev::readBytes(
			m_devAddr,
			regAddr,
			sizeof(buffer),
			reinterpret_cast<uint8_t*>(&buffer)
		);
		return buffer;
	}

	void writeReg(uint8_t regAddr, uint8_t value) const override {
		I2Cdev::writeByte(m_devAddr, regAddr, value);
	}

	void writeReg16(uint8_t regAddr, uint16_t value) const override {
		I2Cdev::writeBytes(
			m_devAddr,
			regAddr,
			sizeof(value),
			reinterpret_cast<uint8_t*>(&value)
		);
	}

	void readBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const override {
		I2Cdev::readBytes(m_devAddr, regAddr, size, buffer);
	}

	void writeBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const override {
		I2Cdev::writeBytes(m_devAddr, regAddr, size, buffer);
	}

	bool hasSensorOnBus() {
		// Ask twice, because we're nice like this
		return I2CSCAN::hasDevOnBus(m_devAddr) || I2CSCAN::hasDevOnBus(m_devAddr);
	}

	uint8_t getAddress() const override { return m_devAddr; }

	std::string toString() const {
		return std::string("I2C(") + std::to_string(m_devAddr) + std::string(")");
	}

private:
	uint8_t m_devAddr;
};

}  // namespace SlimeVR::Sensors
