/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Eiren Rain & SlimeVR Contributors

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
#include <SPI.h>

#include <cstdint>

#include "../logging/Logger.h"
#include "RegisterInterface.h"

#define ICM_READ_FLAG 0x80

namespace SlimeVR::Sensors {

struct SPIImpl : public RegisterInterface {
	SPIImpl(SPIClass& spiClass, SPISettings spiSettings, PinInterface* csPin)
		: m_spiClass(spiClass)
		, m_spiSettings(spiSettings)
		, m_csPin(csPin) {
		m_Logger.info(
			Logs::SPIInfo,
			"SPI settings: clock: %d, bit order: 0x%02X, data mode: 0x%02X",
			spiSettings._clock,
			spiSettings._bitOrder,
			spiSettings._dataMode
		);
		csPin->pinMode(OUTPUT);
		csPin->digitalWrite(HIGH);
		spiClass.begin();
	}

	uint8_t readReg(uint8_t regAddr) const override {
		m_spiClass.beginTransaction(m_spiSettings);
		m_csPin->digitalWrite(LOW);

		m_spiClass.transfer(regAddr | ICM_READ_FLAG);
		uint8_t buffer = m_spiClass.transfer(0);

		m_csPin->digitalWrite(HIGH);
		m_spiClass.endTransaction();

		return buffer;
	}

	uint16_t readReg16(uint8_t regAddr) const override {
		m_spiClass.beginTransaction(m_spiSettings);
		m_csPin->digitalWrite(LOW);

		m_spiClass.transfer(regAddr | ICM_READ_FLAG);
		uint8_t b1 = m_spiClass.transfer(0);
		uint8_t b2 = m_spiClass.transfer(0);

		m_csPin->digitalWrite(HIGH);
		m_spiClass.endTransaction();
		return b2 << 8 | b1;
	}

	void writeReg(uint8_t regAddr, uint8_t value) const override {
		m_spiClass.beginTransaction(m_spiSettings);
		m_csPin->digitalWrite(LOW);

		m_spiClass.transfer(regAddr);
		m_spiClass.transfer(value);

		m_csPin->digitalWrite(HIGH);
		m_spiClass.endTransaction();
	}

	void writeReg16(uint8_t regAddr, uint16_t value) const override {
		m_spiClass.beginTransaction(m_spiSettings);
		m_csPin->digitalWrite(LOW);

		m_spiClass.transfer(regAddr);
		m_spiClass.transfer(value & 0xFF);
		m_spiClass.transfer(value >> 8);

		m_csPin->digitalWrite(HIGH);
		m_spiClass.endTransaction();
	}

	void readBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const override {
		m_spiClass.beginTransaction(m_spiSettings);
		m_csPin->digitalWrite(LOW);
		;

		m_spiClass.transfer(regAddr | ICM_READ_FLAG);
		for (uint8_t i = 0; i < size; ++i) {
			buffer[i] = m_spiClass.transfer(0);
		}

		m_csPin->digitalWrite(HIGH);
		m_spiClass.endTransaction();
	}

	void writeBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const override {
		m_spiClass.beginTransaction(m_spiSettings);
		m_csPin->digitalWrite(LOW);

		m_spiClass.transfer(regAddr);
		for (uint8_t i = 0; i < size; ++i) {
			m_spiClass.transfer(buffer[i]);
		}

		m_csPin->digitalWrite(HIGH);
		m_spiClass.endTransaction();
	}

	bool hasSensorOnBus() override {
		return true;  // TODO
	}

	uint8_t getAddress() const override { return 0; }

	std::string toString() const override { return std::string("SPI"); }

private:
	SPIClass& m_spiClass;
	SPISettings m_spiSettings;
	PinInterface* m_csPin;

	enum class Logs {
		SPIInfo = 0,
	};

	SlimeVR::Logging::Logger<Logs> m_Logger{"SPI", "spi"};
};

}  // namespace SlimeVR::Sensors
