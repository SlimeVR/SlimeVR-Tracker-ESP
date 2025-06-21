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
#include "DirectSPIInterface.h"
#include "RegisterInterface.h"

#define ICM_READ_FLAG 0x80

namespace SlimeVR::Sensors {

struct SPIImpl : public RegisterInterface {
	SPIImpl(DirectSPIInterface* spi, PinInterface* csPin)
		: m_spi(spi)
		, m_csPin(csPin) {
		auto& spiSettings = spi->getSpiSettings();
		m_Logger.info(
			"SPI settings: clock: %d, bit order: 0x%02X, data mode: 0x%02X",
			spiSettings._clock,
			spiSettings._bitOrder,
			spiSettings._dataMode
		);
		csPin->pinMode(OUTPUT);
		csPin->digitalWrite(HIGH);
	}

	uint8_t readReg(uint8_t regAddr) const override {
		m_spi->beginTransaction(m_csPin);

		m_spi->transfer(regAddr | ICM_READ_FLAG);
		uint8_t buffer = m_spi->transfer(0);

		m_spi->endTransaction(m_csPin);

		return buffer;
	}

	uint16_t readReg16(uint8_t regAddr) const override {
		m_spi->beginTransaction(m_csPin);

		m_spi->transfer(regAddr | ICM_READ_FLAG);
		uint8_t b1 = m_spi->transfer(0);
		uint8_t b2 = m_spi->transfer(0);

		m_spi->endTransaction(m_csPin);
		return b2 << 8 | b1;
	}

	void writeReg(uint8_t regAddr, uint8_t value) const override {
		m_spi->beginTransaction(m_csPin);

		m_spi->transfer(regAddr);
		m_spi->transfer(value);

		m_spi->endTransaction(m_csPin);
	}

	void writeReg16(uint8_t regAddr, uint16_t value) const override {
		m_spi->beginTransaction(m_csPin);

		m_spi->transfer(regAddr);
		m_spi->transfer(value & 0xFF);
		m_spi->transfer(value >> 8);

		m_spi->endTransaction(m_csPin);
	}

	void readBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const override {
		m_spi->beginTransaction(m_csPin);

		m_spi->transfer(regAddr | ICM_READ_FLAG);
		for (uint8_t i = 0; i < size; ++i) {
			buffer[i] = m_spi->transfer(0);
		}

		m_spi->endTransaction(m_csPin);
	}

	void writeBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer) const override {
		m_spi->beginTransaction(m_csPin);

		m_spi->transfer(regAddr);
		for (uint8_t i = 0; i < size; ++i) {
			m_spi->transfer(buffer[i]);
		}

		m_spi->endTransaction(m_csPin);
	}

	bool hasSensorOnBus() override {
		return true;  // TODO
	}

	uint8_t getAddress() const override { return 0; }

	std::string toString() const override { return std::string("SPI"); }

private:
	DirectSPIInterface* m_spi;
	PinInterface* m_csPin;
	SlimeVR::Logging::Logger m_Logger = SlimeVR::Logging::Logger("SPI");
};

}  // namespace SlimeVR::Sensors
