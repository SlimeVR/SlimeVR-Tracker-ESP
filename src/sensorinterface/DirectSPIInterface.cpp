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

#include "DirectSPIInterface.h"

#include <Arduino.h>
#include <PinInterface.h>

namespace SlimeVR {

DirectSPIInterface::DirectSPIInterface(
	SPIClass* spiClass,
	SPISettings spiSettings,
	uint8_t sck,
	uint8_t miso,
	uint8_t mosi
)
	: m_spiClass{spiClass}
	, m_spiSettings{spiSettings}
	, m_sck{sck}
	, m_miso{miso}
	, m_mosi{mosi} {}

bool DirectSPIInterface::init() {
	const bool customPinsConfigured = m_sck != 255 && m_miso != 255 && m_mosi != 255;
#if defined(ESP32)
	// To make sure SCK, MISO, MOSI pins have already defined
	if (customPinsConfigured) {
		// SPIClass::begin() requires int8_t
		// for kepping uint_8 style, doing transform here
		m_spiClass->begin((int8_t)m_sck, (int8_t)m_miso, (int8_t)m_mosi);
	} else {
		// or use the default pin defines
		m_spiClass->begin();
	}

#elif defined(ESP8266)
	if (customPinsConfigured) {
		const bool overlapPins = m_sck == 6 && m_miso == 7 && m_mosi == 8;
		// SPIClass::pins() requires an SS argument even though SlimeVR manages CS
		// separately for each sensor through PinInterface. ESP8266 overlap mode
		// specifically requires GPIO0, while the standard HSPI pin set uses the
		// core-defined SS pin (GPIO15).
		const int8_t hardwareSs = overlapPins ? 0 : static_cast<int8_t>(SS);

		if (!m_spiClass->pins(
				static_cast<int8_t>(m_sck),
				static_cast<int8_t>(m_miso),
				static_cast<int8_t>(m_mosi),
				hardwareSs
			)) {
			return false;
		}
	}

	m_spiClass->begin();

	// controls each sensor's CS through PinInterface,
	// turn off the Hardware cs control
	m_spiClass->setHwCs(false);
#else
	m_spiClass->begin();
#endif
	return true;
}

void DirectSPIInterface::swapIn() {}

void DirectSPIInterface::beginTransaction(PinInterface* csPin) {
	m_spiClass->beginTransaction(m_spiSettings);
	csPin->digitalWrite(LOW);
}

void DirectSPIInterface::endTransaction(PinInterface* csPin) {
	csPin->digitalWrite(HIGH);
	m_spiClass->endTransaction();
}

const SPISettings& DirectSPIInterface::getSpiSettings() { return m_spiSettings; }

}  // namespace SlimeVR
