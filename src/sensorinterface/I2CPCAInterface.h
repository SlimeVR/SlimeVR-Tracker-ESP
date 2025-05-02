/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Eiren Rain & SlimeVR Contributors

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
#ifndef I2C_PCA_INTERFACE_H
#define I2C_PCA_INTERFACE_H

#include "I2CWireSensorInterface.h"

namespace SlimeVR {
/**
 * I2C Sensor interface for use with PCA9547 (8-channel I2C-buss multiplexer)
 * or PCA9546A (4-channel I2C-bus multiplexer) or analogs
 */
class I2CPCASensorInterface : public SensorInterface {
public:
	I2CPCASensorInterface(
		uint8_t sclpin,
		uint8_t sdapin,
		uint8_t address,
		uint8_t channel
	)
		: m_Wire(sclpin, sdapin)
		, m_Address(address)
		, m_Channel(channel){};
	~I2CPCASensorInterface(){};

	bool init() override final;
	void swapIn() override final;

	[[nodiscard]] std::string toString() const final {
		using namespace std::string_literals;
		return "PCAWire("s + std::to_string(m_Channel) + ")";
	}

protected:
	I2CWireSensorInterface m_Wire;
	uint8_t m_Address;
	uint8_t m_Channel;
};

}  // namespace SlimeVR

#endif
