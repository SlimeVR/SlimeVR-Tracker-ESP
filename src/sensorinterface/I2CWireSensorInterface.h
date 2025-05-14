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

#ifndef SENSORINTERFACE_I2CWIRE_H
#define SENSORINTERFACE_I2CWIRE_H

#include <Arduino.h>
#include <i2cscan.h>

#include "SensorInterface.h"
#include "globals.h"

namespace SlimeVR {
void swapI2C(uint8_t sclPin, uint8_t sdaPin);
void disconnectI2C();

/**
 * I2C Sensor interface using direct arduino Wire on provided pins
 *
 */
class I2CWireSensorInterface : public SensorInterface {
public:
	I2CWireSensorInterface(uint8_t sclpin, uint8_t sdapin)
		: _sdaPin(sdapin)
		, _sclPin(sclpin){};
	~I2CWireSensorInterface(){};

	bool init() override final { return true; }
	void swapIn() override final { swapI2C(_sclPin, _sdaPin); }
	void disconnect() { disconnectI2C(); }

	[[nodiscard]] std::string toString() const final {
		using namespace std::string_literals;
		return "Wire("s + std::to_string(_sclPin) + ": " + std::to_string(_sdaPin)
			 + ")"s;
	}

protected:
	uint8_t _sdaPin;
	uint8_t _sclPin;
};

}  // namespace SlimeVR

#endif  // SENSORINTERFACE_I2CWIRE_H
