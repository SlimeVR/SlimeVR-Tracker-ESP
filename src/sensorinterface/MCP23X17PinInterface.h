/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Eiren Rain & SlimeVR contributors

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
#ifndef _H_MCP23X17PinInterface_
#define _H_MCP23X17PinInterface_

#include <Adafruit_MCP23X17.h>
#include <PinInterface.h>

#define MCP_GPA0 0
#define MCP_GPA1 1
#define MCP_GPA2 2
#define MCP_GPA3 3
#define MCP_GPA4 4
#define MCP_GPA5 5
#define MCP_GPA6 6
#define MCP_GPA7 7
#define MCP_GPB0 8
#define MCP_GPB1 9
#define MCP_GPB2 10
#define MCP_GPB3 11
#define MCP_GPB4 12
#define MCP_GPB5 13
#define MCP_GPB6 14
#define MCP_GPB7 15

/**
 * Pin interface to use MCP23008/17 I2C GPIO port extenders
 */
class MCP23X17PinInterface : public PinInterface {
public:
	MCP23X17PinInterface(Adafruit_MCP23X17* mcp, uint8_t pin)
		: _mcp23x17(mcp)
		, _pinNum(pin){};

	int digitalRead() override final;
	void pinMode(uint8_t mode) override final;
	void digitalWrite(uint8_t val) override final;

	[[nodiscard]] std::string toString() const final {
		using namespace std::string_literals;
		return "MCPPin("s + std::to_string(_pinNum) + ")";
	}

private:
	Adafruit_MCP23X17* _mcp23x17;
	uint8_t _pinNum;
};

#endif  // _H_MCP23X17PinInterface_
