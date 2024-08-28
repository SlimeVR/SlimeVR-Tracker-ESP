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

#include <PinInterface.h>
#include <Adafruit_MCP23X17.h>

class MCP23X17PinInterface: public PinInterface
{

public:
    MCP23X17PinInterface(Adafruit_MCP23X17* mcp, uint8_t pin)
        : _mcp23x17(mcp), _pinNum(pin) {};

    int digitalRead() override final;
    void pinMode(uint8_t mode) override final;
    void digitalWrite(uint8_t val) override final;

private:
    Adafruit_MCP23X17* _mcp23x17;
    uint8_t _pinNum;
};

#endif // _H_MCP23X17PinInterface_