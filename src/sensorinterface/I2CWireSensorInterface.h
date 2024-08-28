/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2024 Eiren & SlimeVR Contributors

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

#include <Wire.h>
#include "SensorInterface.h"
#include <i2cscan.h>

namespace SlimeVR
{
	void swapI2C(uint8_t sclPin, uint8_t sdaPin);

	class I2CWireSensorInterface : public SensorInterface {
		public:
			I2CWireSensorInterface(uint8_t sdapin, uint8_t sclpin) :
				sdaPin(sdapin), sclPin(sclpin){};
			~I2CWireSensorInterface(){};

			void init() override final {
				I2CSCAN::clearBus(sdaPin, sclPin);
			}
			void swapIn() override final {
				swapI2C(sclPin, sdaPin);
			}
		protected:
			uint8_t sdaPin;
			uint8_t sclPin;
	};


}



#endif // SENSORINTERFACE_I2CWIRE_H
