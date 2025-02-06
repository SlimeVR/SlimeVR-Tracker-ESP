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

#include "I2CWireSensorInterface.h"

#if ESP32
#include "driver/i2c.h"
#endif

uint8_t activeSCLPin;
uint8_t activeSDAPin;
bool isI2CActive = false;

namespace SlimeVR {
void swapI2C(uint8_t sclPin, uint8_t sdaPin) {
	if (sclPin != activeSCLPin || sdaPin != activeSDAPin || !isI2CActive) {
		Wire.flush();
#if ESP32
		if (!isI2CActive) {
			// Reset HWI2C to avoid being affected by I2CBUS reset
			Wire.end();
		}
		// Disconnect pins from HWI2C
		gpio_set_direction((gpio_num_t)activeSCLPin, GPIO_MODE_INPUT);
		gpio_set_direction((gpio_num_t)activeSDAPin, GPIO_MODE_INPUT);

		if (isI2CActive) {
			i2c_set_pin(I2C_NUM_0, sdaPin, sclPin, false, false, I2C_MODE_MASTER);
		} else {
			Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin), I2C_SPEED);
			Wire.setTimeOut(150);
		}
#else
		Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin));
#endif

		activeSCLPin = sclPin;
		activeSDAPin = sdaPin;
		isI2CActive = true;
	}
}

void disconnectI2C() {
	Wire.flush();
	isI2CActive = false;
#if ESP32
	Wire.end();
#endif
}
}  // namespace SlimeVR
