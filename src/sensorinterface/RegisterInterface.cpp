/* SlimeVR Code is placed under the MIT license
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

#include "RegisterInterface.h"

namespace SlimeVR::Sensors {

[[nodiscard]] uint8_t EmptyRegisterInterface::readReg(uint8_t regAddr) const {
	return 0;
};
[[nodiscard]] uint16_t EmptyRegisterInterface::readReg16(uint8_t regAddr) const {
	return 0;
};
void EmptyRegisterInterface::writeReg(uint8_t regAddr, uint8_t value) const {};
void EmptyRegisterInterface::writeReg16(uint8_t regAddr, uint16_t value) const {};
void EmptyRegisterInterface::readBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer)
	const {};
void EmptyRegisterInterface::writeBytes(uint8_t regAddr, uint8_t size, uint8_t* buffer)
	const {};
[[nodiscard]] uint8_t EmptyRegisterInterface::getAddress() const { return 0; };
bool EmptyRegisterInterface::hasSensorOnBus() { return true; };
[[nodiscard]] std::string EmptyRegisterInterface::toString() const { return "Empty"; };

EmptyRegisterInterface EmptyRegisterInterface::instance;

}  // namespace SlimeVR::Sensors
