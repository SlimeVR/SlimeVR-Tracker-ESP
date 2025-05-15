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
#include <memory>

namespace SlimeVR::Sensors {
class ImuAddress {
public:
	ImuAddress(bool isPrimary)
		: m_isPrimary(isPrimary) {}

	ImuAddress(int fixedAddress)
		: m_isDefined(true)
		, m_definedAs(fixedAddress) {}

	bool m_isPrimary = false;
	bool m_isDefined = false;
	uint8_t m_definedAs = 0;

	uint8_t getAddress(uint8_t imuDefaultAddress) {
		if (m_isDefined) {
			return m_definedAs;
		}
		if (m_isPrimary) {
			return imuDefaultAddress;
		}
		return imuDefaultAddress + 1;
	}
};
}  // namespace SlimeVR::Sensors
