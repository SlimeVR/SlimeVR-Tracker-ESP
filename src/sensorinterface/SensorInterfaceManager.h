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

#pragma once

#include <functional>
#include <map>
#include <optional>

#include "DirectPinInterface.h"
#include "I2CPCAInterface.h"
#include "I2CWireSensorInterface.h"
#include "MCP23X17PinInterface.h"

namespace SlimeVR {

class SensorInterfaceManager {
private:
	template <typename InterfaceClass, typename... Args>
	struct SensorInterface {
		explicit SensorInterface(
			std::function<bool(Args...)> validate = [](Args...) { return true; }
		)
			: validate{validate} {}

		InterfaceClass* get(Args... args) {
			if (validate && !validate(args...)) {
				return static_cast<InterfaceClass*>(nullptr);
			}

			auto key = std::make_tuple(args...);

			if (!cache.contains(key)) {
				auto ptr = new InterfaceClass(args...);
				if (!ptr->init()) {
					cache[key] = nullptr;
					return nullptr;
				}

				cache[key] = ptr;
			}
			return cache[key];
		}

	private:
		std::map<std::tuple<Args...>, InterfaceClass*> cache;
		std::function<bool(Args...)> validate;
	};

public:
	inline auto& directPinInterface() { return directPinInterfaces; }
	inline auto& mcpPinInterface() { return mcpPinInterfaces; }
	inline auto& i2cWireInterface() { return i2cWireInterfaces; }
	inline auto& pcaWireInterface() { return pcaWireInterfaces; }

private:
	SensorInterface<DirectPinInterface, int> directPinInterfaces{[](int pin) {
		return pin != 255 && pin != -1;
	}};
	SensorInterface<MCP23X17PinInterface, Adafruit_MCP23X17*, int> mcpPinInterfaces;
	SensorInterface<I2CWireSensorInterface, int, int> i2cWireInterfaces;
	SensorInterface<I2CPCASensorInterface, int, int, int, int> pcaWireInterfaces;
};

}  // namespace SlimeVR
