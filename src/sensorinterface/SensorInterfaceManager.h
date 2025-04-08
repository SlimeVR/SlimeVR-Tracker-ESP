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

#include <PinInterface.h>

#include <functional>
#include <map>
#include <optional>

#include "ADS111xInterface.h"
#include "ADS111xPin.h"
#include "Adafruit_MCP23X17.h"
#include "DirectPinInterface.h"
#include "I2CPCAInterface.h"
#include "I2CWireSensorInterface.h"
#include "MCP23X17PinInterface.h"
#include "ParallelMuxInterface.h"
#include "SensorInterface.h"
#include "sensorinterface/ParallelMuxPin.h"

namespace SlimeVR {

template <typename T>
static bool operator<(std::initializer_list<T*>& lhs, std::initializer_list<T*>& rhs) {
	size_t minLength = std::min(lhs.size(), rhs.size());
	for (size_t i = 0; i < minLength; i++) {
		if (lhs[i] < rhs[i]) {
			return true;
		}
		if (lhs[i] > rhs[i]) {
			return false;
		}
	}
	return lhs.size() < rhs.size();
}

class SensorInterfaceManager {
private:
	template <typename InterfaceClass, typename... Args>
	struct InterfaceCache {
		explicit InterfaceCache(
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
	inline auto& adsInterface() { return adsInterfaces; }
	inline auto& adsPinInterface() { return adsPinInterfaces; }
	inline auto& parallelMuxInterface() { return parallelMuxInterfaces; }
	inline auto& parallelMuxPinInterface() { return parallelMuxPinInterfaces; }

private:
	InterfaceCache<DirectPinInterface, int> directPinInterfaces{
		[](int pin) { return pin != 255 && pin != -1; }};
	InterfaceCache<MCP23X17PinInterface, Adafruit_MCP23X17*, int> mcpPinInterfaces;
	InterfaceCache<I2CWireSensorInterface, int, int> i2cWireInterfaces;
	InterfaceCache<I2CPCASensorInterface, int, int, int, int> pcaWireInterfaces;
	InterfaceCache<ADS111xInterface, SensorInterface*, PinInterface*, int>
		adsInterfaces;
	InterfaceCache<ADS111xPin, ADS111xInterface*, int> adsPinInterfaces;
	InterfaceCache<
		ParallelMuxInterface,
		PinInterface*,
		std::vector<PinInterface*>,
		PinInterface*,
		bool,
		bool>
		parallelMuxInterfaces;
	InterfaceCache<ParallelMuxPin, ParallelMuxInterface*, uint8_t>
		parallelMuxPinInterfaces;
};

}  // namespace SlimeVR
