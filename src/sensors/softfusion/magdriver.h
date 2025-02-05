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

#include <cstdint>
#include <functional>
#include <vector>

#include "../../logging/Logger.h"

namespace SlimeVR::Sensors::SoftFusion {

using I2CWriteFunc = std::function<void(uint8_t, uint8_t)>;
using I2CReadFunc = std::function<uint8_t(uint8_t)>;
using I2CSetIdFunc = std::function<void(uint8_t)>;

struct MagDefinition {
	enum class DataWidth {
		SixByte,
		NineByte,
	};

	const char* name;

	uint8_t deviceId;

	uint8_t whoAmIReg;
	uint8_t expectedWhoAmI;

	std::function<void(const I2CWriteFunc&)> setup;

	uint8_t dataReg;
	DataWidth dataWidth;

	float resolution;
};

class MagDriver {
public:
	void init(
		const I2CSetIdFunc& setId,
		const I2CReadFunc& readI2C,
		const I2CWriteFunc& writeI2C
	);
	void scaleMagSample(const uint8_t* rawData, float outData[3]);

private:
	static std::vector<MagDefinition> mags;
	enum class State {
		NotSetup,
		Ok,
		Error,
	};

	State state = State::NotSetup;
	MagDefinition selectedMag;
	Logging::Logger m_Logger = Logging::Logger("MagDriver");
};

};  // namespace SlimeVR::Sensors::SoftFusion
