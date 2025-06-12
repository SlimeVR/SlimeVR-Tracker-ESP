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
#include <optional>

#include "logging/Logger.h"
#include "sensorinterface/RegisterInterface.h"

namespace SlimeVR::Sensors::SoftFusion {

enum class MagDataWidth {
	SixByte,
	NineByte,
};

struct MagInterface {
	std::function<uint8_t(uint8_t)> readByte;
	std::function<void(uint8_t, uint8_t)> writeByte;
	std::function<void(uint8_t)> setDeviceId;
	std::function<void(uint8_t, MagDataWidth)> startPolling;
	std::function<void()> stopPolling;
};

struct MagDefinition {
	const char* name;

	uint8_t deviceId;

	uint8_t whoAmIReg;
	uint8_t expectedWhoAmI;

	MagDataWidth dataWidth;
	uint8_t dataReg;

	std::function<bool(MagInterface& interface)> setup;
};

class MagDriver {
public:
	bool init(MagInterface&& interface, bool supports9ByteMags);
	void startPolling() const;
	void stopPolling() const;
	[[nodiscard]] const char* getAttachedMagName() const;

private:
	std::optional<MagDefinition> detectedMag;
	MagInterface interface;

	static std::vector<MagDefinition> supportedMags;

	Logging::Logger logger{"MagDriver"};
};

}  // namespace SlimeVR::Sensors::SoftFusion
