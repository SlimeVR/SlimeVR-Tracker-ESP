/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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

#include "icm45base.h"
#include "vqf.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 32g
// and gyroscope range at 4000dps
// using high resolution mode
// Uses 32.768kHz clock
// Gyroscope ODR = 409.6Hz, accel ODR = 204.8Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

struct ICM45605 : public ICM45Base {
	static constexpr auto Name = "ICM-45605";
	static constexpr auto Type = SensorTypeID::ICM45605;

	static constexpr VQFParams SensorVQFParams{
		.motionBiasEstEnabled = true,
		.biasSigmaInit = 0.3f,
		.biasClip = 0.6f,
		.restThGyr = 0.3f,
		.restThAcc = 0.0098f,
	};

	ICM45605(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: ICM45Base{registerInterface, logger} {}

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x72;
			static constexpr uint8_t value = 0xe5;
		};
	};

	bool initialize() {
		ICM45Base::softResetIMU();
		return ICM45Base::initializeBase();
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
