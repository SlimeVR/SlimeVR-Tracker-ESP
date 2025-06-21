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
// Gyroscope ODR = 409.6Hz, accel ODR = 102.4Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

struct ICM45686 : public ICM45Base {
	static constexpr auto Name = "ICM-45686";
	static constexpr auto Type = SensorTypeID::ICM45686;

	static constexpr VQFParams SensorVQFParams{
		.tauAcc = 7.171490,
		.biasSigmaInit = 0.337976,
		.biasForgettingTime = 352.235500,
		.biasClip = 5.0,
		.biasSigmaMotion = 0.985346,
		.biasVerticalForgettingFactor = 0.007959,
		.biasSigmaRest = 0.028897,
		.restMinT = 4.648680,
		.restFilterTau = 1.900166,
		.restThGyr = 2.620598,
		.restThAcc = 2.142593,
	};

	ICM45686(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: ICM45Base{registerInterface, logger} {}

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x72;
			static constexpr uint8_t value = 0xe9;
		};

		struct Pin9Config {
			static constexpr uint8_t reg = 0x31;
			static constexpr uint8_t value = 0b00000110;  // pin 9 to clkin
		};

		struct RtcConfig {
			static constexpr uint8_t reg = 0x26;
			static constexpr uint8_t value = 0b00100011;  // enable RTC
		};
	};

	bool initialize() {
		ICM45Base::softResetIMU();
		m_RegisterInterface.writeReg(Regs::Pin9Config::reg, Regs::Pin9Config::value);
		m_RegisterInterface.writeReg(Regs::RtcConfig::reg, Regs::RtcConfig::value);
		return ICM45Base::initializeBase();
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
