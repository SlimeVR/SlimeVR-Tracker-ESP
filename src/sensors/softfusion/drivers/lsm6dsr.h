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

#include <algorithm>
#include <array>
#include <cstdint>

#include "lsm6ds-common.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 8g
// and gyroscope range at 1000dps
// Gyroscope ODR = 416Hz, accel ODR = 104Hz

struct LSM6DSR : LSM6DSOutputHandler {
	static constexpr uint8_t Address = 0x6a;
	static constexpr auto Name = "LSM6DSR";
	static constexpr auto Type = SensorTypeID::LSM6DSR;

	static constexpr float GyrFreq = 416;
	static constexpr float AccFreq = 104;
	static constexpr float MagFreq = 120;
	static constexpr float TempFreq = 52;

	static constexpr float GyrTs = 1.0 / GyrFreq;
	static constexpr float AccTs = 1.0 / AccFreq;
	static constexpr float MagTs = 1.0 / MagFreq;
	static constexpr float TempTs = 1.0 / TempFreq;

	static constexpr float GyroSensitivity = 1000 / 35.0f;
	static constexpr float AccelSensitivity = 1000 / 0.244f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 256.0f;

	static constexpr float TemperatureZROChange = 20.0f;

	static constexpr VQFParams SensorVQFParams{
		.motionBiasEstEnabled = true,
		.biasSigmaInit = 1.0f,
		.biasClip = 2.0f,
		.restThGyr = 1.0f,
		.restThAcc = 0.192f,
	};

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x0f;
			static constexpr uint8_t value = 0x6b;
		};
		struct Ctrl1XL {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value = (0b01001100);  // XL at 104 Hz, 8g FS
		};
		struct Ctrl2GY {
			static constexpr uint8_t reg = 0x11;
			static constexpr uint8_t value = (0b01101000);  // GY at 416 Hz, 1000dps FS
		};
		struct Ctrl3C {
			static constexpr uint8_t reg = 0x12;
			static constexpr uint8_t valueSwReset = 1;
			static constexpr uint8_t value = (1 << 6) | (1 << 2);  // BDU = 1, IF_INC =
																   // 1
		};
		struct FifoCtrl3BDR {
			static constexpr uint8_t reg = 0x09;
			static constexpr uint8_t value
				= (0b0110) | (0b0110 << 4);  // gyro and accel batched at 417Hz
		};
		struct FifoCtrl4Mode {
			static constexpr uint8_t reg = 0x0a;
			static constexpr uint8_t value = (0b110110);  // continuous mode,
														  // temperature at 52Hz
		};

		static constexpr uint8_t FifoStatus = 0x3a;
		static constexpr uint8_t FifoData = 0x78;
	};

	LSM6DSR(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: LSM6DSOutputHandler(registerInterface, logger) {}

	bool initialize() {
		// perform initialization step
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
		delay(20);
		m_RegisterInterface.writeReg(Regs::Ctrl1XL::reg, Regs::Ctrl1XL::value);
		m_RegisterInterface.writeReg(Regs::Ctrl2GY::reg, Regs::Ctrl2GY::value);
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);
		m_RegisterInterface.writeReg(
			Regs::FifoCtrl3BDR::reg,
			Regs::FifoCtrl3BDR::value
		);
		m_RegisterInterface.writeReg(
			Regs::FifoCtrl4Mode::reg,
			Regs::FifoCtrl4Mode::value
		);
		return true;
	}

	template <typename AccelCall, typename GyroCall, typename TempCall>
	void bulkRead(
		AccelCall&& processAccelSample,
		GyroCall&& processGyroSample,
		TempCall&& processTempSample
	) {
		LSM6DSOutputHandler::template bulkRead<AccelCall, GyroCall, TempCall, Regs>(
			processAccelSample,
			processGyroSample,
			processTempSample,
			GyrTs,
			AccTs,
			TempTs
		);
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
