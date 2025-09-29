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
#include "vqf.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 4g
// and gyroscope range at 1000dps
// Gyroscope ODR = 240Hz, accel ODR = 120Hz

struct LSM6DSV : LSM6DSOutputHandler {
	static constexpr uint8_t Address = 0x6a;
	static constexpr auto Name = "LSM6DSV";
	static constexpr auto Type = SensorTypeID::LSM6DSV;

	static constexpr float GyrFreq = 240;
	static constexpr float AccFreq = 120;
	static constexpr float MagFreq = 120;
	static constexpr float TempFreq = 60;

	static constexpr float GyrTs = 1.0 / GyrFreq;
	static constexpr float AccTs = 1.0 / AccFreq;
	static constexpr float MagTs = 1.0 / MagFreq;
	static constexpr float TempTs = 1.0 / TempFreq;

	static constexpr float GyroSensitivity = 1000 / 35.0f;
	static constexpr float AccelSensitivity = 1000 / 0.244f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 256.0f;

	static constexpr float TemperatureZROChange = 16.667f;

	static constexpr VQFParams SensorVQFParams{};

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x0f;
			static constexpr uint8_t value = 0x70;
		};
		struct HAODRCFG {
			static constexpr uint8_t reg = 0x62;
			static constexpr uint8_t value = (0b00);  // 1st ODR table
		};
		struct Ctrl1XLODR {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value = (0b0010110);  // 120Hz, HAODR
		};
		struct Ctrl2GODR {
			static constexpr uint8_t reg = 0x11;
			static constexpr uint8_t value = (0b0010111);  // 240Hz, HAODR
		};
		struct Ctrl3C {
			static constexpr uint8_t reg = 0x12;
			static constexpr uint8_t valueSwReset = 1;
			static constexpr uint8_t value = (1 << 6) | (1 << 2);  // BDU = 1, IF_INC =
																   // 1
		};
		struct Ctrl6GFS {
			static constexpr uint8_t reg = 0x15;
			static constexpr uint8_t value = (0b0011);  // 1000dps
		};
		struct Ctrl8XLFS {
			static constexpr uint8_t reg = 0x17;
			static constexpr uint8_t value = (0b01);  // 4g
		};
		struct FifoCtrl3BDR {
			static constexpr uint8_t reg = 0x09;
			static constexpr uint8_t value
				= 0b01110110;  // Gyroscope batched into FIFO at 240Hz, Accel at 120Hz
		};
		struct FifoCtrl4Mode {
			static constexpr uint8_t reg = 0x0a;
			static constexpr uint8_t value = (0b110110);  // continuous mode,
														  // temperature at 60Hz
		};

		static constexpr uint8_t FifoStatus = 0x1b;
		static constexpr uint8_t FifoData = 0x78;
	};

	LSM6DSV(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: LSM6DSOutputHandler(registerInterface, logger) {}

	bool initialize() {
		// perform initialization step
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
		delay(20);
		m_RegisterInterface.writeReg(Regs::HAODRCFG::reg, Regs::HAODRCFG::value);
		m_RegisterInterface.writeReg(Regs::Ctrl1XLODR::reg, Regs::Ctrl1XLODR::value);
		m_RegisterInterface.writeReg(Regs::Ctrl2GODR::reg, Regs::Ctrl2GODR::value);
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);
		m_RegisterInterface.writeReg(Regs::Ctrl6GFS::reg, Regs::Ctrl6GFS::value);
		m_RegisterInterface.writeReg(Regs::Ctrl8XLFS::reg, Regs::Ctrl8XLFS::value);
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

	void bulkRead(DriverCallbacks<int16_t>&& callbacks) {
		LSM6DSOutputHandler::template bulkRead<Regs>(
			std::move(callbacks),
			GyrTs,
			AccTs,
			TempTs
		);
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
