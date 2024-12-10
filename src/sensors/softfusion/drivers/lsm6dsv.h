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
// Gyroscope ODR = 480Hz, accel ODR = 120Hz

template <typename I2CImpl>
struct LSM6DSV : LSM6DSOutputHandler<I2CImpl> {
	static constexpr uint8_t Address = 0x6a;
	static constexpr auto Name = "LSM6DSV";
	static constexpr auto Type = ImuID::LSM6DSV;

	static constexpr float GyrFreq = 480;
	static constexpr float AccFreq = 120;
	static constexpr float MagFreq = 120;

	static constexpr float GyrTs = 1.0 / GyrFreq;
	static constexpr float AccTs = 1.0 / AccFreq;
	static constexpr float MagTs = 1.0 / MagFreq;

	static constexpr float GyroSensitivity = 1000 / 35.0f;
	static constexpr float AccelSensitivity = 1000 / 0.244f;

	using LSM6DSOutputHandler<I2CImpl>::i2c;

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x0f;
			static constexpr uint8_t value = 0x70;
		};
		static constexpr uint8_t OutTemp = 0x20;
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
			static constexpr uint8_t value = (0b0011000);  // 480Hz, HAODR
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
			static constexpr uint8_t value = (0b10);  // 8g
		};
		struct FifoCtrl3BDR {
			static constexpr uint8_t reg = 0x09;
			static constexpr uint8_t value
				= (0b1000) | (0b1000 << 4);  // gyro and accel batched at 480Hz
		};
		struct FifoCtrl4Mode {
			static constexpr uint8_t reg = 0x0a;
			static constexpr uint8_t value = (0b110);  // continuous mode
		};

		static constexpr uint8_t FifoStatus = 0x1b;
		static constexpr uint8_t FifoData = 0x78;
	};

	LSM6DSV(I2CImpl i2c, SlimeVR::Logging::Logger& logger)
		: LSM6DSOutputHandler<I2CImpl>(i2c, logger) {}

	bool initialize() {
		// perform initialization step
		i2c.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
		delay(20);
		i2c.writeReg(Regs::HAODRCFG::reg, Regs::HAODRCFG::value);
		i2c.writeReg(Regs::Ctrl1XLODR::reg, Regs::Ctrl1XLODR::value);
		i2c.writeReg(Regs::Ctrl2GODR::reg, Regs::Ctrl2GODR::value);
		i2c.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);
		i2c.writeReg(Regs::Ctrl6GFS::reg, Regs::Ctrl6GFS::value);
		i2c.writeReg(Regs::Ctrl8XLFS::reg, Regs::Ctrl8XLFS::value);
		i2c.writeReg(Regs::FifoCtrl3BDR::reg, Regs::FifoCtrl3BDR::value);
		i2c.writeReg(Regs::FifoCtrl4Mode::reg, Regs::FifoCtrl4Mode::value);
		return true;
	}

	float getDirectTemp() const {
		return LSM6DSOutputHandler<I2CImpl>::template getDirectTemp<Regs>();
	}

	template <typename AccelCall, typename GyroCall>
	void bulkRead(AccelCall&& processAccelSample, GyroCall&& processGyroSample) {
		LSM6DSOutputHandler<I2CImpl>::template bulkRead<AccelCall, GyroCall, Regs>(
			processAccelSample,
			processGyroSample,
			GyrTs,
			AccTs
		);
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers