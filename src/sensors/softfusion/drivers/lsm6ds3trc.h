/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Tailsy13 & SlimeVR Contributors

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

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 8g
// and gyroscope range at 1000dps
// Gyroscope ODR = 416Hz, accel ODR = 416Hz

template <typename I2CImpl>
struct LSM6DS3TRC {
	static constexpr uint8_t Address = 0x6a;
	static constexpr auto Name = "LSM6DS3TR-C";
	static constexpr auto Type = ImuID::LSM6DS3TRC;

	static constexpr float Freq = 416;

	static constexpr float GyrTs = 1.0 / Freq;
	static constexpr float AccTs = 1.0 / Freq;
	static constexpr float MagTs = 1.0 / Freq;

	static constexpr float GyroSensitivity = 28.571428571f;
	static constexpr float AccelSensitivity = 4098.360655738f;

	I2CImpl i2c;
	SlimeVR::Logging::Logger logger;
	LSM6DS3TRC(I2CImpl i2c, SlimeVR::Logging::Logger& logger)
		: i2c(i2c)
		, logger(logger) {}

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x0f;
			static constexpr uint8_t value = 0x6a;
		};
		static constexpr uint8_t OutTemp = 0x20;
		struct Ctrl1XL {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value = (0b11 << 2) | (0b0110 << 4);  // 8g, 416Hz
		};
		struct Ctrl2G {
			static constexpr uint8_t reg = 0x11;
			static constexpr uint8_t value
				= (0b10 << 2) | (0b0110 << 4);  // 1000dps, 416Hz
		};
		struct Ctrl3C {
			static constexpr uint8_t reg = 0x12;
			static constexpr uint8_t valueSwReset = 1;
			static constexpr uint8_t value = (1 << 6) | (1 << 2);  // BDU = 1, IF_INC =
																   // 1
		};
		struct FifoCtrl3 {
			static constexpr uint8_t reg = 0x08;
			static constexpr uint8_t value
				= 0b001 | (0b001 << 3);  // accel no decimation, gyro no decimation
		};
		struct FifoCtrl5 {
			static constexpr uint8_t reg = 0x0a;
			static constexpr uint8_t value
				= 0b110 | (0b0111 << 3);  // continuous mode, odr = 833Hz
		};

		static constexpr uint8_t FifoStatus = 0x3a;
		static constexpr uint8_t FifoData = 0x3e;
	};

	bool initialize() {
		// perform initialization step
		i2c.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
		delay(20);
		i2c.writeReg(Regs::Ctrl1XL::reg, Regs::Ctrl1XL::value);
		i2c.writeReg(Regs::Ctrl2G::reg, Regs::Ctrl2G::value);
		i2c.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);
		i2c.writeReg(Regs::FifoCtrl3::reg, Regs::FifoCtrl3::value);
		i2c.writeReg(Regs::FifoCtrl5::reg, Regs::FifoCtrl5::value);
		return true;
	}

	float getDirectTemp() const {
		const auto value = static_cast<int16_t>(i2c.readReg16(Regs::OutTemp));
		float result = ((float)value / 256.0f) + 25.0f;

		return result;
	}

	template <typename AccelCall, typename GyroCall>
	void bulkRead(AccelCall&& processAccelSample, GyroCall&& processGyroSample) {
		const auto read_result = i2c.readReg16(Regs::FifoStatus);
		if (read_result & 0x4000) {  // overrun!
			// disable and re-enable fifo to clear it
			logger.debug("Fifo overrun, resetting...");
			i2c.writeReg(Regs::FifoCtrl5::reg, 0);
			i2c.writeReg(Regs::FifoCtrl5::reg, Regs::FifoCtrl5::value);
			return;
		}
		const auto unread_entries = read_result & 0x7ff;
		constexpr auto single_measurement_words = 6;
		constexpr auto single_measurement_bytes
			= sizeof(uint16_t) * single_measurement_words;

		std::array<int16_t, 60>
			read_buffer;  // max 10 packages of 6 16bit values of data form fifo
		const auto bytes_to_read = std::min(
									   static_cast<size_t>(read_buffer.size()),
									   static_cast<size_t>(unread_entries)
								   )
								 * sizeof(uint16_t) / single_measurement_bytes
								 * single_measurement_bytes;

		i2c.readBytes(
			Regs::FifoData,
			bytes_to_read,
			reinterpret_cast<uint8_t*>(read_buffer.data())
		);
		for (uint16_t i = 0; i < bytes_to_read / sizeof(uint16_t);
			 i += single_measurement_words) {
			processGyroSample(reinterpret_cast<const int16_t*>(&read_buffer[i]), GyrTs);
			processAccelSample(
				reinterpret_cast<const int16_t*>(&read_buffer[i + 3]),
				AccTs
			);
		}
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers