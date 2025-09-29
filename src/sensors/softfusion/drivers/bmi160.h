/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 kounocom & SlimeVR Contributors

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
#include <cstring>
#include <limits>

#include "../../../sensorinterface/RegisterInterface.h"
#include "callbacks.h"
#include "vqf.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 4G
// and gyroscope range at 1000DPS
// Gyroscope ODR = 200Hz, accel ODR = 100Hz
// Timestamps reading are not used

// Sensorhub to be implemented

struct BMI160 {
	static constexpr uint8_t Address = 0x68;
	static constexpr auto Name = "BMI160";
	static constexpr auto Type = SensorTypeID::BMI160;

	static constexpr float GyrTs = 1.0 / 200.0;
	static constexpr float AccTs = 1.0 / 100.0;

	static constexpr float MagTs = 1.0 / 100;

	static constexpr float GyroSensitivity = 32.768f;
	static constexpr float AccelSensitivity = 8192.0f;

	static constexpr float TemperatureZROChange
		= 2.0f;  // wow maybe BMI270 isn't that bad actually

	static constexpr VQFParams SensorVQFParams{};

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;

	BMI160(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger) {}

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x00;
			static constexpr std::array<uint8_t, 2> values
				= {0xD1, 0xD3};  // 0xD3 for rev3 (thanks bosch)
		};
		static constexpr uint8_t TempData = 0x20;

		struct Cmd {
			static constexpr uint8_t reg = 0x7E;
			static constexpr uint8_t valueSoftReset = 0xB6;
			static constexpr uint8_t valueFifoFlush = 0xB0;
			static constexpr uint8_t valueAccPowerNormal = 0x11;
			static constexpr uint8_t valueGyrPowerNormal = 0x15;
		};

		struct AccelConf {
			static constexpr uint8_t reg = 0x40;
			static constexpr uint8_t value = 0b0101000;  // 100Hz, filter mode normal
		};

		struct AccelRange {
			static constexpr uint8_t reg = 0x41;
			static constexpr uint8_t value = 0b0101;  // 4G range
		};

		struct GyrConf {
			static constexpr uint8_t reg = 0x42;
			static constexpr uint8_t value = 0b0101001;  // 200Hz, filter mode normal
		};

		struct GyrRange {
			static constexpr uint8_t reg = 0x43;
			static constexpr uint8_t value = 0b001;  // 1000 DPS range
		};

		struct FifoConfig {
			static constexpr uint8_t reg = 0x47;
			static constexpr uint8_t value
				= 0b11010000;  // Gyro and accel data in FIFO, enable FIFO header
		};

		static constexpr uint8_t FifoLength = 0x22;
		static constexpr uint8_t FifoData = 0x24;
		static constexpr uint8_t ErrReg = 0x02;
	};

	struct Fifo {
		static constexpr uint8_t ModeMask = 0b11000000;
		static constexpr uint8_t SkipFrame = 0b01000000;
		static constexpr uint8_t DataFrame = 0b10000000;

		static constexpr uint8_t GyrDataBit = 0b00001000;
		static constexpr uint8_t AccelDataBit = 0b00000100;
	};

	bool initialize() {
		m_RegisterInterface.writeReg(Regs::Cmd::reg, Regs::Cmd::valueSoftReset);
		delay(12);
		m_RegisterInterface.writeReg(Regs::AccelConf::reg, Regs::AccelConf::value);
		delay(1);
		m_RegisterInterface.writeReg(Regs::AccelRange::reg, Regs::AccelRange::value);
		delay(1);
		m_RegisterInterface.writeReg(Regs::GyrConf::reg, Regs::GyrConf::value);
		delay(1);
		m_RegisterInterface.writeReg(Regs::GyrRange::reg, Regs::GyrRange::value);
		delay(1);
		m_RegisterInterface.writeReg(Regs::Cmd::reg, Regs::Cmd::valueAccPowerNormal);
		delay(10);
		m_RegisterInterface.writeReg(Regs::Cmd::reg, Regs::Cmd::valueGyrPowerNormal);
		delay(100);
		m_RegisterInterface.writeReg(Regs::FifoConfig::reg, Regs::FifoConfig::value);
		delay(4);
		m_RegisterInterface.writeReg(Regs::Cmd::reg, Regs::Cmd::valueFifoFlush);
		delay(2);  // delay values ripped straight from old BMI160 driver, could maybe
				   // be lower?

		if (m_RegisterInterface.readReg(Regs::ErrReg) != 0) {
			m_Logger.error(
				"BMI160 error: 0x%x",
				m_RegisterInterface.readReg(Regs::ErrReg)
			);
			return false;
		} else {
			return true;
		}
	}

	float getDirectTemp() const {
		// 0x0 is 23C
		// Resolution is 1/2^9 K / LSB
		constexpr float step = 1 / 512.0f;
		const uint16_t value = m_RegisterInterface.readReg16(Regs::TempData);
		return value * step + 23.0f;
	}

	using FifoBuffer = std::array<uint8_t, RegisterInterface::MaxTransactionLength>;
	FifoBuffer read_buffer;

	template <typename T>  // sorry tailsy I ripped all of this from BMI270 driver
						   // because I don't even want to try and understand the FIFO
						   // format
	inline T getFromFifo(uint32_t& position, FifoBuffer& fifo) {
		T to_ret;
		std::memcpy(&to_ret, &fifo[position], sizeof(T));
		position += sizeof(T);
		return to_ret;
	}

	void bulkRead(DriverCallbacks<int16_t>&& callbacks) {
		const auto fifo_bytes = m_RegisterInterface.readReg16(Regs::FifoLength) & 0x7FF;

		const auto bytes_to_read = std::min(
			static_cast<size_t>(read_buffer.size()),
			static_cast<size_t>(fifo_bytes)
		);
		m_RegisterInterface
			.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());

		for (uint32_t i = 0u; i < bytes_to_read;) {
			const uint8_t header = getFromFifo<uint8_t>(i, read_buffer);
			if ((header & Fifo::ModeMask) == Fifo::SkipFrame) {
				if (i + 1 > bytes_to_read) {
					// incomplete frame, nothing left to process
					break;
				}
				getFromFifo<uint8_t>(i, read_buffer);  // skip 1 byte
			} else if ((header & Fifo::ModeMask) == Fifo::DataFrame) {
				uint8_t gyro_data_length = header & Fifo::GyrDataBit ? 6 : 0;
				uint8_t accel_data_length = header & Fifo::AccelDataBit ? 6 : 0;
				uint8_t required_length = gyro_data_length + accel_data_length;
				if (i + required_length > bytes_to_read) {
					// incomplete frame, will be re-read next time
					break;
				}
				if (header & Fifo::GyrDataBit) {
					int16_t gyro[3];
					gyro[0] = getFromFifo<uint16_t>(i, read_buffer);
					gyro[1] = getFromFifo<uint16_t>(i, read_buffer);
					gyro[2] = getFromFifo<uint16_t>(i, read_buffer);
					callbacks.processGyroSample(gyro, GyrTs);
				}

				if (header & Fifo::AccelDataBit) {
					int16_t accel[3];
					accel[0] = getFromFifo<uint16_t>(i, read_buffer);
					accel[1] = getFromFifo<uint16_t>(i, read_buffer);
					accel[2] = getFromFifo<uint16_t>(i, read_buffer);
					callbacks.processAccelSample(accel, AccTs);
				}
			}
		}
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
