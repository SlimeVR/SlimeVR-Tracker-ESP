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

#include "callbacks.h"
#include "vqf.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 8g
// and gyroscope range at 1000dps
// Gyroscope ODR = 200Hz, accel ODR = 100Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

struct ICM42688 {
	static constexpr uint8_t Address = 0x68;
	static constexpr auto Name = "ICM-42688";
	static constexpr auto Type = SensorTypeID::ICM42688;

	static constexpr float GyrTs = 1.0 / 200.0;
	static constexpr float AccTs = 1.0 / 100.0;
	static constexpr float TempTs = 1.0 / 500.0;

	static constexpr float MagTs = 1.0 / 100;

	static constexpr float GyroSensitivity = 32.8f;
	static constexpr float AccelSensitivity = 4096.0f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 2.07f;

	static constexpr float TemperatureZROChange = 20.0f;

	static constexpr VQFParams SensorVQFParams{};

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;
	ICM42688(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger) {}

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x75;
			static constexpr uint8_t value = 0x47;
		};

		struct DeviceConfig {
			static constexpr uint8_t reg = 0x11;
			static constexpr uint8_t valueSwReset = 1;
		};
		struct IntfConfig0 {
			static constexpr uint8_t reg = 0x4c;
			static constexpr uint8_t value
				= (0 << 4) | (0 << 5)
				| (0 << 6);  // fifo count in LE, sensor data in LE, fifo size in bytes
		};
		struct FifoConfig0 {
			static constexpr uint8_t reg = 0x16;
			static constexpr uint8_t value = (0b01 << 6);  // stream to FIFO mode
		};
		struct FifoConfig1 {
			static constexpr uint8_t reg = 0x5f;
			static constexpr uint8_t value
				= 0b1 | (0b1 << 1) | (0b1 << 2)
				| (0b0 << 4);  // fifo accel en=1, gyro=1, temp=1, hires=1
		};
		struct GyroConfig {
			static constexpr uint8_t reg = 0x4f;
			static constexpr uint8_t value
				= (0b001 << 5) | 0b0111;  // 1000dps, odr=200Hz
		};
		struct AccelConfig {
			static constexpr uint8_t reg = 0x50;
			static constexpr uint8_t value = (0b001 << 5) | 0b1000;  // 8g, odr = 100Hz
		};
		struct PwrMgmt {
			static constexpr uint8_t reg = 0x4e;
			static constexpr uint8_t value
				= 0b11 | (0b11 << 2);  // accel in low noise mode, gyro in low noise
		};

		// TODO: might be worth checking
		// GYRO_CONFIG1
		// GYRO_ACCEL_CONFIG0
		// ACCEL_CONFIG1

		static constexpr uint8_t FifoCount = 0x2e;
		static constexpr uint8_t FifoData = 0x30;
	};

#pragma pack(push, 1)
	struct FifoEntryAligned {
		union {
			struct {
				int16_t accel[3];
				int16_t gyro[3];
				uint16_t temp;
				uint16_t timestamp;
				uint8_t xlsb;
				uint8_t ylsb;
				uint8_t zlsb;
			} part;
			uint8_t raw[19];
		};
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	bool initialize() {
		// perform initialization step
		m_RegisterInterface.writeReg(
			Regs::DeviceConfig::reg,
			Regs::DeviceConfig::valueSwReset
		);
		delay(20);

		m_RegisterInterface.writeReg(Regs::IntfConfig0::reg, Regs::IntfConfig0::value);
		m_RegisterInterface.writeReg(Regs::GyroConfig::reg, Regs::GyroConfig::value);
		m_RegisterInterface.writeReg(Regs::AccelConfig::reg, Regs::AccelConfig::value);
		m_RegisterInterface.writeReg(Regs::FifoConfig0::reg, Regs::FifoConfig0::value);
		m_RegisterInterface.writeReg(Regs::FifoConfig1::reg, Regs::FifoConfig1::value);
		m_RegisterInterface.writeReg(Regs::PwrMgmt::reg, Regs::PwrMgmt::value);
		delay(1);

		return true;
	}

	void bulkRead(DriverCallbacks<int32_t>&& callbacks) {
		const auto fifo_bytes = m_RegisterInterface.readReg16(Regs::FifoCount);

		std::array<uint8_t, FullFifoEntrySize * 8> read_buffer;  // max 8 readings
		const auto bytes_to_read = std::min(
									   static_cast<size_t>(read_buffer.size()),
									   static_cast<size_t>(fifo_bytes)
								   )
								 / FullFifoEntrySize * FullFifoEntrySize;
		m_RegisterInterface
			.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());
		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			FifoEntryAligned entry;
			memcpy(
				entry.raw,
				&read_buffer[i + 0x1],
				sizeof(FifoEntryAligned)
			);  // skip fifo header

			const int32_t gyroData[3]{
				static_cast<int32_t>(entry.part.gyro[0]) << 4 | (entry.part.xlsb & 0xf),
				static_cast<int32_t>(entry.part.gyro[1]) << 4 | (entry.part.ylsb & 0xf),
				static_cast<int32_t>(entry.part.gyro[2]) << 4 | (entry.part.zlsb & 0xf),
			};
			callbacks.processGyroSample(gyroData, GyrTs);

			if (entry.part.accel[0] != -32768) {
				const int32_t accelData[3]{
					static_cast<int32_t>(entry.part.accel[0]) << 4
						| (static_cast<int32_t>(entry.part.xlsb) & 0xf0 >> 4),
					static_cast<int32_t>(entry.part.accel[1]) << 4
						| (static_cast<int32_t>(entry.part.ylsb) & 0xf0 >> 4),
					static_cast<int32_t>(entry.part.accel[2]) << 4
						| (static_cast<int32_t>(entry.part.zlsb) & 0xf0 >> 4),
				};
				callbacks.processAccelSample(accelData, AccTs);
			}

			if (entry.part.temp != 0x8000) {
				callbacks.processTempSample(
					static_cast<int16_t>(entry.part.temp),
					TempTs
				);
			}
		}
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
