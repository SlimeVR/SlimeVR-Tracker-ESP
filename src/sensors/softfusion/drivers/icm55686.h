/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 Gorbit99 & SlimeVR Contributors

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

#include "../../../sensorinterface/RegisterInterface.h"
#include "callbacks.h"
#include "vqf.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 32g
// and gyroscope range at 4000dps
// using high resolution mode
// Uses 32.768kHz clock
// Gyroscope ODR = 204.8Hz, accel ODR = 102.4Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

struct ICM55686 {
	static constexpr auto Name = "ICM-55686";
	static constexpr auto Type = SensorTypeID::ICM55686;
	static constexpr uint8_t Address = 0x68;

	static constexpr float GyrTs = 1.0 / 204.8;
	static constexpr float AccTs = 1.0 / 102.4;
	static constexpr float TempTs = 1.0 / 409.6;

	static constexpr float MagTs = 1.0 / 100;

	static constexpr float GyroSensitivity = (1 << 19) / 4000.0f;
	static constexpr float AccelSensitivity = (1 << 19) / 32.0f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 128.0f;

	static constexpr float TemperatureZROChange = 20.0f;

	static constexpr VQFParams SensorVQFParams{};

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;
	ICM55686(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: m_RegisterInterface{registerInterface}
		, m_Logger{logger} {}

	struct Regs {
		static constexpr uint8_t TempData = 0x0c;

		struct WhoAmI {
			static constexpr uint8_t reg = 0x72;
			static constexpr uint8_t value = 0x08;
		};

		struct DeviceConfig {
			static constexpr uint8_t reg = 0x7f;
			static constexpr uint8_t valueSwReset = 0b11;
		};

		struct GyroConfig {
			static constexpr uint8_t reg = 0x20;
			static constexpr uint8_t value
				= (0b0000 << 4) | 0b1000;  // 4000dps, odr=204.8Hz
		};

		struct AccelConfig {
			static constexpr uint8_t reg = 0x1f;
			static constexpr uint8_t value
				= (0b000 << 4) | 0b1001;  // 32g, odr = 102.4Hz
		};

		struct FifoConfig0 {
			static constexpr uint8_t reg = 0x21;
			static constexpr uint8_t value
				= (0b10 << 6) | (0b001111);  // stop on full mode, FIFO depth
											 // 8k bytes <-- this disables all APEX
											 // features, but we don't need them
		};

		struct FifoConfig3 {
			static constexpr uint8_t reg = 0x25;
			static constexpr uint8_t value = (0b1 << 0) | (0b1 << 1) | (0b1 << 2)
										   | (0b1 << 3);  // enable FIFO,
														  // enable accel,
														  // enable gyro,
														  // enable hires mode
		};

		struct PwrMgmt0 {
			static constexpr uint8_t reg = 0x14;
			static constexpr uint8_t value
				= (0b11 << 0)
				| (0b11 << 2);  // accel in low noise mode, gyro in low noise
		};

		struct IOCPADScenarioOvrd {
			static constexpr uint8_t reg = 0x35;
			static constexpr uint8_t value = 0b00000110;  // pin 9 to clkin
		};

		struct RtcConfig {
			static constexpr uint8_t reg = 0x2a;
			static constexpr uint8_t value = 0b00100011;  // enable RTC
		};

		static constexpr uint8_t FifoCount = 0x16;
		static constexpr uint8_t FifoData = 0x18;
	};

#pragma pack(push, 1)
	struct FifoEntryAligned {
		uint16_t accel[3];
		uint16_t gyro[3];
		uint16_t temp;
		uint16_t timestamp;
		uint8_t lsb[3];
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;
	static_assert(FullFifoEntrySize == 20);

	bool initialize() {
		m_RegisterInterface.writeReg(
			Regs::DeviceConfig::reg,
			Regs::DeviceConfig::valueSwReset
		);

		delay(35);

#if IMU_USE_EXTERNAL_CLOCK
		m_RegisterInterface.writeReg(
			Regs::IOCPADScenarioOvrd::reg,
			Regs::IOCPADScenarioOvrd::value
		);
		m_RegisterInterface.writeReg(Regs::RtcConfig::reg, Regs::RtcConfig::value);
#endif

		m_RegisterInterface.writeReg(Regs::GyroConfig::reg, Regs::GyroConfig::value);
		m_RegisterInterface.writeReg(Regs::AccelConfig::reg, Regs::AccelConfig::value);
		m_RegisterInterface.writeReg(Regs::FifoConfig0::reg, Regs::FifoConfig0::value);
		m_RegisterInterface.writeReg(Regs::FifoConfig3::reg, Regs::FifoConfig3::value);
		m_RegisterInterface.writeReg(Regs::PwrMgmt0::reg, Regs::PwrMgmt0::value);

		read_buffer.resize(FullFifoEntrySize * MaxReadings);

		delay(1);

		return true;
	}

	static constexpr size_t MaxReadings = 8;
	// Allocate on heap so that it does not take up stack space, which can result in
	// stack overflow and panic
	std::vector<uint8_t> read_buffer;

	bool bulkRead(DriverCallbacks<int32_t>&& callbacks) {
		constexpr auto InvalidReading = static_cast<int16_t>(0x8000);

		size_t fifo_packets
			= __builtin_bswap16(m_RegisterInterface.readReg16(Regs::FifoCount));
		auto packets_to_read = std::min(fifo_packets, MaxReadings);

		size_t bytes_to_read = packets_to_read * FullFifoEntrySize;
		m_RegisterInterface
			.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());

		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			uint8_t header = read_buffer[i];
			bool has_gyro = header & (1 << 5);
			bool has_accel = header & (1 << 6);

			FifoEntryAligned entry;
			memcpy(
				&entry,
				&read_buffer[i + 0x1],
				sizeof(FifoEntryAligned)
			);  // skip fifo header

			int16_t gyro[3] = {
				static_cast<int16_t>(__builtin_bswap16(entry.gyro[0])),
				static_cast<int16_t>(__builtin_bswap16(entry.gyro[1])),
				static_cast<int16_t>(__builtin_bswap16(entry.gyro[2])),
			};
			if (has_gyro && gyro[0] != InvalidReading) {
				const int32_t gyroData[3]{
					static_cast<int32_t>(gyro[0]) << 4 | (entry.lsb[0] & 0xf),
					static_cast<int32_t>(gyro[1]) << 4 | (entry.lsb[1] & 0xf),
					static_cast<int32_t>(gyro[2]) << 4 | (entry.lsb[2] & 0xf),
				};
				callbacks.processGyroSample(gyroData, GyrTs);
			}

			int16_t accel[3] = {
				static_cast<int16_t>(__builtin_bswap16(entry.accel[0])),
				static_cast<int16_t>(__builtin_bswap16(entry.accel[1])),
				static_cast<int16_t>(__builtin_bswap16(entry.accel[2])),
			};
			if (has_accel && accel[0] != InvalidReading) {
				const int32_t accelData[3]{
					static_cast<int32_t>(accel[0]) << 4
						| (static_cast<int32_t>((entry.lsb[0]) & 0xf0) >> 4),
					static_cast<int32_t>(accel[1]) << 4
						| (static_cast<int32_t>((entry.lsb[1]) & 0xf0) >> 4),
					static_cast<int32_t>(accel[2]) << 4
						| (static_cast<int32_t>((entry.lsb[2]) & 0xf0) >> 4),
				};
				callbacks.processAccelSample(accelData, AccTs);
			}

			auto temp = static_cast<int16_t>(__builtin_bswap16(entry.temp));
			if (temp != InvalidReading) {
				callbacks.processTempSample(temp, TempTs);
			}
		}

		return fifo_packets > MaxReadings;
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
