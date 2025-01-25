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

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 32g
// and gyroscope range at 4000dps
// using high resolution mode
// Uses 32.768kHz clock
// Gyroscope ODR = 409.6Hz, accel ODR = 204.8Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

template <typename I2CImpl>
struct ICM45Base {
	static constexpr uint8_t Address = 0x68;

	static constexpr float GyrTs = 1.0 / 409.6;
	static constexpr float AccTs = 1.0 / 204.8;
	static constexpr float TempTs = 1.0 / 409.6;

	static constexpr float MagTs = 1.0 / 100;

	static constexpr float GyroSensitivity = 131.072f;
	static constexpr float AccelSensitivity = 16384.0f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 128.0f;

	static constexpr bool Uses32BitSensorData = true;

	I2CImpl i2c;
	SlimeVR::Logging::Logger& logger;
	ICM45Base(I2CImpl i2c, SlimeVR::Logging::Logger& logger)
		: i2c(i2c)
		, logger(logger) {}

	struct BaseRegs {
		static constexpr uint8_t TempData = 0x0c;

		struct DeviceConfig {
			static constexpr uint8_t reg = 0x7f;
			static constexpr uint8_t valueSwReset = 0b11;
		};

		struct GyroConfig {
			static constexpr uint8_t reg = 0x1c;
			static constexpr uint8_t value
				= (0b0000 << 4) | 0b0111;  // 4000dps, odr=409.6Hz
		};

		struct AccelConfig {
			static constexpr uint8_t reg = 0x1b;
			static constexpr uint8_t value
				= (0b000 << 4) | 0b1000;  // 32g, odr = 204.8Hz
		};

		struct FifoConfig0 {
			static constexpr uint8_t reg = 0x1d;
			static constexpr uint8_t value
				= (0b01 << 6) | (0b011111);  // stream to FIFO mode, FIFO depth
											 // 8k bytes <-- this disables all APEX
											 // features, but we don't need them
		};

		struct FifoConfig3 {
			static constexpr uint8_t reg = 0x21;
			static constexpr uint8_t value = (0b1 << 0) | (0b1 << 1) | (0b1 << 2)
										   | (0b1 << 3);  // enable FIFO,
														  // enable accel,
														  // enable gyro,
														  // enable hires mode
		};

		struct PwrMgmt0 {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value
				= 0b11 | (0b11 << 2);  // accel in low noise mode, gyro in low noise
		};

		static constexpr uint8_t FifoCount = 0x12;
		static constexpr uint8_t FifoData = 0x14;
	};

#pragma pack(push, 1)
	struct FifoEntryAligned {
		union {
			struct {
				int16_t accel[3];
				int16_t gyro[3];
				uint16_t temp;
				uint16_t timestamp;
				uint8_t lsb[3];
			} part;
			uint8_t raw[19];
		};
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	void softResetIMU() {
		i2c.writeReg(BaseRegs::DeviceConfig::reg, BaseRegs::DeviceConfig::valueSwReset);
		delay(35);
	}

	bool initializeBase() {
		// perform initialization step
		i2c.writeReg(BaseRegs::GyroConfig::reg, BaseRegs::GyroConfig::value);
		i2c.writeReg(BaseRegs::AccelConfig::reg, BaseRegs::AccelConfig::value);
		i2c.writeReg(BaseRegs::FifoConfig0::reg, BaseRegs::FifoConfig0::value);
		i2c.writeReg(BaseRegs::FifoConfig3::reg, BaseRegs::FifoConfig3::value);
		i2c.writeReg(BaseRegs::PwrMgmt0::reg, BaseRegs::PwrMgmt0::value);
		delay(1);

		return true;
	}

	float getDirectTemp() const {
		const auto value = static_cast<int16_t>(i2c.readReg16(BaseRegs::TempData));
		float result = ((float)value / 132.48f) + 25.0f;
		return result;
	}

	template <typename AccelCall, typename GyroCall>
	void bulkRead(AccelCall&& processAccelSample, GyroCall&& processGyroSample) {
		const auto fifo_packets = i2c.readReg16(BaseRegs::FifoCount);
		const auto fifo_bytes = fifo_packets * sizeof(FullFifoEntrySize);

		std::array<uint8_t, FullFifoEntrySize * 8> read_buffer;  // max 8 readings
		const auto bytes_to_read = std::min(
									   static_cast<size_t>(read_buffer.size()),
									   static_cast<size_t>(fifo_bytes)
								   )
								 / FullFifoEntrySize * FullFifoEntrySize;
		i2c.readBytes(BaseRegs::FifoData, bytes_to_read, read_buffer.data());
		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			FifoEntryAligned entry;
			memcpy(
				entry.raw,
				&read_buffer[i + 0x1],
				sizeof(FifoEntryAligned)
			);  // skip fifo header
			const int32_t gyroData[3]{
				static_cast<int32_t>(entry.part.gyro[0]) << 4
					| (entry.part.lsb[0] & 0xf),
				static_cast<int32_t>(entry.part.gyro[1]) << 4
					| (entry.part.lsb[1] & 0xf),
				static_cast<int32_t>(entry.part.gyro[2]) << 4
					| (entry.part.lsb[2] & 0xf),
			};
			processGyroSample(gyroData, GyrTs);

			if (entry.part.accel[0] != -32768) {
				const int32_t accelData[3]{
					static_cast<int32_t>(entry.part.accel[0]) << 4
						| (static_cast<int32_t>(entry.part.lsb[0]) & 0xf0 >> 4),
					static_cast<int32_t>(entry.part.accel[1]) << 4
						| (static_cast<int32_t>(entry.part.lsb[1]) & 0xf0 >> 4),
					static_cast<int32_t>(entry.part.accel[2]) << 4
						| (static_cast<int32_t>(entry.part.lsb[2]) & 0xf0 >> 4),
				};
				processAccelSample(accelData, AccTs);
			}
		}
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
