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
struct ICM45686 {
	static constexpr uint8_t Address = 0x68;
	static constexpr auto Name = "ICM-45688";
	static constexpr auto Type = ImuID::ICM45686;

	static constexpr float GyrTs = 1.0 / 409.6;
	static constexpr float AccTs = 1.0 / 204.8;
	static constexpr float TempTs = 1.0 / 409.6;

	static constexpr float MagTs = 1.0 / 100;

	static constexpr float GyroSensitivity = 131.072f;
	static constexpr float AccelSensitivity = 16384.0f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 128.0f;

	static constexpr bool Uses32BitSensorData = true;

	// Temperature stability constant - how many degrees of temperature for the bias to
	// change by 0.01 Though I don't know if it should be 0.1 or 0.01, this is a guess
	// and seems to work better than 0.1
	static constexpr float TemperatureZROChange = 2.0f;

	// VQF parameters
	// biasSigmaInit and and restThGyr should be the sensor's typical gyro bias
	// biasClip should be 2x the sensor's typical gyro bias
	// restThAcc should be the sensor's typical acceleration bias
	static constexpr VQFParams SensorVQFParams{
		.motionBiasEstEnabled = true,
		.biasSigmaInit = 0.3f,
		.biasClip = 0.6f,
		.restThGyr = 0.3f,
		.restThAcc = 0.0098f,
	};

	I2CImpl i2c;
	SlimeVR::Logging::Logger& logger;
	ICM45686(I2CImpl i2c, SlimeVR::Logging::Logger& logger)
		: i2c(i2c)
		, logger(logger) {}

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x72;
			static constexpr uint8_t value = 0xe9;
		};
		static constexpr uint8_t TempData = 0x0c;

		struct DeviceConfig {
			static constexpr uint8_t reg = 0x7f;
			static constexpr uint8_t valueSwReset = 0b11;
		};

		struct Pin9Config {
			static constexpr uint8_t reg = 0x31;
			static constexpr uint8_t value = 0b00000110;  // pin 9 to clkin
		};

		struct RtcConfig {
			static constexpr uint8_t reg = 0x26;
			static constexpr uint8_t value = 0b00100011;  // enable RTC
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

	bool initialize() {
		// perform initialization step
		i2c.writeReg(Regs::DeviceConfig::reg, Regs::DeviceConfig::valueSwReset);
		delay(35);
		i2c.writeReg(Regs::Pin9Config::reg, Regs::Pin9Config::value);
		i2c.writeReg(Regs::RtcConfig::reg, Regs::RtcConfig::value);
		i2c.writeReg(Regs::GyroConfig::reg, Regs::GyroConfig::value);
		i2c.writeReg(Regs::AccelConfig::reg, Regs::AccelConfig::value);
		i2c.writeReg(Regs::FifoConfig0::reg, Regs::FifoConfig0::value);
		i2c.writeReg(Regs::FifoConfig3::reg, Regs::FifoConfig3::value);
		i2c.writeReg(Regs::PwrMgmt0::reg, Regs::PwrMgmt0::value);
		delay(1);

		return true;
	}

	template <typename AccelCall, typename GyroCall, typename TemperatureCall>
	void bulkRead(
		AccelCall&& processAccelSample,
		GyroCall&& processGyroSample,
		TemperatureCall&& processTemperatureSample
	) {
		const auto fifo_packets = i2c.readReg16(Regs::FifoCount);
		const auto fifo_bytes = fifo_packets * sizeof(FullFifoEntrySize);

		std::array<uint8_t, FullFifoEntrySize * 8> read_buffer;  // max 8 readings
		const auto bytes_to_read = std::min(
									   static_cast<size_t>(read_buffer.size()),
									   static_cast<size_t>(fifo_bytes)
								   )
								 / FullFifoEntrySize * FullFifoEntrySize;
		i2c.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());
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

			if (entry.part.temp != 0x8000) {
				processTemperatureSample(static_cast<int16_t>(entry.part.temp), TempTs);
			}
		}
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
