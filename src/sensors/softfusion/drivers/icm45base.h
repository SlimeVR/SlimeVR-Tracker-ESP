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

#include "../magdriver.h"
#include "GlobalVars.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 32g
// and gyroscope range at 4000dps
// using high resolution mode
// Uses 32.768kHz clock
// Gyroscope ODR = 409.6Hz, accel ODR = 102.4Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

template <typename I2CImpl>
struct ICM45Base {
	static constexpr uint8_t Address = 0x68;

	static constexpr float GyrTs = 1.0 / 409.6;
	static constexpr float AccTs = 1.0 / 102.4;
	static constexpr float TempTs = 1.0 / 409.6;

	static constexpr float MagTs = 1.0 / 100;

	static constexpr float GyroSensitivity = 131.072f;
	static constexpr float AccelSensitivity = 16384.0f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 128.0f;

	static constexpr float TemperatureZROChange = 20.0f;

	static constexpr bool Uses32BitSensorData = USE_6_AXIS;

	static constexpr int fifoReadSize
		= 8;  // Can't be more than 12 or it will overflow i2c read size, default 8

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
				= (0b000 << 4) | 0b1001;  // 32g, odr = 102.4Hz
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
			static constexpr uint8_t valueAux = (0b1 << 0) | (0b1 << 1) | (0b1 << 2)
											  | (0b0 << 3)
											  | (0b1 << 4);  // enable FIFO,
															 // enable accel,
															 // enable gyro,
															 // disable hires,
															 // enable es0
		};

		struct FifoConfig4 {
			static constexpr uint8_t reg = 0x22;
			static constexpr uint8_t value6Byte = 0b0;
			static constexpr uint8_t value9Byte = 0b0;
		};

		struct PwrMgmt0 {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value
				= 0b11 | (0b11 << 2);  // accel in low noise mode, gyro in low noise
		};

		static constexpr uint8_t IRegAddr = 0x7c;
		static constexpr uint8_t IRegData = 0x7e;

		struct IOCPadScenarioAuxOvrd {
			static constexpr uint8_t reg = 0x30;
			static constexpr uint8_t value
				= 0b1 | (0b1 << 1) | (0b01 << 2) | (0b1 << 4);  // override aux
																// enable and mode
		};

		struct I2CMCommand {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x06;
		};

		struct I2CMDevProfile0 {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x0e;
		};

		struct I2CMDevProfile1 {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x0f;
		};

		struct I2CMWRData {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x33;
		};

		struct I2CMRDData {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x1b;
		};

		struct I2CMControl {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x16;
		};

		struct RegMisc1 {
			static constexpr uint8_t reg = 0x35;
			static constexpr uint8_t value = 0b0100;  // internal relaxation
													  // oscillator
		};

		struct DMPExtSenOdrCfg {
			static constexpr uint8_t reg = 0x27;
			static constexpr uint8_t value
				= 0b000 | (0b010 << 3) | (0b1 << 6);  // apex odr irrelevant,
													  // ext odr at 12.5Hz,
													  // enable ext sensor odr
		};

		static constexpr uint8_t FifoCount = 0x12;
		static constexpr uint8_t FifoData = 0x14;
	};

#pragma pack(push, 1)
	struct FifoEntryAligned {
		uint8_t header;
		int16_t accel[3];
		int16_t gyro[3];
		uint16_t temp;
		uint16_t timestamp;
		uint8_t lsb[3];
	};
	struct FifoEntryAlignedAux {
		uint8_t header1;
		uint8_t header2;
		int16_t accel[3];
		int16_t gyro[3];
		uint8_t aux1[9];
		uint8_t aux2[6];
		uint8_t temp;
		uint16_t timestamp;
	};

	using FifoEntry
		= std::conditional<USE_6_AXIS, FifoEntryAligned, FifoEntryAlignedAux>::type;

	static_assert(sizeof(FifoEntry) == 32);

	struct FifoBuffer {
		FifoEntry entry[fifoReadSize] = {};
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntry);
	static constexpr size_t FullFifoBufferSize = sizeof(FifoBuffer);

	void softResetIMU() {
		i2c.writeReg(BaseRegs::DeviceConfig::reg, BaseRegs::DeviceConfig::valueSwReset);
		delay(35);
	}

	template <typename R>
	void writeBankRegister() {
		uint8_t value = R::value;
		writeBankRegister<R>(&value, sizeof(value));
	}

	template <typename R>
	void writeBankRegister(uint8_t value) {
		writeBankRegister<R>(&value, sizeof(value));
	}

	template <typename R>
	void writeBankRegister(uint8_t* value, size_t size) {
		uint8_t data[] = {R::bank, R::reg, value[0]};
		i2c.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(4);
		for (size_t i = 1; i < size; i++) {
			i2c.writeReg(BaseRegs::IRegData, value[i]);
			delayMicroseconds(4);
		}
	}

	template <typename R>
	uint8_t readBankRegister() {
		uint8_t out;
		readBankRegister<R, uint8_t>(&out, sizeof(out));
		return out;
	}

	template <typename R, typename T>
	void readBankRegister(T* outData, size_t count) {
		uint8_t data[] = {R::bank, R::reg};
		i2c.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(4);
		auto* buffer = reinterpret_cast<uint8_t*>(outData);
		for (size_t i = 0; i < count * sizeof(T); i++) {
			buffer[i] = i2c.readReg(BaseRegs::IRegData);
			delayMicroseconds(4);
		}
	}

	bool initializeBase() {
		// perform initialization step
		i2c.writeReg(BaseRegs::GyroConfig::reg, BaseRegs::GyroConfig::value);
		i2c.writeReg(BaseRegs::AccelConfig::reg, BaseRegs::AccelConfig::value);
		i2c.writeReg(BaseRegs::FifoConfig0::reg, BaseRegs::FifoConfig0::value);
		i2c.writeReg(BaseRegs::PwrMgmt0::reg, BaseRegs::PwrMgmt0::value);

		if constexpr (USE_6_AXIS) {
			i2c.writeReg(BaseRegs::FifoConfig3::reg, BaseRegs::FifoConfig3::value);
		} else {
			i2c.writeReg(BaseRegs::FifoConfig3::reg, BaseRegs::FifoConfig3::valueAux);
			i2c.writeReg(
				BaseRegs::DMPExtSenOdrCfg::reg,
				BaseRegs::DMPExtSenOdrCfg::value
			);
			i2c.writeReg(
				BaseRegs::IOCPadScenarioAuxOvrd::reg,
				BaseRegs::IOCPadScenarioAuxOvrd::value
			);
			i2c.writeReg(BaseRegs::RegMisc1::reg, BaseRegs::RegMisc1::value);
		}

		delay(1);

		return true;
	}

	float getDirectTemp() const {
		const auto value = static_cast<int16_t>(i2c.readReg16(BaseRegs::TempData));
		float result = ((float)value / 132.48f) + 25.0f;
		return result;
	}

	template <typename AccelCall, typename GyroCall, typename MagCall>
	void bulkRead(
		AccelCall&& processAccelSample,
		GyroCall&& processGyroSample,
		MagCall&& processMagSample
	) {
		const auto fifo_packets = i2c.readReg16(BaseRegs::FifoCount);
		const auto fifo_bytes = fifo_packets * sizeof(FullFifoEntrySize);

		FifoBuffer read_buffer;
		const auto bytes_to_read = std::min(
									   static_cast<size_t>(FullFifoBufferSize),
									   static_cast<size_t>(fifo_bytes)
								   )
								 / FullFifoEntrySize * FullFifoEntrySize;
		i2c.readBytes(BaseRegs::FifoData, bytes_to_read, (uint8_t*)&read_buffer);
		uint8_t index = 0;
		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			if constexpr (USE_6_AXIS) {
				FifoEntryAligned entry = read_buffer.entry[index++];
				const int32_t gyroData[3]{
					static_cast<int32_t>(entry.gyro[0]) << 4 | (entry.lsb[0] & 0xf),
					static_cast<int32_t>(entry.gyro[1]) << 4 | (entry.lsb[1] & 0xf),
					static_cast<int32_t>(entry.gyro[2]) << 4 | (entry.lsb[2] & 0xf),
				};
				processGyroSample(gyroData, GyrTs);

				if (entry.accel[0] != -32768) {
					const int32_t accelData[3]{
						static_cast<int32_t>(entry.accel[0]) << 4
							| (static_cast<int32_t>(entry.lsb[0]) & 0xf0 >> 4),
						static_cast<int32_t>(entry.accel[1]) << 4
							| (static_cast<int32_t>(entry.lsb[1]) & 0xf0 >> 4),
						static_cast<int32_t>(entry.accel[2]) << 4
							| (static_cast<int32_t>(entry.lsb[2]) & 0xf0 >> 4),
					};
					processAccelSample(accelData, AccTs);
				}
			} else {
				FifoEntryAlignedAux entry = read_buffer.entry[index++];
				processGyroSample(entry.gyro, GyrTs);

				if (entry.accel[0] != -32768) {
					processAccelSample(entry.accel, AccTs);
				}

				if (entry.aux1[0] != 0xff || entry.aux1[1] != 0xff) {
					processMagSample(entry.aux1);
				}
			}
		}
	}

	void setAuxDeviceId(uint8_t id) {
		writeBankRegister<typename BaseRegs::I2CMDevProfile1>(id);
	}

	void writeAux(uint8_t address, uint8_t value) {
		uint8_t data[] = {address, value};
		writeBankRegister<typename BaseRegs::I2CMWRData>(data, sizeof(data));
		writeBankRegister<typename BaseRegs::I2CMCommand>(
			0b0010 | (0b00 << 4) | (0b0 << 6) | (0b1 << 7)
		);  // Write 2 bytes on channel 0, last transmission
		writeBankRegister<typename BaseRegs::I2CMControl>(
			0b1 | (0b0 << 3) | (0b0 << 6)
		);  // Start i2cm, fast mode, no restarts
		while ((readBankRegister<typename BaseRegs::I2CMControl>() & 0b1) != 0)
			;  // Wait until operation finishes
	}

	uint8_t readAux(uint8_t address) {
		uint8_t out;
		readAux<uint8_t>(address, &out, 1);
		return out;
	}

	template <typename T>
	void readAux(uint8_t address, T* outData, size_t count) {
		writeBankRegister<typename BaseRegs::I2CMDevProfile0>(address);
		writeBankRegister<typename BaseRegs::I2CMCommand>(
			0b0001 | (0b01 << 4) | (0b0 << 6) | (0b1 << 7)
		);  // Read (with address) on channel 0, last transmission
		writeBankRegister<typename BaseRegs::I2CMControl>(
			0b1 | (0b0 << 3) | (0b0 << 6)
		);  // Start i2cm, fast mode, no restarts
		while ((readBankRegister<typename BaseRegs::I2CMControl>() & 0b1) != 0)
			;  // Wait until operation finishes

		readBankRegister<typename BaseRegs::I2CMRDData, T>(outData, count);
	}

	void setAuxByteWidth(MagDefinition::DataWidth width) {
		i2c.writeReg(
			BaseRegs::FifoConfig4::reg,
			width == MagDefinition::DataWidth::SixByte
				? BaseRegs::FifoConfig4::value6Byte
				: BaseRegs::FifoConfig4::value9Byte
		);
	}

	void setupAuxSensorPolling(uint8_t address, MagDefinition::DataWidth byteWidth) {
		writeBankRegister<typename BaseRegs::I2CMDevProfile0>(address);
		uint8_t lengthBits
			= byteWidth == MagDefinition::DataWidth::SixByte ? 0b0110 : 0b1001;
		writeBankRegister<typename BaseRegs::I2CMCommand>(
			lengthBits | (0b01 << 3) | (0b0 << 5) | (0b1 << 6)
		);  // Read (with address) on channel 0, last transmission
	};
};

};  // namespace SlimeVR::Sensors::SoftFusion::Drivers
