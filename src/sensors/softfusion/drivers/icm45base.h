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

#include "../../../sensorinterface/RegisterInterface.h"
#include "../magdriver.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 32g
// and gyroscope range at 4000dps
// using high resolution mode
// Uses 32.768kHz clock
// Gyroscope ODR = 409.6Hz, accel ODR = 102.4Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

struct ICM45Base {
	static constexpr uint8_t Address = 0x68;

	static constexpr float GyrTs = 1.0 / 409.6;
	static constexpr float AccTs = 1.0 / 102.4;
	static constexpr float TempTs = 1.0 / 409.6;

	static constexpr float MagTs = 1.0 / 100;

	// static constexpr float GyroSensitivity = 131.072f;
	// static constexpr float AccelSensitivity = 16384.0f;
	static constexpr float GyroSensitivity = (1 << 15) / 4000.0f;
	static constexpr float AccelSensitivity = (1 << 15) / 32.0f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 128.0f;

	static constexpr float TemperatureZROChange = 20.0f;

	static constexpr bool Uses32BitSensorData = true;

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;
	ICM45Base(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger) {}

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
				= (0b10 << 6) | (0b011111);  // stream to FIFO mode, FIFO depth
											 // 8k bytes <-- this disables all APEX
											 // features, but we don't need them
			static constexpr uint8_t valueDisable
				= (0b00 << 6) | (0b011111);  // disable FIFO
		};

		struct FifoConfig3 {
			static constexpr uint8_t reg = 0x21;
			static constexpr uint8_t value = (0b1 << 0) | (0b1 << 1) | (0b1 << 2)
										   | (0b1 << 3);  // enable FIFO,
														  // enable accel,
														  // enable gyro,
														  // enable hires mode
			static constexpr uint8_t valueExt = (0b1 << 0) | (0b1 << 1) | (0b1 << 2)
											  | (0b0 << 3) | (0b1 << 4)
											  | (0b1 << 5);  // enable FIFO,
															 // enable accel,
															 // enable gyro,
															 // disable hires mode,
															 // enable ext1
															 // enable ext2
		};

		struct FifoConfig4 {
			static constexpr uint8_t reg = 0x22;
			static constexpr uint8_t value6Byte = 0b0;
			static constexpr uint8_t value9Byte = 0b1;
		};

		struct PwrMgmt0 {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value
				= 0b11 | (0b11 << 2);  // accel in low noise mode, gyro in low noise
		};

		static constexpr uint8_t FifoCount = 0x12;
		static constexpr uint8_t FifoData = 0x14;

		struct IOCPadScenarioOvrd {
			static constexpr uint8_t reg = 0x30;
			static constexpr uint8_t value
				= 0b0000'1'01'1'1;  // override aux enable to true
									// override aux mode to i2cm
		};

		struct I2CMCommand0 {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x06;
		};

		struct I2CMDevProfile0 {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x0e;
		};

		struct I2CMWRData0 {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x33;
		};

		struct I2CMRDData0 {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x1b;
		};

		struct DMPExtSenOdrCfg {
			static constexpr uint8_t reg = 0x27;
			static constexpr uint8_t value
				= 0b0'1'100'001;  // enable odr generation for
								  // ext sensor, ext sensor odr
								  // at 50Hz, apex odr at 50Hz
		};

		struct I2CMControl {
			static constexpr uint8_t bank = 0xa2;
			static constexpr uint8_t reg = 0x16;
		};

		static constexpr uint8_t IRegAddr = 0x7c;
		static constexpr uint8_t IRegData = 0x7e;
	};

#pragma pack(push, 1)
	struct FifoEntryAligned {
		int16_t accel[3];
		int16_t gyro[3];
		uint8_t ext1[9];
		uint8_t ext2[6];
		uint8_t temp;
		uint16_t timestamp;
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 2;

	void softResetIMU() {
		m_RegisterInterface.writeReg(
			BaseRegs::DeviceConfig::reg,
			BaseRegs::DeviceConfig::valueSwReset
		);
		delay(35);
	}

	template <typename R, size_t Offset = 0x00>
	void writeBankRegister() {
		uint8_t value = R::value;
		writeBankRegister<R, Offset>(&value, sizeof(value));
	}

	template <typename R, size_t Offset = 0x00>
	void writeBankRegister(uint8_t value) {
		writeBankRegister<R, Offset>(&value, sizeof(value));
	}

	template <typename R, size_t Offset = 0x00>
	void writeBankRegister(uint8_t* value, size_t size) {
		uint8_t data[] = {R::bank, R::reg + Offset, value[0]};
		m_RegisterInterface.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(4);
		for (size_t i = 1; i < size; i++) {
			m_RegisterInterface.writeReg(BaseRegs::IRegData, value[i]);
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
		m_RegisterInterface.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(4);
		auto* buffer = reinterpret_cast<uint8_t*>(outData);
		for (size_t i = 0; i < count * sizeof(T); i++) {
			buffer[i] = m_RegisterInterface.readReg(BaseRegs::IRegData);
			delayMicroseconds(4);
		}
	}

	bool initializeBase() {
		// perform initialization step
		m_RegisterInterface.writeReg(
			BaseRegs::GyroConfig::reg,
			BaseRegs::GyroConfig::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::AccelConfig::reg,
			BaseRegs::AccelConfig::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig0::reg,
			BaseRegs::FifoConfig0::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig3::reg,
			BaseRegs::FifoConfig3::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::PwrMgmt0::reg,
			BaseRegs::PwrMgmt0::value
		);
		delay(1);

		if constexpr (!USE_6_AXIS) {
			m_RegisterInterface.writeReg(
				BaseRegs::IOCPadScenarioOvrd::reg,
				BaseRegs::IOCPadScenarioOvrd::value
			);
		}

		return true;
	}

	std::array<uint8_t, FullFifoEntrySize * 12> read_buffer;

	template <
		typename AccelCall,
		typename GyroCall,
		typename TempCall,
		typename MagCall>
	void bulkRead(
		AccelCall&& processAccelSample,
		GyroCall&& processGyroSample,
		TempCall&& processTemperatureSample,
		MagCall&& processMagSample
	) {
		const auto fifo_packets = m_RegisterInterface.readReg16(BaseRegs::FifoCount);
		const auto fifo_bytes = fifo_packets * sizeof(FullFifoEntrySize);

		std::array<uint8_t, FullFifoEntrySize * 8> read_buffer;  // max 8 readings
		const auto bytes_to_read = std::min(
									   static_cast<size_t>(read_buffer.size()),
									   static_cast<size_t>(fifo_bytes)
								   )
								 / FullFifoEntrySize * FullFifoEntrySize;
		m_RegisterInterface
			.readBytes(BaseRegs::FifoData, bytes_to_read, read_buffer.data());
		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			FifoEntryAligned entry;
			memcpy(
				&entry,
				&read_buffer[i + 0x2],
				sizeof(FifoEntryAligned)
			);  // skip fifo header
			const int32_t gyroData[3]{
				// static_cast<int32_t>(entry.gyro[0]) << 4 | (entry.lsb[0] & 0xf),
				// static_cast<int32_t>(entry.gyro[1]) << 4 | (entry.lsb[1] & 0xf),
				// static_cast<int32_t>(entry.gyro[2]) << 4 | (entry.lsb[2] & 0xf),
				static_cast<int32_t>(entry.gyro[0]),
				static_cast<int32_t>(entry.gyro[1]),
				static_cast<int32_t>(entry.gyro[2]),
			};
			processGyroSample(gyroData, GyrTs);

			if (entry.accel[0] != -32768) {
				const int32_t accelData[3]{
					// static_cast<int32_t>(entry.accel[0]) << 4
					// 	| (static_cast<int32_t>((entry.lsb[0]) & 0xf0) >> 4),
					// static_cast<int32_t>(entry.accel[1]) << 4
					// 	| (static_cast<int32_t>((entry.lsb[1]) & 0xf0) >> 4),
					// static_cast<int32_t>(entry.accel[2]) << 4
					// 	| (static_cast<int32_t>((entry.lsb[2]) & 0xf0) >> 4),
					static_cast<int32_t>(entry.accel[0]),
					static_cast<int32_t>(entry.accel[1]),
					static_cast<int32_t>(entry.accel[2]),
				};
				processAccelSample(accelData, AccTs);
			}

			// if (entry.temp != 0x8000) {
			// 	processTemperatureSample(static_cast<int16_t>(entry.temp), TempTs);
			// }
		}
	}

	void setAuxDeviceId(uint8_t id) {
		writeBankRegister<typename BaseRegs::I2CMDevProfile0, 1>(id);
	}

	void writeAux(uint8_t address, uint8_t value) {
		uint8_t data[] = {address, value};
		writeBankRegister<typename BaseRegs::I2CMWRData0>(data, sizeof(data));
		writeBankRegister<typename BaseRegs::I2CMCommand0>(0b1'0'00'0001
		);  // Write 1 byte on channel 0, last transmission
		writeBankRegister<typename BaseRegs::I2CMControl>(
			0b1 | (0b0 << 3) | (0b0 << 6)
		);  // Start i2cm, fast mode, no restarts
		while ((readBankRegister<typename BaseRegs::I2CMControl>() & 0b1) != 0)
			;  // Wait until operation finishes
	}

	uint8_t readAux(uint8_t address) {
		writeBankRegister<typename BaseRegs::I2CMDevProfile0, 0>(address);
		writeBankRegister<typename BaseRegs::I2CMCommand0>(0b1'0'01'0001
		);  // Read one byte, last transaction
		writeBankRegister<typename BaseRegs::I2CMControl>(
			0b1 | (0b0 << 3) | (0b0 << 6)
		);  // Start i2cm transaction
		while ((readBankRegister<typename BaseRegs::I2CMControl>() & 0b1) != 0)
			;
		return readBankRegister<typename BaseRegs::I2CMRDData0>();
	}

	void setAuxByteWidth(MagDefinition::DataWidth width) {
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig4::reg,
			width == MagDefinition::DataWidth::SixByte
				? BaseRegs::FifoConfig4::value6Byte
				: BaseRegs::FifoConfig4::value9Byte
		);
	}

	void setupAuxSensorPolling(uint8_t address, MagDefinition::DataWidth byteWidth) {
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig0::reg,
			BaseRegs::FifoConfig0::valueDisable
		);
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig3::reg,
			BaseRegs::FifoConfig3::valueExt
		);
		writeBankRegister<typename BaseRegs::I2CMDevProfile0>(address);
		uint8_t lengthBits
			= byteWidth == MagDefinition::DataWidth::SixByte ? 0b0110 : 0b1001;
		writeBankRegister<typename BaseRegs::I2CMCommand0>(
			lengthBits | (0b01 << 4) | (0b0 << 6) | (0b1 << 7)
		);  // Read (with address) on channel 0, last transmission
		m_RegisterInterface.writeReg(
			BaseRegs::DMPExtSenOdrCfg::reg,
			BaseRegs::DMPExtSenOdrCfg::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig0::reg,
			BaseRegs::FifoConfig0::value
		);
	};
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
