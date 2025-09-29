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
#include "callbacks.h"
#include "sensors/softfusion/magdriver.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 32g
// and gyroscope range at 4000dps
// using high resolution mode
// Uses 32.768kHz clock
// Gyroscope ODR = 204.8Hz, accel ODR = 102.4Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

struct ICM45Base {
	static constexpr uint8_t Address = 0x68;

	static constexpr float GyrTs = 1.0 / 204.8;
	static constexpr float AccTs = 1.0 / 102.4;
	static constexpr float TempTs = 1.0 / 409.6;

	static constexpr float MagTs = 1.0 / 100;

	static constexpr float GyroSensitivity = 131.072f;
	static constexpr float AccelSensitivity = 16384.0f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 128.0f;

	static constexpr float TemperatureZROChange = 20.0f;

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
				= (0b0000 << 4) | 0b1000;  // 4000dps, odr=204.8Hz
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
		};

		struct PwrMgmt0 {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value
				= 0b11 | (0b11 << 2);  // accel in low noise mode, gyro in low noise
		};

		static constexpr uint8_t FifoCount = 0x12;
		static constexpr uint8_t FifoData = 0x14;

		// Indirect Register Access

		static constexpr uint32_t IRegWaitTimeMicros = 4;

		enum class Bank : uint8_t {
			IMemSram = 0x00,
			IPregBar = 0xa0,
			IPregSys1 = 0xa4,
			IPregSys2 = 0xa5,
			IPregTop1 = 0xa2,
		};

		static constexpr uint8_t IRegAddr = 0x7c;
		static constexpr uint8_t IRegData = 0x7e;

		// Mag Support

		struct IOCPadScenarioAuxOvrd {
			static constexpr uint8_t reg = 0x30;
			static constexpr uint8_t value = (0b1 << 4)  // Enable AUX1 override
										   | (0b01 << 2)  // Enable I2CM master
										   | (0b1 << 1)  // Enable AUX1 enable override
										   | (0b1 << 0);  // Enable AUX1
		};

		struct I2CMCommand0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x06;
		};

		struct I2CMDevProfile0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x0e;
		};

		struct I2CMDevProfile1 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x0f;
		};

		struct I2CMWrData0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x33;
		};

		struct I2CMRdData0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x1b;
		};

		struct DmpExtSenOdrCfg {
			// TODO: todo
		};

		struct I2CMControl {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x16;
		};

		struct I2CMStatus {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x18;

			static constexpr uint8_t SDAErr = 0b1 << 5;
			static constexpr uint8_t SCLErr = 0b1 << 4;
			static constexpr uint8_t SRSTErr = 0b1 << 3;
			static constexpr uint8_t TimeoutErr = 0b1 << 2;
			static constexpr uint8_t Done = 0b1 << 1;
			static constexpr uint8_t Busy = 0b1 << 0;
		};
	};

#pragma pack(push, 1)
	struct FifoEntryAligned {
		int16_t accel[3];
		int16_t gyro[3];
		uint16_t temp;
		uint16_t timestamp;
		uint8_t lsb[3];
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	void softResetIMU() {
		m_RegisterInterface.writeReg(
			BaseRegs::DeviceConfig::reg,
			BaseRegs::DeviceConfig::valueSwReset
		);
		delay(35);
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

		m_RegisterInterface.writeReg(
			BaseRegs::IOCPadScenarioAuxOvrd::reg,
			BaseRegs::IOCPadScenarioAuxOvrd::value
		);

		read_buffer.resize(FullFifoEntrySize * MaxReadings);

		delay(1);

		return true;
	}

	static constexpr size_t MaxReadings = 8;
	// Allocate on heap so that it does not take up stack space, which can result in
	// stack overflow and panic
	std::vector<uint8_t> read_buffer;

	void bulkRead(DriverCallbacks<int32_t>&& callbacks) {
		constexpr int16_t InvalidReading = -32768;

		size_t fifo_packets = m_RegisterInterface.readReg16(BaseRegs::FifoCount);

		if (fifo_packets <= 1) {
			return;
		}

		// AN-000364
		// 2.16 FIFO EMPTY EVENT IN STREAMING MODE CAN CORRUPT FIFO DATA
		//
		// Description: When in FIFO streaming mode, a FIFO empty event
		// (caused by host reading the last byte of the last FIFO frame) can
		// cause FIFO data corruption in the first FIFO frame that arrives
		// after the FIFO empty condition. Once the issue is triggered, the
		// FIFO state is compromised and cannot recover. FIFO must be set in
		// bypass mode to flush out the wrong state
		//
		// When operating in FIFO streaming mode, if FIFO threshold
		// interrupt is triggered with M number of FIFO frames accumulated
		// in the FIFO buffer, the host should only read the first M-1
		// number of FIFO frames. This prevents the FIFO empty event, that
		// can cause FIFO data corruption, from happening.
		--fifo_packets;

		fifo_packets = std::min(fifo_packets, MaxReadings);

		size_t bytes_to_read = fifo_packets * FullFifoEntrySize;
		m_RegisterInterface
			.readBytes(BaseRegs::FifoData, bytes_to_read, read_buffer.data());

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

			if (has_gyro && entry.gyro[0] != InvalidReading) {
				const int32_t gyroData[3]{
					static_cast<int32_t>(entry.gyro[0]) << 4 | (entry.lsb[0] & 0xf),
					static_cast<int32_t>(entry.gyro[1]) << 4 | (entry.lsb[1] & 0xf),
					static_cast<int32_t>(entry.gyro[2]) << 4 | (entry.lsb[2] & 0xf),
				};
				callbacks.processGyroSample(gyroData, GyrTs);
			}

			if (has_accel && entry.accel[0] != InvalidReading) {
				const int32_t accelData[3]{
					static_cast<int32_t>(entry.accel[0]) << 4
						| (static_cast<int32_t>((entry.lsb[0]) & 0xf0) >> 4),
					static_cast<int32_t>(entry.accel[1]) << 4
						| (static_cast<int32_t>((entry.lsb[1]) & 0xf0) >> 4),
					static_cast<int32_t>(entry.accel[2]) << 4
						| (static_cast<int32_t>((entry.lsb[2]) & 0xf0) >> 4),
				};
				callbacks.processAccelSample(accelData, AccTs);
			}

			if (entry.temp != 0x8000) {
				callbacks.processTempSample(static_cast<int16_t>(entry.temp), TempTs);
			}
		}
	}

	template <typename Reg>
	uint8_t readBankRegister() {
		uint8_t buffer;
		readBankRegister<Reg>(&buffer, sizeof(buffer));
		return buffer;
	}

	template <typename Reg, typename T>
	void readBankRegister(T* buffer, size_t length) {
		uint8_t data[] = {
			static_cast<uint8_t>(Reg::bank),
			Reg::reg,
		};

		auto* bufferBytes = reinterpret_cast<uint8_t*>(buffer);
		m_RegisterInterface.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		for (size_t i = 0; i < length * sizeof(T); i++) {
			bufferBytes[i] = m_RegisterInterface.readReg(BaseRegs::IRegData);
			delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		}
	}

	template <typename Reg>
	void writeBankRegister() {
		writeBankRegister<Reg>(&Reg::value, sizeof(Reg::value));
	}

	template <typename Reg, typename T>
	void writeBankRegister(T* buffer, size_t length) {
		auto* bufferBytes = reinterpret_cast<uint8_t*>(buffer);

		uint8_t data[] = {
			static_cast<uint8_t>(Reg::bank),
			Reg::reg,
			bufferBytes[0],
		};

		m_RegisterInterface.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		for (size_t i = 1; i < length * sizeof(T); i++) {
			m_RegisterInterface.writeReg(BaseRegs::IRegData, bufferBytes[i]);
			delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		}
	}

	template <typename Reg>
	void writeBankRegister(uint8_t value) {
		writeBankRegister<Reg>(&value, sizeof(value));
	}

	void setAuxId(uint8_t deviceId) {
		writeBankRegister<typename BaseRegs::I2CMDevProfile1>(deviceId);
	}

	uint8_t readAux(uint8_t address) {
		writeBankRegister<typename BaseRegs::I2CMDevProfile0>(address);

		writeBankRegister<typename BaseRegs::I2CMCommand0>(
			(0b1 << 7)  // Last transaction
			| (0b0 << 6)  // Channel 0
			| (0b01 << 4)  // Read with register
			| (0b0001 << 0)  // Read 1 byte
		);
		writeBankRegister<typename BaseRegs::I2CMControl>(
			(0b0 << 6)  // No restarts
			| (0b0 << 3)  // Fast mode
			| (0b1 << 0)  // Start transaction
		);

		uint8_t lastStatus;
		while ((lastStatus = readBankRegister<typename BaseRegs::I2CMStatus>())
			   & BaseRegs::I2CMStatus::Busy)
			;

		if (lastStatus != BaseRegs::I2CMStatus::Done) {
			m_Logger.error(
				"Aux read from address 0x%02x returned status 0x%02x",
				address,
				lastStatus
			);
		}

		return readBankRegister<typename BaseRegs::I2CMRdData0>();
	}

	void writeAux(uint8_t address, uint8_t value) {
		writeBankRegister<typename BaseRegs::I2CMDevProfile0>(address);
		writeBankRegister<typename BaseRegs::I2CMWrData0>(value);
		writeBankRegister<typename BaseRegs::I2CMCommand0>(
			(0b1 << 7)  // Last transaction
			| (0b0 << 6)  // Channel 0
			| (0b01 << 4)  // Read with register
			| (0b0001 << 0)  // Read 1 byte
		);
		writeBankRegister<typename BaseRegs::I2CMControl>(
			(0b0 << 6)  // No restarts
			| (0b0 << 3)  // Fast mode
			| (0b1 << 0)  // Start transaction
		);

		uint8_t lastStatus;
		while ((lastStatus = readBankRegister<typename BaseRegs::I2CMStatus>())
			   & BaseRegs::I2CMStatus::Busy)
			;

		if (lastStatus != BaseRegs::I2CMStatus::Done) {
			m_Logger.error(
				"Aux write to address 0x%02x with value 0x%02x returned status 0x%02x",
				address,
				value,
				lastStatus
			);
		}
	}

	void startAuxPolling(uint8_t dataReg, MagDataWidth dataWidth) {
		// TODO:
	}

	void stopAuxPolling() {
		// TODO:
	}
};

};  // namespace SlimeVR::Sensors::SoftFusion::Drivers
