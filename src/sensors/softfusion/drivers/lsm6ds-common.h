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
#include <cassert>
#include <cstdint>

#include "../../../sensorinterface/RegisterInterface.h"
#include "../magdriver.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

struct LSM6DSOutputHandler {
	LSM6DSOutputHandler(
		RegisterInterface& registerInterface,
		SlimeVR::Logging::Logger& logger
	)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger) {}

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;

#pragma pack(push, 1)
	struct FifoEntryAligned {
		union {
			int16_t xyz[3];
			uint8_t raw[6];
		};
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	template <
		typename AccelCall,
		typename GyroCall,
		typename TempCall,
		typename MagCall,
		typename Regs>
	void bulkRead(
		AccelCall& processAccelSample,
		GyroCall& processGyroSample,
		TempCall& processTempSample,
		MagCall& processMagSample,
		float GyrTs,
		float AccTs,
		float TempTs,
		float MagTs
	) {
		constexpr auto FIFO_SAMPLES_MASK = 0x3ff;
		constexpr auto FIFO_OVERRUN_LATCHED_MASK = 0x800;

		const auto fifo_status = m_RegisterInterface.readReg16(Regs::FifoStatus);
		const auto available_axes = fifo_status & FIFO_SAMPLES_MASK;
		const auto fifo_bytes = available_axes * FullFifoEntrySize;
		if (fifo_status & FIFO_OVERRUN_LATCHED_MASK) {
			// FIFO overrun is expected to happen during startup and calibration
			m_Logger.error(
				"FIFO OVERRUN! This occuring during normal usage is an issue."
			);
		}

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
			uint8_t tag = read_buffer[i] >> 3;
			memcpy(
				entry.raw,
				&read_buffer[i + 0x1],
				sizeof(FifoEntryAligned)
			);  // skip fifo header

			switch (tag) {
				case 0x01:  // Gyro NC
					processGyroSample(entry.xyz, GyrTs);
					break;
				case 0x02:  // Accel NC
					processAccelSample(entry.xyz, AccTs);
					break;
				case 0x03:  // Temperature
					processTempSample(entry.xyz[0], TempTs);
					break;
				case 0x0e:  // Sensor Hub Slave 0
					processMagSample(entry.raw, MagTs);
					break;
			}
		}
	}

	uint8_t currentAuxDeviceId = 0;
	void setAuxDeviceId(uint8_t id) { currentAuxDeviceId = id; }

	template <typename Regs>
	void pollUntilSet(uint8_t address, uint8_t bits) {
		uint8_t value;
		while (true) {
			value = m_RegisterInterface.readReg(address);
			if ((value & bits) == bits) {
				return;
			}
		}
	}

	template <typename Regs>
	void writeAux(uint8_t address, uint8_t value) {
		m_RegisterInterface.writeReg(
			Regs::FuncCFGAccess::reg,
			Regs::FuncCFGAccess::sensorHubAccessOn
		);
		m_RegisterInterface.writeReg(Regs::SLV0Add, currentAuxDeviceId << 1 | 0b0);
		m_RegisterInterface.writeReg(Regs::SLV0Subadd, address);
		m_RegisterInterface.writeReg(Regs::SLV0Config::reg, 0x00);
		m_RegisterInterface.writeReg(Regs::DatawriteSLV0, value);
		m_RegisterInterface.writeReg(
			Regs::MasterConfig::reg,
			Regs::MasterConfig::valueOneShot
		);
		pollUntilSet<Regs>(Regs::StatusMaster, 0x80);
		m_RegisterInterface.writeReg(
			Regs::MasterConfig::reg,
			Regs::MasterConfig::valueDisable
		);
		delayMicroseconds(300);
		m_RegisterInterface.writeReg(
			Regs::FuncCFGAccess::reg,
			Regs::FuncCFGAccess::sensorHubAccessOff
		);
	}

	template <typename Regs>
	uint8_t readAux(uint8_t address) {
		uint8_t result;
		readAux<uint8_t, Regs>(address, &result, 1);
		return result;
	}

	template <typename T, typename Regs>
	void readAux(uint8_t address, T* outData, size_t count) {
		assert(sizeof(T) * count <= 6);

		m_RegisterInterface.writeReg(
			Regs::FuncCFGAccess::reg,
			Regs::FuncCFGAccess::sensorHubAccessOn
		);
		m_RegisterInterface.writeReg(Regs::SLV0Add, currentAuxDeviceId << 1 | 0b1);
		m_RegisterInterface.writeReg(Regs::SLV0Subadd, address);
		m_RegisterInterface.writeReg(Regs::SLV0Config::reg, sizeof(T) * count);
		m_RegisterInterface.writeReg(
			Regs::MasterConfig::reg,
			Regs::MasterConfig::valueOneShot
		);
		m_RegisterInterface.writeReg(
			Regs::FuncCFGAccess::reg,
			Regs::FuncCFGAccess::sensorHubAccessOff
		);
		pollUntilSet<Regs>(Regs::StatusReg, 0x01);
		pollUntilSet<Regs>(Regs::StatusMasterMainPage, 0x01);
		m_RegisterInterface.writeReg(
			Regs::FuncCFGAccess::reg,
			Regs::FuncCFGAccess::sensorHubAccessOn
		);
		m_RegisterInterface.writeReg(
			Regs::MasterConfig::reg,
			Regs::MasterConfig::valueDisable
		);
		delayMicroseconds(300);
		m_RegisterInterface.readBytes(Regs::SensorHub1, sizeof(T) * count, outData);
		m_RegisterInterface.writeReg(
			Regs::FuncCFGAccess::reg,
			Regs::FuncCFGAccess::sensorHubAccessOff
		);
	}

	template <typename Regs>
	void setAuxByteWidth(MagDefinition::DataWidth width) {}

	template <typename Regs>
	void setupAuxSensorPolling(uint8_t address, MagDefinition::DataWidth byteWidth) {
		assert(byteWidth == MagDefinition::DataWidth::SixByte);
		m_RegisterInterface.writeReg(
			Regs::FuncCFGAccess::reg,
			Regs::FuncCFGAccess::sensorHubAccessOn
		);
		m_RegisterInterface.writeReg(Regs::SLV0Add, currentAuxDeviceId << 1 | 0b1);
		m_RegisterInterface.writeReg(Regs::SLV0Subadd, address);
		m_RegisterInterface.writeReg(Regs::SLV0Config::reg, Regs::SLV0Config::value);
		m_RegisterInterface.writeReg(
			Regs::MasterConfig::reg,
			Regs::MasterConfig::value
		);
		m_RegisterInterface.writeReg(
			Regs::FuncCFGAccess::reg,
			Regs::FuncCFGAccess::sensorHubAccessOff
		);
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
