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
#include <cstring>
#include <limits>

#include "../../../sensorinterface/RegisterInterface.h"
#include "bmi270fw.h"
#include "callbacks.h"
#include "vqf.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 16g
// and gyroscope range at 1000dps
// Gyroscope ODR = 200Hz, accel ODR = 100Hz
// Timestamps reading are not used

struct BMI270 {
	static constexpr uint8_t Address = 0x68;
	static constexpr auto Name = "BMI270";
	static constexpr auto Type = SensorTypeID::BMI270;

	static constexpr float GyrTs = 1.0 / 200.0;
	static constexpr float AccTs = 1.0 / 100.0;

	static constexpr float MagTs = 1.0 / 100;

	static constexpr float GyroSensitivity = 32.768f;
	static constexpr float AccelSensitivity = 2048.0f;

	static constexpr float TemperatureZROChange = 6.667f;

	static constexpr VQFParams SensorVQFParams{};

	struct MotionlessCalibrationData {
		bool valid;
		uint8_t x, y, z;
	};

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;
	int8_t m_zxFactor;
	BMI270(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger)
		, m_zxFactor(0) {}

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x00;
			static constexpr uint8_t value = 0x24;
		};
		static constexpr uint8_t TempData = 0x22;

		struct Cmd {
			static constexpr uint8_t reg = 0x7e;
			static constexpr uint8_t valueSwReset = 0xb6;
			static constexpr uint8_t valueFifoFlush = 0xb0;
			static constexpr uint8_t valueGTrigger = 0x02;
		};

		struct PwrConf {
			static constexpr uint8_t reg = 0x7c;
			static constexpr uint8_t valueNoPowerSaving = 0x0;
			static constexpr uint8_t valueFifoSelfWakeup = 0x2;
		};

		struct PwrCtrl {
			static constexpr uint8_t reg = 0x7d;
			static constexpr uint8_t valueOff = 0x0;
			static constexpr uint8_t valueGyrAccTempOn = 0b1110;  // aux off
			static constexpr uint8_t valueAccOn = 0b0100;  // aux, gyr, temp off
		};

		struct InitCtrl {
			static constexpr uint8_t reg = 0x59;
			static constexpr uint8_t valueStartInit = 0x00;
			static constexpr uint8_t valueEndInit = 0x01;
		};

		static constexpr uint8_t InitAddr = 0x5b;
		static constexpr uint8_t InitData = 0x5e;

		struct InternalStatus {
			static constexpr uint8_t reg = 0x21;
			static constexpr uint8_t initializedBit = 0x01;
		};

		struct GyrConf {
			static constexpr uint8_t reg = 0x42;

			static constexpr uint8_t rate25Hz = 6;
			static constexpr uint8_t rate50Hz = 7;
			static constexpr uint8_t rate100Hz = 8;
			static constexpr uint8_t rate200Hz = 9;
			static constexpr uint8_t rate400Hz = 10;
			static constexpr uint8_t rate800Hz = 11;
			static constexpr uint8_t rate1600Hz = 12;
			static constexpr uint8_t rate3200Hz = 13;

			static constexpr uint8_t DLPFModeOsr4 = 0 << 4;
			static constexpr uint8_t DLPFModeOsr2 = 1 << 4;
			static constexpr uint8_t DLPFModeNorm = 2 << 4;

			static constexpr uint8_t noisePerfMode = 1 << 6;
			static constexpr uint8_t filterHighPerfMode = 1 << 7;

			static constexpr uint8_t value
				= rate200Hz | DLPFModeNorm | noisePerfMode | filterHighPerfMode;
		};

		struct GyrRange {
			static constexpr uint8_t reg = 0x43;

			static constexpr uint8_t range125dps = 4;
			static constexpr uint8_t range250dps = 3;
			static constexpr uint8_t range500dps = 2;
			static constexpr uint8_t range1000dps = 1;
			static constexpr uint8_t range2000dps = 0;

			static constexpr uint8_t value = range1000dps;
		};

		struct AccConf {
			static constexpr uint8_t reg = 0x40;

			static constexpr uint8_t rate0_78Hz = 1;
			static constexpr uint8_t rate1_5Hz = 2;
			static constexpr uint8_t rate3_1Hz = 3;
			static constexpr uint8_t rate6_25Hz = 4;
			static constexpr uint8_t rate12_5Hz = 5;
			static constexpr uint8_t rate25Hz = 6;
			static constexpr uint8_t rate50Hz = 7;
			static constexpr uint8_t rate100Hz = 8;
			static constexpr uint8_t rate200Hz = 9;
			static constexpr uint8_t rate400Hz = 10;
			static constexpr uint8_t rate800Hz = 11;
			static constexpr uint8_t rate1600Hz = 12;

			static constexpr uint8_t DLPFModeAvg1 = 0 << 4;
			static constexpr uint8_t DLPFModeAvg2 = 1 << 4;
			static constexpr uint8_t DLPFModeAvg4 = 2 << 4;
			static constexpr uint8_t DLPFModeAvg8 = 3 << 4;

			static constexpr uint8_t filterHighPerfMode = 1 << 7;

			static constexpr uint8_t value
				= rate100Hz | DLPFModeAvg4 | filterHighPerfMode;
		};

		struct AccRange {
			static constexpr uint8_t reg = 0x41;

			static constexpr uint8_t range2G = 0;
			static constexpr uint8_t range4G = 1;
			static constexpr uint8_t range8G = 2;
			static constexpr uint8_t range16G = 3;

			static constexpr uint8_t value = range16G;
		};

		struct FifoConfig0 {
			static constexpr uint8_t reg = 0x48;
			static constexpr uint8_t value
				= 0x01;  // fifo_stop_on_full=1, fifo_time_en=0
		};

		struct FifoConfig1 {
			static constexpr uint8_t reg = 0x49;
			static constexpr uint8_t value
				= (1 << 4) | (1 << 6) | (1 << 7);  // header en, acc en, gyr en
		};

		struct GyrCrtConf {
			static constexpr uint8_t reg = 0x69;
			static constexpr uint8_t valueRunning = (1 << 2);  // crt_running = 1
			static constexpr uint8_t valueStopped = 0x0;  // crt_running = 0
		};

		struct GTrig1 {  // on feature page 1!
			static constexpr uint8_t reg = 0x32;
			static constexpr uint16_t valueTriggerCRT = (1 << 8);  // select=crt
		};

		struct GyrGainStatus {  // on feature page 0!
			static constexpr uint8_t reg = 0x38;
			static constexpr uint8_t statusOffset = 3;
		};

		struct Offset6 {  // on feature page 0!
			static constexpr uint8_t reg = 0x77;
			static constexpr uint8_t value = (1 << 7);  // gyr_gain_en = 1
		};

		static constexpr uint8_t FeatPage = 0x2f;

		static constexpr uint8_t GyrUserGain
			= 0x78;  // undocumented reg, got from official bmi270 driver

		static constexpr uint8_t FifoCount = 0x24;
		static constexpr uint8_t FifoData = 0x26;
		static constexpr uint8_t RaGyrCas = 0x3c;  // on feature page 0!
	};

	struct Fifo {
		static constexpr uint8_t ModeMask = 0b11000000;
		static constexpr uint8_t SkipFrame = 0b01000000;
		static constexpr uint8_t DataFrame = 0b10000000;

		static constexpr uint8_t GyrDataBit = 0b00001000;
		static constexpr uint8_t AccelDataBit = 0b00000100;
	};

	bool restartAndInit() {
		// perform initialization step
		m_RegisterInterface.writeReg(Regs::Cmd::reg, Regs::Cmd::valueSwReset);
		delay(12);
		// disable power saving
		m_RegisterInterface.writeReg(
			Regs::PwrConf::reg,
			Regs::PwrConf::valueNoPowerSaving
		);
		delay(1);

		// firmware upload
		m_RegisterInterface.writeReg(
			Regs::InitCtrl::reg,
			Regs::InitCtrl::valueStartInit
		);
		auto* firmware_buffer = new uint8_t[RegisterInterface::MaxTransactionLength];
		for (uint16_t pos = 0; pos < sizeof(bmi270_firmware);) {
			// tell the device current position

			// this thing is little endian, but it requires address in bizzare form
			// LSB register is only 4 bits, while MSB register is 8bits
			// also value requested is in words (16bit) not in bytes (8bit)

			const uint16_t pos_words = pos >> 1;  // convert current position to words
			const uint16_t position = (pos_words & 0x0F) | ((pos_words << 4) & 0xff00);
			m_RegisterInterface.writeReg16(Regs::InitAddr, position);
			// write actual payload chunk
			const uint16_t burstWrite = std::min(
				static_cast<size_t>(sizeof(bmi270_firmware) - pos),
				RegisterInterface::MaxTransactionLength
			);
			memcpy_P(firmware_buffer, bmi270_firmware + pos, burstWrite);
			m_RegisterInterface.writeBytes(Regs::InitData, burstWrite, firmware_buffer);
			pos += burstWrite;
		}
		delete[] firmware_buffer;
		m_RegisterInterface.writeReg(Regs::InitCtrl::reg, Regs::InitCtrl::valueEndInit);
		delay(140);

		// leave fifo_self_wakeup enabled
		m_RegisterInterface.writeReg(
			Regs::PwrConf::reg,
			Regs::PwrConf::valueFifoSelfWakeup
		);
		// check if IMU initialized correctly
		if (!(m_RegisterInterface.readReg(Regs::InternalStatus::reg)
			  & Regs::InternalStatus::initializedBit)) {
			// firmware upload fail or sensor not initialized
			return false;
		}

		// read zx factor used to reduce gyro cross-sensitivity error
		const uint8_t zx_factor_reg = m_RegisterInterface.readReg(Regs::RaGyrCas);
		const uint8_t sign_byte = (zx_factor_reg << 1) & 0x80;
		m_zxFactor = static_cast<int8_t>(zx_factor_reg | sign_byte);
		return true;
	}

	void setNormalConfig(MotionlessCalibrationData& gyroSensitivity) {
		m_RegisterInterface.writeReg(Regs::GyrConf::reg, Regs::GyrConf::value);
		m_RegisterInterface.writeReg(Regs::GyrRange::reg, Regs::GyrRange::value);

		m_RegisterInterface.writeReg(Regs::AccConf::reg, Regs::AccConf::value);
		m_RegisterInterface.writeReg(Regs::AccRange::reg, Regs::AccRange::value);

		if (gyroSensitivity.valid) {
			m_RegisterInterface.writeReg(Regs::Offset6::reg, Regs::Offset6::value);
			m_RegisterInterface.writeBytes(Regs::GyrUserGain, 3, &gyroSensitivity.x);
		}

		m_RegisterInterface.writeReg(
			Regs::PwrCtrl::reg,
			Regs::PwrCtrl::valueGyrAccTempOn
		);
		delay(100);  // power up delay
		m_RegisterInterface.writeReg(Regs::FifoConfig0::reg, Regs::FifoConfig0::value);
		m_RegisterInterface.writeReg(Regs::FifoConfig1::reg, Regs::FifoConfig1::value);

		delay(4);
		m_RegisterInterface.writeReg(Regs::Cmd::reg, Regs::Cmd::valueFifoFlush);
		delay(2);
	}

	bool initialize(MotionlessCalibrationData& gyroSensitivity) {
		if (!restartAndInit()) {
			return false;
		}

		setNormalConfig(gyroSensitivity);

		return true;
	}

	bool motionlessCalibration(MotionlessCalibrationData& gyroSensitivity) {
		// perfrom gyroscope motionless sensitivity calibration (CRT)
		// need to start from clean state according to spec
		restartAndInit();
		// only Accel ON
		m_RegisterInterface.writeReg(Regs::PwrCtrl::reg, Regs::PwrCtrl::valueAccOn);
		delay(100);
		m_RegisterInterface.writeReg(
			Regs::GyrCrtConf::reg,
			Regs::GyrCrtConf::valueRunning
		);
		m_RegisterInterface.writeReg(Regs::FeatPage, 1);
		m_RegisterInterface.writeReg16(
			Regs::GTrig1::reg,
			Regs::GTrig1::valueTriggerCRT
		);
		m_RegisterInterface.writeReg(Regs::Cmd::reg, Regs::Cmd::valueGTrigger);
		delay(200);

		while (m_RegisterInterface.readReg(Regs::GyrCrtConf::reg)
			   == Regs::GyrCrtConf::valueRunning) {
			m_Logger.info("CRT running. Do not move tracker!");
			delay(200);
		}

		m_RegisterInterface.writeReg(Regs::FeatPage, 0);

		uint8_t status = m_RegisterInterface.readReg(Regs::GyrGainStatus::reg)
					  >> Regs::GyrGainStatus::statusOffset;
		// turn gyroscope back on
		m_RegisterInterface.writeReg(
			Regs::PwrCtrl::reg,
			Regs::PwrCtrl::valueGyrAccTempOn
		);
		delay(100);

		bool success;

		if (status != 0) {
			m_Logger.error(
				"CRT failed with status 0x%x. Recalibrate again to enable CRT.",
				status
			);
			if (status == 0x03) {
				m_Logger.error("Reason: tracker was moved during CRT!");
			}

			success = false;
		} else {
			std::array<uint8_t, 3> crt_values;
			m_RegisterInterface
				.readBytes(Regs::GyrUserGain, crt_values.size(), crt_values.data());
			m_Logger.debug(
				"CRT finished successfully, result 0x%x, 0x%x, 0x%x",
				crt_values[0],
				crt_values[1],
				crt_values[2]
			);
			gyroSensitivity.valid = true;
			gyroSensitivity.x = crt_values[0];
			gyroSensitivity.y = crt_values[1];
			gyroSensitivity.z = crt_values[2];

			success = true;
		}

		// CRT seems to leave some state behind which isn't persisted after
		// restart. If we continue without restarting, the gyroscope will behave
		// differently on this run compared to subsequent restarts.
		restartAndInit();

		setNormalConfig(gyroSensitivity);

		return success;
	}

	float getDirectTemp() const {
		// middle value is 23 degrees C (0x0000)
		// temperature per step from -41 + 1/2^9 degrees C (0x8001) to 87 - 1/2^9
		// degrees C (0x7FFF)
		constexpr float TempStep = 128. / 65535;
		const auto value
			= static_cast<int16_t>(m_RegisterInterface.readReg16(Regs::TempData));
		return static_cast<float>(value) * TempStep + 23.0f;
	}

	using FifoBuffer = std::array<uint8_t, RegisterInterface::MaxTransactionLength>;
	FifoBuffer read_buffer;

	template <typename T>
	inline T getFromFifo(uint32_t& position, FifoBuffer& fifo) {
		T to_ret;
		std::memcpy(&to_ret, &fifo[position], sizeof(T));
		position += sizeof(T);
		return to_ret;
	}

	void bulkRead(DriverCallbacks<int16_t>&& callbacks) {
		const auto fifo_bytes = m_RegisterInterface.readReg16(Regs::FifoCount);

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
					using ShortLimit = std::numeric_limits<int16_t>;
					// apply zx factor, todo: this awful line should be simplified and
					// validated
					gyro[0] = std::clamp(
						static_cast<int32_t>(gyro[0])
							- static_cast<int16_t>(
								(static_cast<int32_t>(m_zxFactor) * gyro[2]) / 512
							),
						static_cast<int32_t>(ShortLimit::min()),
						static_cast<int32_t>(ShortLimit::max())
					);
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
