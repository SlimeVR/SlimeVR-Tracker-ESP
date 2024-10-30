/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 furrycoding & SlimeVR Contributors

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

#include <MPU6050.h>

#include <algorithm>
#include <array>
#include <cstdint>

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 8g
// and gyroscope range at 1000dps
// Gyroscope ODR = accel ODR = 250Hz

template <typename I2CImpl>
struct MPU6050 {
	struct FifoSample {
		uint8_t accel_x_h, accel_x_l;
		uint8_t accel_y_h, accel_y_l;
		uint8_t accel_z_h, accel_z_l;

		// We don't need temperature in FIFO
		// uint8_t temp_h, temp_l;

		uint8_t gyro_x_h, gyro_x_l;
		uint8_t gyro_y_h, gyro_y_l;
		uint8_t gyro_z_h, gyro_z_l;
	};
#define MPU6050_FIFO_VALUE(fifo, name) \
	(((int16_t)fifo->name##_h << 8) | ((int16_t)fifo->name##_l))

	static constexpr uint8_t Address = 0x68;
	static constexpr auto Name = "MPU-6050";
	static constexpr auto Type = ImuID::MPU6050;

	static constexpr float Freq = 250;

	static constexpr float GyrTs = 1.0 / Freq;
	static constexpr float AccTs = 1.0 / Freq;
	static constexpr float MagTs = 1.0 / Freq;

	static constexpr float GyroSensitivity = 32.8f;
	static constexpr float AccelSensitivity = 4096.0f;

	I2CImpl i2c;
	SlimeVR::Logging::Logger& logger;
	MPU6050(I2CImpl i2c, SlimeVR::Logging::Logger& logger)
		: i2c(i2c)
		, logger(logger) {}

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x75;
			static constexpr uint8_t value = 0x68;
		};

		struct UserCtrl {
			static constexpr uint8_t reg = 0x6A;
			static constexpr uint8_t fifoResetValue
				= (1 << MPU6050_USERCTRL_FIFO_EN_BIT)
				| (1 << MPU6050_USERCTRL_FIFO_RESET_BIT);
		};

		struct GyroConfig {
			static constexpr uint8_t reg = 0x1b;
			static constexpr uint8_t value = 0b10 << 3;  // 1000dps
		};

		struct AccelConfig {
			static constexpr uint8_t reg = 0x1c;
			static constexpr uint8_t value = 0b10 << 3;  // 8g
		};

		static constexpr uint8_t OutTemp = MPU6050_RA_TEMP_OUT_H;

		static constexpr uint8_t IntStatus = MPU6050_RA_INT_STATUS;

		static constexpr uint8_t FifoCount = MPU6050_RA_FIFO_COUNTH;
		static constexpr uint8_t FifoData = MPU6050_RA_FIFO_R_W;
	};

	inline uint16_t byteSwap(uint16_t value) const {
		// Swap bytes because MPU is big-endian
		return (value >> 8) | (value << 8);
	}

	void resetFIFO() {
		i2c.writeReg(Regs::UserCtrl::reg, Regs::UserCtrl::fifoResetValue);
	}

	bool initialize() {
		// Reset
		i2c.writeReg(
			MPU6050_RA_PWR_MGMT_1,
			0x80
		);  // PWR_MGMT_1: reset with 100ms delay (also disables sleep)
		delay(100);
		i2c.writeReg(
			MPU6050_RA_SIGNAL_PATH_RESET,
			0x07
		);  // full SIGNAL_PATH_RESET: with another 100ms delay
		delay(100);

		// Configure
		i2c.writeReg(
			MPU6050_RA_PWR_MGMT_1,
			0x01
		);  // 0000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
		i2c.writeReg(
			MPU6050_RA_USER_CTRL,
			0x00
		);  // 0000 0000 USER_CTRL: Disable FIFO / I2C master / DMP
		i2c.writeReg(
			MPU6050_RA_INT_ENABLE,
			0x10
		);  // 0001 0000 INT_ENABLE: only FIFO overflow interrupt
		i2c.writeReg(Regs::GyroConfig::reg, Regs::GyroConfig::value);
		i2c.writeReg(Regs::AccelConfig::reg, Regs::AccelConfig::value);
		i2c.writeReg(
			MPU6050_RA_CONFIG,
			0x02
		);  // 0000 0010 CONFIG: No EXT_SYNC_SET, DLPF set to 98Hz(also lowers gyro
			// output rate to 1KHz)
		i2c.writeReg(
			MPU6050_RA_SMPLRT_DIV,
			0x03
		);  // 0000 0011 SMPLRT_DIV: Divides the internal sample rate 250Hz (Sample Rate
			// = Gyroscope Output Rate / (1 + SMPLRT_DIV))

		i2c.writeReg(
			MPU6050_RA_FIFO_EN,
			0x78
		);  // 0111 1000 FIFO_EN: All gyro axes + Accel

		resetFIFO();

		return true;
	}

	float getDirectTemp() const {
		auto value = byteSwap(i2c.readReg16(Regs::OutTemp));
		float result = (static_cast<int16_t>(value) / 340.0f) + 36.53f;
		return result;
	}

	template <typename AccelCall, typename GyroCall>
	void bulkRead(AccelCall&& processAccelSample, GyroCall&& processGyroSample) {
		const auto status = i2c.readReg(Regs::IntStatus);

		if (status & (1 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) {
			// Overflows make it so we lose track of which packet is which
			// This necessitates a reset
			logger.debug("Fifo overrun, resetting...");
			resetFIFO();
			return;
		}

		std::array<uint8_t, 12 * 10>
			readBuffer;  // max 10 packages of 12byte values (sample) of data form fifo
		auto byteCount = byteSwap(i2c.readReg16(Regs::FifoCount));

		auto readBytes = min(static_cast<size_t>(byteCount), readBuffer.size())
					   / sizeof(FifoSample) * sizeof(FifoSample);
		if (!readBytes) {
			return;
		}

		i2c.readBytes(Regs::FifoData, readBytes, readBuffer.data());
		for (auto i = 0u; i < readBytes; i += sizeof(FifoSample)) {
			const FifoSample* sample = reinterpret_cast<FifoSample*>(&readBuffer[i]);

			int16_t xyz[3];

			xyz[0] = MPU6050_FIFO_VALUE(sample, accel_x);
			xyz[1] = MPU6050_FIFO_VALUE(sample, accel_y);
			xyz[2] = MPU6050_FIFO_VALUE(sample, accel_z);
			processAccelSample(xyz, AccTs);

			xyz[0] = MPU6050_FIFO_VALUE(sample, gyro_x);
			xyz[1] = MPU6050_FIFO_VALUE(sample, gyro_y);
			xyz[2] = MPU6050_FIFO_VALUE(sample, gyro_z);
			processGyroSample(xyz, GyrTs);
		}
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
