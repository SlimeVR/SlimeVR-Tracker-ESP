/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors

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

#include <array>
#include <cstdint>
#include <type_traits>

#include "../../motionprocessing/types.h"
#include "drivers/callbacks.h"

template <typename IMU>
struct IMUConsts {
	static constexpr bool Uses32BitSensorData
		= requires(IMU& i, DriverCallbacks<int32_t> callbacks) {
			  i.bulkRead(std::move(callbacks));
		  };

	static constexpr bool DirectTempReadOnly = requires(IMU& i) { i.getDirectTemp(); };

	using RawSensorT =
		typename std::conditional<Uses32BitSensorData, int32_t, int16_t>::type;
	using RawVectorT = std::array<RawSensorT, 3>;

	static constexpr float GScale
		= ((32768. / IMU::GyroSensitivity) / 32768.) * (PI / 180.0);
	static constexpr float AScale = CONST_EARTH_GRAVITY / IMU::AccelSensitivity;

	static constexpr float DirectTempReadFreq = 15;
	static constexpr float DirectTempReadTs = 1.0f / DirectTempReadFreq;
	static constexpr sensor_real_t getDefaultTempTs() {
		if constexpr (DirectTempReadOnly) {
			return DirectTempReadTs;
		} else {
			return IMU::TempTs;
		}
	}

	static constexpr bool SupportsMags = requires(IMU& i) { i.readAux(0x00); };
	static constexpr bool Supports9ByteMag = []() constexpr {
		if constexpr (requires { IMU::Supports9ByteMag; }) {
			return IMU::Supports9ByteMag;
		} else {
			return true;
		}
	}();
};
