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

#include "magdriver.h"

namespace SlimeVR::Sensors::SoftFusion {

std::vector<MagDefinition> MagDriver::mags = {MagDefinition{
	.name = "IST8306",

	.deviceId = 0x19,

	.whoAmIReg = 0x00,
	.expectedWhoAmI = 0x06,

	.setup =
		[](const I2CWriteFunc& writeI2C) {
			writeI2C(0x32, 0x01);  // Soft reset
			delay(50);
			writeI2C(0x30, 0x20);  // Noise suppression: low
			writeI2C(0x41, 0x2d);  // Oversampling: 32X
			writeI2C(0x31, 0x02);  // Continuous measurement @ 10Hz
		},

	.dataReg = 0x11,
	.dataWidth = MagDefinition::DataWidth::SixByte,

	.resolution = 0.3,
}};

void MagDriver::init(AuxInterface auxInterface) {
	for (auto& mag : mags) {
		auxInterface.setId(mag.deviceId);
		m_Logger.info("Trying mag %s", mag.name);
		if (auxInterface.readI2C(mag.whoAmIReg) != mag.expectedWhoAmI) {
			continue;
		}

		selectedMag = mag;
		state = State::Ok;

		m_Logger.info("Found mag of type %s, initializing!", mag.name);
		auxInterface.setByteWidth(mag.dataWidth);
		selectedMag.setup(auxInterface.writeI2C);
		auxInterface.setupPolling(mag.dataReg, mag.dataWidth);

		return;
	}

	m_Logger.info("No mag found!");
}

#pragma pack(push, 1)
struct ThreeByteValues {
	int32_t x : 24;
	int32_t y : 24;
	int32_t z : 24;
};
#pragma pack(pop)
static_assert(sizeof(ThreeByteValues) == 9);

void MagDriver::scaleMagSample(const uint8_t* rawData, float outData[3]) {
	if (state != State::Ok) {
		return;
	}

	int32_t rawDataConcat[3];

	if (selectedMag.dataWidth == MagDefinition::DataWidth::SixByte) {
		rawDataConcat[0] = reinterpret_cast<const int16_t*>(rawData)[0];
		rawDataConcat[1] = reinterpret_cast<const int16_t*>(rawData)[1];
		rawDataConcat[2] = reinterpret_cast<const int16_t*>(rawData)[2];
	} else {
		rawDataConcat[0] = reinterpret_cast<const ThreeByteValues*>(rawData)->x;
		rawDataConcat[1] = reinterpret_cast<const ThreeByteValues*>(rawData)->y;
		rawDataConcat[2] = reinterpret_cast<const ThreeByteValues*>(rawData)->z;
	}

	outData[0] = rawDataConcat[0] * selectedMag.resolution;
	outData[1] = rawDataConcat[1] * selectedMag.resolution;
	outData[2] = rawDataConcat[2] * selectedMag.resolution;
}

}  // namespace SlimeVR::Sensors::SoftFusion
