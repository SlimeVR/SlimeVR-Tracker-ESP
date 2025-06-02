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

std::vector<MagDefinition> MagDriver::supportedMags{
	MagDefinition{
		.name = "QMC6309",

		.deviceId = 0x7c,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x90,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x01,

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x0b, 0x80);
				interface.writeByte(0x0b, 0x00);  // Soft reset
				delay(10);
				interface.writeByte(0x0b, 0x48);  // Set/reset on, 8g full range, 200Hz
				interface.writeByte(
					0x0a,
					0x21
				);  // LP filter 2, 8x Oversampling, normal mode
				return true;
			},
	},
	MagDefinition{
		.name = "IST8306",

		.deviceId = 0x19,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x06,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x11,

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x32, 0x01);  // Soft reset
				delay(50);
				interface.writeByte(0x30, 0x20);  // Noise suppression: low
				interface.writeByte(0x41, 0x2d);  // Oversampling: 32X
				interface.writeByte(0x31, 0x02);  // Continuous measurement @ 10Hz
				return true;
			},
	},
};

bool MagDriver::init(MagInterface&& interface, bool supports9ByteMags) {
	for (auto& mag : supportedMags) {
		interface.setDeviceId(mag.deviceId);

		logger.info("Trying mag %s!", mag.name);

		uint8_t whoAmI = interface.readByte(mag.whoAmIReg);
		if (whoAmI != mag.expectedWhoAmI) {
			continue;
		}

		if (!supports9ByteMags && mag.dataWidth == MagDataWidth::NineByte) {
			logger.error("The sensor doesn't support this mag!");
			return false;
		}

		logger.info("Found mag %s! Initializing", mag.name);

		if (!mag.setup(interface)) {
			logger.error("Mag %s failed to initialize!", mag.name);
			return false;
		}

		detectedMag = mag;

		break;
	}

	this->interface = interface;
	return detectedMag.has_value();
}

void MagDriver::startPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.startPolling(detectedMag->dataReg, detectedMag->dataWidth);
}

void MagDriver::stopPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.stopPolling();
}

const char* MagDriver::getAttachedMagName() const {
	if (!detectedMag) {
		return nullptr;
	}

	return detectedMag->name;
}

}  // namespace SlimeVR::Sensors::SoftFusion
