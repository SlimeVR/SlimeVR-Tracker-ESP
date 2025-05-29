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
	},
	MagDefinition{
		.name = "IST8306",

		.deviceId = 0x19,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x06,
	},
};

bool MagDriver::init(MagInterface&& interface) {
	for (auto& mag : supportedMags) {
		interface.setDeviceId(mag.deviceId);

		logger.info("Trying mag %s!", mag.name);

		uint8_t whoAmI = interface.readByte(mag.whoAmIReg);
		if (whoAmI != mag.expectedWhoAmI) {
			continue;
		}

		// TODO: check mag compatibility

		detectedMag = mag;

		logger.info("Found mag %s! Initializing", mag.name);
		// TODO: initialize mag

		break;
	}

	this->interface = interface;

	return detectedMag.has_value();
}

}  // namespace SlimeVR::Sensors::SoftFusion
