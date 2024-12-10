/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 TheDevMinerTV

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

#include "SensorConfig.h"

namespace SlimeVR {
namespace Configuration {
const char* calibrationConfigTypeToString(SensorConfigType type) {
	switch (type) {
		case SensorConfigType::NONE:
			return "NONE";
		case SensorConfigType::BMI160:
			return "BMI160";
		case SensorConfigType::MPU6050:
			return "MPU6050";
		case SensorConfigType::MPU9250:
			return "MPU9250";
		case SensorConfigType::ICM20948:
			return "ICM20948";
		case SensorConfigType::SFUSION:
			return "SoftFusion (common)";
		case SensorConfigType::BNO0XX:
			return "BNO0XX";
		default:
			return "UNKNOWN";
	}
}

// 1st bit specifies if magnetometer is enabled or disabled
// 2nd bit specifies if magnetometer is supported
uint16_t configDataToNumber(SensorConfig* sensorConfig, bool magSupported) {
	uint16_t data = 0;
	data += magSupported << 1;
	switch (sensorConfig->type) {
		case SensorConfigType::BNO0XX: {
			auto config = &sensorConfig->data.bno0XX;
			data += config->magEnabled;
			break;
		}
		case SensorConfigType::NONE:
		default:
			break;
	}
	return data;
}
}  // namespace Configuration
}  // namespace SlimeVR
