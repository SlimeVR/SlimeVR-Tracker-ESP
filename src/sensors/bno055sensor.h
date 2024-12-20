/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#ifndef SENSORS_BNO055SENSOR_H
#define SENSORS_BNO055SENSOR_H

#include <Adafruit_BNO055.h>

#include "sensor.h"

class BNO055Sensor : public Sensor {
public:
	static constexpr auto TypeID = ImuID::BNO055;
	static constexpr uint8_t Address = 0x28;

	BNO055Sensor(
		uint8_t id,
		uint8_t i2cAddress,
		float rotation,
		uint8_t sclPin,
		uint8_t sdaPin,
		uint8_t
	)
		: Sensor(
			"BNO055Sensor",
			ImuID::BNO055,
			id,
			i2cAddress,
			rotation,
			sclPin,
			sdaPin
		){};
	~BNO055Sensor(){};
	void motionSetup() override final;
	void motionLoop() override final;
	void startCalibration(int calibrationType) override final;

private:
	Adafruit_BNO055 imu;
	SlimeVR::Configuration::BNO0XXSensorConfig m_Config = {};
};

#endif
