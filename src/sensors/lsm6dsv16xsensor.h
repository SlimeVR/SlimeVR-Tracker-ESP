/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 Eiren Rain & SlimeVR contributors

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

#ifndef SENSORS_LSM6DSV16X_H
#define SENSORS_LSM6DSV16X_H

#include <LSM6DSV16X.h>

#include "sensor.h"

#ifndef LSM6DSV16X_ACCEL_MAX
#define LSM6DSV16X_ACCEL_MAX 4
#endif

#ifndef LSM6DSV16X_GYRO_MAX
#define LSM6DSV16X_GYRO_MAX 2000
#endif

#ifndef LSM6DSV16X_FIFO_MAX_ENTRIES
#define LSM6DSV16X_FIFO_MAX_ENTRIES 32
#endif

#ifndef LSM6DSV16X_FIFO_DATA_RATE
#define LSM6DSV16X_FIFO_DATA_RATE 120
#endif

class LSM6DSV16XSensor : public Sensor {
public:
	LSM6DSV16XSensor(
		uint8_t id,
		uint8_t type,
		uint8_t address,
		float rotation,
		uint8_t sclPin,
		uint8_t sdaPin,
		uint8_t intPin
	)
		: Sensor("LSM6DSV16XSensor", type, id, address, rotation, sclPin, sdaPin)
		, imu(&Wire, addr)
		, m_IntPin(intPin){};
	~LSM6DSV16XSensor(){};
	void motionSetup() override final;
	void postSetup() override { lastData = millis(); }

	void motionLoop() override final;
	void sendData() override final;
	void startCalibration(int calibrationType) override final;
	SensorStatus getSensorState() override final;

private:
	// void interruptHandler();
	// volatile bool imuEvent; //the interrupt cant be a class function

	LSM6DSV16X imu;
	uint8_t m_IntPin;
	uint8_t errorCounter = 0;  // Error is -1, OK is 0
	uint8_t tap = 0;
	unsigned long lastData = 0;

	uint8_t lastReset = 0;
};

#endif
