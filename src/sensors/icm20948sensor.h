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
#ifndef SLIMEVR_ICM20948SENSOR_H_
#define SLIMEVR_ICM20948SENSOR_H_

#include <ICM_20948.h>

#include "SensorFusionDMP.h"
#include "sensor.h"

class ICM20948Sensor : public Sensor {
public:
	static constexpr auto TypeID = ImuID::ICM20948;
	static constexpr uint8_t Address = 0x68;

	ICM20948Sensor(
		uint8_t id,
		uint8_t i2cAddress,
		float rotation,
		uint8_t sclPin,
		uint8_t sdaPin,
		uint8_t
	)
		: Sensor(
			"ICM20948Sensor",
			ImuID::ICM20948,
			id,
			i2cAddress,
			rotation,
			sclPin,
			sdaPin
		) {}
	~ICM20948Sensor() override = default;
	void motionSetup() override final;
	void postSetup() override { this->lastData = millis(); }

	void motionLoop() override final;
	void sendData() override final;
	void startCalibration(int calibrationType) override final;

private:
	void calculateAccelerationWithoutGravity(Quat* quaternion);
	unsigned long lastData = 0;
	int bias_save_counter = 0;
	bool hasdata = false;
	// Performance test
	/*
		uint8_t cntbuf = 0;
		int32_t cntrounds = 0;
		unsigned long lastData2 = 0;
	*/

#define DMPNUMBERTODOUBLECONVERTER 1073741824.0;

	ICM_20948_I2C imu;
	ICM_20948_Device_t pdev;
	icm_20948_DMP_data_t dmpData{};
	icm_20948_DMP_data_t dmpDataTemp{};

	SlimeVR::Configuration::ICM20948SensorConfig m_Config = {};

	SlimeVR::Sensors::SensorFusionDMP sfusion;

	void saveCalibration(bool repeat);
	void loadCalibration();
	void startCalibrationAutoSave();
	void startDMP();
	void connectSensor();
	void startMotionLoop();
	void checkSensorTimeout();
	void readRotation();
	void readFIFOToEnd();

#define OVERRIDEDMPSETUP true
	// TapDetector tapDetector;
};

#endif  // SLIMEVR_ICM20948SENSOR_H_
