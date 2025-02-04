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

#ifndef SLIMEVR_SENSOR_H_
#define SLIMEVR_SENSOR_H_

#include <Arduino.h>
#include <quat.h>
#include <vector3.h>

#include <memory>

#include "PinInterface.h"
#include "configuration/Configuration.h"
#include "globals.h"
#include "logging/Logger.h"
#include "sensorinterface/SensorInterface.h"
#include "status/TPSCounter.h"
#include "utils.h"

#define DATA_TYPE_NORMAL 1
#define DATA_TYPE_CORRECTION 2

enum class SensorStatus : uint8_t {
	SENSOR_OFFLINE = 0,
	SENSOR_OK = 1,
	SENSOR_ERROR = 2
};

enum class MagnetometerStatus : uint8_t {
	MAG_NOT_SUPPORTED = 0,
	MAG_DISABLED = 1,
	MAG_ENABLED = 2,
};

class Sensor {
public:
	Sensor(
		const char* sensorName,
		ImuID type,
		uint8_t id,
		uint8_t address,
		float rotation,
		SlimeVR::SensorInterface* sensorInterface = nullptr
	)
		: m_hwInterface(sensorInterface)
		, addr(address)
		, sensorId(id)
		, sensorType(type)
		, sensorOffset({Quat(Vector3(0, 0, 1), rotation)})
		, m_Logger(SlimeVR::Logging::Logger(sensorName)) {
		char buf[4];
		sprintf(buf, "%u", id);
		m_Logger.setTag(buf);
	}

	virtual ~Sensor(){};
	virtual void motionSetup(){};
	virtual void postSetup(){};
	virtual void motionLoop(){};
	virtual void sendData();
	virtual void setAcceleration(Vector3 a);
	virtual void setFusedRotation(Quat r);
	virtual void startCalibration(int calibrationType){};
	virtual SensorStatus getSensorState();
	virtual void printTemperatureCalibrationState();
	virtual void printDebugTemperatureCalibrationState();
	virtual void resetTemperatureCalibrationState();
	virtual void saveTemperatureCalibration();
	virtual void setFlag(uint16_t flagId, bool state){};
	virtual uint16_t getSensorConfigData();
	bool isWorking() { return working; };
	bool getHadData() const { return hadData; };
	bool isValid() { return m_hwInterface != nullptr; };
	bool isMagEnabled() { return magStatus == MagnetometerStatus::MAG_ENABLED; };
	uint8_t getSensorId() { return sensorId; };
	ImuID getSensorType() { return sensorType; };
	MagnetometerStatus getMagStatus() { return magStatus; };
	const Vector3& getAcceleration() { return acceleration; };
	const Quat& getFusedRotation() { return fusedRotation; };
	bool hasNewDataToSend() { return newFusedRotation || newAcceleration; };
	inline bool hasCompletedRestCalibration() { return restCalibrationComplete; }

	virtual uint8_t getDataType() { return SENSOR_DATATYPE_ROTATION; };

	uint16_t getSensorPosition() { return m_SensorPosition; };

	void setSensorInfo(uint16_t sensorPosition) { m_SensorPosition = sensorPosition; };

	TPSCounter m_tpsCounter;
	TPSCounter m_dataCounter;
	std::shared_ptr<SlimeVR::SensorInterface> m_hwInterface;

protected:
	uint8_t addr = 0;
	uint8_t sensorId = 0;
	ImuID sensorType = ImuID::Unknown;
	bool working = false;
	bool hadData = false;
	uint8_t calibrationAccuracy = 0;
	MagnetometerStatus magStatus = MagnetometerStatus::MAG_NOT_SUPPORTED;
	Quat sensorOffset;

	bool newFusedRotation = false;
	Quat fusedRotation{};
	Quat lastFusedRotationSent{};

	bool newAcceleration = false;
	Vector3 acceleration{};

	uint16_t m_SensorPosition = POSITION_NO;
	
	void markRestCalibrationComplete(bool completed = true);

	mutable SlimeVR::Logging::Logger m_Logger;

private:
	void printTemperatureCalibrationUnsupported();

	bool restCalibrationComplete;
};

const char* getIMUNameByType(ImuID imuType);

#endif  // SLIMEVR_SENSOR_H_
