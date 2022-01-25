/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

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
#include "configuration.h"
#include "globals.h"

#define DATA_TYPE_NORMAL 1
#define DATA_TYPE_CORRECTION 2

class Sensor
{
public:
    Sensor(){};
    virtual ~Sensor(){};
    void setupSensor(uint8_t expectedSensorType, uint8_t sensorId, uint8_t addr, uint8_t intPin);
    virtual void motionSetup(){};
    virtual void motionLoop(){};
    virtual void sendData();
    virtual void startCalibration(int calibrationType){};
    virtual uint8_t getSensorState();
    bool isWorking()
    {
        return working;
    };
    uint8_t getSensorId() {
        return sensorId;
    };
    uint8_t getSensorType() {
        return sensorType;
    };

protected:
    uint8_t addr = 0;
    uint8_t intPin = 255;
    uint8_t sensorId = 0;
    uint8_t sensorType = 0;
    bool configured = false;
    bool newData = false;
    bool working = false;
    uint8_t calibrationAccuracy = 0;
    Quat sensorOffset = Quat(Vector3(0, 0, 1), IMU_ROTATION);

    Quat quaternion{};
    Quat lastQuatSent{};
};

class EmptySensor : public Sensor
{
public:
    EmptySensor(){};
    ~EmptySensor(){};
    void motionSetup() override final{};
    void motionLoop() override final{};
    void sendData() override final{};
    void startCalibration(int calibrationType) override final{};
};

const char * getIMUNameByType(int imuType);

enum SensorStatus {
    SENSOR_OFFLINE = 0,
    SENSOR_OK = 1,
    SENSOR_ERROR = 2
};


#endif // SLIMEVR_SENSOR_H_
