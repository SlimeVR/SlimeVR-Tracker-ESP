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

#ifndef SENSORS_LSM303SENSOR_H
#define SENSORS_LSM303SENSOR_H

#include "magneto1.4.h"
#include "sensor.h"
#include "mahony.h"
#include <LSM303.h>

class LSM303Sensor : public Sensor
{
public:
    LSM303Sensor(uint8_t id, uint8_t address, float rotation) : Sensor("LSM303Sensor", IMU_LSM303, id, address, rotation){};
    ~LSM303Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;
    void getScaledValues(float AGMxyz[9]);

private:
    LSM303 imu{};
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f};
    // Loop timing globals
    uint32_t now = 0, last = 0;
    float deltat = 0;

    SlimeVR::Configuration::LSM303CalibrationConfig m_Calibration;
};

#endif
