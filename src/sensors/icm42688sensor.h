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

#ifndef SENSORS_ICM42688SENSOR_H
#define SENSORS_ICM42688SENSOR_H

#include "sensor.h"
#include "logging/Logger.h"

#include <ICM42688.h>
#include <MMC5983MA.h>
#include "I2Cdev.h"

class ICM42688Sensor : public Sensor
{
public:
    ICM42688Sensor(uint8_t id, uint8_t address, float rotation) : Sensor("ICM42688Sensor", IMU_ICM42688, id, address, rotation){};
    ~ICM42688Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;

private:
    uint8_t addr;
    uint8_t addr_mag = 0x30;
    bool magExists = false;

    // raw data and scaled as vector
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f}; // for raw filter
    float Axyz[3]{};
    float Gxyz[3]{};
    float Mxyz[3]{};
    Quat correction{0, 0, 0, 0};
    // Loop timing globals
    float deltat = 0;                // sample time in seconds

    SlimeVR::Configuration::ICM42688CalibrationConfig m_Calibration;

    void accel_read();
    void gyro_read();
    void mag_read();

    void parseAccelData();
    void parseGyroData();
    void parseMagData();
};

#endif
