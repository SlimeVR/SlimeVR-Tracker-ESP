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

#ifndef SENSORS_MPU9250SENSOR_H
#define SENSORS_MPU9250SENSOR_H

#include "sensor.h"
#include "logging/Logger.h"

#include <MPU9250_6Axis_MotionApps_V6_12.h>

class MPU9250Sensor : public Sensor
{
public:
    MPU9250Sensor(uint8_t id, uint8_t address, float rotation) : Sensor("MPU9250Sensor", IMU_MPU9250, id, address, rotation){};
    ~MPU9250Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;
    void getMPUScaled();

private:
    MPU9250 imu{};
    bool dmpReady = false;    // set true if DMP init was successful
    // TODO: actually check interrupt status
    // uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
    uint16_t packetSize;      // expected DMP packet size (default is 42 bytes)

    // raw data and scaled as vector
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f}; // for raw filter
    float Axyz[3]{};
    float Gxyz[3]{};
    float Mxyz[3]{};
    VectorInt16 rawAccel{};
    Quat correction{0, 0, 0, 0};
    // Loop timing globals
    float deltat = 0;                // sample time in seconds

    SlimeVR::Configuration::MPU9250CalibrationConfig m_Calibration;

    // outputs to respective member variables
    void parseAccelData(int16_t data[3]);
    void parseGyroData(int16_t data[3]);
    void parseMagData(int16_t data[3]);

    // 6 bytes for gyro, 6 bytes for accel, 7 bytes for magnetometer
    static constexpr uint16_t sensor_data_len = 19;

    struct fifo_sample {
        int16_t accel[3];
        int16_t gyro[3];
        int16_t mag[3];
        uint8_t mag_status;
    };

    // acts as a memory space for getNextSample. upon success, can read from the sample member
    // TODO: this may be overcomplicated, we may be able to just use fifo_sample and i misunderstood strict aliasing rules.
    union fifo_sample_raw {
        uint8_t raw[sensor_data_len];
        struct fifo_sample sample;
    };

    // returns true if sample was read, outputs number of waiting samples in remaining_count if not null.
    bool getNextSample(union fifo_sample_raw *buffer, uint16_t *remaining_count);
    static void swapFifoData(union fifo_sample_raw* sample);
};

#endif
