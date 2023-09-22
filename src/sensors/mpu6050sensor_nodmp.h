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

#ifndef SENSORS_MPU6050SENSOR_NODMP_H
#define SENSORS_MPU6050SENSOR_NODMP_H

#include "sensor.h"
#include <MPU6050.h>
#include "SensorFusionRestDetect.h"

#include "axisremap.h"


#include "defines_mpu6050.h"


#define HIGH_SPEED false


#if HIGH_SPEED

#define MPU6050_SAMPLE_RATE 800

#else

// With digital low-pass filter enabled we get 250Hz sample rate
#define MPU6050_SAMPLE_RATE 250

#endif

class MPU6050NoDMPSensor : public Sensor
{
public:
    MPU6050NoDMPSensor(uint8_t id, uint8_t type, uint8_t address, float rotation, uint8_t sclPin, uint8_t sdaPin, int axisRemap=AXIS_REMAP_DEFAULT)
        : Sensor("MPU6050Sensor_NoDMP", type, id, address, rotation, sclPin, sdaPin),
        axisRemap(axisRemap),
        sfusion(1.0f / ((float)MPU6050_SAMPLE_RATE))
        {};
    ~MPU6050NoDMPSensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;

    void sendData() override final;

    void printTemperatureCalibrationState() override final;
    void printDebugTemperatureCalibrationState() override final;
    void resetTemperatureCalibrationState() override final {
        gyroTempCalibrator->reset();
        m_Logger.info("Temperature calibration state has been reset for sensorId:%i", sensorId);
    };
    void saveTemperatureCalibration() override final;

public:
    void getRemappedRotation(int16_t* x, int16_t* y, int16_t* z) {
        imu.getRotation(x, y, z);
        remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), x, y, z);
    }
    void getRemappedAcceleration(int16_t* x, int16_t* y, int16_t* z) {
        imu.getAcceleration(x, y, z);
        remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), x, y, z);
    }

private:
    void onRawGyroSample(int16_t x, int16_t y, int16_t z);
    void onRawAccelSample(int16_t x, int16_t y, int16_t z);
    void onRawTempSample(int16_t t);

    void checkFlipCalibration();
    void calibrateGyro();
    void calibrateAccel();

private:
    MPU6050 imu{};
    Quaternion rawQuat{};
    VectorInt16 rawAccel{};

    bool initialized = false;
    GyroTemperatureCalibrator* gyroTempCalibrator = nullptr;
    // An offset to the whole temperature calibration curve,
    // such that, if we are at the temperature we did static calibration at,
    // the final estimated bias will be the same as we calculated during static calibration
    sensor_real_t tempCalOffset[3];

    int axisRemap;

    bool newTemperature;
    float temperature;

    SlimeVR::Sensors::SensorFusionRestDetect sfusion;
    SlimeVR::Configuration::BMI160CalibrationConfig m_Calibration;
};

#endif
