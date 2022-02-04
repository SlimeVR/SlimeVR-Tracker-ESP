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
#include <i2cscan.h>
#include "sensorfactory.h"

#include "bno055sensor.h"
#include "bno080sensor.h"
#include "mpu9250sensor.h"
#include "mpu6050sensor.h"
#include "bmi160sensor.h"
#include "icm20948sensor.h"

SensorFactory::SensorFactory()
{
}

SensorFactory::~SensorFactory()
{
    delete sensor1;
    delete sensor2;
}

void SensorFactory::create()
{
    uint8_t first_addr = 0;
    #if IMU == IMU_BNO080 || IMU == IMU_BNO085 || IMU == IMU_BNO086
        this->sensor1 = new BNO080Sensor();
        first_addr = I2CSCAN::pickDevice(0x4A, 0x4B, true);
    #elif IMU == IMU_BNO055
        this->sensor1 = new BNO055Sensor();
        first_addr = I2CSCAN::pickDevice(0x29, 0x28, true);
    #elif IMU == IMU_MPU9250
        this->sensor1 = new MPU9250Sensor();
        first_addr = I2CSCAN::pickDevice(0x68, 0x69, true);
    #elif IMU == IMU_BMI160
        this->sensor1 = new BMI160Sensor();
        first_addr = I2CSCAN::pickDevice(0x68, 0x69, true);
    #elif IMU == IMU_MPU6500 || IMU == IMU_MPU6050
        this->sensor1 = new MPU6050Sensor();
        first_addr = I2CSCAN::pickDevice(0x68, 0x69, true);
    #elif IMU == IMU_ICM20948
        this->sensor1 = new ICM20948Sensor();
        first_addr = I2CSCAN::pickDevice(0x68, 0x69, true);
    #else
    #error Unsupported IMU
    #endif

    if(first_addr == 0)
        this->sensor1 = new EmptySensor();
    this->sensor1->setupSensor(IMU, 0, first_addr, PIN_IMU_INT);

    uint8_t second_addr = 0;
    #ifndef SECOND_IMU
        this->sensor2 = new EmptySensor();
    #elif SECOND_IMU == IMU_BNO080 || SECOND_IMU == IMU_BNO085 || SECOND_IMU == IMU_BNO086
        this->sensor2 = new BNO080Sensor();
        second_addr = I2CSCAN::pickDevice(0x4B, 0x4A, false);
    #elif SECOND_IMU == IMU_BNO055
        this->sensor2 = new BNO055Sensor();
        second_addr = I2CSCAN::pickDevice(0x28, 0x29, false);
    #elif SECOND_IMU == IMU_MPU9250
        this->sensor2 = new MPU9250Sensor();
        second_addr = I2CSCAN::pickDevice(0x69, 0x68, false);
    #elif SECOND_IMU == IMU_BMI160
        this->sensor2 = new BMI160Sensor();
        second_addr = I2CSCAN::pickDevice(0x69, 0x68, false);
    #elif SECOND_IMU == IMU_MPU6500 || SECOND_IMU == IMU_MPU6050
        this->sensor2 = new MPU6050Sensor();
        second_addr = I2CSCAN::pickDevice(0x69, 0x68, false);
    #elif SECOND_IMU == IMU_ICM20948
        this->sensor2 = new ICM20948Sensor();
        second_addr = I2CSCAN::pickDevice(0x69, 0x68, true);
    #else
    #error Unsupported secondary IMU
    #endif

    if(first_addr == second_addr)
        this->sensor2 = new EmptySensor();
    this->sensor2->setupSensor(SECOND_IMU, 1, second_addr, PIN_IMU_INT_2);
}

void SensorFactory::motionSetup()
{
    sensor1->motionSetup();
    #ifdef SECOND_IMU
        sensor2->motionSetup();
    #endif
}

void SensorFactory::motionLoop()
{
    sensor1->motionLoop();
    #ifdef SECOND_IMU
        sensor2->motionLoop();
    #endif
}

void SensorFactory::sendData()
{
    sensor1->sendData();
    #ifdef SECOND_IMU
        sensor2->sendData();
    #endif
}

void SensorFactory::startCalibration(int sensorId, int calibrationType)
{
    switch(sensorId) {
        case 0:
            sensor1->startCalibration(calibrationType);
        break;
        case 1:
            sensor2->startCalibration(calibrationType);
        break;
    }
}