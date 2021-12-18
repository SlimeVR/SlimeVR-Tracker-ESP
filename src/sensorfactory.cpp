#include "sensorfactory.h"
#include <i2cscan.h>

SensorFactory::SensorFactory()
{
}

SensorFactory::~SensorFactory()
{
    delete sensor1;
    delete sensor2;
}

bool SensorFactory::create()
{
    uint8_t first_addr = 0;
    #if IMU == IMU_BNO080 || IMU == IMU_BNO085
        this->sensor1 = new BNO080Sensor();
        first_addr = I2CSCAN::pickDevice(0x4A, 0x4B, true);
    #elif IMU == IMU_BNO055
        this->sensor1 = new BNO055Sensor();
        first_addr = I2CSCAN::pickDevice(0x29, 0x28, true);
    #elif IMU == IMU_MPU9250
        this->sensor1 = new MPU9250Sensor();
        first_addr = I2CSCAN::pickDevice(0x68, 0x69, true);
    #elif IMU == IMU_MPU6500 || IMU == IMU_MPU6050
        this->sensor1 = new MPU6050Sensor();
        first_addr = I2CSCAN::pickDevice(0x68, 0x69, true);
    #else
    #error Unsupported IMU
    #endif

    if (first_addr == 0)
        return false;
    this->sensor1->setupSensor(0, first_addr, PIN_IMU_INT);

    // Deprecated block: Checks for old type of config. Remove after a grace period to allow people to use outdated defines.h for a while.
    #ifndef SECOND_IMU
    #if IMU == IMU_BNO085
    #define SECOND_IMU IMU_BNO085
    #elif IMU == IMU_BNO080
    #define SECOND_IMU IMU_BNO080
    #elif IMU == IMU_MPU6500
    #define SECOND_IMU IMU_MPU6500
    #elif IMU == IMU_MPU6050
    #define SECOND_IMU IMU_MPU6050
    #endif
    #endif
    // End deprecated block

    uint8_t second_addr = 0;
    #ifndef SECOND_IMU
        this->sensor2 = new EmptySensor();
    #elif SECOND_IMU == IMU_BNO080 || SECOND_IMU == IMU_BNO085
        this->sensor2 = new BNO080Sensor();
        second_addr = I2CSCAN::pickDevice(0x4B, 0x4A, false);
    #elif IMU == IMU_BNO055
        this->sensor2 = new BNO055Sensor();
        second_addr = I2CSCAN::pickDevice(0x28, 0x29, false);
    #elif IMU == IMU_MPU9250
        this->sensor2 = new MPU9250Sensor();
        second_addr = I2CSCAN::pickDevice(0x69, 0x68, false);
    #elif SECOND_IMU == IMU_MPU6500 || SECOND_IMU == IMU_MPU6050
        this->sensor2 = new MPU6050Sensor();
        second_addr = I2CSCAN::pickDevice(0x69, 0x68, false);
    #else
    #error Unsupported secondary IMU
    #endif

    if (first_addr == second_addr)
        this->sensor2 = new EmptySensor();
    this->sensor2->setupSensor(1, second_addr, PIN_IMU_INT_2);
    return true;
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

void SensorFactory::startCalibration(int calibrationType)
{
    sensor1->startCalibration(calibrationType);
    // #ifdef SECOND_IMU
    // sensor2->startCalibration(calibrationType);
    // #endif
}