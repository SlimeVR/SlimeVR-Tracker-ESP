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
#include "sensor.h"
#include "network/network.h"
#include <i2cscan.h>
#include "calibration.h"

uint8_t Sensor::getSensorState() {
    return isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

void Sensor::sendData() {
    if(newData) {
        newData = false;
        Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, calibrationAccuracy, sensorId);

#if SEND_ACCELERATION
        {
            Network::sendAccel(acceleration, sensorId);
        }
#endif

#ifdef DEBUG_SENSOR
        m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(quaternion));
#endif
    }
}

const char * getIMUNameByType(int imuType) {
    switch(imuType) {
        case IMU_MPU9250:
            return "MPU9250";
        case IMU_MPU6500:
            return "MPU6500";
        case IMU_BNO080:
            return "BNO080";
        case IMU_BNO085:
            return "BNO085";
        case IMU_BNO055:
            return "BNO055";
        case IMU_MPU6050:
            return "MPU6050";
        case IMU_BNO086:
            return "BNO086";
        case IMU_BMI160:
            return "BMI160";
        case IMU_ICM20948:
            return "ICM20948";
    }
    return "Unknown";
}
