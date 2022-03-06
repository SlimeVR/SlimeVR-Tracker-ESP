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

#ifndef SLIMEVR_PACKETS_H_
#define SLIMEVR_PACKETS_H_

#include "sensors/sensor.h"

#define PACKET_HEARTBEAT 0
//#define PACKET_ROTATION 1 // Deprecated
//#define PACKET_GYRO 2 // Deprecated
#define PACKET_HANDSHAKE 3
#define PACKET_ACCEL 4
//#define PACKET_MAG 5 // Deprecated
#define PACKET_RAW_CALIBRATION_DATA 6
#define PACKET_CALIBRATION_FINISHED 7
#define PACKET_CONFIG 8
//#define PACKET_RAW_MAGNETOMETER 9 // Deprecated
#define PACKET_PING_PONG 10
#define PACKET_SERIAL 11
#define PACKET_BATTERY_LEVEL 12
#define PACKET_TAP 13
#define PACKET_ERROR 14
#define PACKET_SENSOR_INFO 15
//#define PACKET_ROTATION_2 16 // Deprecated
#define PACKET_ROTATION_DATA 17
#define PACKET_MAGNETOMETER_ACCURACY 18
#define PACKET_SIGNAL_STRENGTH 19
#define PACKET_TEMPERATURE 20

#define PACKET_INSPECTION 105 // 0x69

#define PACKET_RECEIVE_HEARTBEAT 1
#define PACKET_RECEIVE_VIBRATE 2
#define PACKET_RECEIVE_HANDSHAKE 3
#define PACKET_RECEIVE_COMMAND 4

#define PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA 1
#define PACKET_INSPECTION_PACKETTYPE_FUSED_IMU_DATA 2
#define PACKET_INSPECTION_PACKETTYPE_CORRECTION_DATA 3
#define PACKET_INSPECTION_DATATYPE_INT 1
#define PACKET_INSPECTION_DATATYPE_FLOAT 2

namespace Network {
    // PACKET_HEARTBEAT 0
    void sendHeartbeat();

    // PACKET_HANDSHAKE 3
    void sendHandshake();

    // PACKET_ACCEL 4
    void sendAccel(float* vector, uint8_t sensorId);

    // PACKET_RAW_CALIBRATION_DATA 6
    void sendRawCalibrationData(float* vector, uint8_t calibrationType, uint8_t sensorId);
    void sendRawCalibrationData(int* vector, uint8_t calibrationType, uint8_t sensorId);

    // PACKET_CALIBRATION_FINISHED 7
    void sendCalibrationFinished(uint8_t calibrationType, uint8_t sensorId);

    // PACKET_BATTERY_LEVEL 12
    void sendBatteryLevel(float batteryVoltage, float batteryPercentage);

    // PACKET_TAP 13
    void sendTap(uint8_t value, uint8_t sensorId);

    // PACKET_ERROR 14
    void sendError(uint8_t reason, uint8_t sensorId);

    // PACKET_SENSOR_INFO 15
    void sendSensorInfo(Sensor * sensor);

    // PACKET_ROTATION_DATA 17
    void sendRotationData(Quat * const quaternion, uint8_t dataType, uint8_t accuracyInfo, uint8_t sensorId);

    // PACKET_MAGNETOMETER_ACCURACY 18
    void sendMagnetometerAccuracy(float accuracyInfo, uint8_t sensorId);

    // PACKET_SIGNAL_STRENGTH 19
    void sendSignalStrength(uint8_t signalStrength);

    // PACKET_TEMPERATURE 20
    void sendTemperature(float temperature, uint8_t sensorId);

#if ENABLE_INSPECTION
    void sendInspectionRawIMUData(uint8_t sensorId, int16_t rX, int16_t rY, int16_t rZ, uint8_t rA, int16_t aX, int16_t aY, int16_t aZ, uint8_t aA, int16_t mX, int16_t mY, int16_t mZ, uint8_t mA);
    void sendInspectionRawIMUData(uint8_t sensorId, float rX, float rY, float rZ, uint8_t rA, float aX, float aY, float aZ, uint8_t aA, float mX, float mY, float mZ, uint8_t mA);

    void sendInspectionFusedIMUData(uint8_t sensorId, Quat quaternion);

    void sendInspectionCorrectionData(uint8_t sensorId, Quat quaternion);
#endif
}

namespace DataTransfer {
    bool beginPacket();
    bool endPacket();
    void sendPacketType(uint8_t type);
    void sendPacketNumber();

    void sendFloat(float f);
    void sendByte(uint8_t c);
    void sendInt(int i);
    void sendLong(uint64_t l);
    void sendBytes(const uint8_t * c, size_t length);
    void sendShortString(const char * str);
    void sendLongString(const char * str);

    int getWriteError();
}

#endif // SLIMEVR_PACKETS_H_
