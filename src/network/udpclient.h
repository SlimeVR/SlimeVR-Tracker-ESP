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

#ifndef SLIMEVR_UDP_CLIENT_H_
#define SLIMEVR_UDP_CLIENT_H_

#include <WiFiUdp.h>
#include <Arduino.h>
#include "quat.h"
#include "configuration.h"
#include "sensors/sensor.h"
#include "wifihandler.h"
#include "defines.h"

// TODO : Hide this behind stuff in packet.h
void sendQuat(float * const quaternion, int type);
void sendQuat(Quat * const quaternion, int type);
void sendRotationData(Quat * const quaternion, uint8_t dataType, uint8_t accuracyInfo, uint8_t sensorId, int type);
void sendMagnetometerAccuracy(float accuracyInfo, uint8_t sensorId, int type);
void sendVector(float * const result, int type);
void sendConfig(DeviceConfig * const config, int type);
void sendFloat(float const value, int type);
void send2Floats(float const value1, float const value2, int type);
void sendByte(unsigned char const value, int type);
void sendByte(uint8_t const value, uint8_t sensorId, int type);
void sendSensorInfo(Sensor & sensor, int type);
void sendRawCalibrationData(int * const data, int calibrationType, unsigned char const sensorId, int type);
void sendRawCalibrationData(float * const data, int calibrationType, unsigned char const sensorId, int type);
void sendCalibrationFinished(int calibrationType, unsigned char const sensorId, int type);
void sendSerial(uint8_t *const data, int length, int type);

namespace Network {
    void connect();
    void update(Sensor * const sensor, Sensor * const sensor2);
    void onWiFiConnected();
    bool isConnected();
}

#endif // SLIMEVR_UDP_CLIENT_H_