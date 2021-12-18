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
#include "defines.h"
#include "sensor.h"
#include "udpclient.h"
#include <i2cscan.h>
#include "calibration.h"
#include "configuration.h"

void Sensor::setupSensor(uint8_t sensorId, uint8_t addr, uint8_t intPin) {
    this->addr = addr;
    this->intPin = intPin;
    this->sensorId = sensorId;
    this->sensorOffset = {Quat(Vector3(0, 0, 1), sensorId == 0 ? IMU_ROTATION : SECOND_IMU_ROTATION)};
}

void Sensor::sendData() {
    if(newData) {
        newData = false;
        sendRotationData(&quaternion, DATA_TYPE_NORMAL, calibrationAccuracy, sensorId, PACKET_ROTATION_DATA);
        #ifdef FULL_DEBUG
            Serial.print("[DBG] Quaternion: ");
            Serial.print(quaternion.x);
            Serial.print(",");
            Serial.print(quaternion.y);
            Serial.print(",");
            Serial.print(quaternion.z);
            Serial.print(",");
            Serial.print(quaternion.w);
            Serial.print("\n");
        #endif
    }
}