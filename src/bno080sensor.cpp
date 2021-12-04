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

#include "BNO080.h"
#include "sensor.h"
#include "udpclient.h"
#include "defines.h"
#include <i2cscan.h>
#include "ledstatus.h"

namespace {
    void signalAssert() {
        for(int i = 0; i < 200; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }
    }
    
    void sendResetReason(uint8_t reason, uint8_t sensorId) {
        sendByte(reason, sensorId, PACKET_RESET_REASON);
    }
}

void BNO080Sensor::setupBNO080(uint8_t sensorId, uint8_t addr, uint8_t intPin) {
    this->addr = addr;
    this->intPin = intPin;
    this->sensorId = sensorId;
    this->sensorOffset = {Quat(Vector3(0, 0, 1), sensorId == 0 ? IMU_ROTATION : SECOND_IMU_ROTATION)};
}

void BNO080Sensor::motionSetup()
{
#ifdef FULL_DEBUG
        imu.enableDebugging(Serial);
#endif
    if(!imu.begin(addr, Wire, intPin)) {
        Serial.print("[ERR] IMU BNO08X: Can't connect to ");
        Serial.println(IMU_NAME);
        signalAssert();
        return;
    }
    Serial.print("[NOTICE] IMU BNO08X: Connected to ");
    Serial.print(IMU_NAME);
    Serial.print(" on 0x");
    Serial.print(addr, HEX);
    Serial.print(". Info: SW Version Major: 0x");
    Serial.print(imu.swMajor, HEX);
    Serial.print(" SW Version Minor: 0x");
    Serial.print(imu.swMinor, HEX);
    Serial.print(" SW Part Number: 0x");
    Serial.print(imu.swPartNumber, HEX);
    Serial.print(" SW Build Number: 0x");
    Serial.print(imu.swBuildNumber, HEX);
    Serial.print(" SW Version Patch: 0x");
    Serial.println(imu.swVersionPatch, HEX);
#if defined(BNO_HAS_ARVR_STABILIZATION) && BNO_HAS_ARVR_STABILIZATION
        if(useMagnetometerAllTheTime) {
            imu.enableARVRStabilizedRotationVector(10);
        } else {
            imu.enableARVRStabilizedGameRotationVector(10);
            if(useMagnetometerCorrection)
                imu.enableRotationVector(1000);
        }
#else
    if(useMagnetometerAllTheTime) {
        imu.enableRotationVector(10);
    } else {
        imu.enableGameRotationVector(10);
        if(useMagnetometerCorrection)
            imu.enableRotationVector(1000);
    }
#endif
    imu.enableTapDetector(100);
    lastReset = imu.resetReason();
    lastData = millis();
    working = true;
    setUp = true;
}

void BNO080Sensor::motionLoop()
{
    //Look for reports from the IMU
    while(imu.dataAvailable())
    {
        lastReset = -1;
        lastData = millis();
        if(useMagnetometerAllTheTime || !useMagnetometerCorrection) {
            if(imu.hasNewQuat()) {
                imu.getQuat(quaternion.x, quaternion.y, quaternion.z, quaternion.w, magneticAccuracyEstimate, calibrationAccuracy);
                quaternion *= sensorOffset;
                if(!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion)) {
                    newData = true;
                    lastQuatSent = quaternion;
                }
            }
        } else {
            if(imu.hasNewGameQuat()) {
                imu.getGameQuat(quaternion.x, quaternion.y, quaternion.z, quaternion.w, calibrationAccuracy);
                quaternion *= sensorOffset;
                newData = true;
            }
            if(imu.hasNewMagQuat()) {
                imu.getMagQuat(magQuaternion.x, magQuaternion.y, magQuaternion.z, magQuaternion.w, magneticAccuracyEstimate, magCalibrationAccuracy);
                magQuaternion *= sensorOffset;
                newMagData = true;
            }
        }
        if(imu.getTapDetected()) {
            tap = imu.getTapDetector();
        }
        if(imu.hasNewAccel()) {
            float v[3];
            uint8_t acc;
            imu.getAccel(v[0], v[1], v[2], acc);
            sendVector(v, PACKET_ACCEL);
        }
        if(intPin == 255 || imu.I2CTimedOut())
            break;
    }
    if(lastData + 1000 < millis() && setUp) {
        setLedStatus(LED_STATUS_IMU_ERROR);
        working = false;
        lastData = millis();
        uint8_t rr = imu.resetReason();
        if(rr != lastReset) {
            lastReset = rr;
            sendResetReason(rr, this->sensorId);
        }
        Serial.print("[ERR] Sensor ");
        Serial.print(sensorId);
        Serial.print(" was reset: ");
        Serial.println(rr);
    }
}

void BNO080Sensor::sendData() {
    if(newData) {
        newData = false;
        sendRotationData(&quaternion, DATA_TYPE_NORMAL, calibrationAccuracy, sensorId, PACKET_ROTATION_DATA);
        if(useMagnetometerAllTheTime)
            sendMagnetometerAccuracy(magneticAccuracyEstimate, sensorId, PACKET_MAGNETOMETER_ACCURACY);
#ifdef FULL_DEBUG
            Serial.print("[DBG] Quaternion: ");
            Serial.print(quaternion.x);
            Serial.print(",");
            Serial.print(quaternion.y);
            Serial.print(",");
            Serial.print(quaternion.z);
            Serial.print(",");
            Serial.println(quaternion.w);
#endif
    }
    if(newMagData) {
        newMagData = false;
        sendRotationData(&magQuaternion, DATA_TYPE_CORRECTION, magCalibrationAccuracy, sensorId, PACKET_ROTATION_DATA);
        sendMagnetometerAccuracy(magneticAccuracyEstimate, sensorId, PACKET_MAGNETOMETER_ACCURACY);
    }
    if(tap != 0) {
        sendByte(tap, sensorId, PACKET_TAP);
        tap = 0;
    }
}

void BNO080Sensor::startCalibration(int calibrationType) {
    // TODO It only calibrates gyro, it should have multiple calibration modes, and check calibration status in motionLoop()
    for(int i = 0; i < 10; ++i) {
        digitalWrite(CALIBRATING_LED, LOW);
        delay(20);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(20);
    }
    digitalWrite(CALIBRATING_LED, LOW);
    delay(2000);
    digitalWrite(CALIBRATING_LED, HIGH);
    imu.calibrateGyro();
    do {
        digitalWrite(CALIBRATING_LED, LOW);
        imu.requestCalibrationStatus();
        delay(20);
        imu.getReadings();
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(20);
    } while(!imu.calibrationComplete());
    imu.saveCalibration();
}