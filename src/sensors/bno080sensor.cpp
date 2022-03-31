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

#include "sensors/bno080sensor.h"
#include "network/network.h"
#include "utils.h"
#include "GlobalVars.h"

void BNO080Sensor::motionSetup()
{
#ifdef DEBUG_SENSOR
    imu.enableDebugging(Serial);
#endif
    if(!imu.begin(addr, Wire, m_IntPin)) {
        m_Logger.fatal("Can't connect to %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to %s on 0x%02x. "
                  "Info: SW Version Major: 0x%02x "
                  "SW Version Minor: 0x%02x "
                  "SW Part Number: 0x%02x "
                  "SW Build Number: 0x%02x "
                  "SW Version Patch: 0x%02x", 
                  getIMUNameByType(sensorType), 
                  addr, 
                  imu.swMajor, 
                  imu.swMinor, 
                  imu.swPartNumber, 
                  imu.swBuildNumber, 
                  imu.swVersionPatch
                );

    bool useStabilization = false;
#ifdef BNO_USE_ARVR_STABILIZATION
    useStabilization = true;
#endif
    if(useStabilization && (sensorType == IMU_BNO085 || sensorType == IMU_BNO086)) {
        if(useMagnetometerAllTheTime) {
            imu.enableARVRStabilizedRotationVector(10);
        } else {
            imu.enableARVRStabilizedGameRotationVector(10);
            if (useMagnetometerCorrection)
                imu.enableRotationVector(1000);
        }
    } else {
        if(useMagnetometerAllTheTime) {
            imu.enableRotationVector(10);
        } else {
            imu.enableGameRotationVector(10);
            if(useMagnetometerCorrection)
                imu.enableRotationVector(1000);
        }
    }
    imu.enableTapDetector(100);

#ifdef ENABLE_INSPECTION
    imu.enableRawGyro(10);
    imu.enableRawAccelerometer(10);
    imu.enableRawMagnetometer(10);
#endif

    lastReset = 0;
    lastData = millis();
    working = true;
    configured = true;
}

void BNO080Sensor::motionLoop()
{
    //Look for reports from the IMU
    while (imu.dataAvailable())
    {
#if ENABLE_INSPECTION
        {
            int16_t rX = imu.getRawGyroX();
            int16_t rY = imu.getRawGyroY();
            int16_t rZ = imu.getRawGyroZ();
            uint8_t rA = imu.getGyroAccuracy();

            int16_t aX = imu.getRawAccelX();
            int16_t aY = imu.getRawAccelY();
            int16_t aZ = imu.getRawAccelZ();
            uint8_t aA = imu.getAccelAccuracy();

            int16_t mX = imu.getRawMagX();
            int16_t mY = imu.getRawMagY();
            int16_t mZ = imu.getRawMagZ();
            uint8_t mA = imu.getMagAccuracy();

            Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, rA, aX, aY, aZ, aA, mX, mY, mZ, mA);
        }
#endif

        lastReset = 0;
        lastData = millis();
        if (useMagnetometerAllTheTime || !useMagnetometerCorrection)
        {
            if (imu.hasNewQuat())
            {
                imu.getQuat(quaternion.x, quaternion.y, quaternion.z, quaternion.w, magneticAccuracyEstimate, calibrationAccuracy);
                quaternion *= sensorOffset;

#if ENABLE_INSPECTION
                {
                    Network::sendInspectionFusedIMUData(sensorId, quaternion);
                }
#endif

                if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
                {
                    newData = true;
                    lastQuatSent = quaternion;
                }
            }
        }
        else
        {
            if (imu.hasNewGameQuat())
            {
                imu.getGameQuat(quaternion.x, quaternion.y, quaternion.z, quaternion.w, calibrationAccuracy);
                quaternion *= sensorOffset;

#if ENABLE_INSPECTION
                {
                    Network::sendInspectionFusedIMUData(sensorId, quaternion);
                }
#endif

                newData = true;
            }
            if (imu.hasNewMagQuat())
            {
                imu.getMagQuat(magQuaternion.x, magQuaternion.y, magQuaternion.z, magQuaternion.w, magneticAccuracyEstimate, magCalibrationAccuracy);
                magQuaternion *= sensorOffset;

#if ENABLE_INSPECTION
                {
                    Network::sendInspectionCorrectionData(sensorId, quaternion);
                }
#endif

                newMagData = true;
            }
        }
        if (imu.getTapDetected())
        {
            tap = imu.getTapDetector();
        }
        if (imu.hasNewAccel())
        {
            float v[3];
            uint8_t acc;
            imu.getAccel(v[0], v[1], v[2], acc);
            Network::sendAccel(v, PACKET_ACCEL);
        }
        if (m_IntPin == 255 || imu.I2CTimedOut())
            break;
    }
    if (lastData + 1000 < millis() && configured)
    {
        while(true) {
            BNO080Error error = imu.readError();
            if(error.error_source == 255)
                break;
            lastError = error;
            m_Logger.error("BNO08X error. Severity: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d",
                error.severity, error.error_sequence_number, error.error_source, error.error, error.error_module, error.error_code);
        }
        statusManager.setStatus(SlimeVR::Status::IMU_ERROR, true);
        working = false;
        lastData = millis();
        uint8_t rr = imu.resetReason();
        if (rr != lastReset)
        {
            lastReset = rr;
            Network::sendError(rr, this->sensorId);
        }
        m_Logger.error("Sensor %d doesn't respond. Last reset reason:", sensorId, lastReset);
        m_Logger.error("Last error: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d",
                lastError.severity, lastError.error_sequence_number, lastError.error_source, lastError.error, lastError.error_module, lastError.error_code);
    }
}

uint8_t BNO080Sensor::getSensorState() {
    return lastReset > 0 ? SensorStatus::SENSOR_ERROR : isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

void BNO080Sensor::sendData()
{
    if (newData)
    {
        newData = false;
        Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, calibrationAccuracy, sensorId);
        if (useMagnetometerAllTheTime)
            Network::sendMagnetometerAccuracy(magneticAccuracyEstimate, sensorId);

#ifdef DEBUG_SENSOR
        m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(quaternion));
#endif
    }
    if (newMagData)
    {
        newMagData = false;
        Network::sendRotationData(&magQuaternion, DATA_TYPE_CORRECTION, magCalibrationAccuracy, sensorId);
        Network::sendMagnetometerAccuracy(magneticAccuracyEstimate, sensorId);
    }
    if (tap != 0)
    {
        Network::sendTap(tap, sensorId);
        tap = 0;
    }
}

void BNO080Sensor::startCalibration(int calibrationType)
{
    // TODO It only calibrates gyro, it should have multiple calibration modes, and check calibration status in motionLoop()
    ledManager.pattern(20, 20, 10);
    ledManager.blink(2000);
    imu.calibrateGyro();
    do
    {
        ledManager.on();
        imu.requestCalibrationStatus();
        delay(20);
        imu.getReadings();
        ledManager.off();
        delay(20);
    } while (!imu.calibrationComplete());
    imu.saveCalibration();
}