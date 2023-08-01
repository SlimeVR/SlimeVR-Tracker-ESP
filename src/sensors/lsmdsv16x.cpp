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

#include "sensors/lsmdsv16x.h"
#include "utils.h"
#include "GlobalVars.h"

void LSMDSV16XSensor::motionSetup()
{
#ifdef DEBUG_SENSOR
    imu.enableDebugging(Serial);
#endif
    imu = LSM6DSV16XSensor(&Wire, addr);
    if(!imu.begin()) {
        m_Logger.fatal("Can't connect to %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    //if(imu.ReadID())

    m_Logger.info("Connected to %s on 0x%02x. "
                  "Info: SW Version Major: 0x%02x "
                  "SW Version Minor: 0x%02x "
                  "SW Part Number: 0x%02x "
                  "SW Build Number: 0x%02x "
                  "SW Version Patch: 0x%02x",
                  getIMUNameByType(sensorType),
                  addr
                );

    if(!imu.Enable_X()) { //accel
        m_Logger.fatal("Error enabling accelerometer on %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    imu.Enable_G(); //gyro
    //make an interupt
    attachInterrupt(m_IntPin, DoSomething, RISING);
    imu.Enable_6D_Orientation(LSM6DSV16X_INT1_PIN);

    lastReset = 0;
    lastData = millis();
    working = true;
    configured = true;
}

void LSMDSV16XSensor::motionLoop()
{
    //Look for reports from the IMU
    while (imu.dataAvailable())
    {
        hadData = true;
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

            networkConnection.sendInspectionRawIMUData(sensorId, rX, rY, rZ, rA, aX, aY, aZ, aA, mX, mY, mZ, mA);
        }
#endif

        uint8_t xl = 0;
        uint8_t xh = 0;
        uint8_t yl = 0;
        uint8_t yh = 0;
        uint8_t zl = 0;
        uint8_t zh = 0;
  
        imu.Get_6D_Orientation_XL(&xl);
        imu.Get_6D_Orientation_XH(&xh);
        imu.Get_6D_Orientation_YL(&yl);
        imu.Get_6D_Orientation_YH(&yh);
        imu.Get_6D_Orientation_ZL(&zl);
        imu.Get_6D_Orientation_ZH(&zh);

        lastReset = 0;
        lastData = millis();

        if (imu.hasNewGameQuat()) // New quaternion if context
        {
            imu.getGameQuat(fusedRotation.x, fusedRotation.y, fusedRotation.z, fusedRotation.w, calibrationAccuracy);
            fusedRotation *= sensorOffset;

            if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES || !lastFusedRotationSent.equalsWithEpsilon(fusedRotation))
            {
                newFusedRotation = true;
                lastFusedRotationSent = fusedRotation;
            }
            // Leave new quaternion if context open, it's closed later

            // Continuation of the new quaternion if context, used for both 6 and 9 axis
#if SEND_ACCELERATION
            {
                uint8_t acc;
                this->imu.getLinAccel(this->acceleration[0], this->acceleration[1], this->acceleration[2], acc);
                this->newAcceleration = true;
            }
#endif // SEND_ACCELERATION
        } // Closing new quaternion if context


        if (imu.getTapDetected())
        {
            tap = imu.getTapDetector();
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
            networkConnection.sendSensorError(this->sensorId, rr);
        }

        m_Logger.error("Sensor %d doesn't respond. Last reset reason:", sensorId, lastReset);
        m_Logger.error("Last error: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d",
                lastError.severity, lastError.error_sequence_number, lastError.error_source, lastError.error, lastError.error_module, lastError.error_code);
    }
}

SensorStatus BNO080Sensor::getSensorState() {
    return lastReset > 0 ? SensorStatus::SENSOR_ERROR : isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

void LSMDSV16XSensor::sendData()
{
    if (newFusedRotation)
    {
        newFusedRotation = false;
        networkConnection.sendRotationData(sensorId, &fusedRotation, DATA_TYPE_NORMAL, calibrationAccuracy);

#ifdef DEBUG_SENSOR
        m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(fusedRotation));
#endif
    }

#if SEND_ACCELERATION
    if(newAcceleration)
    {
        newAcceleration = false;
        networkConnection.sendSensorAcceleration(this->sensorId, this->acceleration);
    }
#endif

#if !USE_6_AXIS
        networkConnection.sendMagnetometerAccuracy(sensorId, magneticAccuracyEstimate);
#endif

#if USE_6_AXIS && BNO_USE_MAGNETOMETER_CORRECTION
    if (newMagData)
    {
        newMagData = false;
        networkConnection.sendRotationData(sensorId, &magQuaternion, DATA_TYPE_CORRECTION, magCalibrationAccuracy);
        networkConnection.sendMagnetometerAccuracy(sensorId, magneticAccuracyEstimate);
    }
#endif

    if (tap != 0)
    {
        networkConnection.sendSensorTap(sensorId, tap);
        tap = 0;
    }
}

void LSMDSV16XSensor::startCalibration(int calibrationType)
{
    //TODO: Look up in data sheet, does not look like it is in the lib
}
