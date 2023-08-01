/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2023 Eiren Rain & SlimeVR contributors

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

volatile bool imuEvent = false;

void LSMDSV16XSensor::motionSetup()
{
#ifdef DEBUG_SENSOR
    //TODO: Can anything go here
#endif
    errorCounter = 0; //Either subtract to the error counter or handle the error
    imu = LSM6DSV16XSensor(&Wire, addr);
    if(imu.begin() == LSM6DSV16X_ERROR) {
        m_Logger.fatal("Can't connect to %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }
    uint8_t deviceId = 0;
    if(imu.ReadID(&deviceId) == LSM6DSV16X_ERROR) {
        m_Logger.fatal("The IMU returned an error when reading the device ID of: 0x%02x", deviceId);
        ledManager.pattern(50, 50, 200);
        return;
    }

    if (deviceId != 0x00) { //TODO: Look up device ID
        m_Logger.fatal("The IMU returned an invalid device ID of: 0x%02x when it should have been: 0x%02x", deviceId, 0x00);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to %s on 0x%02x. ", getIMUNameByType(sensorType), addr);

    if(imu.Enable_X() == LSM6DSV16X_ERROR) { //accel
        m_Logger.fatal("Error enabling accelerometer on %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    if(imu.Enable_G() == LSM6DSV16X_ERROR) { //gyro
        m_Logger.fatal("Error enabling gyroscope on %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }


    attachInterrupt(m_IntPin, interruptHandler, RISING);


    errorCounter -= imu.Enable_6D_Orientation(LSM6DSV16X_INT1_PIN);
    errorCounter -= imu.Enable_Single_Tap_Detection(LSM6DSV16X_INT1_PIN);
    errorCounter -= imu.Enable_Double_Tap_Detection(LSM6DSV16X_INT1_PIN);

    if (errorCounter) {
        m_Logger.fatal("%d Error(s) occured enabling imu features on %s at address 0x%02x", errorCounter, getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    lastReset = 0;
    lastData = millis();
    working = true;
    configured = true;
}

void interruptHandler() {
    imuEvent = true;
}

void LSMDSV16XSensor::motionLoop()
{
    if (imuEvent) {
        imuEvent = false;
        LSM6DSV16X_Event_Status_t status;
        errorCounter -= imu.Get_X_Event_Status(&status);

        if (status.TapStatus) {
            tap++;
        }
        if (status.DoubleTapStatus) {
            tap += 2;
        }
        if (status.D6DOrientationStatus) {
            hadData = true;
#if ENABLE_INSPECTION
            {
                int16_t accelerometer[3];
                int16_t gyroscope[3];
            
                errorCounter += imu.Get_X_AxesRaw(accelerometer);
                errorCounter += imu.Get_G_AxesRaw(gyroscope);

                networkConnection.sendInspectionRawIMUData(sensorId, gyroscope[0], gyroscope[1], gyroscope[2], 0, accelerometer[0], accelerometer[1], accelerometer[2], 0, 0, 0, 0, 0);
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

            
            //We have imu position (x,y,z) in high/low registers, we need to convert this to a float16 and then to a quaternion


            /* bno code
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
            } // Closing new quaternion if context
            */


#if SEND_ACCELERATION
            {
                int32_t accelerometerInt[3];
                errorCounter -= imu.Get_X_Axes(accelerometerInt);
                acceleration[0] = accelerometerInt[0] * 1000.0; //convert from mg to g
                acceleration[1] = accelerometerInt[1] * 1000.0;
                acceleration[2] = accelerometerInt[2] * 1000.0;
                newAcceleration = true;
            }
#endif // SEND_ACCELERATION
        }





        if ((lastData + 1000 < millis() || errorCounter) && configured) //Errors
        {
            if (errorCounter) {
               m_Logger.error("The %s at address 0x%02x, had %d error(s) in the motion processing loop", getIMUNameByType(sensorType), addr, errorCounter);
            }

            if (lastData + 1000 < millis()) {
                m_Logger.error("The %s at address 0x%02x, has not responded in the last second", getIMUNameByType(sensorType), addr);
            }

            statusManager.setStatus(SlimeVR::Status::IMU_ERROR, true);
            working = false;
            lastData = millis();
            errorCounter = 0;
        }
    }
}

SensorStatus LSMDSV16XSensor::getSensorState() {
    return errorCounter > 0 ? SensorStatus::SENSOR_ERROR : isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

void LSMDSV16XSensor::sendData()
{
    if (newFusedRotation) //IDK how to get
    {
        newFusedRotation = false;
        networkConnection.sendRotationData(sensorId, &fusedRotation, DATA_TYPE_NORMAL, calibrationAccuracy);

#ifdef DEBUG_SENSOR
        m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(fusedRotation));
#endif
    }

#if SEND_ACCELERATION
    if(newAcceleration) //Returns acceleration in G's
    {
        newAcceleration = false;
        networkConnection.sendSensorAcceleration(this->sensorId, this->acceleration);
    }
#endif

    if (tap != 0) //chip supports tap and double tap
    {
        networkConnection.sendSensorTap(sensorId, tap);
        tap = 0;
    }
}

void LSMDSV16XSensor::startCalibration(int calibrationType)
{
    //TODO: Look up in data sheet, does not look like it is in the lib
}
