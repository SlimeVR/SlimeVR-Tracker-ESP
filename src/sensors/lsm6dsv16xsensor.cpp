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

#include "sensors/lsm6dsv16xsensor.h"
#include "utils.h"
#include "customConversions.h"
#include "GlobalVars.h"

//#define INTERRUPTFREE  //TODO: change based on int pin number (255 = interruptFree)

volatile bool imuEvent = false;

void IRAM_ATTR interruptHandler() {
    imuEvent = true;
}

void LSM6DSV16XSensor::motionSetup()
{
#ifdef DEBUG_SENSOR
    //TODO: Should anything go here
#endif
    errorCounter = 0; //Either subtract to the error counter or handle the error
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

    if (deviceId != LSM6DSV16X_ID) {
        m_Logger.fatal("The IMU returned an invalid device ID of: 0x%02x when it should have been: 0x%02x", deviceId, LSM6DSV16X_ID);
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

#ifndef INTERRUPTFREE
    attachInterrupt(m_IntPin, interruptHandler, RISING);
#endif

    errorCounter -= imu.Enable_6D_Orientation(LSM6DSV16X_INT1_PIN);
    errorCounter -= imu.Enable_Single_Tap_Detection(LSM6DSV16X_INT1_PIN);
    errorCounter -= imu.Enable_Double_Tap_Detection(LSM6DSV16X_INT1_PIN);

    if (errorCounter) {
        m_Logger.fatal("%d Error(s) occured enabling imu features on %s at address 0x%02x", errorCounter, getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    //errorCounter -= imu.Set_G_Filter_Mode(0, 0) //Look up filter setup
    //errorCounter -= imu.Set_G_FS(2000dps);
    //errorCounter -= imu.Set_G_ODR(120.0F, LSM6DSV16X_GYRO_HIGH_ACCURACY_MODE) //High accuracy mode is not implemented

    //errorCounter -= imu.Set_X_Filter_Mode(0, 0) //Look up filter setup
    //errorCounter -= imu.Set_X_FS(8LSM6DSV16X_ACC_SENSITIVITY_FS_8G);
    //errorCounter -= imu.Set_X_ODR(120.0F, LSM6DSV16X_ACC_HIGH_ACCURACY_MODE) //High accuracy mode is not implemented

    lastReset = 0;
    lastData = millis();
    working = true;
    configured = true;
}

void LSM6DSV16XSensor::motionLoop()
{
#ifdef INTERRUPTFREE 
    //TODO: Check if data is actually availiable in the fifo instead of just hoping
    if (millis() - 100 > lastData) //10Hz refresh rate TODO: make faster
        imuEvent = true;
#endif

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
            
                errorCounter -= imu.Get_X_AxesRaw(accelerometer);
                errorCounter -= imu.Get_G_AxesRaw(gyroscope);

                networkConnection.sendInspectionRawIMUData(sensorId, gyroscope[0], gyroscope[1], gyroscope[2], 0, accelerometer[0], accelerometer[1], accelerometer[2], 0, 0, 0, 0, 0);
            }
#endif
            uint8_t dataLow = 0;
            uint8_t dataHigh = 0;
            errorCounter -= imu.Get_6D_Orientation_XL(&dataLow);
            errorCounter -= imu.Get_6D_Orientation_XH(&dataHigh);
            fusedRotation.x = Conversions::convertBytesToFloat(dataLow, dataHigh);

            errorCounter -= imu.Get_6D_Orientation_YL(&dataLow);
            errorCounter -= imu.Get_6D_Orientation_YH(&dataHigh);
            fusedRotation.y = Conversions::convertBytesToFloat(dataLow, dataHigh);

            errorCounter -= imu.Get_6D_Orientation_ZL(&dataLow);
            errorCounter -= imu.Get_6D_Orientation_ZH(&dataHigh);
            fusedRotation.z = Conversions::convertBytesToFloat(dataLow, dataHigh);

            fusedRotation.w = sqrtf(1.0F - sq(fusedRotation.x) - sq(fusedRotation.y) - sq(fusedRotation.z));

            lastReset = 0;
            lastData = millis();

            if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES || !lastFusedRotationSent.equalsWithEpsilon(fusedRotation))
            {
                newFusedRotation = true;
                lastFusedRotationSent = fusedRotation;
            }


#if SEND_ACCELERATION
            {
                int32_t accelerometerInt[3];
                errorCounter -= imu.Get_X_Axes(accelerometerInt);
                acceleration[0] = accelerometerInt[0] * 1000.0F; //convert from mg to g
                acceleration[1] = accelerometerInt[1] * 1000.0F;
                acceleration[2] = accelerometerInt[2] * 1000.0F;
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

SensorStatus LSM6DSV16XSensor::getSensorState() {
    //TODO: this may need to be redone, errorCounter gets reset at the end of the loop
    return errorCounter > 0 ? SensorStatus::SENSOR_ERROR : isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

void LSM6DSV16XSensor::sendData()
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
    if(newAcceleration) //Returns acceleration in G's
    {
        newAcceleration = false;
        networkConnection.sendSensorAcceleration(sensorId, acceleration);
    }
#endif

    if (tap != 0) //chip supports tap and double tap
    {
        networkConnection.sendSensorTap(sensorId, tap);
        tap = 0;
    }
}

void LSM6DSV16XSensor::startCalibration(int calibrationType)
{
    //These IMU are factory calibrated.
    //The register might be able to be changed but it could break the device
    //I don't think we will need to mess with them
}
