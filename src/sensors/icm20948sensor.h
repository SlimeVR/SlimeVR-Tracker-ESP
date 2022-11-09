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
#ifndef SLIMEVR_ICM20948SENSOR_H_
#define SLIMEVR_ICM20948SENSOR_H_

#include <ICM_20948.h>
#include "sensor.h"
#include <arduino-timer.h> // Used for periodically saving bias

class ICM20948Sensor : public Sensor
{
public:
    ICM20948Sensor(uint8_t id, uint8_t address, float rotation) : Sensor("ICM20948Sensor", IMU_ICM20948, id, address, rotation) {}
    ~ICM20948Sensor() override = default;
    void motionSetup() override final;
    void postSetup() override {
        this->lastData = millis();
    }

    void motionLoop() override final;
    void sendData() override final;
    void startCalibration(int calibrationType) override final;

private:
    void calculateAccelerationWithoutGravity(Quat *quaternion);
    unsigned long lastData = 0;
    unsigned long lastDataSent = 0;
    int bias_save_counter = 0;
    bool newTap;
    int16_t rawAccel[3];
    
    #define DMPNUMBERTODOUBLECONVERTER 1073741824.0;

    ICM_20948_I2C imu;
    ICM_20948_Device_t pdev;
    icm_20948_DMP_data_t dmpData{};
    icm_20948_DMP_data_t dmpDataTemp{};

    SlimeVR::Configuration::ICM20948CalibrationConfig m_Calibration;

    void saveCalibration(bool repeat);
    void loadCalibration();
    void startCalibrationAutoSave();
    void startDMP();
    void connectSensor();
    void startMotionLoop();
    void checkSensorTimeout();
    void readRotation();
    void readFIFOToEnd();

#define OVERRIDEDMPSETUP true

    Timer<> timer = timer_create_default();
    // TapDetector tapDetector;
};

#endif // SLIMEVR_ICM20948SENSOR_H_
