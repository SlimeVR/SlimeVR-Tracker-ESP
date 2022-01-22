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

#ifndef SLIMEVR_SENSOR_H_
#define SLIMEVR_SENSOR_H_

#include <BNO080.h>
#include <MPU9250.h>
#include <MPU6050.h>
#include <ICM_20948.h>
#include <quat.h>
#include <vector3.h>
#include "configuration.h"
#include <Adafruit_BNO055.h>
#include "defines.h"
#include <arduino-timer.h> // Used for periodically saving bias
#ifdef ESP32
    #include <Preferences.h> // ICM bias saving. ESP8266 use eprom
#endif


#define DATA_TYPE_NORMAL 1
#define DATA_TYPE_CORRECTION 2

class Sensor {
    public:
        Sensor() = default;
        virtual ~Sensor() = default;
        virtual void motionSetup() = 0;
        virtual void motionLoop() = 0;
        virtual void sendData() = 0;
        virtual void startCalibration(int calibrationType) = 0;
        bool isWorking() {
            return working;
        }
    protected:
        Quat quaternion {};
        Quat sensorOffset {Quat(Vector3(0, 0, 1), IMU_ROTATION)};
        Quat lastQuatSent {};
        bool working {false};
        uint8_t sensorId {0};
};

class EmptySensor : public Sensor {
    public:
        EmptySensor() = default;
        ~EmptySensor() override = default;
        void motionSetup() override final;
        void motionLoop() override final;
        void sendData() override final;
        void startCalibration(int calibrationType) override final;
};

class BNO080Sensor : public Sensor {
    public:
        BNO080Sensor() = default;
        ~BNO080Sensor() override = default;
        void motionSetup() override final;
        void motionLoop() override final;
        void sendData() override final;
        void startCalibration(int calibrationType) override final;
        void setupBNO080(uint8_t sensorId = 0, uint8_t addr = 0x4B, uint8_t intPin = 255);
    private:
        BNO080 imu {};
        bool newData {false};
        bool newMagData {false};
        uint8_t tap;
        unsigned long lastData = 0;
        uint8_t lastReset = 0;
        uint8_t addr = 0x4B;
        uint8_t intPin = 255;
        bool setUp {false};
        Quat magQuaternion {};
        float magneticAccuracyEstimate {999};
        uint8_t calibrationAccuracy {0};
        uint8_t magCalibrationAccuracy {0};
        bool useMagnetometerAllTheTime {false};
        bool useMagnetometerCorrection {false};
};

class BNO055Sensor : public Sensor {
    public:
        BNO055Sensor() = default;
        ~BNO055Sensor() override = default;
        void motionSetup() override final;
        void motionLoop() override final;
        void sendData() override final;
        void startCalibration(int calibrationType) override final;
    private:
        Adafruit_BNO055  imu {Adafruit_BNO055(55, 0x28)};
        bool newData {false};
};

class MPUSensor : public Sensor {
    public:
        MPUSensor() = default;
        ~MPUSensor() override = default;
    protected:
        float q[4] {1.0, 0.0, 0.0, 0.0};
};

class MPU6050Sensor : public MPUSensor {
    public:
        MPU6050Sensor() = default;
        ~MPU6050Sensor() override  = default;
        void motionSetup() override final;
        void setSecond();
        void motionLoop() override final;
        void sendData() override final;
        void startCalibration(int calibrationType) override final;
    private:
        MPU6050 imu {};
        bool isSecond{false};
        bool newData{false};
        Quaternion rawQuat {};
        // MPU dmp control/status vars
        bool dmpReady{false};  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64] {}; // FIFO storage buffer
};

class MPU9250Sensor : public MPUSensor {
    public:
        MPU9250Sensor() = default;
        ~MPU9250Sensor() override  = default;
        void motionSetup() override final;
        void motionLoop() override final;
        void sendData() override final;
        void startCalibration(int calibrationType) override final;
        void getMPUScaled();
        void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
    private:
        MPU9250 imu {};
        //raw data and scaled as vector
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t mx, my, mz;
        float Axyz[3] {};
        float Gxyz[3] {};
        float Mxyz[3] {};
        float rawMag[3] {};
        // Loop timing globals
        unsigned long now = 0, last = 0;   //micros() timers
        float deltat = 0;                  //loop time in seconds
        bool newData {false};
};

class ICM20948Sensor : public Sensor {
public:
    ICM20948Sensor() = default;
    ~ICM20948Sensor() override = default;
    void motionSetup() override final;
    void motionLoop() override final;
    void sendData() override final;
    void startCalibration(int calibrationType) override final;
    void save_bias(bool repeat);
    void setupICM20948(bool auxiliary = false, uint8_t addr = 0x69);

private:

    void i2c_scan();
    bool auxiliary{ false };
    unsigned long lastData = 0;
    uint8_t first_imu;
    uint8_t second_imu;
    uint8_t addr = 0x69;
    int bias_save_counter = 0;
    uint8_t ICM_address;
    bool ICM_found = false;
    bool ICM_init = false;
    bool newData = false;
    bool newTap;
    
    uint8_t lastReset = 0;
    ICM_20948_I2C imu;
    ICM_20948_Device_t pdev;
    icm_20948_DMP_data_t dmpData {};

    #define OVERRIDEDMPSETUP false
#ifdef ESP32
    Preferences prefs;
    Timer<> timer = timer_create_default();
#endif
#ifdef ESP8266
    Timer<> timer = timer_create_default();
#endif
    //TapDetector tapDetector;
};

#endif // SLIMEVR_SENSOR_H_
