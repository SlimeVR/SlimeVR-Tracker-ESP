#ifndef _SENSOR_H_
#define _SENSOR_H_ 1

#include <BNO080.h>
#include <MPU9250.h>
#include <quat.h>
#include <vector3.h>
#include "configuration.h"

class Sensor {
    public:
        Sensor() = default;
        virtual ~Sensor() = default;
        virtual void motionSetup(DeviceConfig * config) = 0;
        virtual void motionLoop() = 0;
        virtual void sendData() = 0;
        virtual void startCalibration(int calibrationType) = 0;
    protected:
        Quat quaternion {};
        Quat sensorOffset {Quat(Vector3(0, 0, 1), PI / 2.0)};
};

class BNO080Sensor : public Sensor {
    public:
        BNO080Sensor() = default;
        ~BNO080Sensor() = default;
        void motionSetup(DeviceConfig * config) final;
        void motionLoop() final;
        void sendData() final;
        void startCalibration(int calibrationType) final;
    private:
        BNO080 imu {};
        bool newData {false};
};

class MPUSensor : public Sensor {
    public:
        MPUSensor() = default;
        ~MPUSensor() = default;
    protected:
        MPU9250 imu {};
        float q[4] {1.0, 0.0, 0.0, 0.0};
};

class MPU6050Sensor : public MPUSensor {
    public:
        MPU6050Sensor() = default;
        ~MPU6050Sensor() = default;
        void motionSetup(DeviceConfig * config) final;
        void motionLoop() final;
        void sendData() final;
        void startCalibration(int calibrationType) final;
    private:
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
        ~MPU9250Sensor() = default;
        void motionSetup(DeviceConfig * config) final;
        void motionLoop() final;
        void sendData() final;
        void startCalibration(int calibrationType) final;
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
};

#endif /* _SENSOR_H_ */
