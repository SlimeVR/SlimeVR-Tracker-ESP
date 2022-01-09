#include "sensor.h"
#include <MPU6050.h>

class MPU6050Sensor : public Sensor
{
public:
    MPU6050Sensor(){};
    ~MPU6050Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;

private:
    MPU6050 imu{};
    Quaternion rawQuat{};
    // MPU dmp control/status vars
    bool dmpReady = false;    // set true if DMP init was successful
    uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
    uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;      // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;       // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]{}; // FIFO storage buffer
};