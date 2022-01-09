#include "sensor.h"

#include <MPU9250.h>

class MPU9250Sensor : public Sensor
{
public:
    MPU9250Sensor(){};
    ~MPU9250Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;
    void getMPUScaled();
    void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

private:
    MPU9250 imu{};
    //raw data and scaled as vector
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    float Axyz[3]{};
    float Gxyz[3]{};
    float Mxyz[3]{};
    float rawMag[3]{};
    float q[4]{1.0, 0.0, 0.0, 0.0};
    // Loop timing globals
    unsigned long now = 0, last = 0; //micros() timers
    float deltat = 0;                //loop time in seconds
};