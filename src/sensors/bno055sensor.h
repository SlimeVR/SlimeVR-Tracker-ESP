#include "sensor.h"

#include <Adafruit_BNO055.h>

class BNO055Sensor : public Sensor
{
public:
    BNO055Sensor(){};
    ~BNO055Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;

private:
    Adafruit_BNO055 imu;
};