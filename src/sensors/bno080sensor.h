#include "sensor.h"
#include <BNO080.h>

class BNO080Sensor : public Sensor
{
public:
    BNO080Sensor(){};
    ~BNO080Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void sendData() override final;
    void startCalibration(int calibrationType) override final;

private:
    BNO080 imu{};

    uint8_t tap;
    unsigned long lastData = 0;
    uint8_t lastReset = 0;

    // Magnetometer specific members
    Quat magQuaternion{};
    uint8_t magCalibrationAccuracy = 0;
    float magneticAccuracyEstimate = 999;
    bool useMagnetometerAllTheTime = false;
    bool useMagnetometerCorrection = false;
    bool newMagData = false;
};