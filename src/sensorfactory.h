#ifndef SLIMEVR_SENSORFACTORY_H_
#define SLIMEVR_SENSORFACTORY_H_

#include "defines.h"
#include "sensor.h"

class SensorFactory
{
public:
    SensorFactory();
    ~SensorFactory();
    bool create();
    void motionSetup();
    void motionLoop();
    void sendData();
    void startCalibration(int calibrationType);
    Sensor *getFirst() { return sensor1; };
    Sensor *getSecond() { return sensor2; };

protected:
    Sensor *sensor1;
    Sensor *sensor2;
};

#endif // SLIMEVR_SENSORFACTORY_H_