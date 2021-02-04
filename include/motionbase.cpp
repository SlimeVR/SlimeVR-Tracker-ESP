#include "I2Cdev.cpp"
#include "MPU9250MotionApps.h"
#include "defines.h"
#include "quat.cpp"
#include "configuration.cpp"
#include "util.cpp"
#include "udpclient.cpp"

MPU9250 accelgyro;
DeviceConfig config;
const CalibrationConfig &calibration = config.calibration;

// Vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};
Quaternion rawQuat = Quaternion();
const Quat rotationQuat = Quat(Vector3(0, 0, 1), PI / 2.0); // Adjust rotation to match Android rotations
Quat cq = Quat();

void motionSetup();

void motionLoop();

void sendData();