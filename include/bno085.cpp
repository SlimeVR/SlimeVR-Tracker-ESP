#include "motionbase.h"
#include "BNO080.cpp"

BNO080 imu;

void motionSetup()
{
    delay(500);
    if(!imu.begin(BNO080_DEFAULT_ADDRESS, Wire)) {
        Serial.println("Can't connect to BNO08X");
        for(int i = 0; i < 500; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }
    }
    Serial.println("Connected to BNO08X");
    Wire.setClock(400000);
    imu.enableARVRStabilizedRotationVector(30);
}

void motionLoop()
{
    //Look for reports from the IMU
    if (imu.dataAvailable() == true)
    {
        cq.x = imu.getQuatI();
        cq.y = imu.getQuatJ();
        cq.z = imu.getQuatK();
        cq.w = imu.getQuatReal();
        cq *= rotationQuat;
    }
}

void sendData() {
    sendQuat(&cq, PACKET_ROTATION);
}

void performCalibration() {
    for(int i = 0; i < 10; ++i) {
        digitalWrite(CALIBRATING_LED, LOW);
        delay(20);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(20);
    }
    digitalWrite(CALIBRATING_LED, LOW);
    delay(2000);
    digitalWrite(CALIBRATING_LED, HIGH);
    imu.calibrateGyro();
    imu.requestCalibrationStatus(); // use this
    while(!imu.calibrationComplete()) {
        digitalWrite(CALIBRATING_LED, LOW);
        delay(20);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(20);
        imu.getReadings();
    }
    digitalWrite(CALIBRATING_LED, LOW);
    delay(2000);
    digitalWrite(CALIBRATING_LED, HIGH);
    imu.calibrateAccelerometer();
    while(!imu.calibrationComplete()) {
        digitalWrite(CALIBRATING_LED, LOW);
        delay(20);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(20);
        imu.getReadings();
    }
    digitalWrite(CALIBRATING_LED, LOW);
    delay(2000);
    digitalWrite(CALIBRATING_LED, HIGH);
    imu.calibrateMagnetometer();
    while(!imu.calibrationComplete()) {
        digitalWrite(CALIBRATING_LED, LOW);
        delay(20);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(20);
        imu.getReadings();
    }
    imu.saveCalibration();
}