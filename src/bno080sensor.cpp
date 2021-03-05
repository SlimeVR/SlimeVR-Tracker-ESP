#include "BNO080.h"
#include "sensor.h"
#include "udpclient.h"

void BNO080Sensor::motionSetup(DeviceConfig * config)
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
    imu.enableARVRStabilizedGameRotationVector(10);
}

void BNO080Sensor::motionLoop()
{
    //Look for reports from the IMU
    if (imu.dataAvailable() == true)
    {
        quaternion.x = imu.getQuatI();
        quaternion.y = imu.getQuatJ();
        quaternion.z = imu.getQuatK();
        quaternion.w = imu.getQuatReal();
        quaternion *= sensorOffset;
        newData = true;
    }
}

void BNO080Sensor::sendData() {
    if(newData) {
        newData = false;
        sendQuat(&quaternion, PACKET_ROTATION);
    }
}

void BNO080Sensor::startCalibration(int calibrationType) {
    // TODO It only calibrates gyro, it should have multiple calibration modes, and check calibration status in motionLoop()
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
    do {
        digitalWrite(CALIBRATING_LED, LOW);
        imu.requestCalibrationStatus();
        delay(20);
        imu.getReadings();
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(20);
    } while(!imu.calibrationComplete());
    imu.saveCalibration();
}