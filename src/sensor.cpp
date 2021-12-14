#include "defines.h"
#include "sensor.h"
#include "udpclient.h"
#include <i2cscan.h>
#include "calibration.h"
#include "configuration.h"

void Sensor::setupSensor(uint8_t sensorId, uint8_t addr, uint8_t intPin) {
    this->addr = addr;
    this->intPin = intPin;
    this->sensorId = sensorId;
    this->sensorOffset = {Quat(Vector3(0, 0, 1), sensorId == 0 ? IMU_ROTATION : SECOND_IMU_ROTATION)};
    };

void Sensor::sendData() {
    if(newData) {
        newData = false;
        sendRotationData(&quaternion, DATA_TYPE_NORMAL, calibrationAccuracy, sensorId, PACKET_ROTATION_DATA);
        #ifdef FULL_DEBUG
            Serial.print("Quaternion: ");
            Serial.print(quaternion.x);
            Serial.print(",");
            Serial.print(quaternion.y);
            Serial.print(",");
            Serial.print(quaternion.z);
            Serial.print(",");
            Serial.print(quaternion.w);
            Serial.print("\n");
        #endif
    }
};