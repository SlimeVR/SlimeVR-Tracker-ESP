// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

#ifndef REST_DETECTION_H
#define REST_DETECTION_H

#include <Arduino.h>
#include <quat.h>
#include <vector3.h>
#include <vqf.h>
#include "types.h"

struct RestDetectionParams {
    sensor_real_t biasClip;
    uint32_t restMinTimeMicros;
    sensor_real_t restThGyr;
    sensor_real_t restThAcc;
    sensor_real_t restThMag;
    RestDetectionParams():
        biasClip(2.0f),
        restMinTimeMicros(1.5 * 1e6),
        restThGyr(2.0f),
        restThAcc(0.5f),
        restThMag(0.1f)
    { }
};

class RestDetection {
public:
    RestDetection()
    { }
    RestDetection(const RestDetectionParams &params) {
        this->params = params;
    }

    void updateGyr(uint32_t dtMicros, sensor_real_t inxyz[3]);
    void updateAcc(uint32_t dtMicros, sensor_real_t inxyz[3]);
    void updateMag(uint32_t dtMicros, sensor_real_t inxyz[3]);
    bool getRestDetected();

private:
    RestDetectionParams params;
    bool restDetected;
    uint32_t restTimeMicros;
    struct {
        float gyr[3];
        float acc[3];
        float mag[3];
    } lastSample;
    sensor_real_t gyrLastSquaredDeviation = 0;
    sensor_real_t accLastSquaredDeviation = 0;
    sensor_real_t magLastSquaredDeviation = 0;
};

#endif