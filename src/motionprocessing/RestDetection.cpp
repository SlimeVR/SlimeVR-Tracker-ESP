// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

#include "RestDetection.h"

inline sensor_real_t square(sensor_real_t x) { return x * x; }

void RestDetection::updateGyr(uint32_t dtMicros, sensor_real_t gyr[3]) {
    gyrLastSquaredDeviation =
        square(gyr[0] - lastSample.gyr[0]) +
        square(gyr[1] - lastSample.gyr[1]) +
        square(gyr[2] - lastSample.gyr[2]);

    sensor_real_t biasClip = params.biasClip*sensor_real_t(M_PI/180.0);
    if (gyrLastSquaredDeviation >= square(params.restThGyr*sensor_real_t(M_PI/180.0))
            || fabs(lastSample.gyr[0]) > biasClip || fabs(lastSample.gyr[1]) > biasClip
            || fabs(lastSample.gyr[2]) > biasClip) {
        restTimeMicros = 0;
        restDetected = false;
    }

    lastSample.gyr[0] = gyr[0];
    lastSample.gyr[1] = gyr[1];
    lastSample.gyr[2] = gyr[2];
}

void RestDetection::updateAcc(uint32_t dtMicros, sensor_real_t acc[3]) {
    if (acc[0] == sensor_real_t(0.0) && acc[1] == sensor_real_t(0.0) && acc[2] == sensor_real_t(0.0)) {
        return;
    }

    accLastSquaredDeviation =
        square(acc[0] - lastSample.acc[0]) +
        square(acc[1] - lastSample.acc[1]) +
        square(acc[2] - lastSample.acc[2]);

    if (accLastSquaredDeviation >= square(params.restThAcc)) {
        restTimeMicros = 0;
        restDetected = false;
    } else {
        restTimeMicros += dtMicros;
        if (restTimeMicros >= params.restMinTimeMicros) {
            restDetected = true;
        }
    }

    lastSample.acc[0] = acc[0];
    lastSample.acc[1] = acc[1];
    lastSample.acc[2] = acc[2];
}

void RestDetection::updateMag(uint32_t dtMicros, sensor_real_t mag[3]) {
    if (mag[0] == sensor_real_t(0.0) && mag[1] == sensor_real_t(0.0) && mag[2] == sensor_real_t(0.0)) {
        return;
    }

    magLastSquaredDeviation =
        square(mag[0] - lastSample.mag[0]) +
        square(mag[1] - lastSample.mag[1]) +
        square(mag[2] - lastSample.mag[2]);

    sensor_real_t magNormSquared =
        square(lastSample.mag[0]) +
        square(lastSample.mag[1]) +
        square(lastSample.mag[2]);
    if (magLastSquaredDeviation >= square(params.restThMag)*magNormSquared) {
        restTimeMicros = 0;
        restDetected = false;
    }

    lastSample.mag[0] = mag[0];
    lastSample.mag[1] = mag[1];
    lastSample.mag[2] = mag[2];
}

bool RestDetection::getRestDetected() {
    return restDetected;
}
