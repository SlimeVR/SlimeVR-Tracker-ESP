#pragma once
#ifndef QUATERNIONFILTER_H
#define QUATERNIONFILTER_H

#include <Arduino.h>

enum class QuatFilterSel {
    NONE,
    MADGWICK,
    MAHONY,
};

class QuaternionFilter {
    // for madgwick
    float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = PI * (0.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    // for mahony
    float Kp = 30.0;
    float Ki = 0.0;

    QuatFilterSel filter_sel{QuatFilterSel::MADGWICK};
    double deltaT{0.};
    uint32_t newTime{0}, oldTime{0};

public:
    void select_filter(QuatFilterSel sel);

    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);

    void no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
    void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
    void mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
};

#endif  // QUATERNIONFILTER_H
