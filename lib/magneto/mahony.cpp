/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "mahony.h"

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
#define Kp 10.0f
#define Ki 0.0f

static float ix = 0.0f, iy = 0.0f, iz = 0.0f;  //integral feedback terms

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// reference vectors are Up (Acc) and West (Acc cross Mag)
// sjr 12/2020
// gx, gy, gz must be in units of radians/second
void mahonyQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, hz;  //observed West vector W = AxM
    float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
    float ex, ey, ez;
    float qa, qb, qc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Compute feedback only if magnetometer measurement valid (avoids NaN in magnetometer normalisation)
    float tmp = mx * mx + my * my + mz * mz;
    if (tmp == 0.0f) {
        mahonyQuaternionUpdate(q, ax, ay, az, gx, gy, gz, deltat);
        return;
    }
    // Normalise magnetometer
    norm = invSqrt(tmp);
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0f)
    {
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        norm = invSqrt(tmp);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Measured horizon vector = a x m (in body frame)
        hx = ay * mz - az * my;
        hy = az * mx - ax * mz;
        hz = ax * my - ay * mx;

        // Normalise horizon vector
        norm = invSqrt(hx * hx + hy * hy + hz * hz);
        hx *= norm;
        hy *= norm;
        hz *= norm;

        // Estimated direction of Up reference vector
        ux = 2.0f * (q2q4 - q1q3);
        uy = 2.0f * (q1q2 + q3q4);
        uz = q1q1 - q2q2 - q3q3 + q4q4;

        // estimated direction of horizon (West) reference vector
        wx = 2.0f * (q2q3 + q1q4);
        wy = q1q1 - q2q2 + q3q3 - q4q4;
        wz = 2.0f * (q3q4 - q1q2);

        // Error is cross product between estimated direction and measured direction of the reference vectors
        ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
        ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
        ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f) {
            ix += Ki * ex * deltat;  // integral error scaled by Ki
            iy += Ki * ey * deltat;
            iz += Ki * ez * deltat;
            gx += ix;  // apply integral feedback
            gy += iy;
            gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // Integrate rate of change of quaternion
    // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
    deltat *= 0.5f;
    gx *= deltat;   // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = q1;
    qb = q2;
    qc = q3;
    q1 += (-qb * gx - qc * gy - q4 * gz);
    q2 += (qa * gx + qc * gz - q4 * gy);
    q3 += (qa * gy - qb * gz + q4 * gx);
    q4 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void mahonyQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float deltat)
{
    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;  //error terms
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    float tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0f)
    {
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        norm = invSqrt(tmp);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = q2 * q4 - q1 * q3;
        vy = q1 * q2 + q3 * q4;
        vz = q1 * q1 - 0.5f + q4 * q4;

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f) {
            ix += Ki * ex * deltat;  // integral error scaled by Ki
            iy += Ki * ey * deltat;
            iz += Ki * ez * deltat;
            gx += ix;  // apply integral feedback
            gy += iy;
            gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // Integrate rate of change of quaternion, q cross gyro term
    deltat *= 0.5f;
    gx *= deltat;   // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = q1;
    qb = q2;
    qc = q3;
    q1 += (-qb * gx - qc * gy - q4 * gz);
    q2 += (qa * gx + qc * gz - q4 * gy);
    q3 += (qa * gy - qb * gz + q4 * gx);
    q4 += (qa * gz + qb * gy - qc * gx);

    // normalise quaternion
    norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}