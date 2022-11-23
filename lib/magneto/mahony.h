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

#ifndef _MAHONY_H_
#define _MAHONY_H_

#include "helper_3dmath.h"

void mahonyQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float deltat);
void mahonyQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

template<typename T>
class Mahony {
public:
    void updateInto(T q[4], T ax, T ay, T az, T gx, T gy, T gz, T deltat)
    {
        constexpr double Kp = 10.0;
        constexpr double Ki = 0.0;
        // short name local variable for readability
        T q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
        T norm;
        T vx, vy, vz;
        T ex, ey, ez;  //error terms
        T qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        T tmp = ax * ax + ay * ay + az * az;
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
private:
    T ix = 0.0;
    T iy = 0.0;
    T iz = 0.0;
};

#endif /* _MAHONY_H_ */