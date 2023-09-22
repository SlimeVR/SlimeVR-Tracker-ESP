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

template<typename T>
class Mahony {

    // These are the free parameters in the Mahony filter and fusion scheme,
    // Kp for proportional feedback, Ki for integral
    // with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
    static constexpr float Kp = 10.0f;
    static constexpr float Ki = 0.0f;

public:
    void update(T q[4], T ax, T ay, T az, T gx, T gy, T gz, T mx, T my, T mz, T deltat);
    void update(T q[4], T ax, T ay, T az, T gx, T gy, T gz, T deltat);
    
private:
    T ix = 0.0;
    T iy = 0.0;
    T iz = 0.0;
};

#include "mahony.hpp"

#endif /* _MAHONY_H_ */
