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

#include "helper_3dmath.h"

// Fast inverse square root function (https://en.wikipedia.org/wiki/Fast_inverse_square_root).
// Source: https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/
float invSqrt(float x)
{
    union { float f; uint32_t u; } y = {x};
    y.u = 0x5F1FFF77ul - (y.u >> 1);
    return 0.703974056f * y.f * (2.38919526f - x * y.f * y.f);
}

float vector_dot(float a[3], float b[3])
{
    return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}

void vector_normalize(float a[3])
{
    float norm = invSqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    a[0] *= norm;
    a[1] *= norm;
    a[2] *= norm;
}