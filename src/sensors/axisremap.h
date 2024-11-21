/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 SlimeVR Contributors

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
#ifndef AXISREMAP_H
#define AXISREMAP_H

// Number label for axis selections
// 3 bits for each axis, 9 bits for one sensor, 18 bits at total
// bit 1-0
#define AXIS_REMAP_USE_X 0
#define AXIS_REMAP_USE_Y 1
#define AXIS_REMAP_USE_Z 2
// bit 2
#define AXIS_REMAP_INVERT 4

#define AXIS_REMAP_USE_XN (AXIS_REMAP_USE_X | AXIS_REMAP_INVERT)
#define AXIS_REMAP_USE_YN (AXIS_REMAP_USE_Y | AXIS_REMAP_INVERT)
#define AXIS_REMAP_USE_ZN (AXIS_REMAP_USE_Z | AXIS_REMAP_INVERT)

// Macros to extract axis selection from remap descriptor
#define AXIS_REMAP_GET_X(desc) (desc & 0x07)
#define AXIS_REMAP_GET_Y(desc) ((desc >> 3) & 0x07)
#define AXIS_REMAP_GET_Z(desc) ((desc >> 6) & 0x07)
#define AXIS_REMAP_GET_MAGX(desc) ((desc >> 9) & 0x07)
#define AXIS_REMAP_GET_MAGY(desc) ((desc >> 12) & 0x07)
#define AXIS_REMAP_GET_MAGZ(desc) ((desc >> 15) & 0x07)
#define AXIS_REMAP_GET_ALL_IMU(desc) (desc & 0x1FF)
#define AXIS_REMAP_GET_ALL_MAG(desc) ((desc >> 9) & 0x1FF)

// Macro to build remap descriptor
#define AXIS_REMAP_AXIS_X(x) (x & 0x07)
#define AXIS_REMAP_AXIS_Y(y) ((y & 0x07) << 3)
#define AXIS_REMAP_AXIS_Z(z) ((z & 0x07) << 6)
#define AXIS_REMAP_AXIS_MAGX(x) ((x & 0x07) << 9)
#define AXIS_REMAP_AXIS_MAGY(y) ((y & 0x07) << 12)
#define AXIS_REMAP_AXIS_MAGZ(z) ((z & 0x07) << 15)

#define AXIS_REMAP_BUILD(x, y, z, mx, my, mz)                           \
	(AXIS_REMAP_AXIS_X(x) | AXIS_REMAP_AXIS_Y(y) | AXIS_REMAP_AXIS_Z(z) \
	 | AXIS_REMAP_AXIS_MAGX(mx) | AXIS_REMAP_AXIS_MAGY(my) | AXIS_REMAP_AXIS_MAGZ(mz))

#define AXIS_REMAP_DEFAULT \
	AXIS_REMAP_BUILD(      \
		AXIS_REMAP_USE_X,  \
		AXIS_REMAP_USE_Y,  \
		AXIS_REMAP_USE_Z,  \
		AXIS_REMAP_USE_X,  \
		AXIS_REMAP_USE_Y,  \
		AXIS_REMAP_USE_Z   \
	)

// Template functions for remapping
template <typename T>
T inline remapOneAxis(int axisdesc, T x, T y, T z) {
	T result;
	switch (axisdesc & 0x3) {
		case AXIS_REMAP_USE_X:
			result = x;
			break;
		case AXIS_REMAP_USE_Y:
			result = y;
			break;
		case AXIS_REMAP_USE_Z:
			result = z;
			break;
		default:
			result = 0;
	}
	return (axisdesc & AXIS_REMAP_INVERT) ? -result : result;
}

template <typename T>
void inline remapAllAxis(int axisdesc, T* x, T* y, T* z) {
	T bx = *x, by = *y, bz = *z;
	*x = remapOneAxis(AXIS_REMAP_GET_X(axisdesc), bx, by, bz);
	*y = remapOneAxis(AXIS_REMAP_GET_Y(axisdesc), bx, by, bz);
	*z = remapOneAxis(AXIS_REMAP_GET_Z(axisdesc), bx, by, bz);
}

#endif  // AXISREMAP_H
