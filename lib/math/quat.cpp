/*************************************************************************/
/*  quat.cpp                                                             */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2021 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2021 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#include "quat.h"
#include "basis.h"

// set_euler_xyz expects a vector containing the Euler angles in the format
// (ax,ay,az), where ax is the angle of rotation around x axis,
// and similar for other axes.
// This implementation uses XYZ convention (Z is the first rotation).
void Quat::set_euler_xyz(const Vector3& p_euler) {
	float half_a1 = p_euler.x * 0.5;
	float half_a2 = p_euler.y * 0.5;
	float half_a3 = p_euler.z * 0.5;

	// R = X(a1).Y(a2).Z(a3) convention for Euler angles.
	// Conversion to quaternion as listed in https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770024290.pdf (page A-2)
	// a3 is the angle of the first rotation, following the notation in this reference.

	float cos_a1 = std::cos(half_a1);
	float sin_a1 = std::sin(half_a1);
	float cos_a2 = std::cos(half_a2);
	float sin_a2 = std::sin(half_a2);
	float cos_a3 = std::cos(half_a3);
	float sin_a3 = std::sin(half_a3);

	set(sin_a1 * cos_a2 * cos_a3 + sin_a2 * sin_a3 * cos_a1,
		-sin_a1 * sin_a3 * cos_a2 + sin_a2 * cos_a1 * cos_a3,
		sin_a1 * sin_a2 * cos_a3 + sin_a3 * cos_a1 * cos_a2,
		-sin_a1 * sin_a2 * sin_a3 + cos_a1 * cos_a2 * cos_a3);
}


// set_euler_yxz expects a vector containing the Euler angles in the format
// (ax,ay,az), where ax is the angle of rotation around x axis,
// and similar for other axes.
// This implementation uses YXZ convention (Z is the first rotation).
void Quat::set_euler_yxz(const Vector3& p_euler) {
	float half_a1 = p_euler.y * 0.5;
	float half_a2 = p_euler.x * 0.5;
	float half_a3 = p_euler.z * 0.5;

	// R = Y(a1).X(a2).Z(a3) convention for Euler angles.
	// Conversion to quaternion as listed in https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770024290.pdf (page A-6)
	// a3 is the angle of the first rotation, following the notation in this reference.

	float cos_a1 = std::cos(half_a1);
	float sin_a1 = std::sin(half_a1);
	float cos_a2 = std::cos(half_a2);
	float sin_a2 = std::sin(half_a2);
	float cos_a3 = std::cos(half_a3);
	float sin_a3 = std::sin(half_a3);

	set(sin_a1 * cos_a2 * sin_a3 + cos_a1 * sin_a2 * cos_a3,
		sin_a1 * cos_a2 * cos_a3 - cos_a1 * sin_a2 * sin_a3,
		-sin_a1 * sin_a2 * cos_a3 + cos_a1 * cos_a2 * sin_a3,
		sin_a1 * sin_a2 * sin_a3 + cos_a1 * cos_a2 * cos_a3);
}

void Quat::operator*=(const Quat& q) {
	set(w * q.x + x * q.w + y * q.z - z * q.y,
		w * q.y + y * q.w + z * q.x - x * q.z,
		w * q.z + z * q.w + x * q.y - y * q.x,
		w * q.w - x * q.x - y * q.y - z * q.z);
}

Quat Quat::operator*(const Quat& q) const {
	Quat r = *this;
	r *= q;
	return r;
}

bool Quat::is_equal_approx(const Quat& p_quat) const {
	return Math::is_equal_approx(x, p_quat.x) && Math::is_equal_approx(y, p_quat.y) && Math::is_equal_approx(z, p_quat.z) && Math::is_equal_approx(w, p_quat.w);
}

float Quat::length() const {
	return std::sqrt(length_squared());
}

void Quat::normalize() {
	*this /= length();
}

Quat Quat::normalized() const {
	return *this / length();
}

bool Quat::is_normalized() const {
	return Math::is_equal_approx(length_squared(), 1.0, UNIT_EPSILON); //use less epsilon
}

bool Quat::equalsWithEpsilon(const Quat& q2) {
	return ABS(x - q2.x) < 0.0001f && ABS(y - q2.y) < 0.0001f && ABS(z - q2.z) < 0.0001f && ABS(w - q2.w) < 0.0001f;
}

Quat Quat::inverse() const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!is_normalized(), Quat(), "The quaternion must be normalized.");
#endif
	return Quat(-x, -y, -z, w);
}

Quat Quat::slerp(const Quat& q, const float& t) const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!is_normalized(), Quat(), "The start quaternion must be normalized.");
	ERR_FAIL_COND_V_MSG(!q.is_normalized(), Quat(), "The end quaternion must be normalized.");
#endif
	Quat to1;
	float omega, cosom, sinom, scale0, scale1;

	// calc cosine
	cosom = dot(q);

	// adjust signs (if necessary)
	if (cosom < 0.0) {
		cosom = -cosom;
		to1.x = -q.x;
		to1.y = -q.y;
		to1.z = -q.z;
		to1.w = -q.w;
	}
	else {
		to1.x = q.x;
		to1.y = q.y;
		to1.z = q.z;
		to1.w = q.w;
	}

	// calculate coefficients

	if ((1.0 - cosom) > CMP_EPSILON) {
		// standard case (slerp)
		omega = std::acos(cosom);
		sinom = std::sin(omega);
		scale0 = std::sin((1.0 - t) * omega) / sinom;
		scale1 = std::sin(t * omega) / sinom;
	}
	else {
		// "from" and "to" quaternions are very close
		//  ... so we can do a linear interpolation
		scale0 = 1.0 - t;
		scale1 = t;
	}
	// calculate final values
	return Quat(
		scale0 * x + scale1 * to1.x,
		scale0 * y + scale1 * to1.y,
		scale0 * z + scale1 * to1.z,
		scale0 * w + scale1 * to1.w);
}

Quat Quat::slerpni(const Quat& q, const float& t) const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!is_normalized(), Quat(), "The start quaternion must be normalized.");
	ERR_FAIL_COND_V_MSG(!q.is_normalized(), Quat(), "The end quaternion must be normalized.");
#endif
	const Quat& from = *this;

	float dot = from.dot(q);

	if (std::abs(dot) > 0.9999) {
		return from;
	}

	float theta = std::acos(dot),
		sinT = 1.0 / std::sin(theta),
		newFactor = std::sin(t * theta) * sinT,
		invFactor = std::sin((1.0 - t) * theta) * sinT;

	return Quat(invFactor * from.x + newFactor * q.x,
		invFactor * from.y + newFactor * q.y,
		invFactor * from.z + newFactor * q.z,
		invFactor * from.w + newFactor * q.w);
}

Quat Quat::cubic_slerp(const Quat& q, const Quat& prep, const Quat& postq, const float& t) const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!is_normalized(), Quat(), "The start quaternion must be normalized.");
	ERR_FAIL_COND_V_MSG(!q.is_normalized(), Quat(), "The end quaternion must be normalized.");
#endif
	//the only way to do slerp :|
	float t2 = (1.0 - t) * t * 2;
	Quat sp = this->slerp(q, t);
	Quat sq = prep.slerpni(postq, t);
	return sp.slerpni(sq, t2);
}

void Quat::set_axis_angle(const Vector3& axis, const float& angle) {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_MSG(!axis.is_normalized(), "The axis Vector3 must be normalized.");
#endif
	float d = axis.length();
	if (d == 0) {
		set(0, 0, 0, 0);
	}
	else {
		float sin_angle = std::sin(angle * 0.5);
		float cos_angle = std::cos(angle * 0.5);
		float s = sin_angle / d;
		set(axis.x * s, axis.y * s, axis.z * s,
			cos_angle);
	}
}
