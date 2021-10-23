/*************************************************************************/
/*  quat.h                                                             */
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

// Circular dependency between Vector3 and Basis :/
#include "vector3.h"

#ifndef QUAT_H
#define QUAT_H

#include "shared.h"
#include <cmath>

class Basis;

class Quat {
public:
	union {
		struct {
			float x;
			float y;
			float z;
			float w;
		};
		float components[4] = { 0, 0, 0, 1.0 };
	};

	inline float& operator[](int idx) {
		return components[idx];
	}
	inline const float& operator[](int idx) const {
		return components[idx];
	}
	inline float length_squared() const;
	bool is_equal_approx(const Quat& p_quat) const;
	float length() const;
	void normalize();
	Quat normalized() const;
	bool is_normalized() const;
	Quat inverse() const;
	inline float dot(const Quat& q) const;

	void set_euler_xyz(const Vector3& p_euler);
	Vector3 get_euler_xyz() const;
	void set_euler_yxz(const Vector3& p_euler);
	Vector3 get_euler_yxz() const;

	void set_euler(const Vector3& p_euler) { set_euler_yxz(p_euler); };
	Vector3 get_euler() const { return get_euler_yxz(); };

	Quat slerp(const Quat& q, const float& t) const;
	Quat slerpni(const Quat& q, const float& t) const;
	Quat cubic_slerp(const Quat& q, const Quat& prep, const Quat& postq, const float& t) const;
	bool equalsWithEpsilon(const Quat& q2);

	void set_axis_angle(const Vector3& axis, const float& angle);
	inline void get_axis_angle(Vector3& r_axis, double& r_angle) const {
		r_angle = 2 * std::acos(w);
		double r = ((double)1) / std::sqrt(1 - w * w);
		r_axis.x = x * r;
		r_axis.y = y * r;
		r_axis.z = z * r;
	}

	void operator*=(const Quat& q);
	Quat operator*(const Quat& q) const;

	Quat operator*(const Vector3& v) const {
		return Quat(w * v.x + y * v.z - z * v.y,
			w * v.y + z * v.x - x * v.z,
			w * v.z + x * v.y - y * v.x,
			-x * v.x - y * v.y - z * v.z);
	}

	inline Vector3 xform(const Vector3& v) const {
#ifdef MATH_CHECKS
		ERR_FAIL_COND_V_MSG(!is_normalized(), v, "The quaternion must be normalized.");
#endif
		Vector3 u(x, y, z);
		Vector3 uv = u.cross(v);
		return v + ((uv * w) + u.cross(uv)) * ((float)2);
	}

	inline Vector3 xform_inv(const Vector3& v) const {
		return inverse().xform(v);
	}

	inline void operator+=(const Quat& q);
	inline void operator-=(const Quat& q);
	inline void operator*=(const float& s);
	inline void operator/=(const float& s);
	inline Quat operator+(const Quat& q2) const;
	inline Quat operator-(const Quat& q2) const;
	inline Quat operator-() const;
	inline Quat operator*(const float& s) const;
	inline Quat operator/(const float& s) const;

	inline bool operator==(const Quat& p_quat) const;
	inline bool operator!=(const Quat& p_quat) const;

	inline void set(float p_x, float p_y, float p_z, float p_w) {
		x = p_x;
		y = p_y;
		z = p_z;
		w = p_w;
	}

	inline Quat() {}
	inline Quat(float p_x, float p_y, float p_z, float p_w) :
		x(p_x),
		y(p_y),
		z(p_z),
		w(p_w) {
	}
	Quat(const Vector3& axis, const float& angle) { set_axis_angle(axis, angle); }

	Quat(const Vector3& euler) { set_euler(euler); }
	Quat(const Quat& q) :
		x(q.x),
		y(q.y),
		z(q.z),
		w(q.w) {
	}

	Quat& operator=(const Quat& q) {
		x = q.x;
		y = q.y;
		z = q.z;
		w = q.w;
		return *this;
	}

	Quat(const Vector3& v0, const Vector3& v1) // shortest arc
	{
		Vector3 c = v0.cross(v1);
		float d = v0.dot(v1);

		if (d < -1.0 + CMP_EPSILON) {
			x = 0;
			y = 1;
			z = 0;
			w = 0;
		}
		else {
			float s = std::sqrt((1.0 + d) * 2.0);
			float rs = 1.0 / s;

			x = c.x * rs;
			y = c.y * rs;
			z = c.z * rs;
			w = s * 0.5;
		}
	}
};

float Quat::dot(const Quat& q) const {
	return x * q.x + y * q.y + z * q.z + w * q.w;
}

float Quat::length_squared() const {
	return dot(*this);
}

void Quat::operator+=(const Quat& q) {
	x += q.x;
	y += q.y;
	z += q.z;
	w += q.w;
}

void Quat::operator-=(const Quat& q) {
	x -= q.x;
	y -= q.y;
	z -= q.z;
	w -= q.w;
}

void Quat::operator*=(const float& s) {
	x *= s;
	y *= s;
	z *= s;
	w *= s;
}

void Quat::operator/=(const float& s) {
	*this *= 1.0 / s;
}

Quat Quat::operator+(const Quat& q2) const {
	const Quat& q1 = *this;
	return Quat(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w);
}

Quat Quat::operator-(const Quat& q2) const {
	const Quat& q1 = *this;
	return Quat(q1.x - q2.x, q1.y - q2.y, q1.z - q2.z, q1.w - q2.w);
}

Quat Quat::operator-() const {
	const Quat& q2 = *this;
	return Quat(-q2.x, -q2.y, -q2.z, -q2.w);
}

Quat Quat::operator*(const float& s) const {
	return Quat(x * s, y * s, z * s, w * s);
}

Quat Quat::operator/(const float& s) const {
	return *this * (1.0 / s);
}

bool Quat::operator==(const Quat& p_quat) const {
	return x == p_quat.x && y == p_quat.y && z == p_quat.z && w == p_quat.w;
}

bool Quat::operator!=(const Quat& p_quat) const {
	return x != p_quat.x || y != p_quat.y || z != p_quat.z || w != p_quat.w;
}

inline Quat operator*(const float& p_real, const Quat& p_quat) {
	return p_quat * p_real;
}

#endif