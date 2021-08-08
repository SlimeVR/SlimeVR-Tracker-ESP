/*************************************************************************/
/*  vector3.h                                                            */
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

#ifndef VECTOR3_H
#define VECTOR3_H

#include "shared.h"

class Basis;

struct Vector3 {
	enum Axis {
		AXIS_X,
		AXIS_Y,
		AXIS_Z,
	};

	union {
		struct {
			float x;
			float y;
			float z;
		};

		float coord[3] = { 0 };
	};

	inline const float& operator[](int p_axis) const {
		return coord[p_axis];
	}

	inline float& operator[](int p_axis) {
		return coord[p_axis];
	}

	void set_axis(int p_axis, float p_value);
	float get_axis(int p_axis) const;

	int min_axis() const;
	int max_axis() const;

	inline float length() const;
	inline float length_squared() const;

	inline void normalize();
	inline Vector3 normalized() const;
	inline bool is_normalized() const;
	inline Vector3 inverse() const;

	inline void zero();

	// void snap(Vector3 p_val);
	// Vector3 snapped(Vector3 p_val) const;

	void rotate(const Vector3& p_axis, float p_phi);
	Vector3 rotated(const Vector3& p_axis, float p_phi) const;

	/* Static Methods between 2 vector3s */

	inline Vector3 lerp(const Vector3& p_b, float p_t) const;
	inline Vector3 slerp(const Vector3& p_b, float p_t) const;
	Vector3 cubic_interpolate(const Vector3& p_b, const Vector3& p_pre_a, const Vector3& p_post_b, float p_t) const;
	Vector3 cubic_interpolaten(const Vector3& p_b, const Vector3& p_pre_a, const Vector3& p_post_b, float p_t) const;
	Vector3 move_toward(const Vector3& p_to, const float p_delta) const;

	inline Vector3 cross(const Vector3& p_b) const;
	inline float dot(const Vector3& p_b) const;
	Basis outer(const Vector3& p_b) const;
	Basis to_diagonal_matrix() const;

	inline Vector3 abs() const;
	inline Vector3 floor() const;
	inline Vector3 sign() const;
	inline Vector3 ceil() const;

	inline float distance_to(const Vector3& p_b) const;
	inline float distance_squared_to(const Vector3& p_b) const;

	inline Vector3 posmod(const float p_mod) const;
	inline Vector3 posmodv(const Vector3& p_modv) const;
	inline Vector3 project(const Vector3& p_b) const;

	inline float angle_to(const Vector3& p_b) const;
	inline Vector3 direction_to(const Vector3& p_b) const;

	inline Vector3 slide(const Vector3& p_normal) const;
	inline Vector3 bounce(const Vector3& p_normal) const;
	inline Vector3 reflect(const Vector3& p_normal) const;

	bool is_equal_approx(const Vector3& p_v) const;

	/* Operators */

	inline Vector3& operator+=(const Vector3& p_v);
	inline Vector3 operator+(const Vector3& p_v) const;
	inline Vector3& operator-=(const Vector3& p_v);
	inline Vector3 operator-(const Vector3& p_v) const;
	inline Vector3& operator*=(const Vector3& p_v);
	inline Vector3 operator*(const Vector3& p_v) const;
	inline Vector3& operator/=(const Vector3& p_v);
	inline Vector3 operator/(const Vector3& p_v) const;

	inline Vector3& operator*=(float p_scalar);
	inline Vector3 operator*(float p_scalar) const;
	inline Vector3& operator/=(float p_scalar);
	inline Vector3 operator/(float p_scalar) const;

	inline Vector3 operator-() const;

	inline bool operator==(const Vector3& p_v) const;
	inline bool operator!=(const Vector3& p_v) const;
	inline bool operator<(const Vector3& p_v) const;
	inline bool operator<=(const Vector3& p_v) const;
	inline bool operator>(const Vector3& p_v) const;
	inline bool operator>=(const Vector3& p_v) const;

	inline Vector3() {}
	inline Vector3(float p_x, float p_y, float p_z) {
		x = p_x;
		y = p_y;
		z = p_z;
	}
};

Vector3 Vector3::cross(const Vector3& p_b) const {
	Vector3 ret(
		(y * p_b.z) - (z * p_b.y),
		(z * p_b.x) - (x * p_b.z),
		(x * p_b.y) - (y * p_b.x));

	return ret;
}

float Vector3::dot(const Vector3& p_b) const {
	return x * p_b.x + y * p_b.y + z * p_b.z;
}

Vector3 Vector3::abs() const {
	return Vector3(std::abs(x), std::abs(y), std::abs(z));
}

Vector3 Vector3::sign() const {
	return Vector3(Math::sign(x), Math::sign(y), Math::sign(z));
}

Vector3 Vector3::floor() const {
	return Vector3(std::floor(x), std::floor(y), std::floor(z));
}

Vector3 Vector3::ceil() const {
	return Vector3(std::ceil(x), std::ceil(y), std::ceil(z));
}

Vector3 Vector3::lerp(const Vector3& p_b, float p_t) const {
	return Vector3(
		x + (p_t * (p_b.x - x)),
		y + (p_t * (p_b.y - y)),
		z + (p_t * (p_b.z - z)));
}

Vector3 Vector3::slerp(const Vector3& p_b, float p_t) const {
	float theta = angle_to(p_b);
	return rotated(cross(p_b).normalized(), theta * p_t);
}

float Vector3::distance_to(const Vector3& p_b) const {
	return (p_b - *this).length();
}

float Vector3::distance_squared_to(const Vector3& p_b) const {
	return (p_b - *this).length_squared();
}

Vector3 Vector3::posmod(const float p_mod) const {
	return Vector3(Math::fposmod(x, p_mod), Math::fposmod(y, p_mod), Math::fposmod(z, p_mod));
}

Vector3 Vector3::posmodv(const Vector3& p_modv) const {
	return Vector3(Math::fposmod(x, p_modv.x), Math::fposmod(y, p_modv.y), Math::fposmod(z, p_modv.z));
}

Vector3 Vector3::project(const Vector3& p_b) const {
	return p_b * (dot(p_b) / p_b.length_squared());
}

float Vector3::angle_to(const Vector3& p_b) const {
	return std::atan2(cross(p_b).length(), dot(p_b));
}

Vector3 Vector3::direction_to(const Vector3& p_b) const {
	Vector3 ret(p_b.x - x, p_b.y - y, p_b.z - z);
	ret.normalize();
	return ret;
}

/* Operators */

Vector3& Vector3::operator+=(const Vector3& p_v) {
	x += p_v.x;
	y += p_v.y;
	z += p_v.z;
	return *this;
}

Vector3 Vector3::operator+(const Vector3& p_v) const {
	return Vector3(x + p_v.x, y + p_v.y, z + p_v.z);
}

Vector3& Vector3::operator-=(const Vector3& p_v) {
	x -= p_v.x;
	y -= p_v.y;
	z -= p_v.z;
	return *this;
}

Vector3 Vector3::operator-(const Vector3& p_v) const {
	return Vector3(x - p_v.x, y - p_v.y, z - p_v.z);
}

Vector3& Vector3::operator*=(const Vector3& p_v) {
	x *= p_v.x;
	y *= p_v.y;
	z *= p_v.z;
	return *this;
}

Vector3 Vector3::operator*(const Vector3& p_v) const {
	return Vector3(x * p_v.x, y * p_v.y, z * p_v.z);
}

Vector3& Vector3::operator/=(const Vector3& p_v) {
	x /= p_v.x;
	y /= p_v.y;
	z /= p_v.z;
	return *this;
}

Vector3 Vector3::operator/(const Vector3& p_v) const {
	return Vector3(x / p_v.x, y / p_v.y, z / p_v.z);
}

Vector3& Vector3::operator*=(float p_scalar) {
	x *= p_scalar;
	y *= p_scalar;
	z *= p_scalar;
	return *this;
}

inline Vector3 operator*(float p_scalar, const Vector3& p_vec) {
	return p_vec * p_scalar;
}

Vector3 Vector3::operator*(float p_scalar) const {
	return Vector3(x * p_scalar, y * p_scalar, z * p_scalar);
}

Vector3& Vector3::operator/=(float p_scalar) {
	x /= p_scalar;
	y /= p_scalar;
	z /= p_scalar;
	return *this;
}

Vector3 Vector3::operator/(float p_scalar) const {
	return Vector3(x / p_scalar, y / p_scalar, z / p_scalar);
}

Vector3 Vector3::operator-() const {
	return Vector3(-x, -y, -z);
}

bool Vector3::operator==(const Vector3& p_v) const {
	return x == p_v.x && y == p_v.y && z == p_v.z;
}

bool Vector3::operator!=(const Vector3& p_v) const {
	return x != p_v.x || y != p_v.y || z != p_v.z;
}

bool Vector3::operator<(const Vector3& p_v) const {
	if (x == p_v.x) {
		if (y == p_v.y) {
			return z < p_v.z;
		}
		else {
			return y < p_v.y;
		}
	}
	else {
		return x < p_v.x;
	}
}

bool Vector3::operator>(const Vector3& p_v) const {
	if (x == p_v.x) {
		if (y == p_v.y) {
			return z > p_v.z;
		}
		else {
			return y > p_v.y;
		}
	}
	else {
		return x > p_v.x;
	}
}

bool Vector3::operator<=(const Vector3& p_v) const {
	if (x == p_v.x) {
		if (y == p_v.y) {
			return z <= p_v.z;
		}
		else {
			return y < p_v.y;
		}
	}
	else {
		return x < p_v.x;
	}
}

bool Vector3::operator>=(const Vector3& p_v) const {
	if (x == p_v.x) {
		if (y == p_v.y) {
			return z >= p_v.z;
		}
		else {
			return y > p_v.y;
		}
	}
	else {
		return x > p_v.x;
	}
}

inline Vector3 vec3_cross(const Vector3& p_a, const Vector3& p_b) {
	return p_a.cross(p_b);
}

inline float vec3_dot(const Vector3& p_a, const Vector3& p_b) {
	return p_a.dot(p_b);
}

float Vector3::length() const {
	float x2 = x * x;
	float y2 = y * y;
	float z2 = z * z;

	return std::sqrt(x2 + y2 + z2);
}

float Vector3::length_squared() const {
	float x2 = x * x;
	float y2 = y * y;
	float z2 = z * z;

	return x2 + y2 + z2;
}

void Vector3::normalize() {
	float lengthsq = length_squared();
	if (lengthsq == 0) {
		x = y = z = 0;
	}
	else {
		float length = std::sqrt(lengthsq);
		x /= length;
		y /= length;
		z /= length;
	}
}

Vector3 Vector3::normalized() const {
	Vector3 v = *this;
	v.normalize();
	return v;
}

bool Vector3::is_normalized() const {
	// use length_squared() instead of length() to avoid sqrt(), makes it more stringent.
	return Math::is_equal_approx(length_squared(), 1.0, UNIT_EPSILON);
}

Vector3 Vector3::inverse() const {
	return Vector3(1.0 / x, 1.0 / y, 1.0 / z);
}

void Vector3::zero() {
	x = y = z = 0;
}

// slide returns the component of the vector along the given plane, specified by its normal vector.
Vector3 Vector3::slide(const Vector3& p_normal) const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!p_normal.is_normalized(), Vector3(), "The normal Vector3 must be normalized.");
#endif
	return *this - p_normal * this->dot(p_normal);
}

Vector3 Vector3::bounce(const Vector3& p_normal) const {
	return -reflect(p_normal);
}

Vector3 Vector3::reflect(const Vector3& p_normal) const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!p_normal.is_normalized(), Vector3(), "The normal Vector3 must be normalized.");
#endif
	return 2.0 * p_normal * this->dot(p_normal) - *this;
}

#endif