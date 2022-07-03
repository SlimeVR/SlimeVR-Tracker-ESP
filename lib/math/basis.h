/*************************************************************************/
/*  basis.h                                                              */
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

#ifndef BASIS_H
#define BASIS_H

#include "vector3.h"
#include "quat.h"

class Basis {
public:
	Vector3 elements[3] = {
		Vector3(1, 0, 0),
		Vector3(0, 1, 0),
		Vector3(0, 0, 1)
	};

	inline const Vector3& operator[](int axis) const {
		return elements[axis];
	}
	inline Vector3& operator[](int axis) {
		return elements[axis];
	}

	void invert();
	void transpose();

	Basis inverse() const;
	Basis transposed() const;

	inline double determinant() const;

	void from_z(const Vector3& p_z);

	inline Vector3 get_axis(int p_axis) const {
		// get actual basis axis (elements is transposed for performance)
		return Vector3(elements[0][p_axis], elements[1][p_axis], elements[2][p_axis]);
	}
	inline void set_axis(int p_axis, const Vector3& p_value) {
		// get actual basis axis (elements is transposed for performance)
		elements[0][p_axis] = p_value.x;
		elements[1][p_axis] = p_value.y;
		elements[2][p_axis] = p_value.z;
	}

	void rotate(const Vector3& p_axis, double p_phi);
	Basis rotated(const Vector3& p_axis, double p_phi) const;

	void rotate_local(const Vector3& p_axis, double p_phi);
	Basis rotated_local(const Vector3& p_axis, double p_phi) const;

	void rotate(const Vector3& p_euler);
	Basis rotated(const Vector3& p_euler) const;

	void rotate(const Quat& p_quat);
	Basis rotated(const Quat& p_quat) const;

	Vector3 get_rotation_euler() const;
	void get_rotation_axis_angle(Vector3& p_axis, double& p_angle) const;
	void get_rotation_axis_angle_local(Vector3& p_axis, double& p_angle) const;
	Quat get_rotation_quat() const;
	Vector3 get_rotation() const { return get_rotation_euler(); };

	Vector3 rotref_posscale_decomposition(Basis& rotref) const;

	Vector3 get_euler_xyz() const;
	void set_euler_xyz(const Vector3& p_euler);

	Vector3 get_euler_xzy() const;
	void set_euler_xzy(const Vector3& p_euler);

	Vector3 get_euler_yzx() const;
	void set_euler_yzx(const Vector3& p_euler);

	Vector3 get_euler_yxz() const;
	void set_euler_yxz(const Vector3& p_euler);

	Vector3 get_euler_zxy() const;
	void set_euler_zxy(const Vector3& p_euler);

	Vector3 get_euler_zyx() const;
	void set_euler_zyx(const Vector3& p_euler);

	Quat get_quat() const;

	Vector3 get_euler() const { return get_euler_yxz(); }
	void set_euler(const Vector3& p_euler) { set_euler_yxz(p_euler); }

	void get_axis_angle(Vector3& r_axis, double& r_angle) const;
	void set_axis_angle(const Vector3& p_axis, double p_phi);

	void scale(const Vector3& p_scale);
	Basis scaled(const Vector3& p_scale) const;

	void scale_local(const Vector3& p_scale);
	Basis scaled_local(const Vector3& p_scale) const;

	void make_scale_uniform();
	float get_uniform_scale() const;

	Vector3 get_scale() const;
	Vector3 get_scale_abs() const;
	Vector3 get_scale_local() const;

	void set_axis_angle_scale(const Vector3& p_axis, double p_phi, const Vector3& p_scale);
	void set_euler_scale(const Vector3& p_euler, const Vector3& p_scale);
	void set_quat_scale(const Quat& p_quat, const Vector3& p_scale);

	// transposed dot products
	inline double tdotx(const Vector3& v) const {
		return elements[0][0] * v[0] + elements[1][0] * v[1] + elements[2][0] * v[2];
	}
	inline double tdoty(const Vector3& v) const {
		return elements[0][1] * v[0] + elements[1][1] * v[1] + elements[2][1] * v[2];
	}
	inline double tdotz(const Vector3& v) const {
		return elements[0][2] * v[0] + elements[1][2] * v[1] + elements[2][2] * v[2];
	}

	bool is_equal_approx(const Basis& p_basis) const;

	bool operator==(const Basis& p_matrix) const;
	bool operator!=(const Basis& p_matrix) const;

	inline Vector3 xform(const Vector3& p_vector) const;
	inline Vector3 xform_inv(const Vector3& p_vector) const;
	inline void operator*=(const Basis& p_matrix);
	inline Basis operator*(const Basis& p_matrix) const;
	inline void operator+=(const Basis& p_matrix);
	inline Basis operator+(const Basis& p_matrix) const;
	inline void operator-=(const Basis& p_matrix);
	inline Basis operator-(const Basis& p_matrix) const;
	inline void operator*=(double p_val);
	inline Basis operator*(double p_val) const;

	int get_orthogonal_index() const;
	void set_orthogonal_index(int p_index);

	void set_diagonal(const Vector3& p_diag);

	bool is_orthogonal() const;
	bool is_diagonal() const;
	bool is_rotation() const;

	Basis slerp(const Basis& target, const double& t) const;
	void rotate_sh(double* p_values);

	/* create / set */

	inline void set(double xx, double xy, double xz, double yx, double yy, double yz, double zx, double zy, double zz) {
		elements[0][0] = xx;
		elements[0][1] = xy;
		elements[0][2] = xz;
		elements[1][0] = yx;
		elements[1][1] = yy;
		elements[1][2] = yz;
		elements[2][0] = zx;
		elements[2][1] = zy;
		elements[2][2] = zz;
	}
	inline void set(const Vector3& p_x, const Vector3& p_y, const Vector3& p_z) {
		set_axis(0, p_x);
		set_axis(1, p_y);
		set_axis(2, p_z);
	}
	inline Vector3 get_column(int i) const {
		return Vector3(elements[0][i], elements[1][i], elements[2][i]);
	}

	inline Vector3 get_row(int i) const {
		return Vector3(elements[i][0], elements[i][1], elements[i][2]);
	}
	inline Vector3 get_main_diagonal() const {
		return Vector3(elements[0][0], elements[1][1], elements[2][2]);
	}

	inline void set_row(int i, const Vector3& p_row) {
		elements[i][0] = p_row.x;
		elements[i][1] = p_row.y;
		elements[i][2] = p_row.z;
	}

	inline void set_zero() {
		elements[0].zero();
		elements[1].zero();
		elements[2].zero();
	}

	inline Basis transpose_xform(const Basis& m) const {
		return Basis(
			elements[0].x * m[0].x + elements[1].x * m[1].x + elements[2].x * m[2].x,
			elements[0].x * m[0].y + elements[1].x * m[1].y + elements[2].x * m[2].y,
			elements[0].x * m[0].z + elements[1].x * m[1].z + elements[2].x * m[2].z,
			elements[0].y * m[0].x + elements[1].y * m[1].x + elements[2].y * m[2].x,
			elements[0].y * m[0].y + elements[1].y * m[1].y + elements[2].y * m[2].y,
			elements[0].y * m[0].z + elements[1].y * m[1].z + elements[2].y * m[2].z,
			elements[0].z * m[0].x + elements[1].z * m[1].x + elements[2].z * m[2].x,
			elements[0].z * m[0].y + elements[1].z * m[1].y + elements[2].z * m[2].y,
			elements[0].z * m[0].z + elements[1].z * m[1].z + elements[2].z * m[2].z);
	}
	Basis(double xx, double xy, double xz, double yx, double yy, double yz, double zx, double zy, double zz) {
		set(xx, xy, xz, yx, yy, yz, zx, zy, zz);
	}

	void orthonormalize();
	Basis orthonormalized() const;

#ifdef MATH_CHECKS
	bool is_symmetric() const;
#endif
	Basis diagonalize();

	operator Quat() const { return get_quat(); }

	Basis(const Quat& p_quat, const Vector3& p_scale) { set_quat_scale(p_quat, p_scale); }

	Basis(const Vector3& p_euler) { set_euler(p_euler); }
	Basis(const Vector3& p_euler, const Vector3& p_scale) { set_euler_scale(p_euler, p_scale); }

	Basis(const Vector3& p_axis, double p_phi) { set_axis_angle(p_axis, p_phi); }
	Basis(const Vector3& p_axis, double p_phi, const Vector3& p_scale) { set_axis_angle_scale(p_axis, p_phi, p_scale); }

	inline Basis(const Vector3& row0, const Vector3& row1, const Vector3& row2) {
		elements[0] = row0;
		elements[1] = row1;
		elements[2] = row2;
	}

	inline Basis() {}
};

inline void Basis::operator*=(const Basis& p_matrix) {
	set(
		p_matrix.tdotx(elements[0]), p_matrix.tdoty(elements[0]), p_matrix.tdotz(elements[0]),
		p_matrix.tdotx(elements[1]), p_matrix.tdoty(elements[1]), p_matrix.tdotz(elements[1]),
		p_matrix.tdotx(elements[2]), p_matrix.tdoty(elements[2]), p_matrix.tdotz(elements[2]));
}

inline Basis Basis::operator*(const Basis& p_matrix) const {
	return Basis(
		p_matrix.tdotx(elements[0]), p_matrix.tdoty(elements[0]), p_matrix.tdotz(elements[0]),
		p_matrix.tdotx(elements[1]), p_matrix.tdoty(elements[1]), p_matrix.tdotz(elements[1]),
		p_matrix.tdotx(elements[2]), p_matrix.tdoty(elements[2]), p_matrix.tdotz(elements[2]));
}

inline void Basis::operator+=(const Basis& p_matrix) {
	elements[0] += p_matrix.elements[0];
	elements[1] += p_matrix.elements[1];
	elements[2] += p_matrix.elements[2];
}

inline Basis Basis::operator+(const Basis& p_matrix) const {
	Basis ret(*this);
	ret += p_matrix;
	return ret;
}

inline void Basis::operator-=(const Basis& p_matrix) {
	elements[0] -= p_matrix.elements[0];
	elements[1] -= p_matrix.elements[1];
	elements[2] -= p_matrix.elements[2];
}

inline Basis Basis::operator-(const Basis& p_matrix) const {
	Basis ret(*this);
	ret -= p_matrix;
	return ret;
}

inline void Basis::operator*=(double p_val) {
	elements[0] *= p_val;
	elements[1] *= p_val;
	elements[2] *= p_val;
}

inline Basis Basis::operator*(double p_val) const {
	Basis ret(*this);
	ret *= p_val;
	return ret;
}

Vector3 Basis::xform(const Vector3& p_vector) const {
	return Vector3(
		elements[0].dot(p_vector),
		elements[1].dot(p_vector),
		elements[2].dot(p_vector));
}

Vector3 Basis::xform_inv(const Vector3& p_vector) const {
	return Vector3(
		(elements[0][0] * p_vector.x) + (elements[1][0] * p_vector.y) + (elements[2][0] * p_vector.z),
		(elements[0][1] * p_vector.x) + (elements[1][1] * p_vector.y) + (elements[2][1] * p_vector.z),
		(elements[0][2] * p_vector.x) + (elements[1][2] * p_vector.y) + (elements[2][2] * p_vector.z));
}

double Basis::determinant() const {
	return elements[0][0] * (elements[1][1] * elements[2][2] - elements[2][1] * elements[1][2]) -
		elements[1][0] * (elements[0][1] * elements[2][2] - elements[2][1] * elements[0][2]) +
		elements[2][0] * (elements[0][1] * elements[1][2] - elements[1][1] * elements[0][2]);
}

#endif