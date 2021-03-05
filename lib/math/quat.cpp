
#include "quat.h"
#include "basis.h"

// set_euler_xyz expects a vector containing the Euler angles in the format
// (ax,ay,az), where ax is the angle of rotation around x axis,
// and similar for other axes.
// This implementation uses XYZ convention (Z is the first rotation).
void Quat::set_euler_xyz(const Vector3& p_euler) {
	double half_a1 = p_euler.x * 0.5;
	double half_a2 = p_euler.y * 0.5;
	double half_a3 = p_euler.z * 0.5;

	// R = X(a1).Y(a2).Z(a3) convention for Euler angles.
	// Conversion to quaternion as listed in https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770024290.pdf (page A-2)
	// a3 is the angle of the first rotation, following the notation in this reference.

	double cos_a1 = std::cos(half_a1);
	double sin_a1 = std::sin(half_a1);
	double cos_a2 = std::cos(half_a2);
	double sin_a2 = std::sin(half_a2);
	double cos_a3 = std::cos(half_a3);
	double sin_a3 = std::sin(half_a3);

	set(sin_a1 * cos_a2 * cos_a3 + sin_a2 * sin_a3 * cos_a1,
		-sin_a1 * sin_a3 * cos_a2 + sin_a2 * cos_a1 * cos_a3,
		sin_a1 * sin_a2 * cos_a3 + sin_a3 * cos_a1 * cos_a2,
		-sin_a1 * sin_a2 * sin_a3 + cos_a1 * cos_a2 * cos_a3);
}

// get_euler_xyz returns a vector containing the Euler angles in the format
// (ax,ay,az), where ax is the angle of rotation around x axis,
// and similar for other axes.
// This implementation uses XYZ convention (Z is the first rotation).
Vector3 Quat::get_euler_xyz() const {
	Basis m(*this);
	return m.get_euler_xyz();
}

// set_euler_yxz expects a vector containing the Euler angles in the format
// (ax,ay,az), where ax is the angle of rotation around x axis,
// and similar for other axes.
// This implementation uses YXZ convention (Z is the first rotation).
void Quat::set_euler_yxz(const Vector3& p_euler) {
	double half_a1 = p_euler.y * 0.5;
	double half_a2 = p_euler.x * 0.5;
	double half_a3 = p_euler.z * 0.5;

	// R = Y(a1).X(a2).Z(a3) convention for Euler angles.
	// Conversion to quaternion as listed in https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770024290.pdf (page A-6)
	// a3 is the angle of the first rotation, following the notation in this reference.

	double cos_a1 = std::cos(half_a1);
	double sin_a1 = std::sin(half_a1);
	double cos_a2 = std::cos(half_a2);
	double sin_a2 = std::sin(half_a2);
	double cos_a3 = std::cos(half_a3);
	double sin_a3 = std::sin(half_a3);

	set(sin_a1 * cos_a2 * sin_a3 + cos_a1 * sin_a2 * cos_a3,
		sin_a1 * cos_a2 * cos_a3 - cos_a1 * sin_a2 * sin_a3,
		-sin_a1 * sin_a2 * cos_a3 + cos_a1 * cos_a2 * sin_a3,
		sin_a1 * sin_a2 * sin_a3 + cos_a1 * cos_a2 * cos_a3);
}

// get_euler_yxz returns a vector containing the Euler angles in the format
// (ax,ay,az), where ax is the angle of rotation around x axis,
// and similar for other axes.
// This implementation uses YXZ convention (Z is the first rotation).
Vector3 Quat::get_euler_yxz() const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!is_normalized(), Vector3(0, 0, 0), "The quaternion must be normalized.");
#endif
	Basis m(*this);
	return m.get_euler_yxz();
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

double Quat::length() const {
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

Quat Quat::inverse() const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!is_normalized(), Quat(), "The quaternion must be normalized.");
#endif
	return Quat(-x, -y, -z, w);
}

Quat Quat::slerp(const Quat& q, const double& t) const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!is_normalized(), Quat(), "The start quaternion must be normalized.");
	ERR_FAIL_COND_V_MSG(!q.is_normalized(), Quat(), "The end quaternion must be normalized.");
#endif
	Quat to1;
	double omega, cosom, sinom, scale0, scale1;

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

Quat Quat::slerpni(const Quat& q, const double& t) const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!is_normalized(), Quat(), "The start quaternion must be normalized.");
	ERR_FAIL_COND_V_MSG(!q.is_normalized(), Quat(), "The end quaternion must be normalized.");
#endif
	const Quat& from = *this;

	double dot = from.dot(q);

	if (std::abs(dot) > 0.9999) {
		return from;
	}

	double theta = std::acos(dot),
		sinT = 1.0 / std::sin(theta),
		newFactor = std::sin(t * theta) * sinT,
		invFactor = std::sin((1.0 - t) * theta) * sinT;

	return Quat(invFactor * from.x + newFactor * q.x,
		invFactor * from.y + newFactor * q.y,
		invFactor * from.z + newFactor * q.z,
		invFactor * from.w + newFactor * q.w);
}

Quat Quat::cubic_slerp(const Quat& q, const Quat& prep, const Quat& postq, const double& t) const {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_V_MSG(!is_normalized(), Quat(), "The start quaternion must be normalized.");
	ERR_FAIL_COND_V_MSG(!q.is_normalized(), Quat(), "The end quaternion must be normalized.");
#endif
	//the only way to do slerp :|
	double t2 = (1.0 - t) * t * 2;
	Quat sp = this->slerp(q, t);
	Quat sq = prep.slerpni(postq, t);
	return sp.slerpni(sq, t2);
}

void Quat::set_axis_angle(const Vector3& axis, const double& angle) {
#ifdef MATH_CHECKS
	ERR_FAIL_COND_MSG(!axis.is_normalized(), "The axis Vector3 must be normalized.");
#endif
	double d = axis.length();
	if (d == 0) {
		set(0, 0, 0, 0);
	}
	else {
		double sin_angle = std::sin(angle * 0.5);
		double cos_angle = std::cos(angle * 0.5);
		double s = sin_angle / d;
		set(axis.x * s, axis.y * s, axis.z * s,
			cos_angle);
	}
}
