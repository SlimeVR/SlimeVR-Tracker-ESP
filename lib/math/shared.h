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

#pragma once

#include <cmath>

#define UNIT_EPSILON 0.00001

#define CMP_EPSILON 0.00001
#define CMP_EPSILON2 (CMP_EPSILON * CMP_EPSILON)

#define CMP_NORMALIZE_TOLERANCE 0.000001
#define CMP_POINT_IN_PLANE_EPSILON 0.00001

#define Math_SQRT12 0.7071067811865475244008443621048490
#define Math_SQRT2 1.4142135623730950488016887242
#define Math_LN2 0.6931471805599453094172321215
#define Math_TAU 6.2831853071795864769252867666
#define Math_PI 3.14159265358979323846264338332795
#define Math_E 2.7182818284590452353602874714
#define Math_INF INFINITY
#define Math_NAN NAN

// Generic ABS function, for math uses please use Math::abs.
#ifndef ABS
#define ABS(m_v) (((m_v) < 0) ? (-(m_v)) : (m_v))
#endif

#ifndef SGN
#define SGN(m_v) (((m_v) < 0) ? (-1.0) : (+1.0))
#endif

#ifndef MIN
#define MIN(m_a, m_b) (((m_a) < (m_b)) ? (m_a) : (m_b))
#endif

#ifndef MAX
#define MAX(m_a, m_b) (((m_a) > (m_b)) ? (m_a) : (m_b))
#endif

#ifndef CLAMP
#define CLAMP(m_a, m_min, m_max) (((m_a) < (m_min)) ? (m_min) : (((m_a) > (m_max)) ? m_max : m_a))
#endif

// Generic swap template.
#ifndef SWAP
#define SWAP(m_x, m_y) __swap_tmpl((m_x), (m_y))
template <class T>
inline void __swap_tmpl(T& x, T& y) {
	T aux = x;
	x = y;
	y = aux;
}
#endif // SWAP

namespace Math {
	inline double fposmod(double p_x, double p_y) {
		return (p_x >= 0) ? std::fmod(p_x, p_y) : p_y - std::fmod(-p_x, p_y);
	}


	inline bool is_equal_approx(double a, double b) {
		// Check for exact equality first, required to handle "infinity" values.
		if (a == b) {
			return true;
		}
		// Then check for approximate equality.
		double tolerance = UNIT_EPSILON * ABS(a);
		if (tolerance < UNIT_EPSILON) {
			tolerance = UNIT_EPSILON;
		}
		return ABS(a - b) < tolerance;
	}

	inline bool is_equal_approx(double a, double b, double eps) {
		// Check for exact equality first, required to handle "infinity" values.
		if (a == b) {
			return true;
		}
		// Then check for approximate equality.
		return ABS(a - b) < eps;
	}

	inline bool is_zero_approx(double a){
		return (is_equal_approx(a, 0.));
	}



	static inline double lerp(double p_from, double p_to, double p_weight) { return p_from + (p_to - p_from) * p_weight; }
	static inline float lerp(float p_from, float p_to, float p_weight) { return p_from + (p_to - p_from) * p_weight; }

	static inline double lerp_angle(double p_from, double p_to, double p_weight) {
		double difference = fmod(p_to - p_from, Math_TAU);
		double distance = fmod(2.0 * difference, Math_TAU) - difference;
		return p_from + distance * p_weight;
	}
	static inline float lerp_angle(float p_from, float p_to, float p_weight) {
		float difference = fmod(p_to - p_from, (float)Math_TAU);
		float distance = fmod(2.0f * difference, (float)Math_TAU) - difference;
		return p_from + distance * p_weight;
	}

	static inline double inverse_lerp(double p_from, double p_to, double p_value) { return (p_value - p_from) / (p_to - p_from); }
	static inline float inverse_lerp(float p_from, float p_to, float p_value) { return (p_value - p_from) / (p_to - p_from); }

	static inline double range_lerp(double p_value, double p_istart, double p_istop, double p_ostart, double p_ostop) { return Math::lerp(p_ostart, p_ostop, Math::inverse_lerp(p_istart, p_istop, p_value)); }
	static inline float range_lerp(float p_value, float p_istart, float p_istop, float p_ostart, float p_ostop) { return Math::lerp(p_ostart, p_ostop, Math::inverse_lerp(p_istart, p_istop, p_value)); }

	static inline double smoothstep(double p_from, double p_to, double p_s) {
		if (is_equal_approx(p_from, p_to)) {
			return p_from;
		}
		double s = CLAMP((p_s - p_from) / (p_to - p_from), 0.0, 1.0);
		return s * s * (3.0 - 2.0 * s);
	}
	static inline float smoothstep(float p_from, float p_to, float p_s) {
		if (is_equal_approx(p_from, p_to)) {
			return p_from;
		}
		float s = CLAMP((p_s - p_from) / (p_to - p_from), 0.0f, 1.0f);
		return s * s * (3.0f - 2.0f * s);
	}


	inline int sign(double a) {
		return (a > 0) ? 1 : -1;
	}

};