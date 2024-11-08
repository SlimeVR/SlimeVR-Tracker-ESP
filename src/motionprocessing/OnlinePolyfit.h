/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 SlimeVR Contributors

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
#include <Arduino.h>

#ifndef ONLINE_POLYFIT_H
#define ONLINE_POLYFIT_H

template <uint32_t degree, uint32_t dimensions, uint64_t forgettingFactorNumSamples>
class OnlineVectorPolyfit {
public:
	constexpr static int32_t numDimensions = dimensions;
	constexpr static int32_t numCoefficients = degree + 1;
	constexpr static double forgettingFactor
		= std::exp(-1.0 / forgettingFactorNumSamples);

	OnlineVectorPolyfit() { reset(); }

	void reset() {
		std::fill(Rb[0], Rb[0] + rows * cols, 0.0);
		std::fill(coeffs[0], coeffs[0] + numDimensions * rows, 0.0f);
	}

	// Recursive least squares update using QR decomposition by Givens transformations
	void update(double xValue, const double yValues[numDimensions]) {
		double xin[cols];
		xin[0] = 1;
		for (int32_t i = 1; i < cols - numDimensions; i++) {
			xin[i] = xin[i - 1] * xValue;
		}
		for (int32_t i = 0; i < numDimensions; i++) {
			xin[cols - numDimensions + i] = yValues[i];
		}

		// degree = 3, dimensions = 3, yValues = [x, y, z]
		// B I I I Ix Iy Iz       R R R R bx by bz
		// . B I I Ix Iy Iz  ===  . R R R bx by bz
		// . . B I Ix Iy Iz  ===  . . R R bx by bz
		// . . . B Ix Iy Iz       . . . R bx by bz

		// https://www.eecs.harvard.edu/~htk/publication/1981-matrix-triangularization-by-systolic-arrays.pdf

		for (int32_t y = 0; y < rows; y++) {
			double c = 1, s = 0;
			if (xin[y] != 0.0) {
				Rb[y][y] *= forgettingFactor;
				const double norm = sqrt(Rb[y][y] * Rb[y][y] + xin[y] * xin[y]);
				c = Rb[y][y] * (1.0 / norm);
				s = xin[y] * (1.0 / norm);
				Rb[y][y] = norm;
			}
			for (int32_t x = y + 1; x < cols; x++) {
				Rb[y][x] *= forgettingFactor;
				const double xout = (c * xin[x] - s * Rb[y][x]);
				Rb[y][x] = (s * xin[x] + c * Rb[y][x]);
				xin[x] = xout;
			}
		}
	}

	// Back solves upper triangular system
	// Returns float[numDimensions][numCoefficients] coefficients from lowest to highest
	// power
	const auto computeCoefficients() {
		// https://en.wikipedia.org/wiki/Triangular_matrix#Forward_and_back_substitution
		for (int32_t d = 0; d < numDimensions; d++) {
			for (int32_t y = rows - 1; y >= 0; y--) {
				const int32_t bColumn = cols - numDimensions + d;
				coeffs[d][y] = Rb[y][bColumn];
				if (Rb[y][y] == 0.0) {
					continue;
				}
				for (int32_t x = y + 1; x < rows; x++) {
					coeffs[d][y] -= coeffs[d][x] * Rb[y][x];
				}
				coeffs[d][y] /= Rb[y][y];
			}
		}
		return coeffs;
	}

	float predict(int32_t d, float x) {
		if (d >= numDimensions) {
			return 0.0;
		}
		// https://en.wikipedia.org/wiki/Horner%27s_method
		float y = coeffs[d][numCoefficients - 1];
		for (int32_t i = numCoefficients - 2; i >= 0; i--) {
			y = y * x + coeffs[d][i];
		}
		return y;
	}

	std::pair<float, float> tangentAt(float x) {
		float intercept = coeffs[0];
		float slope = coeffs[1];
		for (uint32_t i = 2; i < degree + 1; i++) {
			intercept -= coeffs[i] * (i - 1) * pow(x, i);
			slope += coeffs[i] * i * pow(x, i - 1);
		}
		return std::make_pair(slope, intercept);
	}

private:
	constexpr static int32_t rows = numCoefficients;
	constexpr static int32_t cols = numCoefficients + 3;
	double Rb[rows][cols];
	float coeffs[numDimensions][rows];
};

#endif