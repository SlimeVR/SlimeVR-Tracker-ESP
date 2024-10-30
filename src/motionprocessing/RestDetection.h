// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-FileCopyrightText: 2022 SlimeVR Contributors
//
// SPDX-License-Identifier: MIT

// Separated and modified from VQF

#ifndef REST_DETECTION_H
#define REST_DETECTION_H

// #define REST_DETECTION_DISABLE_LPF

#include <Arduino.h>
#include <basicvqf.h>
#include <vqf.h>

#include "types.h"

#define NaN std::numeric_limits<sensor_real_t>::quiet_NaN()

struct RestDetectionParams {
	sensor_real_t biasClip;
	sensor_real_t biasSigmaRest;
	sensor_real_t restMinTime;
	sensor_real_t restFilterTau;
	sensor_real_t restThGyr;
	sensor_real_t restThAcc;
	RestDetectionParams()
		: biasClip(2.0f)
		, biasSigmaRest(0.03f)
		, restMinTime(1.5)
		, restFilterTau(0.5f)
		, restThGyr(2.0f)
		, restThAcc(0.5f) {}
};

inline sensor_real_t square(sensor_real_t x) { return x * x; }

class RestDetection {
public:
	RestDetection(sensor_real_t gyrTs, sensor_real_t accTs) {
		this->gyrTs = gyrTs;
		this->accTs = accTs;
		setup();
	}
	RestDetection(
		const RestDetectionParams& params,
		sensor_real_t gyrTs,
		sensor_real_t accTs
	) {
		this->params = params;
		this->gyrTs = gyrTs;
		this->accTs = accTs;
		setup();
	}

#ifndef REST_DETECTION_DISABLE_LPF
	void filterInitialState(
		sensor_real_t x0,
		const double b[3],
		const double a[2],
		double out[]
	) {
		// initial state for steady state (equivalent to scipy.signal.lfilter_zi,
		// obtained by setting y=x=x0 in the filter update equation)
		out[0] = x0 * (1 - b[0]);
		out[1] = x0 * (b[2] - a[1]);
	}

	sensor_real_t
	filterStep(sensor_real_t x, const double b[3], const double a[2], double state[2]) {
		// difference equations based on scipy.signal.lfilter documentation
		// assumes that a0 == 1.0
		double y = b[0] * x + state[0];
		state[0] = b[1] * x - a[0] * y + state[1];
		state[1] = b[2] * x - a[1] * y;
		return y;
	}

	void filterVec(
		const sensor_real_t x[],
		size_t N,
		sensor_real_t tau,
		sensor_real_t Ts,
		const double b[3],
		const double a[2],
		double state[],
		sensor_real_t out[]
	) {
		assert(N >= 2);

		// to avoid depending on a single sample, average the first samples (for
		// duration tau) and then use this average to calculate the filter initial state
		if (isnan(state[0])) {  // initialization phase
			if (isnan(state[1])) {  // first sample
				state[1] = 0;  // state[1] is used to store the sample count
				for (size_t i = 0; i < N; i++) {
					state[2 + i] = 0;  // state[2+i] is used to store the sum
				}
			}
			state[1]++;
			for (size_t i = 0; i < N; i++) {
				state[2 + i] += x[i];
				out[i] = state[2 + i] / state[1];
			}
			if (state[1] * Ts >= tau) {
				for (size_t i = 0; i < N; i++) {
					filterInitialState(out[i], b, a, state + 2 * i);
				}
			}
			return;
		}

		for (size_t i = 0; i < N; i++) {
			out[i] = filterStep(x[i], b, a, state + 2 * i);
		}
	}
#endif

	void updateGyr(const sensor_real_t gyr[3]) {
#ifdef REST_DETECTION_DISABLE_LPF
		gyrLastSquaredDeviation = square(gyr[0] - lastSample.gyr[0])
								+ square(gyr[1] - lastSample.gyr[1])
								+ square(gyr[2] - lastSample.gyr[2]);

		sensor_real_t biasClip = params.biasClip * sensor_real_t(M_PI / 180.0);
		if (gyrLastSquaredDeviation
				>= square(params.restThGyr * sensor_real_t(M_PI / 180.0))
			|| fabs(lastSample.gyr[0]) > biasClip || fabs(lastSample.gyr[1]) > biasClip
			|| fabs(lastSample.gyr[2]) > biasClip) {
			restTime = 0;
			restDetected = false;
		}

		lastSample.gyr[0] = gyr[0];
		lastSample.gyr[1] = gyr[1];
		lastSample.gyr[2] = gyr[2];
#else
		filterVec(
			gyr,
			3,
			params.restFilterTau,
			gyrTs,
			restGyrLpB,
			restGyrLpA,
			restGyrLpState,
			restLastGyrLp
		);

		gyrLastSquaredDeviation = square(gyr[0] - restLastGyrLp[0])
								+ square(gyr[1] - restLastGyrLp[1])
								+ square(gyr[2] - restLastGyrLp[2]);

		sensor_real_t biasClip = params.biasClip * sensor_real_t(M_PI / 180.0);
		if (gyrLastSquaredDeviation
				>= square(params.restThGyr * sensor_real_t(M_PI / 180.0))
			|| fabs(restLastGyrLp[0]) > biasClip || fabs(restLastGyrLp[1]) > biasClip
			|| fabs(restLastGyrLp[2]) > biasClip) {
			restTime = 0;
			restDetected = false;
		}
#endif
	}

	void updateAcc(sensor_real_t dt, const sensor_real_t acc[3]) {
		if (acc[0] == sensor_real_t(0.0) && acc[1] == sensor_real_t(0.0)
			&& acc[2] == sensor_real_t(0.0)) {
			return;
		}

#ifdef REST_DETECTION_DISABLE_LPF
		accLastSquaredDeviation = square(acc[0] - lastSample.acc[0])
								+ square(acc[1] - lastSample.acc[1])
								+ square(acc[2] - lastSample.acc[2]);

		if (accLastSquaredDeviation >= square(params.restThAcc)) {
			restTime = 0;
			restDetected = false;
		} else {
			restTime += dt;
			if (restTime >= params.restMinTime) {
				restDetected = true;
			}
		}

		lastSample.acc[0] = acc[0];
		lastSample.acc[1] = acc[1];
		lastSample.acc[2] = acc[2];
#else
		filterVec(
			acc,
			3,
			params.restFilterTau,
			accTs,
			restAccLpB,
			restAccLpA,
			restAccLpState,
			restLastAccLp
		);

		accLastSquaredDeviation = square(acc[0] - restLastAccLp[0])
								+ square(acc[1] - restLastAccLp[1])
								+ square(acc[2] - restLastAccLp[2]);

		if (accLastSquaredDeviation >= square(params.restThAcc)) {
			restTime = 0;
			restDetected = false;
		} else {
			restTime += dt;
			if (restTime >= params.restMinTime) {
				restDetected = true;
			}
		}
#endif
	}

	bool getRestDetected() { return restDetected; }

#ifndef REST_DETECTION_DISABLE_LPF
	void resetState() {
		restDetected = false;

		gyrLastSquaredDeviation = 0.0;
		accLastSquaredDeviation = 0.0;
		restTime = 0.0;
		std::fill(restLastGyrLp, restLastGyrLp + 3, 0.0);
		std::fill(restGyrLpState, restGyrLpState + 3 * 2, NaN);
		std::fill(restLastAccLp, restLastAccLp + 3, 0.0);
		std::fill(restAccLpState, restAccLpState + 3 * 2, NaN);
	}

	void
	filterCoeffs(sensor_real_t tau, sensor_real_t Ts, double outB[], double outA[]) {
		assert(tau > 0);
		assert(Ts > 0);
		// second order Butterworth filter based on https://stackoverflow.com/a/52764064
		double fc
			= (M_SQRT2 / (2.0 * M_PI))
			/ double(tau
			);  // time constant of dampened, non-oscillating part of step response
		double C = tan(M_PI * fc * double(Ts));
		double D = C * C + sqrt(2) * C + 1;
		double b0 = C * C / D;
		outB[0] = b0;
		outB[1] = 2 * b0;
		outB[2] = b0;
		// a0 = 1.0
		outA[0] = 2 * (C * C - 1) / D;  // a1
		outA[1] = (1 - sqrt(2) * C + C * C) / D;  // a2
	}
#endif

	void setup() {
#ifndef REST_DETECTION_DISABLE_LPF
		assert(gyrTs > 0);
		assert(accTs > 0);

		filterCoeffs(params.restFilterTau, gyrTs, restGyrLpB, restGyrLpA);
		filterCoeffs(params.restFilterTau, accTs, restAccLpB, restAccLpA);

		resetState();
#endif
	}

private:
	RestDetectionParams params;
	bool restDetected;
	sensor_real_t restTime;
	sensor_real_t gyrLastSquaredDeviation = 0;
	sensor_real_t accLastSquaredDeviation = 0;

	sensor_real_t gyrTs;
	sensor_real_t accTs;
#ifndef REST_DETECTION_DISABLE_LPF
	sensor_real_t restLastGyrLp[3];
	double restGyrLpState[3 * 2];
	double restGyrLpB[3];
	double restGyrLpA[2];
	sensor_real_t restLastAccLp[3];
	double restAccLpState[3 * 2];
	double restAccLpB[3];
	double restAccLpA[2];
#else
	struct {
		float gyr[3];
		float acc[3];
	} lastSample;
#endif
};

#endif