// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

// Modified to add timestamps in: updateGyr(const vqf_real_t gyr[3], double gyrTs)
// Removed batch update functions

#include "basicvqf.h"

#include <algorithm>
#include <limits>
#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>

#define EPS std::numeric_limits<vqf_real_t>::epsilon()
#define NaN std::numeric_limits<vqf_real_t>::quiet_NaN()

inline vqf_real_t square(vqf_real_t x) { return x*x; }

BasicVQFParams::BasicVQFParams()
    : tauAcc(3.0)
    , tauMag(9.0)
{

}

BasicVQF::BasicVQF(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    coeffs.magTs = magTs > 0 ? magTs : gyrTs;

    setup();
}

BasicVQF::BasicVQF(const BasicVQFParams &params, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    this->params = params;

    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    coeffs.magTs = magTs > 0 ? magTs : gyrTs;

    setup();
}

void BasicVQF::updateGyr(const vqf_real_t gyr[3], double gyrTs)
{
    // gyroscope prediction step
    vqf_real_t gyrNorm = norm(gyr, 3);
    vqf_real_t angle = gyrNorm * gyrTs;
    if (gyrNorm > EPS) {
        vqf_real_t c = cos(angle/2);
        vqf_real_t s = sin(angle/2)/gyrNorm;
        vqf_real_t gyrStepQuat[4] = {c, s*gyr[0], s*gyr[1], s*gyr[2]};
        quatMultiply(state.gyrQuat, gyrStepQuat, state.gyrQuat);
        normalize(state.gyrQuat, 4);
    }
}

void BasicVQF::updateAcc(const vqf_real_t acc[3])
{
    // ignore [0 0 0] samples
    if (acc[0] == vqf_real_t(0.0) && acc[1] == vqf_real_t(0.0) && acc[2] == vqf_real_t(0.0)) {
        return;
    }

    vqf_real_t accEarth[3];

    // filter acc in inertial frame
    quatRotate(state.gyrQuat, acc, accEarth);
    filterVec(accEarth, 3, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.accLpState, state.lastAccLp);

    // transform to 6D earth frame and normalize
    quatRotate(state.accQuat, state.lastAccLp, accEarth);
    normalize(accEarth, 3);

    // inclination correction
    vqf_real_t accCorrQuat[4];
    vqf_real_t q_w = sqrt((accEarth[2]+1)/2);
    if (q_w > 1e-6) {
        accCorrQuat[0] = q_w;
        accCorrQuat[1] = 0.5*accEarth[1]/q_w;
        accCorrQuat[2] = -0.5*accEarth[0]/q_w;
        accCorrQuat[3] = 0;
    } else {
        // to avoid numeric issues when acc is close to [0 0 -1], i.e. the correction step is close (<= 0.00011°) to 180°:
        accCorrQuat[0] = 0;
        accCorrQuat[1] = 1;
        accCorrQuat[2] = 0;
        accCorrQuat[3] = 0;
    }
    quatMultiply(accCorrQuat, state.accQuat, state.accQuat);
    normalize(state.accQuat, 4);
}

void BasicVQF::updateMag(const vqf_real_t mag[3])
{
    // ignore [0 0 0] samples
    if (mag[0] == vqf_real_t(0.0) && mag[1] == vqf_real_t(0.0) && mag[2] == vqf_real_t(0.0)) {
        return;
    }

    vqf_real_t magEarth[3];

    // bring magnetometer measurement into 6D earth frame
    vqf_real_t accGyrQuat[4];
    getQuat6D(accGyrQuat);
    quatRotate(accGyrQuat, mag, magEarth);

    // calculate disagreement angle based on current magnetometer measurement
    vqf_real_t magDisAngle = atan2(magEarth[0], magEarth[1]) - state.delta;

    // make sure the disagreement angle is in the range [-pi, pi]
    if (magDisAngle > vqf_real_t(M_PI)) {
        magDisAngle -= vqf_real_t(2*M_PI);
    } else if (magDisAngle < vqf_real_t(-M_PI)) {
        magDisAngle += vqf_real_t(2*M_PI);
    }

    vqf_real_t k = coeffs.kMag;

    // ensure fast initial convergence
    if (state.kMagInit != vqf_real_t(0.0)) {
        // make sure that the gain k is at least 1/N, N=1,2,3,... in the first few samples
        if (k < state.kMagInit) {
            k = state.kMagInit;
        }

        // iterative expression to calculate 1/N
        state.kMagInit = state.kMagInit/(state.kMagInit+1);

        // disable if t > tauMag
        if (state.kMagInit*params.tauMag < coeffs.magTs) {
            state.kMagInit = 0.0;
        }
    }

    // first-order filter step
    state.delta += k*magDisAngle;

    // make sure delta is in the range [-pi, pi]
    if (state.delta > vqf_real_t(M_PI)) {
        state.delta -= vqf_real_t(2*M_PI);
    } else if (state.delta < vqf_real_t(-M_PI)) {
        state.delta += vqf_real_t(2*M_PI);
    }
}

void BasicVQF::getQuat3D(vqf_real_t out[4]) const
{
    std::copy(state.gyrQuat, state.gyrQuat+4, out);
}

void BasicVQF::getQuat6D(vqf_real_t out[4]) const
{
    quatMultiply(state.accQuat, state.gyrQuat, out);
}

void BasicVQF::getQuat9D(vqf_real_t out[4]) const
{
    quatMultiply(state.accQuat, state.gyrQuat, out);
    quatApplyDelta(out, state.delta, out);
}

vqf_real_t BasicVQF::getDelta() const
{
    return state.delta;
}

void BasicVQF::setTauAcc(vqf_real_t tauAcc)
{
    if (params.tauAcc == tauAcc) {
        return;
    }
    params.tauAcc = tauAcc;
    double newB[3];
    double newA[3];

    filterCoeffs(params.tauAcc, coeffs.accTs, newB, newA);
    filterAdaptStateForCoeffChange(state.lastAccLp, 3, coeffs.accLpB, coeffs.accLpA, newB, newA, state.accLpState);

    std::copy(newB, newB+3, coeffs.accLpB);
    std::copy(newA, newA+2, coeffs.accLpA);
}

void BasicVQF::setTauMag(vqf_real_t tauMag)
{
    params.tauMag = tauMag;
    coeffs.kMag = gainFromTau(params.tauMag, coeffs.magTs);
}

const BasicVQFParams& BasicVQF::getParams() const
{
    return params;
}

const BasicVQFCoefficients& BasicVQF::getCoeffs() const
{
    return coeffs;
}

const BasicVQFState& BasicVQF::getState() const
{
    return state;
}

void BasicVQF::setState(const BasicVQFState& state)
{
    this->state = state;
}

void BasicVQF::resetState()
{
    quatSetToIdentity(state.gyrQuat);
    quatSetToIdentity(state.accQuat);
    state.delta = 0.0;

    std::fill(state.lastAccLp, state.lastAccLp+3, 0);
    std::fill(state.accLpState, state.accLpState + 3*2, NaN);

    state.kMagInit = 1.0;
}

void BasicVQF::quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4])
{
    vqf_real_t w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    vqf_real_t x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    vqf_real_t y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    vqf_real_t z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void BasicVQF::quatConj(const vqf_real_t q[4], vqf_real_t out[4])
{
    vqf_real_t w = q[0];
    vqf_real_t x = -q[1];
    vqf_real_t y = -q[2];
    vqf_real_t z = -q[3];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}


void BasicVQF::quatSetToIdentity(vqf_real_t out[4])
{
    out[0] = 1;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}

void BasicVQF::quatApplyDelta(vqf_real_t q[], vqf_real_t delta, vqf_real_t out[])
{
    // out = quatMultiply([cos(delta/2), 0, 0, sin(delta/2)], q)
    vqf_real_t c = cos(delta/2);
    vqf_real_t s = sin(delta/2);
    vqf_real_t w = c * q[0] - s * q[3];
    vqf_real_t x = c * q[1] - s * q[2];
    vqf_real_t y = c * q[2] + s * q[1];
    vqf_real_t z = c * q[3] + s * q[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void BasicVQF::quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3])
{
    vqf_real_t x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
    vqf_real_t y = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
    vqf_real_t z = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}

vqf_real_t BasicVQF::norm(const vqf_real_t vec[], size_t N)
{
    vqf_real_t s = 0;
    for(size_t i = 0; i < N; i++) {
        s += vec[i]*vec[i];
    }
    return sqrt(s);
}

void BasicVQF::normalize(vqf_real_t vec[], size_t N)
{
    vqf_real_t n = norm(vec, N);
    if (n < EPS) {
        return;
    }
    for(size_t i = 0; i < N; i++) {
        vec[i] /= n;
    }
}

void BasicVQF::clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max)
{
    for(size_t i = 0; i < N; i++) {
        if (vec[i] < min) {
            vec[i] = min;
        } else if (vec[i] > max) {
            vec[i] = max;
        }
    }
}

vqf_real_t BasicVQF::gainFromTau(vqf_real_t tau, vqf_real_t Ts)
{
    assert(Ts > 0);
    if (tau < vqf_real_t(0.0)) {
        return 0; // k=0 for negative tau (disable update)
    } else if (tau == vqf_real_t(0.0)) {
        return 1; // k=1 for tau=0
    } else {
        return 1 - exp(-Ts/tau);  // fc = 1/(2*pi*tau)
    }
}

void BasicVQF::filterCoeffs(vqf_real_t tau, vqf_real_t Ts, double outB[], double outA[])
{
    assert(tau > 0);
    assert(Ts > 0);
    // second order Butterworth filter based on https://stackoverflow.com/a/52764064
    double fc = (M_SQRT2 / (2.0*M_PI))/double(tau); // time constant of dampened, non-oscillating part of step response
    double C = tan(M_PI*fc*double(Ts));
    double D = C*C + sqrt(2)*C + 1;
    double b0 = C*C/D;
    outB[0] = b0;
    outB[1] = 2*b0;
    outB[2] = b0;
    // a0 = 1.0
    outA[0] = 2*(C*C-1)/D; // a1
    outA[1] = (1-sqrt(2)*C+C*C)/D; // a2
}

void BasicVQF::filterInitialState(vqf_real_t x0, const double b[3], const double a[2], double out[])
{
    // initial state for steady state (equivalent to scipy.signal.lfilter_zi, obtained by setting y=x=x0 in the filter
    // update equation)
    out[0] = x0*(1 - b[0]);
    out[1] = x0*(b[2] - a[1]);
}

void BasicVQF::filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const double b_old[],
                                              const double a_old[], const double b_new[],
                                              const double a_new[], double state[])
{
    if (isnan(state[0])) {
        return;
    }
    for (size_t i = 0; i < N; i++) {
        state[0+2*i] = state[0+2*i] + (b_old[0] - b_new[0])*last_y[i];
        state[1+2*i] = state[1+2*i] + (b_old[1] - b_new[1] - a_old[0] + a_new[0])*last_y[i];
    }
}

vqf_real_t BasicVQF::filterStep(vqf_real_t x, const double b[3], const double a[2], double state[2])
{
    // difference equations based on scipy.signal.lfilter documentation
    // assumes that a0 == 1.0
    double y = b[0]*x + state[0];
    state[0] = b[1]*x - a[0]*y + state[1];
    state[1] = b[2]*x - a[1]*y;
    return y;
}

void BasicVQF::filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const double b[3],
                         const double a[2], double state[], vqf_real_t out[])
{
    assert(N>=2);

    // to avoid depending on a single sample, average the first samples (for duration tau)
    // and then use this average to calculate the filter initial state
    if (isnan(state[0])) { // initialization phase
        if (isnan(state[1])) { // first sample
            state[1] = 0; // state[1] is used to store the sample count
            for(size_t i = 0; i < N; i++) {
                state[2+i] = 0; // state[2+i] is used to store the sum
            }
        }
        state[1]++;
        for (size_t i = 0; i < N; i++) {
            state[2+i] += x[i];
            out[i] = state[2+i]/state[1];
        }
        if (state[1]*Ts >= tau) {
            for(size_t i = 0; i < N; i++) {
               filterInitialState(out[i], b, a, state+2*i);
            }
        }
        return;
    }

    for (size_t i = 0; i < N; i++) {
        out[i] = filterStep(x[i], b, a, state+2*i);
    }
}

void BasicVQF::setup()
{
    assert(coeffs.gyrTs > 0);
    assert(coeffs.accTs > 0);
    assert(coeffs.magTs > 0);

    filterCoeffs(params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA);

    coeffs.kMag = gainFromTau(params.tauMag, coeffs.magTs);

    resetState();
}
