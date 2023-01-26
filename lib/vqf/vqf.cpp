// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

// Modified to add timestamps in: updateGyr(const vqf_real_t gyr[3], double gyrTs)
// Removed batch update functions

#include "vqf.h"

#include <algorithm>
#include <limits>
#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>

#define EPS std::numeric_limits<vqf_real_t>::epsilon()
#define NaN std::numeric_limits<vqf_real_t>::quiet_NaN()

inline vqf_real_t square(vqf_real_t x) { return x*x; }


VQFParams::VQFParams()
    : tauAcc(3.0)
    , tauMag(9.0)
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    , motionBiasEstEnabled(true)
#endif
    , restBiasEstEnabled(true)
    , magDistRejectionEnabled(true)
    , biasSigmaInit(0.5)
    , biasForgettingTime(100.0)
    , biasClip(2.0)
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    , biasSigmaMotion(0.1)
    , biasVerticalForgettingFactor(0.0001)
#endif
    , biasSigmaRest(0.03)
    , restMinT(1.5)
    , restFilterTau(0.5)
    , restThGyr(2.0)
    , restThAcc(0.5)
    , magCurrentTau(0.05)
    , magRefTau(20.0)
    , magNormTh(0.1)
    , magDipTh(10.0)
    , magNewTime(20.0)
    , magNewFirstTime(5.0)
    , magNewMinGyr(20.0)
    , magMinUndisturbedTime(0.5)
    , magMaxRejectionTime(60.0)
    , magRejectionFactor(2.0)
{

}

VQF::VQF(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    coeffs.magTs = magTs > 0 ? magTs : gyrTs;

    setup();
}

VQF::VQF(const VQFParams &params, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    this->params = params;

    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    coeffs.magTs = magTs > 0 ? magTs : gyrTs;

    setup();
}

void VQF::updateGyr(const vqf_real_t gyr[3], double gyrTs)
{
    // rest detection
    if (params.restBiasEstEnabled || params.magDistRejectionEnabled) {
        filterVec(gyr, 3, params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA,
                  state.restGyrLpState, state.restLastGyrLp);

        state.restLastSquaredDeviations[0] = square(gyr[0] - state.restLastGyrLp[0])
                + square(gyr[1] - state.restLastGyrLp[1]) + square(gyr[2] - state.restLastGyrLp[2]);

        vqf_real_t biasClip = params.biasClip*vqf_real_t(M_PI/180.0);
        if (state.restLastSquaredDeviations[0] >= square(params.restThGyr*vqf_real_t(M_PI/180.0))
                || fabs(state.restLastGyrLp[0]) > biasClip || fabs(state.restLastGyrLp[1]) > biasClip
                || fabs(state.restLastGyrLp[2]) > biasClip) {
            state.restT = 0.0;
            state.restDetected = false;
        }
    }

    // remove estimated gyro bias
    vqf_real_t gyrNoBias[3] = {gyr[0]-state.bias[0], gyr[1]-state.bias[1], gyr[2]-state.bias[2]};
    // gyroscope prediction step
    vqf_real_t gyrNorm = norm(gyrNoBias, 3);
    vqf_real_t angle = gyrNorm * gyrTs;
    if (gyrNorm > EPS) {
        vqf_real_t c = cos(angle/2);
        vqf_real_t s = sin(angle/2)/gyrNorm;
        vqf_real_t gyrStepQuat[4] = {c, s*gyrNoBias[0], s*gyrNoBias[1], s*gyrNoBias[2]};
        quatMultiply(state.gyrQuat, gyrStepQuat, state.gyrQuat);
        normalize(state.gyrQuat, 4);
    }
}

void VQF::updateAcc(const vqf_real_t acc[3])
{
    // ignore [0 0 0] samples
    if (acc[0] == vqf_real_t(0.0) && acc[1] == vqf_real_t(0.0) && acc[2] == vqf_real_t(0.0)) {
        return;
    }

    // rest detection
    if (params.restBiasEstEnabled) {
        filterVec(acc, 3, params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA,
                  state.restAccLpState, state.restLastAccLp);

        state.restLastSquaredDeviations[1] = square(acc[0] - state.restLastAccLp[0])
                + square(acc[1] - state.restLastAccLp[1]) + square(acc[2] - state.restLastAccLp[2]);

        if (state.restLastSquaredDeviations[1] >= square(params.restThAcc)) {
            state.restT = 0.0;
            state.restDetected = false;
        } else {
            state.restT += coeffs.accTs;
            if (state.restT >= params.restMinT) {
                state.restDetected = true;
            }
        }
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

    // calculate correction angular rate to facilitate debugging
    state.lastAccCorrAngularRate = acos(accEarth[2])/coeffs.accTs;

    // bias estimation
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    if (params.motionBiasEstEnabled || params.restBiasEstEnabled) {
        vqf_real_t biasClip = params.biasClip*vqf_real_t(M_PI/180.0);

        vqf_real_t accGyrQuat[4];
        vqf_real_t R[9];
        vqf_real_t biasLp[2];

        // get rotation matrix corresponding to accGyrQuat
        getQuat6D(accGyrQuat);
        R[0] = 1 - 2*square(accGyrQuat[2]) - 2*square(accGyrQuat[3]); // r11
        R[1] = 2*(accGyrQuat[2]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[3]); // r12
        R[2] = 2*(accGyrQuat[0]*accGyrQuat[2] + accGyrQuat[3]*accGyrQuat[1]); // r13
        R[3] = 2*(accGyrQuat[0]*accGyrQuat[3] + accGyrQuat[2]*accGyrQuat[1]); // r21
        R[4] = 1 - 2*square(accGyrQuat[1]) - 2*square(accGyrQuat[3]); // r22
        R[5] = 2*(accGyrQuat[2]*accGyrQuat[3] - accGyrQuat[1]*accGyrQuat[0]); // r23
        R[6] = 2*(accGyrQuat[3]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[2]); // r31
        R[7] = 2*(accGyrQuat[0]*accGyrQuat[1] + accGyrQuat[3]*accGyrQuat[2]); // r32
        R[8] = 1 - 2*square(accGyrQuat[1]) - 2*square(accGyrQuat[2]); // r33

        // calculate R*b_hat (only the x and y component, as z is not needed)
        biasLp[0] = R[0]*state.bias[0] + R[1]*state.bias[1] + R[2]*state.bias[2];
        biasLp[1] = R[3]*state.bias[0] + R[4]*state.bias[1] + R[5]*state.bias[2];

        // low-pass filter R and R*b_hat
        filterVec(R, 9, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstRLpState, R);
        filterVec(biasLp, 2, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstBiasLpState,
                  biasLp);

        // set measurement error and covariance for the respective Kalman filter update
        vqf_real_t w[3];
        vqf_real_t e[3];
        if (state.restDetected && params.restBiasEstEnabled) {
            e[0] = state.restLastGyrLp[0] - state.bias[0];
            e[1] = state.restLastGyrLp[1] - state.bias[1];
            e[2] = state.restLastGyrLp[2] - state.bias[2];
            matrix3SetToScaledIdentity(1.0, R);
            std::fill(w, w+3, coeffs.biasRestW);
        } else if (params.motionBiasEstEnabled) {
            e[0] = -accEarth[1]/coeffs.accTs + biasLp[0] - R[0]*state.bias[0] - R[1]*state.bias[1] - R[2]*state.bias[2];
            e[1] = accEarth[0]/coeffs.accTs + biasLp[1] - R[3]*state.bias[0] - R[4]*state.bias[1] - R[5]*state.bias[2];
            e[2] = - R[6]*state.bias[0] - R[7]*state.bias[1] - R[8]*state.bias[2];
            w[0] = coeffs.biasMotionW;
            w[1] = coeffs.biasMotionW;
            w[2] = coeffs.biasVerticalW;
        } else {
            std::fill(w, w+3, -1); // disable update
        }

        // Kalman filter update
        // step 1: P = P + V (also increase covariance if there is no measurement update!)
        if (state.biasP[0] < coeffs.biasP0) {
            state.biasP[0] += coeffs.biasV;
        }
        if (state.biasP[4] < coeffs.biasP0) {
            state.biasP[4] += coeffs.biasV;
        }
        if (state.biasP[8] < coeffs.biasP0) {
            state.biasP[8] += coeffs.biasV;
        }
        if (w[0] >= 0) {
            // clip disagreement to -2..2 °/s
            // (this also effectively limits the harm done by the first inclination correction step)
            clip(e, 3, -biasClip, biasClip);

            // step 2: K = P R^T inv(W + R P R^T)
            vqf_real_t K[9];
            matrix3MultiplyTpsSecond(state.biasP, R, K); // K = P R^T
            matrix3Multiply(R, K, K); // K = R P R^T
            K[0] += w[0];
            K[4] += w[1];
            K[8] += w[2]; // K = W + R P R^T
            matrix3Inv(K, K); // K = inv(W + R P R^T)
            matrix3MultiplyTpsFirst(R, K, K); // K = R^T inv(W + R P R^T)
            matrix3Multiply(state.biasP, K, K); // K = P R^T inv(W + R P R^T)

            // step 3: bias = bias + K (y - R bias) = bias + K e
            state.bias[0] += K[0]*e[0] + K[1]*e[1] + K[2]*e[2];
            state.bias[1] += K[3]*e[0] + K[4]*e[1] + K[5]*e[2];
            state.bias[2] += K[6]*e[0] + K[7]*e[1] + K[8]*e[2];

            // step 4: P = P - K R P
            matrix3Multiply(K, R, K); // K = K R
            matrix3Multiply(K, state.biasP, K); // K = K R P
            for(size_t i = 0; i < 9; i++) {
                state.biasP[i] -= K[i];
            }

            // clip bias estimate to -2..2 °/s
            clip(state.bias, 3, -biasClip, biasClip);
        }
    }
#else
    // simplified implementation of bias estimation for the special case in which only rest bias estimation is enabled
    if (params.restBiasEstEnabled) {
        vqf_real_t biasClip = params.biasClip*vqf_real_t(M_PI/180.0);
        if (state.biasP < coeffs.biasP0) {
            state.biasP += coeffs.biasV;
        }
        if (state.restDetected) {
            vqf_real_t e[3];
            e[0] = state.restLastGyrLp[0] - state.bias[0];
            e[1] = state.restLastGyrLp[1] - state.bias[1];
            e[2] = state.restLastGyrLp[2] - state.bias[2];
            clip(e, 3, -biasClip, biasClip);

            // Kalman filter update, simplified scalar version for rest update
            // (this version only uses the first entry of P as P is diagonal and all diagonal elements are the same)
            // step 1: P = P + V (done above!)
            // step 2: K = P R^T inv(W + R P R^T)
            vqf_real_t k = state.biasP/(coeffs.biasRestW + state.biasP);
            // step 3: bias = bias + K (y - R bias) = bias + K e
            state.bias[0] += k*e[0];
            state.bias[1] += k*e[1];
            state.bias[2] += k*e[2];
            // step 4: P = P - K R P
            state.biasP -= k*state.biasP;
            clip(state.bias, 3, -biasClip, biasClip);
        }
    }
#endif
}

void VQF::updateMag(const vqf_real_t mag[3])
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

    if (params.magDistRejectionEnabled) {
        state.magNormDip[0] = norm(magEarth, 3);
        state.magNormDip[1] = -asin(magEarth[2]/state.magNormDip[0]);

        if (params.magCurrentTau > 0) {
            filterVec(state.magNormDip, 2, params.magCurrentTau, coeffs.magTs, coeffs.magNormDipLpB,
                      coeffs.magNormDipLpA, state.magNormDipLpState, state.magNormDip);
        }

        // magnetic disturbance detection
        if (fabs(state.magNormDip[0] - state.magRefNorm) < params.magNormTh*state.magRefNorm
                && fabs(state.magNormDip[1] - state.magRefDip) < params.magDipTh*vqf_real_t(M_PI/180.0)) {
            state.magUndisturbedT += coeffs.magTs;
            if (state.magUndisturbedT >= params.magMinUndisturbedTime) {
                state.magDistDetected = false;
                state.magRefNorm += coeffs.kMagRef*(state.magNormDip[0] - state.magRefNorm);
                state.magRefDip += coeffs.kMagRef*(state.magNormDip[1] - state.magRefDip);
            }
        } else {
            state.magUndisturbedT = 0.0;
            state.magDistDetected = true;
        }

        // new magnetic field acceptance
        if (fabs(state.magNormDip[0] - state.magCandidateNorm) < params.magNormTh*state.magCandidateNorm
                && fabs(state.magNormDip[1] - state.magCandidateDip) < params.magDipTh*vqf_real_t(M_PI/180.0)) {
            if (norm(state.restLastGyrLp, 3) >= params.magNewMinGyr*M_PI/180.0) {
                state.magCandidateT += coeffs.magTs;
            }
            state.magCandidateNorm += coeffs.kMagRef*(state.magNormDip[0] - state.magCandidateNorm);
            state.magCandidateDip += coeffs.kMagRef*(state.magNormDip[1] - state.magCandidateDip);

            if (state.magDistDetected && (state.magCandidateT >= params.magNewTime || (
                    state.magRefNorm == 0.0 && state.magCandidateT >= params.magNewFirstTime))) {
                state.magRefNorm = state.magCandidateNorm;
                state.magRefDip = state.magCandidateDip;
                state.magDistDetected = false;
                state.magUndisturbedT = params.magMinUndisturbedTime;
            }
        } else {
            state.magCandidateT = 0.0;
            state.magCandidateNorm = state.magNormDip[0];
            state.magCandidateDip = state.magNormDip[1];
        }
    }

    // calculate disagreement angle based on current magnetometer measurement
    state.lastMagDisAngle = atan2(magEarth[0], magEarth[1]) - state.delta;

    // make sure the disagreement angle is in the range [-pi, pi]
    if (state.lastMagDisAngle > vqf_real_t(M_PI)) {
        state.lastMagDisAngle -= vqf_real_t(2*M_PI);
    } else if (state.lastMagDisAngle < vqf_real_t(-M_PI)) {
        state.lastMagDisAngle += vqf_real_t(2*M_PI);
    }

    vqf_real_t k = coeffs.kMag;

    if (params.magDistRejectionEnabled) {
        // magnetic disturbance rejection
        if (state.magDistDetected) {
            if (state.magRejectT <= params.magMaxRejectionTime) {
                state.magRejectT += coeffs.magTs;
                k = 0;
            } else {
                k /= params.magRejectionFactor;
            }
        } else {
            state.magRejectT = std::max(state.magRejectT - params.magRejectionFactor*coeffs.magTs, vqf_real_t(0.0));
        }
    }

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
    state.delta += k*state.lastMagDisAngle;
    // calculate correction angular rate to facilitate debugging
    state.lastMagCorrAngularRate = k*state.lastMagDisAngle/coeffs.magTs;

    // make sure delta is in the range [-pi, pi]
    if (state.delta > vqf_real_t(M_PI)) {
        state.delta -= vqf_real_t(2*M_PI);
    } else if (state.delta < vqf_real_t(-M_PI)) {
        state.delta += vqf_real_t(2*M_PI);
    }
}

void VQF::getQuat3D(vqf_real_t out[4]) const
{
    std::copy(state.gyrQuat, state.gyrQuat+4, out);
}

void VQF::getQuat6D(vqf_real_t out[4]) const
{
    quatMultiply(state.accQuat, state.gyrQuat, out);
}

void VQF::getQuat9D(vqf_real_t out[4]) const
{
    quatMultiply(state.accQuat, state.gyrQuat, out);
    quatApplyDelta(out, state.delta, out);
}

vqf_real_t VQF::getDelta() const
{
    return state.delta;
}

vqf_real_t VQF::getBiasEstimate(vqf_real_t out[3]) const
{
    if (out) {
        std::copy(state.bias, state.bias+3, out);
    }
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    // use largest absolute row sum as upper bound estimate for largest eigenvalue (Gershgorin circle theorem)
    // and clip output to biasSigmaInit
    vqf_real_t sum1 = fabs(state.biasP[0]) + fabs(state.biasP[1]) + fabs(state.biasP[2]);
    vqf_real_t sum2 = fabs(state.biasP[3]) + fabs(state.biasP[4]) + fabs(state.biasP[5]);
    vqf_real_t sum3 = fabs(state.biasP[6]) + fabs(state.biasP[7]) + fabs(state.biasP[8]);
    vqf_real_t P = std::min(std::max(std::max(sum1, sum2), sum3), coeffs.biasP0);
#else
    vqf_real_t P = state.biasP;
#endif
    // convert standard deviation from 0.01deg to rad
    return sqrt(P)*vqf_real_t(M_PI/100.0/180.0);
}

void VQF::setBiasEstimate(vqf_real_t bias[3], vqf_real_t sigma)
{
    std::copy(bias, bias+3, state.bias);
    if (sigma > 0) {
        vqf_real_t P = square(sigma*vqf_real_t(180.0*100.0/M_PI));
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        matrix3SetToScaledIdentity(P, state.biasP);
#else
        state.biasP = P;
#endif
    }
}

bool VQF::getRestDetected() const
{
    return state.restDetected;
}

bool VQF::getMagDistDetected() const
{
    return state.magDistDetected;
}

void VQF::getRelativeRestDeviations(vqf_real_t out[2]) const
{
    out[0] = sqrt(state.restLastSquaredDeviations[0]) / (params.restThGyr*vqf_real_t(M_PI/180.0));
    out[1] = sqrt(state.restLastSquaredDeviations[1]) / params.restThAcc;
}

vqf_real_t VQF::getMagRefNorm() const
{
    return state.magRefNorm;
}

vqf_real_t VQF::getMagRefDip() const
{
    return state.magRefDip;
}

void VQF::setMagRef(vqf_real_t norm, vqf_real_t dip)
{
    state.magRefNorm = norm;
    state.magRefDip = dip;
}

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void VQF::setMotionBiasEstEnabled(bool enabled)
{
    if (params.motionBiasEstEnabled == enabled) {
        return;
    }
    params.motionBiasEstEnabled = enabled;
    std::fill(state.motionBiasEstRLpState, state.motionBiasEstRLpState + 9*2, NaN);
    std::fill(state.motionBiasEstBiasLpState, state.motionBiasEstBiasLpState + 2*2, NaN);
}
#endif

void VQF::setRestBiasEstEnabled(bool enabled)
{
    if (params.restBiasEstEnabled == enabled) {
        return;
    }
    params.restBiasEstEnabled = enabled;
    state.restDetected = false;
    std::fill(state.restLastSquaredDeviations, state.restLastSquaredDeviations + 3, 0.0);
    state.restT = 0.0;
    std::fill(state.restLastGyrLp, state.restLastGyrLp + 3, 0.0);
    std::fill(state.restGyrLpState, state.restGyrLpState + 3*2, NaN);
    std::fill(state.restLastAccLp, state.restLastAccLp + 3, 0.0);
    std::fill(state.restAccLpState, state.restAccLpState + 3*2, NaN);
}

void VQF::setMagDistRejectionEnabled(bool enabled)
{
    if (params.magDistRejectionEnabled == enabled) {
        return;
    }
    params.magDistRejectionEnabled = enabled;
    state.magDistDetected = true;
    state.magRefNorm = 0.0;
    state.magRefDip = 0.0;
    state.magUndisturbedT = 0.0;
    state.magRejectT = params.magMaxRejectionTime;
    state.magCandidateNorm = -1.0;
    state.magCandidateDip = 0.0;
    state.magCandidateT = 0.0;
    std::fill(state.magNormDipLpState, state.magNormDipLpState + 2*2, NaN);
}

void VQF::setTauAcc(vqf_real_t tauAcc)
{
    if (params.tauAcc == tauAcc) {
        return;
    }
    params.tauAcc = tauAcc;
    double newB[3];
    double newA[3];

    filterCoeffs(params.tauAcc, coeffs.accTs, newB, newA);
    filterAdaptStateForCoeffChange(state.lastAccLp, 3, coeffs.accLpB, coeffs.accLpA, newB, newA, state.accLpState);

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    // For R and biasLP, the last value is not saved in the state.
    // Since b0 is small (at reasonable settings), the last output is close to state[0].
    vqf_real_t R[9];
    for (size_t i = 0; i < 9; i++) {
        R[i] = state.motionBiasEstRLpState[2*i];
    }
    filterAdaptStateForCoeffChange(R, 9, coeffs.accLpB, coeffs.accLpA, newB, newA, state.motionBiasEstRLpState);
    vqf_real_t biasLp[2];
    for (size_t i = 0; i < 2; i++) {
        biasLp[i] = state.motionBiasEstBiasLpState[2*i];
    }
    filterAdaptStateForCoeffChange(biasLp, 2, coeffs.accLpB, coeffs.accLpA, newB, newA, state.motionBiasEstBiasLpState);
#endif

    std::copy(newB, newB+3, coeffs.accLpB);
    std::copy(newA, newA+2, coeffs.accLpA);
}

void VQF::setTauMag(vqf_real_t tauMag)
{
    params.tauMag = tauMag;
    coeffs.kMag = gainFromTau(params.tauMag, coeffs.magTs);
}

void VQF::setRestDetectionThresholds(vqf_real_t thGyr, vqf_real_t thAcc)
{
    params.restThGyr = thGyr;
    params.restThAcc = thAcc;
}

const VQFParams& VQF::getParams() const
{
    return params;
}

const VQFCoefficients& VQF::getCoeffs() const
{
    return coeffs;
}

const VQFState& VQF::getState() const
{
    return state;
}

void VQF::setState(const VQFState& state)
{
    this->state = state;
}

void VQF::resetState()
{
    quatSetToIdentity(state.gyrQuat);
    quatSetToIdentity(state.accQuat);
    state.delta = 0.0;

    state.restDetected = false;
    state.magDistDetected = true;

    std::fill(state.lastAccLp, state.lastAccLp+3, 0);
    std::fill(state.accLpState, state.accLpState + 3*2, NaN);
    state.lastAccCorrAngularRate = 0.0;

    state.kMagInit = 1.0;
    state.lastMagDisAngle = 0.0;
    state.lastMagCorrAngularRate = 0.0;

    std::fill(state.bias, state.bias+3, 0);
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    matrix3SetToScaledIdentity(coeffs.biasP0, state.biasP);
#else
    state.biasP = coeffs.biasP0;
#endif

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    std::fill(state.motionBiasEstRLpState, state.motionBiasEstRLpState + 9*2, NaN);
    std::fill(state.motionBiasEstBiasLpState, state.motionBiasEstBiasLpState + 2*2, NaN);
#endif

    std::fill(state.restLastSquaredDeviations, state.restLastSquaredDeviations + 3, 0.0);
    state.restT = 0.0;
    std::fill(state.restLastGyrLp, state.restLastGyrLp + 3, 0.0);
    std::fill(state.restGyrLpState, state.restGyrLpState + 3*2, NaN);
    std::fill(state.restLastAccLp, state.restLastAccLp + 3, 0.0);
    std::fill(state.restAccLpState, state.restAccLpState + 3*2, NaN);

    state.magRefNorm = 0.0;
    state.magRefDip = 0.0;
    state.magUndisturbedT = 0.0;
    state.magRejectT = params.magMaxRejectionTime;
    state.magCandidateNorm = -1.0;
    state.magCandidateDip = 0.0;
    state.magCandidateT = 0.0;
    std::fill(state.magNormDip, state.magNormDip + 2, 0);
    std::fill(state.magNormDipLpState, state.magNormDipLpState + 2*2, NaN);
}

void VQF::quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4])
{
    vqf_real_t w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    vqf_real_t x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    vqf_real_t y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    vqf_real_t z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void VQF::quatConj(const vqf_real_t q[4], vqf_real_t out[4])
{
    vqf_real_t w = q[0];
    vqf_real_t x = -q[1];
    vqf_real_t y = -q[2];
    vqf_real_t z = -q[3];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}


void VQF::quatSetToIdentity(vqf_real_t out[4])
{
    out[0] = 1;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}

void VQF::quatApplyDelta(vqf_real_t q[], vqf_real_t delta, vqf_real_t out[])
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

void VQF::quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3])
{
    vqf_real_t x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
    vqf_real_t y = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
    vqf_real_t z = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}

vqf_real_t VQF::norm(const vqf_real_t vec[], size_t N)
{
    vqf_real_t s = 0;
    for(size_t i = 0; i < N; i++) {
        s += vec[i]*vec[i];
    }
    return sqrt(s);
}

void VQF::normalize(vqf_real_t vec[], size_t N)
{
    vqf_real_t n = norm(vec, N);
    if (n < EPS) {
        return;
    }
    for(size_t i = 0; i < N; i++) {
        vec[i] /= n;
    }
}

void VQF::clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max)
{
    for(size_t i = 0; i < N; i++) {
        if (vec[i] < min) {
            vec[i] = min;
        } else if (vec[i] > max) {
            vec[i] = max;
        }
    }
}

vqf_real_t VQF::gainFromTau(vqf_real_t tau, vqf_real_t Ts)
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

void VQF::filterCoeffs(vqf_real_t tau, vqf_real_t Ts, double outB[], double outA[])
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

void VQF::filterInitialState(vqf_real_t x0, const double b[3], const double a[2], double out[])
{
    // initial state for steady state (equivalent to scipy.signal.lfilter_zi, obtained by setting y=x=x0 in the filter
    // update equation)
    out[0] = x0*(1 - b[0]);
    out[1] = x0*(b[2] - a[1]);
}

void VQF::filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const double b_old[],
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

vqf_real_t VQF::filterStep(vqf_real_t x, const double b[3], const double a[2], double state[2])
{
    // difference equations based on scipy.signal.lfilter documentation
    // assumes that a0 == 1.0
    double y = b[0]*x + state[0];
    state[0] = b[1]*x - a[0]*y + state[1];
    state[1] = b[2]*x - a[1]*y;
    return y;
}

void VQF::filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const double b[3],
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

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void VQF::matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t out[9])
{
    out[0] = scale;
    out[1] = 0.0;
    out[2] = 0.0;
    out[3] = 0.0;
    out[4] = scale;
    out[5] = 0.0;
    out[6] = 0.0;
    out[7] = 0.0;
    out[8] = scale;
}

void VQF::matrix3Multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[1]*in2[3] + in1[2]*in2[6];
    tmp[1] = in1[0]*in2[1] + in1[1]*in2[4] + in1[2]*in2[7];
    tmp[2] = in1[0]*in2[2] + in1[1]*in2[5] + in1[2]*in2[8];
    tmp[3] = in1[3]*in2[0] + in1[4]*in2[3] + in1[5]*in2[6];
    tmp[4] = in1[3]*in2[1] + in1[4]*in2[4] + in1[5]*in2[7];
    tmp[5] = in1[3]*in2[2] + in1[4]*in2[5] + in1[5]*in2[8];
    tmp[6] = in1[6]*in2[0] + in1[7]*in2[3] + in1[8]*in2[6];
    tmp[7] = in1[6]*in2[1] + in1[7]*in2[4] + in1[8]*in2[7];
    tmp[8] = in1[6]*in2[2] + in1[7]*in2[5] + in1[8]*in2[8];
    std::copy(tmp, tmp+9, out);
}

void VQF::matrix3MultiplyTpsFirst(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[3]*in2[3] + in1[6]*in2[6];
    tmp[1] = in1[0]*in2[1] + in1[3]*in2[4] + in1[6]*in2[7];
    tmp[2] = in1[0]*in2[2] + in1[3]*in2[5] + in1[6]*in2[8];
    tmp[3] = in1[1]*in2[0] + in1[4]*in2[3] + in1[7]*in2[6];
    tmp[4] = in1[1]*in2[1] + in1[4]*in2[4] + in1[7]*in2[7];
    tmp[5] = in1[1]*in2[2] + in1[4]*in2[5] + in1[7]*in2[8];
    tmp[6] = in1[2]*in2[0] + in1[5]*in2[3] + in1[8]*in2[6];
    tmp[7] = in1[2]*in2[1] + in1[5]*in2[4] + in1[8]*in2[7];
    tmp[8] = in1[2]*in2[2] + in1[5]*in2[5] + in1[8]*in2[8];
    std::copy(tmp, tmp+9, out);
}

void VQF::matrix3MultiplyTpsSecond(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[1]*in2[1] + in1[2]*in2[2];
    tmp[1] = in1[0]*in2[3] + in1[1]*in2[4] + in1[2]*in2[5];
    tmp[2] = in1[0]*in2[6] + in1[1]*in2[7] + in1[2]*in2[8];
    tmp[3] = in1[3]*in2[0] + in1[4]*in2[1] + in1[5]*in2[2];
    tmp[4] = in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5];
    tmp[5] = in1[3]*in2[6] + in1[4]*in2[7] + in1[5]*in2[8];
    tmp[6] = in1[6]*in2[0] + in1[7]*in2[1] + in1[8]*in2[2];
    tmp[7] = in1[6]*in2[3] + in1[7]*in2[4] + in1[8]*in2[5];
    tmp[8] = in1[6]*in2[6] + in1[7]*in2[7] + in1[8]*in2[8];
    std::copy(tmp, tmp+9, out);
}

bool VQF::matrix3Inv(const vqf_real_t in[9], vqf_real_t out[9])
{
    // in = [a b c; d e f; g h i]
    double A = in[4]*in[8] - in[5]*in[7]; // (e*i - f*h)
    double D = in[2]*in[7] - in[1]*in[8]; // -(b*i - c*h)
    double G = in[1]*in[5] - in[2]*in[4]; // (b*f - c*e)
    double B = in[5]*in[6] - in[3]*in[8]; // -(d*i - f*g)
    double E = in[0]*in[8] - in[2]*in[6]; // (a*i - c*g)
    double H = in[2]*in[3] - in[0]*in[5]; // -(a*f - c*d)
    double C = in[3]*in[7] - in[4]*in[6]; // (d*h - e*g)
    double F = in[1]*in[6] - in[0]*in[7]; // -(a*h - b*g)
    double I = in[0]*in[4] - in[1]*in[3]; // (a*e - b*d)

    double det = in[0]*A + in[1]*B + in[2]*C; // a*A + b*B + c*C;

    if (det >= -EPS && det <= EPS) {
        std::fill(out, out+9, 0);
        return false;
    }

    // out = [A D G; B E H; C F I]/det
    out[0] = A/det;
    out[1] = D/det;
    out[2] = G/det;
    out[3] = B/det;
    out[4] = E/det;
    out[5] = H/det;
    out[6] = C/det;
    out[7] = F/det;
    out[8] = I/det;

    return true;
}
#endif

void VQF::setup()
{
    assert(coeffs.gyrTs > 0);
    assert(coeffs.accTs > 0);
    assert(coeffs.magTs > 0);

    filterCoeffs(params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA);

    coeffs.kMag = gainFromTau(params.tauMag, coeffs.magTs);

    coeffs.biasP0 = square(params.biasSigmaInit*100.0);
    // the system noise increases the variance from 0 to (0.1 °/s)^2 in biasForgettingTime seconds
    coeffs.biasV = square(0.1*100.0)*coeffs.accTs/params.biasForgettingTime;

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t pMotion = square(params.biasSigmaMotion*100.0);
    coeffs.biasMotionW = square(pMotion) / coeffs.biasV + pMotion;
    coeffs.biasVerticalW = coeffs.biasMotionW / std::max(params.biasVerticalForgettingFactor, vqf_real_t(1e-10));
#endif

    vqf_real_t pRest = square(params.biasSigmaRest*100.0);
    coeffs.biasRestW = square(pRest) / coeffs.biasV + pRest;

    filterCoeffs(params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA);
    filterCoeffs(params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA);

    coeffs.kMagRef = gainFromTau(params.magRefTau, coeffs.magTs);
    if (params.magCurrentTau > 0) {
        filterCoeffs(params.magCurrentTau, coeffs.magTs, coeffs.magNormDipLpB, coeffs.magNormDipLpA);
    } else {
        std::fill(coeffs.magNormDipLpB, coeffs.magNormDipLpB + 3, NaN);
        std::fill(coeffs.magNormDipLpA, coeffs.magNormDipLpA + 2, NaN);
    }

    resetState();
}
