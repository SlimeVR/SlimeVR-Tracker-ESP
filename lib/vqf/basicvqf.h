// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

// Modified to add timestamps in: updateGyr(const vqf_real_t gyr[3], double gyrTs)
// Removed batch update functions

#ifndef BASICVQF_HPP
#define BASICVQF_HPP

#include <stddef.h>

#define VQF_SINGLE_PRECISION
#define M_PI 3.14159265358979323846
#define M_SQRT2 1.41421356237309504880

/**
 * @brief Typedef for the floating-point data type used for most operations.
 *
 * By default, all floating-point calculations are performed using `double`. Set the `VQF_SINGLE_PRECISION` define to
 * change this type to `float`. Note that the Butterworth filter implementation will always use double precision as
 * using floats can cause numeric issues.
 */
#ifndef VQF_SINGLE_PRECISION
typedef double vqf_real_t;
#else
typedef float vqf_real_t;
#endif

/**
 * @brief Struct containing all tuning parameters used by the BasicVQF class.
 *
 * The parameters influence the behavior of the algorithm and are independent of the sampling rate of the IMU data. The
 * constructor sets all parameters to the default values.
 *
 * The basic version of this algoirthm only has two parameters: The time constants #tauAcc and #tauMag can be tuned to
 * change the trust on the accelerometer and magnetometer measurements, respectively.
 */
struct BasicVQFParams
{
    /**
     * @brief Constructor that initializes the struct with the default parameters.
     */
    BasicVQFParams();

    /**
     * @brief Time constant \f$\tau_\mathrm{acc}\f$ for accelerometer low-pass filtering in seconds.
     *
     * Small values for \f$\tau_\mathrm{acc}\f$ imply trust on the accelerometer measurements and while large values of
     * \f$\tau_\mathrm{acc}\f$ imply trust on the gyroscope measurements.
     *
     * The time constant \f$\tau_\mathrm{acc}\f$ corresponds to the cutoff frequency \f$f_\mathrm{c}\f$ of the
     * second-order Butterworth low-pass filter as follows: \f$f_\mathrm{c} = \frac{\sqrt{2}}{2\pi\tau_\mathrm{acc}}\f$.
     *
     * Default value: 3.0 s
     */
    vqf_real_t tauAcc;
    /**
     * @brief Time constant \f$\tau_\mathrm{mag}\f$ for magnetometer update in seconds.
     *
     * Small values for \f$\tau_\mathrm{mag}\f$ imply trust on the magnetometer measurements and while large values of
     * \f$\tau_\mathrm{mag}\f$ imply trust on the gyroscope measurements.
     *
     * The time constant \f$\tau_\mathrm{mag}\f$ corresponds to the cutoff frequency \f$f_\mathrm{c}\f$ of the
     * first-order low-pass filter for the heading correction as follows:
     * \f$f_\mathrm{c} = \frac{1}{2\pi\tau_\mathrm{mag}}\f$.
     *
     * Default value: 9.0 s
     */
    vqf_real_t tauMag;
};

/**
 * @brief Struct containing the filter state of the BasicVQF class.
 *
 * The relevant parts of the state can be accessed via functions of the BasicVVQF class, e.g. BasicVQF::getQuat6D()
 * and BasicVQF::getQuat9D(). To reset the state to the initial values, use VQF::resetState().
 *
 * Direct access to the full state is typically not needed but can be useful in some cases, e.g. for debugging. For this
 * purpose, the state can be accessed by BasicVQF::getState() and set by BasicVQF::setState().
 */
struct BasicVQFState {
    /**
     * @brief Angular velocity strapdown integration quaternion \f$^{\mathcal{S}_i}_{\mathcal{I}_i}\mathbf{q}\f$.
     */
    vqf_real_t gyrQuat[4];
    /**
     * @brief Inclination correction quaternion \f$^{\mathcal{I}_i}_{\mathcal{E}_i}\mathbf{q}\f$.
     */
    vqf_real_t accQuat[4];
    /**
     * @brief Heading difference \f$\delta\f$ between \f$\mathcal{E}_i\f$ and \f$\mathcal{E}\f$.
     *
     * \f$^{\mathcal{E}_i}_{\mathcal{E}}\mathbf{q} = \begin{bmatrix}\cos\frac{\delta}{2} & 0 & 0 &
     * \sin\frac{\delta}{2}\end{bmatrix}^T\f$.
     */
    vqf_real_t delta;

    /**
     * @brief Last low-pass filtered acceleration in the \f$\mathcal{I}_i\f$ frame.
     */
    vqf_real_t lastAccLp[3];
    /**
     * @brief Internal low-pass filter state for #lastAccLp.
     */
    double accLpState[3*2];

    /**
     * @brief Gain used for heading correction to ensure fast initial convergence.
     *
     * This value is used as the gain for heading correction in the beginning if it is larger than the normal filter
     * gain. It is initialized to 1 and then updated to 0.5, 0.33, 0.25, ... After VQFParams::tauMag seconds, it is
     * set to zero.
     */
    vqf_real_t kMagInit;
};

/**
 * @brief Struct containing coefficients used by the BasicVQF class.
 *
 * Coefficients are values that depend on the parameters and the sampling times, but do not change during update steps.
 * They are calculated in BasicVQF::setup().
 */
struct BasicVQFCoefficients
{
    /**
     * @brief Sampling time of the gyroscope measurements (in seconds).
     */
    vqf_real_t gyrTs;
    /**
     * @brief Sampling time of the accelerometer measurements (in seconds).
     */
    vqf_real_t accTs;
    /**
     * @brief Sampling time of the magnetometer measurements (in seconds).
     */
    vqf_real_t magTs;

    /**
     * @brief Numerator coefficients of the acceleration low-pass filter.
     *
     * The array contains \f$\begin{bmatrix}b_0 & b_1 & b_2\end{bmatrix}\f$.
     */
    double accLpB[3];
    /**
     * @brief Denominator coefficients of the acceleration low-pass filter.
     *
     * The array contains \f$\begin{bmatrix}a_1 & a_2\end{bmatrix}\f$ and \f$a_0=1\f$.
     */
    double accLpA[2];

    /**
     * @brief Gain of the first-order filter used for heading correction.
     */
    vqf_real_t kMag;
};

/**
 * @brief A Versatile Quaternion-based Filter for IMU Orientation Estimation.
 *
 * \rst
 * This class implements the basic version of the orientation estimation filter described in the following publication:
 *
 *
 *     D. Laidig and T. Seel. "VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic
 *     Disturbance Rejection." Information Fusion 2023, 91, 187--204.
 *     `doi:10.1016/j.inffus.2022.10.014 <https://doi.org/10.1016/j.inffus.2022.10.014>`_.
 *     [Accepted manuscript available at `arXiv:2203.17024 <https://arxiv.org/abs/2203.17024>`_.]
 *
 * The filter can perform simultaneous 6D (magnetometer-free) and 9D (gyr+acc+mag) sensor fusion and can also be used
 * without magnetometer data. Different sampling rates for gyroscopes, accelerometers and magnetometers are
 * supported as well. While in most cases, the defaults will be reasonable, the algorithm can be influenced via two
 * tuning parameters.
 *
 * To use this C++ implementation,
 *
 * 1. create a instance of the class and provide the sampling time and, optionally, parameters
 * 2. for every sample, call one of the update functions to feed the algorithm with IMU data
 * 3. access the estimation results with :meth:`getQuat6D() <VQF.getQuat6D>`, :meth:`getQuat9D() <VQF.getQuat9D>` and
 *    the other getter methods.
 *
 * If the full data is available in (row-major) data buffers, you can use :meth:`updateBatch() <VQF.updateBatch>`.
 *
 * This class is the C++ implementation of the basic algorithm version. This version does not include rest detection,
 * gyroscope bias estimation, and magnetic disturbance detection and rejection. It is equivalent to the full version
 * when the parameters :cpp:member:`VQFParams::motionBiasEstEnabled`, :cpp:member:`VQFParams::restBiasEstEnabled` and
 * :cpp:member:`VQFParams::magDistRejectionEnabled` are set to false.
 * Depending on use case and programming language of choice, the following alternatives might be useful:
 *
 * +------------------------+--------------------------+---------------------------+---------------------------+
 * |                        | Full Version             | Basic Version             | Offline Version           |
 * |                        |                          |                           |                           |
 * +========================+==========================+===========================+===========================+
 * | **C++**                | :cpp:class:`VQF`         | **BasicVQF (this class)** | :cpp:func:`offlineVQF`    |
 * +------------------------+--------------------------+---------------------------+---------------------------+
 * | **Python/C++ (fast)**  | :py:class:`vqf.VQF`      | :py:class:`vqf.BasicVQF`  | :py:meth:`vqf.offlineVQF` |
 * +------------------------+--------------------------+---------------------------+---------------------------+
 * | **Pure Python (slow)** | :py:class:`vqf.PyVQF`    | --                        | --                        |
 * +------------------------+--------------------------+---------------------------+---------------------------+
 * | **Pure Matlab (slow)** | :mat:class:`VQF.m <VQF>` | --                        | --                        |
 * +------------------------+--------------------------+---------------------------+---------------------------+
 * \endrst
 */

class BasicVQF
{
public:
    /**
     * Initializes the object with default parameters.
     *
     * In the most common case (using the default parameters and all data being sampled with the same frequency,
     * create the class like this:
     * \rst
     * .. code-block:: c++
     *
     *     BasicVQF vqf(0.01); // 0.01 s sampling time, i.e. 100 Hz
     * \endrst
     *
     * @param gyrTs sampling time of the gyroscope measurements in seconds
     * @param accTs sampling time of the accelerometer measurements in seconds (the value of `gyrTs` is used if set to -1)
     * @param magTs sampling time of the magnetometer measurements in seconds (the value of `gyrTs` is used if set to -1)
     *
     */
    BasicVQF(vqf_real_t gyrTs, vqf_real_t accTs=-1.0, vqf_real_t magTs=-1.0);
    /**
     * @brief Initializes the object with custom parameters.
     *
     * Example code to create an object with a different value for tauAcc:
     * \rst
     * .. code-block:: c++
     *
     *     BasicVQFParams params;
     *     params.tauAcc = 1.0;
     *     BasicVQF vqf(0.01); // 0.01 s sampling time, i.e. 100 Hz
     * \endrst
     *
     * @param params BasicVQFParams struct containing the desired parameters
     * @param gyrTs sampling time of the gyroscope measurements in seconds
     * @param accTs sampling time of the accelerometer measurements in seconds (the value of `gyrTs` is used if set to -1)
     * @param magTs sampling time of the magnetometer measurements in seconds (the value of `gyrTs` is used if set to -1)
     */
    BasicVQF(const BasicVQFParams& params, vqf_real_t gyrTs, vqf_real_t accTs=-1.0, vqf_real_t magTs=-1.0);

    /**
     * @brief Performs gyroscope update step.
     *
     * It is only necessary to call this function directly if gyroscope, accelerometers and magnetometers have
     * different sampling rates. Otherwise, simply use #update().
     *
     * @param gyr gyroscope measurement in rad/s
     */
    void updateGyr(const vqf_real_t gyr[3], double gyrTs);
    /**
     * @brief Performs accelerometer update step.
     *
     * It is only necessary to call this function directly if gyroscope, accelerometers and magnetometers have
     * different sampling rates. Otherwise, simply use #update().
     *
     * Should be called after #updateGyr and before #updateMag.
     *
     * @param acc accelerometer measurement in m/s²
     */
    void updateAcc(const vqf_real_t acc[3]);
    /**
     * @brief Performs magnetometer update step.
     *
     * It is only necessary to call this function directly if gyroscope, accelerometers and magnetometers have
     * different sampling rates. Otherwise, simply use #update().
     *
     * Should be called after #updateAcc.
     *
     * @param mag magnetometer measurement in arbitrary units
     */
    void updateMag(const vqf_real_t mag[3]);

    /**
     * @brief Returns the angular velocity strapdown integration quaternion
     * \f$^{\mathcal{S}_i}_{\mathcal{I}_i}\mathbf{q}\f$.
     * @param out output array for the quaternion
     */
    void getQuat3D(vqf_real_t out[4]) const;
    /**
     * @brief Returns the 6D (magnetometer-free) orientation quaternion
     * \f$^{\mathcal{S}_i}_{\mathcal{E}_i}\mathbf{q}\f$.
     * @param out output array for the quaternion
     */
    void getQuat6D(vqf_real_t out[4]) const;
    /**
     * @brief Returns the 9D (with magnetometers) orientation quaternion
     * \f$^{\mathcal{S}_i}_{\mathcal{E}}\mathbf{q}\f$.
     * @param out output array for the quaternion
     */
    void getQuat9D(vqf_real_t out[4]) const;
    /**
     * @brief Returns the heading difference \f$\delta\f$ between \f$\mathcal{E}_i\f$ and \f$\mathcal{E}\f$.
     *
     * \f$^{\mathcal{E}_i}_{\mathcal{E}}\mathbf{q} = \begin{bmatrix}\cos\frac{\delta}{2} & 0 & 0 &
     * \sin\frac{\delta}{2}\end{bmatrix}^T\f$.
     *
     * @return delta angle in rad (VQFState::delta)
     */
    vqf_real_t getDelta() const;

    /**
     * @brief Sets the time constant for accelerometer low-pass filtering.
     *
     * For more details, see VQFParams.tauAcc.
     *
     * @param tauAcc time constant \f$\tau_\mathrm{acc}\f$ in seconds
     */
    void setTauAcc(vqf_real_t tauAcc);
    /**
     * @brief Sets the time constant for the magnetometer update.
     *
     * For more details, see VQFParams.tauMag.
     *
     * @param tauMag time constant \f$\tau_\mathrm{mag}\f$ in seconds
     */
    void setTauMag(vqf_real_t tauMag);

    /**
     * @brief Returns the current parameters.
     */
    const BasicVQFParams& getParams() const;
    /**
     * @brief Returns the coefficients used by the algorithm.
     */
    const BasicVQFCoefficients& getCoeffs() const;
    /**
     * @brief Returns the current state.
     */
    const BasicVQFState& getState() const;
    /**
     * @brief Overwrites the current state.
     *
     * This method allows to set a completely arbitrary filter state and is intended for debugging purposes. In
     * combination with #getState, individual elements of the state can be modified.
     *
     * @param state A BasicVQFState struct containing the new state
     */
    void setState(const BasicVQFState& state);
    /**
     * @brief Resets the state to the default values at initialization.
     *
     * Resetting the state is equivalent to creating a new instance of this class.
     */
    void resetState();

    /**
     * @brief Performs quaternion multiplication (\f$\mathbf{q}_\mathrm{out} = \mathbf{q}_1 \otimes \mathbf{q}_2\f$).
     */
    static void quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4]);
    /**
     * @brief Calculates the quaternion conjugate (\f$\mathbf{q}_\mathrm{out} = \mathbf{q}^*\f$).
     */
    static void quatConj(const vqf_real_t q[4], vqf_real_t out[4]);
    /**
     * @brief Sets the output quaternion to the identity quaternion (\f$\mathbf{q}_\mathrm{out} =
     * \begin{bmatrix}1 & 0 & 0 & 0\end{bmatrix}\f$).
     */
    static void quatSetToIdentity(vqf_real_t out[4]);
    /**
     * @brief Applies a heading rotation by the angle delta (in rad) to a quaternion.
     *
     * \f$\mathbf{q}_\mathrm{out} = \begin{bmatrix}\cos\frac{\delta}{2} & 0 & 0 &
     * \sin\frac{\delta}{2}\end{bmatrix} \otimes \mathbf{q}\f$
     */
    static void quatApplyDelta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4]);
    /**
     * @brief Rotates a vector with a given quaternion.
     *
     * \f$\begin{bmatrix}0 & \mathbf{v}_\mathrm{out}\end{bmatrix} =
     * \mathbf{q} \otimes \begin{bmatrix}0 & \mathbf{v}\end{bmatrix} \otimes \mathbf{q}^*\f$
     */
    static void quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3]);
    /**
     * @brief Calculates the Euclidean norm of a vector.
     * @param vec pointer to an array of N elements
     * @param N number of elements
     */
    static vqf_real_t norm(const vqf_real_t vec[], size_t N);
    /**
     * @brief Normalizes a vector in-place.
     * @param vec pointer to an array of N elements that will be normalized
     * @param N number of elements
     */
    static void normalize(vqf_real_t vec[], size_t N);
    /**
     * @brief Clips a vector in-place.
     * @param vec pointer to an array of N elements that will be clipped
     * @param N number of elements
     * @param min smallest allowed value
     * @param max largest allowed value
     */
    static void clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max);
    /**
     * @brief Calculates the gain for a first-order low-pass filter from the 1/e time constant.
     *
     * \f$k = 1 - \exp\left(-\frac{T_\mathrm{s}}{\tau}\right)\f$
     *
     * The cutoff frequency of the resulting filter is \f$f_\mathrm{c} = \frac{1}{2\pi\tau}\f$.
     *
     * @param tau time constant \f$\tau\f$ in seconds - use -1 to disable update (\f$k=0\f$) or 0 to obtain
     *        unfiltered values (\f$k=1\f$)
     * @param Ts sampling time \f$T_\mathrm{s}\f$ in seconds
     * @return filter gain *k*
     */
    static vqf_real_t gainFromTau(vqf_real_t tau, vqf_real_t Ts);
    /**
     * @brief Calculates coefficients for a second-order Butterworth low-pass filter.
     *
     * The filter is parametrized via the time constant of the dampened, non-oscillating part of step response and the
     * resulting cutoff frequency is \f$f_\mathrm{c} = \frac{\sqrt{2}}{2\pi\tau}\f$.
     *
     * @param tau time constant \f$\tau\f$ in seconds
     * @param Ts sampling time \f$T_\mathrm{s}\f$ in seconds
     * @param outB output array for numerator coefficients
     * @param outA output array for denominator coefficients (without \f$a_0=1\f$)
     */
    static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, double outB[3], double outA[2]);
    /**
     * @brief Calculates the initial filter state for a given steady-state value.
     * @param x0 steady state value
     * @param b numerator coefficients
     * @param a denominator coefficients (without \f$a_0=1\f$)
     * @param out output array for filter state
     */
    static void filterInitialState(vqf_real_t x0, const double b[], const double a[], double out[2]);
    /**
     * @brief Adjusts the filter state when changing coefficients.
     *
     * This function assumes that the filter is currently in a steady state, i.e. the last input values and the last
     * output values are all equal. Based on this, the filter state is adjusted to new filter coefficients so that the
     * output does not jump.
     *
     * @param last_y last filter output values (array of size N)
     * @param N number of values in vector-valued signal
     * @param b_old previous numerator coefficients
     * @param a_old previous denominator coefficients (without \f$a_0=1\f$)
     * @param b_new new numerator coefficients
     * @param a_new new denominator coefficients (without \f$a_0=1\f$)
     * @param state filter state (array of size N*2, will be modified)
     */
    static void filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const double b_old[3],
                                               const double a_old[2], const double b_new[3],
                                               const double a_new[2], double state[]);
    /**
     * @brief Performs a filter step for a scalar value.
     * @param x input value
     * @param b numerator coefficients
     * @param a denominator coefficients (without \f$a_0=1\f$)
     * @param state filter state array (will be modified)
     * @return filtered value
     */
    static vqf_real_t filterStep(vqf_real_t x, const double b[3], const double a[2], double state[2]);
    /**
     * @brief Performs filter step for vector-valued signal with averaging-based initialization.
     *
     * During the first \f$\tau\f$ seconds, the filter output is the mean of the previous samples. At \f$t=\tau\f$, the
     * initial conditions for the low-pass filter are calculated based on the current mean value and from then on,
     * regular filtering with the rational transfer function described by the coefficients b and a is performed.
     *
     * @param x input values (array of size N)
     * @param N number of values in vector-valued signal
     * @param tau filter time constant \f$\tau\f$ in seconds (used for initialization)
     * @param Ts sampling time \f$T_\mathrm{s}\f$ in seconds (used for initialization)
     * @param b numerator coefficients
     * @param a denominator coefficients (without \f$a_0=1\f$)
     * @param state filter state (array of size N*2, will be modified)
     * @param out output array for filtered values (size N)
     */
    static void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const double b[3],
                          const double a[2], double state[], vqf_real_t out[]);

protected:
    /**
     * @brief Calculates coefficients based on parameters and sampling rates.
     */
    void setup();

    /**
     * @brief Contains the current parameters.
     *
     * See #getParams. To set parameters, pass them to the constructor. The parameters can be changed with
     * #setTauAcc and #setTauMag.
     */
    BasicVQFParams params;
    /**
     * @brief Contains the current state.
     *
     * See #getState, #getState and #resetState.
     */
    BasicVQFState state;
    /**
     * @brief Contains the current coefficients (calculated in #setup).
     *
     * See #getCoeffs.
     */
    BasicVQFCoefficients coeffs;
};

#endif // BASICVQF_HPP
