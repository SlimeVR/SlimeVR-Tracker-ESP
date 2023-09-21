#ifndef _MADGWICK_H_
#define _MADGWICK_H_

#include "helper_3dmath.h"

template<typename T>
class Madgwick {

    static constexpr float madgwickBeta = 0.1f;

public:
    void update(T q[4], T ax, T ay, T az, T gx, T gy, T gz, T mx, T my, T mz, T deltat);
    void update(T q[4], T ax, T ay, T az, T gx, T gy, T gz, T deltat);
};

#include "madgwick.hpp"

#endif /* _MADGWICK_H_ */
