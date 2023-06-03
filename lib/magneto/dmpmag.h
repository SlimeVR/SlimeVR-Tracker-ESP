#ifndef _DMPMAG_H_
#define _DMPMAG_H_

#include "quat.h"

template<typename T>
class DMPMag {
    static constexpr T magCorrRatio = 0.02;
public:
    void update(T oqwxyz[4], const T iqwxyz[4], const T Grav[3], const T Mxyz[3]);
private:
    Quat getQuatDCM(const T acc[3], const T mag[3]);
    Quat getCorrection(const T acc[3], const T mag[3], Quat quat);

    Quat correction{0, 0, 0, 0};
};

#include "dmpmag.hpp"

#endif /* _DMPMAG_H_ */
