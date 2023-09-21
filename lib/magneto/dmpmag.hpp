#include "dmpmag.h"

//Get rotation quaternion from gravity vector and geomagnetic vector by Direction Cosine Matrix
//https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran

template<typename T>
Quat DMPMag<T>::getQuatDCM(const T acc[3], const T mag[3])
{
    Vector3 Mv(mag[0], mag[1], mag[2]);
    Vector3 Dv(acc[0], acc[1], acc[2]);
    Dv.normalize();
    Vector3 Rv = Dv.cross(Mv);
    Rv.normalize();
    Vector3 Fv = Rv.cross(Dv);
    Fv.normalize();
    float q04 = 2*sqrt(1+Fv.x+Rv.y+Dv.z);
    return Quat(Rv.z-Dv.y, Dv.x-Fv.z, Fv.y-Rv.x, q04*q04/4).normalized();    
}

template<typename T>
Quat DMPMag<T>::getCorrection(const T acc[3], const T mag[3], Quat quat)
{
    Quat magQ = getQuatDCM(acc,mag);
    //dmp.w=DCM.z
    //dmp.x=DCM.y
    //dmp.y=-DCM.x
    //dmp.z=DCM.w
    Quat trans(magQ.x, magQ.y, magQ.w, magQ.z);
    Quat result = trans*quat.inverse();
    return result;
}

template<typename T>
void DMPMag<T>::update(T oqwxyz[4], const T iqwxyz[4], const T Grav[3], const T Mxyz[3])
{
    // Map DMP axis to sensor axis
    Quat quat(-iqwxyz[2], iqwxyz[1], iqwxyz[3], iqwxyz[0]);
    if (correction.length_squared() == 0.0f) {
        correction = getCorrection(Grav, Mxyz, quat);
    } else {
        Quat newCorr = getCorrection(Grav, Mxyz, quat);

        if(!__isnanf(newCorr.w)) {
            correction = correction.slerp(newCorr, magCorrRatio);
        }
    }
    Quat fusedquat = correction * quat;
    oqwxyz[0] = fusedquat.w; 
    oqwxyz[1] = fusedquat.x;
    oqwxyz[2] = fusedquat.y;
    oqwxyz[3] = fusedquat.z;
}
