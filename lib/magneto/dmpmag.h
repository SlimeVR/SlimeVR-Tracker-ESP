#include "quat.h"
#include "vector3.h"

//Get rotation quaternion from gravity vector and geomagnetic vector by Direction Cosine Matrix
//https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran
Quat getQuatDCM(float* acc, float* mag){
    Vector3 Mv(mag[0], mag[1], mag[2]);
    Vector3 Dv(acc[0], acc[1], acc[2]);
    Dv.normalize();
    Vector3 Rv = Dv.cross(Mv);
    Rv.normalize();
    Vector3 Fv = Rv.cross(Dv);
    Fv.normalize();
    float q04 = 2*sqrt(1+Fv.x+Rv.y+Dv.z);
    return Quat(Rv.z-Dv.y,Dv.x-Fv.z,Fv.y-Rv.x,q04*q04/4).normalized();    
}
Quat getCorrection(float* acc,float* mag,Quat quat)
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