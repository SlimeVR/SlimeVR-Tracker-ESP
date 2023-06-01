#include "SensorFusion.h"

namespace SlimeVR
{
    namespace Sensors
    {
        
        void SensorFusion::update6D(sensor_real_t Axyz[3], sensor_real_t Gxyz[3], sensor_real_t deltat)
        {
            updateAcc(Axyz, deltat);
            updateGyro(Gxyz, deltat);
        }

        void SensorFusion::update9D(sensor_real_t Axyz[3], sensor_real_t Gxyz[3], sensor_real_t Mxyz[3], sensor_real_t deltat)
        {
            updateMag(Mxyz, deltat);
            updateAcc(Axyz, deltat);
            updateGyro(Gxyz, deltat);
        }

        void SensorFusion::updateAcc(sensor_real_t Axyz[3], sensor_real_t deltat)
        {
            if (deltat < 0) deltat = accTs;

            #if SENSOR_USE_MAHONY || SENSOR_USE_MADGWICK
                std::copy(Axyz, Axyz+3, bAxyz);
                accelUpdated = true;
            #elif SENSOR_USE_BASICVQF
                basicvqf.updateAcc(Axyz);
            #elif SENSOR_USE_VQF
                vqf.updateAcc(Axyz);
            #endif
        }

        void SensorFusion::updateMag(sensor_real_t Mxyz[3], sensor_real_t deltat)
        {
            if (deltat < 0) deltat = magTs;

            if (!magExist) {
                if (Mxyz[0] != 0.0f || Mxyz[1] != 0.0f || Mxyz[2] != 0.0f) {
                    magExist = true;
                } else {
                    return;
                }
            }

            #if SENSOR_USE_MAHONY || SENSOR_USE_MADGWICK
                std::copy(Mxyz, Mxyz+3, bMxyz);
            #elif SENSOR_USE_BASICVQF
                basicvqf.updateMag(Mxyz);
            #elif SENSOR_USE_VQF
                vqf.updateMag(Mxyz);
            #endif
        }

        void SensorFusion::updateGyro(sensor_real_t Gxyz[3], sensor_real_t deltat)
        {
            if (deltat < 0) deltat = gyrTs;

            #if SENSOR_USE_MAHONY
                if (accelUpdated) {
                    if (magExist) {
                        mahony.update(qwxyz, bAxyz[0], bAxyz[1], bAxyz[2],
                                            Gxyz[0],  Gxyz[1],  Gxyz[2],
                                    deltat);
                    } else {
                        mahony.update(qwxyz, bAxyz[0], bAxyz[1], bAxyz[2],
                                            Gxyz[0],  Gxyz[1],  Gxyz[2],
                                            bMxyz[0], bMxyz[1], bMxyz[2],
                                    deltat);
                    }
                }
                accelUpdated = false;
            #elif SENSOR_USE_MADGWICK
                if (accelUpdated) {
                    if (magExist) {
                        madgwick.update(qwxyz, bAxyz[0], bAxyz[1], bAxyz[2],
                                                Gxyz[0],  Gxyz[1],  Gxyz[2],
                                        deltat);
                    } else {
                        madgwick.update(qwxyz, bAxyz[0], bAxyz[1], bAxyz[2],
                                                Gxyz[0],  Gxyz[1],  Gxyz[2],
                                            bMxyz[0], bMxyz[1], bMxyz[2],
                                        deltat);
                    }
                }
                accelUpdated = false;
            #elif SENSOR_USE_BASICVQF
                basicvqf.updateGyr(Gxyz, deltat);
            #elif SENSOR_USE_VQF
                vqf.updateGyr(Gxyz, deltat);
            #endif

            updated = true;
        }

        bool SensorFusion::isUpdated()
        {
            return updated;
        }

        void SensorFusion::clearUpdated()
        {
            updated = false;
        }
        
        sensor_real_t const * SensorFusion::getQuaternion()
        {
            #if SENSOR_USE_BASICVQF
                if(magExist) {
                    basicvqf.getQuat9D(qwxyz);
                } else {
                    basicvqf.getQuat6D(qwxyz);
                }
            #elif SENSOR_USE_VQF
                if(magExist) {
                    vqf.getQuat9D(qwxyz);
                } else {
                    vqf.getQuat6D(qwxyz);
                }
            #endif

            return qwxyz;
        }

    }
}