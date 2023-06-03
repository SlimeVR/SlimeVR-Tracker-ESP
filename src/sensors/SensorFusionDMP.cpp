#include "SensorFusionDMP.h"

namespace SlimeVR
{
    namespace Sensors
    {
        void SensorFusionDMP::updateAcc(sensor_real_t Axyz[3])
        {
            std::copy(Axyz, Axyz+3, bAxyz);
        }

        void SensorFusionDMP::updateMag(sensor_real_t Mxyz[3])
        {
            if (!magExist) {
                if (Mxyz[0] != 0.0f || Mxyz[1] != 0.0f || Mxyz[2] != 0.0f) {
                    magExist = true;
                } else {
                    return;
                }
            }

            getGravityVec();
            dmpmag.update(qwxyz, bqwxyz, vecGravity, Mxyz);
        }

        void SensorFusionDMP::updateQuaternion(sensor_real_t nqwxyz[4])
        {
            std::copy(nqwxyz, nqwxyz+4, bqwxyz);

            updated = true;
            gravityReady = false;
            linaccelReady = false;
        }

        bool SensorFusionDMP::isUpdated()
        {
            return updated;
        }

        void SensorFusionDMP::clearUpdated()
        {
            updated = false;
        }
        
        sensor_real_t const * SensorFusionDMP::getQuaternion()
        {
            if (!magExist) {
                qwxyz[0] = bqwxyz[0];
                qwxyz[1] = -bqwxyz[2];
                qwxyz[2] = bqwxyz[1];
                qwxyz[3] = bqwxyz[3];
            }
            return qwxyz;
        }

        sensor_real_t const * SensorFusionDMP::getGravityVec(sensor_real_t const nqwxyz[4])
        {
            if (nqwxyz == NULL) {
                nqwxyz = bqwxyz;
            } else {
                // Use raw quaternion instead of the one after remap
                gravityReady = false;
            }

            if (!gravityReady) {
                SensorFusion::calcGravityVec(nqwxyz, vecGravity);
                gravityReady = true;
            }
            return vecGravity;
        }

        sensor_real_t const * SensorFusionDMP::getLinearAcc()
        {
            if (!linaccelReady) {
                getGravityVec();
                SensorFusion::calcLinearAcc(bAxyz, vecGravity, linAccel);
                linaccelReady = true;
            }
            return linAccel;
        }

    }
}