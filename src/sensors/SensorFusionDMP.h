#ifndef SLIMEVR_SENSORFUSIONDMP_H
#define SLIMEVR_SENSORFUSIONDMP_H

#include "SensorFusion.h"
#include "dmpmag.h"

namespace SlimeVR
{
    namespace Sensors
    {
        class SensorFusionDMP
        {
        public:
            void updateQuaternion(sensor_real_t nqwxyz[4]);
            void updateAcc(sensor_real_t Axyz[3]);
            void updateMag(sensor_real_t Mxyz[3]);

            bool isUpdated();
            void clearUpdated();
            sensor_real_t const * getQuaternion();
            sensor_real_t const * getGravityVec();
            sensor_real_t const * getLinearAcc();

        protected:
            DMPMag<sensor_real_t> dmpmag;

            sensor_real_t bAxyz[3]{0.0f, 0.0f, 0.0f};
            
            bool magExist = false;
            sensor_real_t bqwxyz[4]{1.0f, 0.0f, 0.0f, 0.0f};
            sensor_real_t qwxyz[4]{1.0f, 0.0f, 0.0f, 0.0f};
            bool updated = false;

            bool gravityReady = false;
            sensor_real_t vecGravity[3]{0.0f, 0.0f, 0.0f};
            bool linaccelReady = false;
            sensor_real_t linAccel[3]{0.0f, 0.0f, 0.0f};
        };
    }
}

#endif // SLIMEVR_SENSORFUSIONDMP_H
