#include "SensorFusionRestDetect.h"

namespace SlimeVR
{
    namespace Sensors
    {
        void SensorFusionRestDetect::updateAcc(sensor_real_t Axyz[3], sensor_real_t deltat)
        {
            #if !SENSOR_WITH_REST_DETECT
                if (deltat < 0) deltat = accTs;
                restDetection.updateAcc(deltat * 1e6, Axyz);
            #endif
            SensorFusion::updateAcc(Axyz, deltat);
        }

        void SensorFusionRestDetect::updateGyro(sensor_real_t Gxyz[3], sensor_real_t deltat)
        {
            #if !SENSOR_WITH_REST_DETECT
                if (deltat < 0) deltat = accTs;
                restDetection.updateGyr(deltat * 1e6, Gxyz);
            #endif
            SensorFusion::updateGyro(Gxyz, deltat);
        }

        void SensorFusionRestDetect::update6D(sensor_real_t Axyz[3], sensor_real_t Gxyz[3], sensor_real_t deltat)
        {
            #if !SENSOR_WITH_REST_DETECT
                if (deltat < 0) deltat = accTs; //They need to be the same
                restDetection.update6D(deltat * 1e6, Axyz, Gxyz);
            #endif
            SensorFusion::update6D(Axyz, Gxyz, deltat);
        }

        bool SensorFusionRestDetect::getRestDetected()
        {
            #if !SENSOR_WITH_REST_DETECT
                return restDetection.getRestDetected();
            #elif SENSOR_USE_VQF
                return vqf.getRestDetected();
            #endif
        }

        void SensorFusionRestDetect::updateRestDetectionParameters(RestDetectionParams newParams)
        {
            restDetection.updateRestDetectionParameters(newParams);
        }
    }
}
