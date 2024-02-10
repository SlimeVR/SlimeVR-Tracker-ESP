#ifndef SLIMEVR_SENSORFUSIONRESTDETECT_H_
#define SLIMEVR_SENSORFUSIONRESTDETECT_H_

#include "SensorFusion.h"

#include "../motionprocessing/RestDetection.h"

#if SENSOR_USE_VQF
    #define SENSOR_FUSION_WITH_RESTDETECT 1
#else
    #define SENSOR_FUSION_WITH_RESTDETECT 0
#endif

namespace SlimeVR
{
    namespace Sensors
    {
        #if !SENSOR_FUSION_WITH_RESTDETECT
        struct SensorRestDetectionParams: RestDetectionParams {
            SensorRestDetectionParams() : RestDetectionParams() {
                restMinTime = 2.0f;
                restThGyr = 0.6f; // 400 norm
                restThAcc = 0.06f; // 100 norm
            }
        };
        #endif

        class SensorFusionRestDetect : public SensorFusion
        {
        public:
            SensorFusionRestDetect(float gyrTs, float accTs=-1.0, float magTs=-1.0)
                : SensorFusion(gyrTs, accTs, magTs)
            #if !SENSOR_FUSION_WITH_RESTDETECT
                , restDetection(restDetectionParams, gyrTs,
                                (accTs<0) ? gyrTs : accTs)
            #endif
            {}

            bool getRestDetected();

            #if !SENSOR_FUSION_WITH_RESTDETECT
                void updateAcc(sensor_real_t Axyz[3], sensor_real_t deltat);
                void updateGyro(sensor_real_t Gxyz[3], sensor_real_t deltat);
            #endif
        protected:
            #if !SENSOR_FUSION_WITH_RESTDETECT
                SensorRestDetectionParams restDetectionParams {};
                RestDetection restDetection;
            #endif
            
        };
    }
}

#endif // SLIMEVR_SENSORFUSIONRESTDETECT_H_
