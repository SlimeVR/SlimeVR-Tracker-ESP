#ifndef SLIMEVR_SENSORFUSION_H
#define SLIMEVR_SENSORFUSION_H

#include "globals.h"
#include "sensor.h"

#define SENSOR_DOUBLE_PRECISION 0

#define SENSOR_FUSION_TYPE SENSOR_FUSION_VQF

#define SENSOR_FUSION_MAHONY 1
#define SENSOR_FUSION_MADGWICK 2
#define SENSOR_FUSION_BASICVQF 3
#define SENSOR_FUSION_VQF 4

#define SENSOR_USE_MAHONY (SENSOR_FUSION_TYPE == SENSOR_FUSION_MAHONY)
#define SENSOR_USE_MADGWICK (SENSOR_FUSION_TYPE == SENSOR_FUSION_MADGWICK)
#define SENSOR_USE_BASICVQF (SENSOR_FUSION_TYPE == SENSOR_FUSION_BASICVQF)
#define SENSOR_USE_VQF (SENSOR_FUSION_TYPE == SENSOR_FUSION_VQF)


#include "../motionprocessing/types.h"

#include "mahony.h"
#include "madgwick.h"
#include <vqf.h>
#include <basicvqf.h>

namespace SlimeVR
{
    namespace Sensors
    {
        #if SENSOR_USE_VQF
        struct SensorVQFParams: VQFParams {
            SensorVQFParams() : VQFParams() {
                #ifndef VQF_NO_MOTION_BIAS_ESTIMATION
                motionBiasEstEnabled = false;
                #endif
                tauAcc = 2.0f;
                restMinT = 2.0f;
                restThGyr = 0.6f; // 400 norm
                restThAcc = 0.06f; // 100 norm
            }
        };
        #endif

        class SensorFusion
        {
        public:
            SensorFusion(float gyrTs, float accTs=-1.0, float magTs=-1.0)
                : gyrTs(gyrTs), 
                  accTs( (accTs<0) ? gyrTs : accTs ), 
                  magTs( (magTs<0) ? gyrTs : magTs )
            #if SENSOR_USE_MAHONY
            #elif SENSOR_USE_MADGWICK
            #elif SENSOR_USE_BASICVQF
                , basicvqf(gyrTs, ((accTs<0) ? gyrTs : accTs),
                                    ((magTs<0) ? gyrTs : magTs))
            #elif SENSOR_USE_VQF
                , vqf(vqfParams, gyrTs, ((accTs<0) ? gyrTs : accTs), 
                                        ((magTs<0) ? gyrTs : magTs))
            #endif
            {}

            void update6D(sensor_real_t Axyz[3], sensor_real_t Gxyz[3], sensor_real_t deltat=-1.0f);
            void update9D(sensor_real_t Axyz[3], sensor_real_t Gxyz[3], sensor_real_t Mxyz[3], sensor_real_t deltat=-1.0f);
            void updateAcc(sensor_real_t Axyz[3], sensor_real_t deltat=-1.0f);
            void updateMag(sensor_real_t Mxyz[3], sensor_real_t deltat=-1.0f);
            void updateGyro(sensor_real_t Gxyz[3], sensor_real_t deltat=-1.0f);

            bool isUpdated();
            void clearUpdated();
            sensor_real_t const * getQuaternion();
            Quat getQuaternionQuat();
            sensor_real_t const * getGravityVec();
            sensor_real_t const * getLinearAcc();
            void getLinearAcc(sensor_real_t outLinAccel[3]);

            static void calcGravityVec(const sensor_real_t qwxyz[4], sensor_real_t gravVec[3]);
            static void calcLinearAcc(const sensor_real_t accin[3], const sensor_real_t gravVec[3], sensor_real_t accout[3]);

        protected:
            sensor_real_t gyrTs;
            sensor_real_t accTs;
            sensor_real_t magTs;

            #if SENSOR_USE_MAHONY
                Mahony<sensor_real_t> mahony;
            #elif SENSOR_USE_MADGWICK
                Madgwick<sensor_real_t> madgwick;
            #elif SENSOR_USE_BASICVQF
                BasicVQF basicvqf;
            #elif SENSOR_USE_VQF
                SensorVQFParams vqfParams {};
                VQF vqf;
            #endif

            // A also used for linear acceleration extraction
            sensor_real_t bAxyz[3]{0.0f, 0.0f, 0.0f};

            #if SENSOR_USE_MAHONY || SENSOR_USE_MADGWICK
                // Buffer M here to keep the behavior of BMI160
                sensor_real_t bMxyz[3]{0.0f, 0.0f, 0.0f};
                bool accelUpdated = false;
            #endif

            bool magExist = false;
            sensor_real_t qwxyz[4]{1.0f, 0.0f, 0.0f, 0.0f};
            bool updated = false;

            bool gravityReady = false;
            sensor_real_t vecGravity[3]{0.0f, 0.0f, 0.0f};
            bool linaccelReady = false;
            sensor_real_t linAccel[3]{0.0f, 0.0f, 0.0f};
            #if ESP32
            sensor_real_t linAccel_guard;   // Temporary patch for some weird ESP32 bug
            #endif
        };
    }
}

#endif // SLIMEVR_SENSORFUSION_H
