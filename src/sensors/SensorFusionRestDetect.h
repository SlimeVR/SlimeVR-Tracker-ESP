#ifndef SLIMEVR_SENSORFUSIONRESTDETECT_H_
#define SLIMEVR_SENSORFUSIONRESTDETECT_H_

#include "../motionprocessing/RestDetection.h"
#include "SensorFusion.h"

namespace SlimeVR::Sensors {

class SensorFusionRestDetect : public SensorFusion {
public:
	SensorFusionRestDetect(float gyrTs, float accTs = -1.0, float magTs = -1.0)
		: SensorFusion(gyrTs, accTs, magTs) {}

	SensorFusionRestDetect(
		VQFParams vqfParams,
		float gyrTs,
		float accTs = -1.0,
		float magTs = -1.0
	)
		: SensorFusion(vqfParams, gyrTs, accTs, magTs) {}

	bool getRestDetected();

protected:
};
}  // namespace SlimeVR::Sensors

#endif  // SLIMEVR_SENSORFUSIONRESTDETECT_H_
