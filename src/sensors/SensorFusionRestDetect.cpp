#include "SensorFusionRestDetect.h"

namespace SlimeVR {
namespace Sensors {
#if !SENSOR_FUSION_WITH_RESTDETECT
void SensorFusionRestDetect::updateAcc(
	const sensor_real_t Axyz[3],
	sensor_real_t deltat
) {
	if (deltat < 0) {
		deltat = accTs;
	}
	restDetection.updateAcc(deltat, Axyz);
	SensorFusion::updateAcc(Axyz, deltat);
}

void SensorFusionRestDetect::updateGyro(
	const sensor_real_t Gxyz[3],
	sensor_real_t deltat
) {
	if (deltat < 0) {
		deltat = gyrTs;
	}
	restDetection.updateGyr(Gxyz);
	SensorFusion::updateGyro(Gxyz, deltat);
}
#endif

bool SensorFusionRestDetect::getRestDetected() {
#if !SENSOR_FUSION_WITH_RESTDETECT
	return restDetection.getRestDetected();
#elif SENSOR_USE_VQF
	return vqf.getRestDetected();
#endif
}
}  // namespace Sensors
}  // namespace SlimeVR
