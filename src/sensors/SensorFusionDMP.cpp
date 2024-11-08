#include "SensorFusionDMP.h"

namespace SlimeVR {
namespace Sensors {
void SensorFusionDMP::updateAcc(sensor_real_t Axyz[3]) {
	std::copy(Axyz, Axyz + 3, bAxyz);
}

void SensorFusionDMP::updateMag(sensor_real_t Mxyz[3]) {
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

void SensorFusionDMP::updateQuaternion(sensor_real_t nqwxyz[4]) {
	std::copy(nqwxyz, nqwxyz + 4, bqwxyz);

	updated = true;
	gravityReady = false;
	linaccelReady = false;
}

void SensorFusionDMP::updateQuaternion(Quaternion const& nq) {
	bqwxyz[0] = nq.w;
	bqwxyz[1] = nq.x;
	bqwxyz[2] = nq.y;
	bqwxyz[3] = nq.z;

	updated = true;
	gravityReady = false;
	linaccelReady = false;
}

void SensorFusionDMP::updateQuaternion(Quat const& nq) {
	bqwxyz[0] = nq.w;
	bqwxyz[1] = nq.x;
	bqwxyz[2] = nq.y;
	bqwxyz[3] = nq.z;

	updated = true;
	gravityReady = false;
	linaccelReady = false;
}

bool SensorFusionDMP::isUpdated() { return updated; }

void SensorFusionDMP::clearUpdated() { updated = false; }

sensor_real_t const* SensorFusionDMP::getQuaternion() {
	if (!magExist) {
		// remap axis from DMP space to sensor space
		qwxyz[0] = bqwxyz[0];
		qwxyz[1] = -bqwxyz[2];
		qwxyz[2] = bqwxyz[1];
		qwxyz[3] = bqwxyz[3];
	}
	// dmpmag remaps axis during Mag update
	return qwxyz;
}

Quat SensorFusionDMP::getQuaternionQuat() {
	getQuaternion();
	return Quat(qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]);
}

sensor_real_t const* SensorFusionDMP::getGravityVec() {
	if (!gravityReady) {
		SensorFusion::calcGravityVec(bqwxyz, vecGravity);
		gravityReady = true;
	}
	return vecGravity;
}

sensor_real_t const* SensorFusionDMP::getLinearAcc() {
	if (!linaccelReady) {
		getGravityVec();
		SensorFusion::calcLinearAcc(bAxyz, vecGravity, linAccel);
		linaccelReady = true;
	}
	return linAccel;
}

void SensorFusionDMP::getLinearAcc(sensor_real_t outLinAccel[3]) {
	getLinearAcc();
	std::copy(linAccel, linAccel + 3, outLinAccel);
}

Vector3 SensorFusionDMP::getLinearAccVec() {
	getLinearAcc();
	return Vector3(linAccel[0], linAccel[1], linAccel[2]);
}
}  // namespace Sensors
}  // namespace SlimeVR
