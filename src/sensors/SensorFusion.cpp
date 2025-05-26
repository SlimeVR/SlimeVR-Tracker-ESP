#include "SensorFusion.h"

namespace SlimeVR::Sensors {

void SensorFusion::update6D(
	sensor_real_t Axyz[3],
	sensor_real_t Gxyz[3],
	sensor_real_t deltat
) {
	updateAcc(Axyz, deltat);
	updateGyro(Gxyz, deltat);
}

void SensorFusion::update9D(
	sensor_real_t Axyz[3],
	sensor_real_t Gxyz[3],
	sensor_real_t Mxyz[3],
	sensor_real_t deltat
) {
	updateMag(Mxyz, deltat);
	updateAcc(Axyz, deltat);
	updateGyro(Gxyz, deltat);
}

void SensorFusion::updateAcc(const sensor_real_t Axyz[3], sensor_real_t deltat) {
	if (deltat < 0) {
		deltat = accTs;
	}

	std::copy(Axyz, Axyz + 3, bAxyz);
	vqf.updateAcc(Axyz);
}

void SensorFusion::updateMag(const sensor_real_t Mxyz[3], sensor_real_t deltat) {
	if (deltat < 0) {
		deltat = magTs;
	}

	if (!magExist) {
		if (Mxyz[0] != 0.0f || Mxyz[1] != 0.0f || Mxyz[2] != 0.0f) {
			magExist = true;
		} else {
			return;
		}
	}

	vqf.updateMag(Mxyz);
}

void SensorFusion::updateGyro(const sensor_real_t Gxyz[3], sensor_real_t deltat) {
	if (deltat < 0) {
		deltat = gyrTs;
	}

	vqf.updateGyr(Gxyz, deltat);

	updated = true;
	gravityReady = false;
	linaccelReady = false;
}

bool SensorFusion::isUpdated() { return updated; }

void SensorFusion::clearUpdated() { updated = false; }

sensor_real_t const* SensorFusion::getQuaternion() {
	if (magExist) {
		vqf.getQuat9D(qwxyz);
	} else {
		vqf.getQuat6D(qwxyz);
	}

	return qwxyz;
}

Quat SensorFusion::getQuaternionQuat() {
	getQuaternion();
	return Quat(qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]);
}

sensor_real_t const* SensorFusion::getGravityVec() {
	if (!gravityReady) {
		calcGravityVec(qwxyz, vecGravity);
		gravityReady = true;
	}
	return vecGravity;
}

sensor_real_t const* SensorFusion::getLinearAcc() {
	if (!linaccelReady) {
		getGravityVec();
		calcLinearAcc(bAxyz, vecGravity, linAccel);
		linaccelReady = true;
	}
	return linAccel;
}

void SensorFusion::getLinearAcc(sensor_real_t outLinAccel[3]) {
	getLinearAcc();
	std::copy(linAccel, linAccel + 3, outLinAccel);
}

Vector3 SensorFusion::getLinearAccVec() {
	getLinearAcc();
	return Vector3(linAccel[0], linAccel[1], linAccel[2]);
}

void SensorFusion::calcGravityVec(
	const sensor_real_t qwxyz[4],
	sensor_real_t gravVec[3]
) {
	gravVec[0] = 2 * (qwxyz[1] * qwxyz[3] - qwxyz[0] * qwxyz[2]);
	gravVec[1] = 2 * (qwxyz[0] * qwxyz[1] + qwxyz[2] * qwxyz[3]);
	gravVec[2] = qwxyz[0] * qwxyz[0] - qwxyz[1] * qwxyz[1] - qwxyz[2] * qwxyz[2]
			   + qwxyz[3] * qwxyz[3];
}

void SensorFusion::calcLinearAcc(
	const sensor_real_t accin[3],
	const sensor_real_t gravVec[3],
	sensor_real_t accout[3]
) {
	accout[0] = accin[0] - gravVec[0] * CONST_EARTH_GRAVITY;
	accout[1] = accin[1] - gravVec[1] * CONST_EARTH_GRAVITY;
	accout[2] = accin[2] - gravVec[2] * CONST_EARTH_GRAVITY;
}

void SensorFusion::updateBiasForgettingTime(float biasForgettingTime) {
	vqf.updateBiasForgettingTime(biasForgettingTime);
}

bool SensorFusion::getRestDetected() const { return vqf.getRestDetected(); }

}  // namespace SlimeVR::Sensors
