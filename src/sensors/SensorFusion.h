#ifndef SLIMEVR_SENSORFUSION_H
#define SLIMEVR_SENSORFUSION_H

#include "globals.h"
#include "sensor.h"

#define SENSOR_DOUBLE_PRECISION 0

#define SENSOR_FUSION_TYPE_STRING "vqf"

#include <vqf.h>

#include "../motionprocessing/types.h"

namespace SlimeVR::Sensors {
constexpr VQFParams DefaultVQFParams = VQFParams{
	.tauAcc = 2.0f,
	.restMinT = 2.0f,
	.restThGyr = 0.6f,
	.restThAcc = 0.06f,
};

class SensorFusion {
public:
	SensorFusion(
		VQFParams vqfParams,
		sensor_real_t gyrTs,
		sensor_real_t accTs = -1.0,
		sensor_real_t magTs = -1.0
	)
		: gyrTs(gyrTs)
		, accTs((accTs < 0) ? gyrTs : accTs)
		, magTs((magTs < 0) ? gyrTs : magTs)
		, vqfParams(vqfParams)
		, vqf(this->vqfParams,
			  gyrTs,
			  ((accTs < 0) ? gyrTs : accTs),
			  ((magTs < 0) ? gyrTs : magTs)) {}

	explicit SensorFusion(
		sensor_real_t gyrTs,
		sensor_real_t accTs = -1.0,
		sensor_real_t magTs = -1.0
	)
		: SensorFusion(DefaultVQFParams, gyrTs, accTs, magTs) {}

	void update6D(
		sensor_real_t Axyz[3],
		sensor_real_t Gxyz[3],
		sensor_real_t deltat = -1.0f
	);
	void update9D(
		sensor_real_t Axyz[3],
		sensor_real_t Gxyz[3],
		sensor_real_t Mxyz[3],
		sensor_real_t deltat = -1.0f
	);
	void updateAcc(const sensor_real_t Axyz[3], sensor_real_t deltat = -1.0f);
	void updateMag(const sensor_real_t Mxyz[3], sensor_real_t deltat = -1.0f);
	void updateGyro(const sensor_real_t Gxyz[3], sensor_real_t deltat = -1.0f);

	bool isUpdated();
	void clearUpdated();
	sensor_real_t const* getQuaternion();
	Quat getQuaternionQuat();
	sensor_real_t const* getGravityVec();
	sensor_real_t const* getLinearAcc();
	void getLinearAcc(sensor_real_t outLinAccel[3]);
	Vector3 getLinearAccVec();

	static void calcGravityVec(const sensor_real_t qwxyz[4], sensor_real_t gravVec[3]);
	static void calcLinearAcc(
		const sensor_real_t accin[3],
		const sensor_real_t gravVec[3],
		sensor_real_t accout[3]
	);

	void updateBiasForgettingTime(float biasForgettingTime);

	[[nodiscard]] bool getRestDetected() const;

protected:
	sensor_real_t gyrTs;
	sensor_real_t accTs;
	sensor_real_t magTs;

	VQFParams vqfParams;
	VQF vqf;

	// A also used for linear acceleration extraction
	sensor_real_t bAxyz[3]{0.0f, 0.0f, 0.0f};

	bool magExist = false;
	sensor_real_t qwxyz[4]{1.0f, 0.0f, 0.0f, 0.0f};
	bool updated = false;

	bool gravityReady = false;
	sensor_real_t vecGravity[3]{0.0f, 0.0f, 0.0f};
	bool linaccelReady = false;
	sensor_real_t linAccel[3]{0.0f, 0.0f, 0.0f};
#ifdef ESP32
	sensor_real_t linAccel_guard;  // Temporary patch for some weird ESP32 bug
#endif
};
}  // namespace SlimeVR::Sensors

#endif  // SLIMEVR_SENSORFUSION_H
