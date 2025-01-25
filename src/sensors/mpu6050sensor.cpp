/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "globals.h"

#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#else
#include "MPU6050_6Axis_MotionApps20.h"
#endif

#include <i2cscan.h>

#include "GlobalVars.h"
#include "calibration.h"
#include "mpu6050sensor.h"

#define ACCEL_SENSITIVITY_2G 16384.0f

// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE_2G
	= ((32768. / ACCEL_SENSITIVITY_2G) / 32768.) * CONST_EARTH_GRAVITY;

void MPU6050Sensor::motionSetup() {
	imu.initialize(addr);
	if (!imu.testConnection()) {
		m_Logger.fatal(
			"Can't connect to %s (reported device ID 0x%02x) at address 0x%02x",
			getIMUNameByType(sensorType),
			imu.getDeviceID(),
			addr
		);
		return;
	}

	m_Logger.info(
		"Connected to %s (reported device ID 0x%02x) at address 0x%02x",
		getIMUNameByType(sensorType),
		imu.getDeviceID(),
		addr
	);

#ifndef IMU_MPU6050_RUNTIME_CALIBRATION
	// Initialize the configuration
	{
		SlimeVR::Configuration::SensorConfig sensorCalibration
			= configuration.getCalibration(sensorId);
		// If no compatible calibration data is found, the calibration data will just be
		// zero-ed out
		switch (sensorCalibration.type) {
			case SlimeVR::Configuration::SensorConfigType::MPU6050:
				m_Config = sensorCalibration.data.mpu6050;
				break;

			case SlimeVR::Configuration::SensorConfigType::NONE:
				m_Logger.warn(
					"No calibration data found for sensor %d, ignoring...",
					sensorId
				);
				m_Logger.info("Calibration is advised");
				break;

			default:
				m_Logger.warn(
					"Incompatible calibration data found for sensor %d, ignoring...",
					sensorId
				);
				m_Logger.info("Calibration is advised");
		}
	}
#endif

	devStatus = imu.dmpInitialize();

	if (devStatus == 0) {
#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
		// We don't have to manually calibrate if we are using the dmp's automatic
		// calibration
#else  // IMU_MPU6050_RUNTIME_CALIBRATION

		m_Logger.debug("Performing startup calibration of accel and gyro...");
		// Do a quick and dirty calibration. As the imu warms up the offsets will change
		// a bit, but this will be good-enough
		delay(1000);  // A small sleep to give the users a chance to stop it from moving

		imu.CalibrateGyro(6);
		imu.CalibrateAccel(6);
		imu.PrintActiveOffsets();
#endif  // IMU_MPU6050_RUNTIME_CALIBRATION

		ledManager.pattern(50, 50, 5);

		// turn on the DMP, now that it's ready
		m_Logger.debug("Enabling DMP...");
		imu.setDMPEnabled(true);

		// TODO: Add interrupt support
		// mpuIntStatus = imu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		m_Logger.debug("DMP ready! Waiting for first interrupt...");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = imu.dmpGetFIFOPacketSize();

		working = true;
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		m_Logger.error("DMP Initialization failed (code %d)", devStatus);
	}
}

void MPU6050Sensor::motionLoop() {
#if ENABLE_INSPECTION
	{
		int16_t rX, rY, rZ, aX, aY, aZ;
		imu.getRotation(&rX, &rY, &rZ);
		imu.getAcceleration(&aX, &aY, &aZ);

		networkConnection.sendInspectionRawIMUData(
			sensorId,
			rX,
			rY,
			rZ,
			255,
			aX,
			aY,
			aZ,
			255,
			0,
			0,
			0,
			255
		);
	}
#endif

	if (!dmpReady) {
		return;
	}

	if (imu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
		imu.dmpGetQuaternion(&rawQuat, fifoBuffer);
		hadData = true;

		sfusion.updateQuaternion(rawQuat);

		setFusedRotation(sfusion.getQuaternionQuat());

#if SEND_ACCELERATION
		{
			this->imu.dmpGetAccel(&this->rawAccel, this->fifoBuffer);

			float Axyz[3]
				= {(float)rawAccel.x * ASCALE_2G,
				   (float)rawAccel.y * ASCALE_2G,
				   (float)rawAccel.z * ASCALE_2G};

			sfusion.updateAcc(Axyz);

			setAcceleration(sfusion.getLinearAccVec());
		}
#endif
	}
}

void MPU6050Sensor::startCalibration(int calibrationType) {
	ledManager.on();

#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
	m_Logger.info(
		"MPU is using automatic runtime calibration. Place down the device and it "
		"should automatically calibrate after a few seconds"
	);
#else  //! IMU_MPU6050_RUNTIME_CALIBRATION
	m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
	delay(2000);

	imu.setDMPEnabled(false);
	imu.CalibrateGyro(6);
	imu.CalibrateAccel(6);
	imu.setDMPEnabled(true);

	m_Logger.debug("Gathered baseline gyro reading");
	m_Logger.debug("Starting offset finder");
	switch (calibrationType) {
		case CALIBRATION_TYPE_INTERNAL_ACCEL:
			imu.CalibrateAccel(10);
			m_Config.A_B[0] = imu.getXAccelOffset();
			m_Config.A_B[1] = imu.getYAccelOffset();
			m_Config.A_B[2] = imu.getZAccelOffset();
			break;
		case CALIBRATION_TYPE_INTERNAL_GYRO:
			imu.CalibrateGyro(10);
			m_Config.G_off[0] = imu.getXGyroOffset();
			m_Config.G_off[1] = imu.getYGyroOffset();
			m_Config.G_off[2] = imu.getZGyroOffset();
			break;
	}

	SlimeVR::Configuration::SensorConfig calibration;
	calibration.type = SlimeVR::Configuration::SensorConfigType::MPU6050;
	calibration.data.mpu6050 = m_Config;
	configuration.setCalibration(sensorId, calibration);
	configuration.save();

	m_Logger.info("Calibration finished");
#endif  // !IMU_MPU6050_RUNTIME_CALIBRATION

	ledManager.off();
}
