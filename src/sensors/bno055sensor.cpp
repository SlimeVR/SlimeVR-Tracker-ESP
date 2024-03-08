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
#include "bno055sensor.h"
#include "globals.h"
#include "GlobalVars.h"

/*
	Info:

	Check the datasheet for the BNO055 for more information on the different
	operation modes.

	OPERATION_MODE_IMUPLUS = OPR_MODE 0x08
		In the IMU mode the relative orientation of the BNO055 in space is
		calculated from the accelerometer and gyroscope data. The calculation
		is fast.

	OPERATION_MODE_NDOF = OPR_MODE 0x0C
		This is a fusion mode with 9 degrees of freedom where the fused
		absolute orientation data is calculated from accelerometer, gyroscope
		and the magnetometer. The advantages of combining all three sensors are
		a fast calculation, resulting in high output data rate, and high
		robustness from magnetic field distortions. In this mode the
		Fast Magnetometer calibration is turned ON and thereby resulting in
		quick calibration of the magnetometer and higher output data accuracy.
*/

void BNO055Sensor::motionSetup() {
    imu = Adafruit_BNO055(sensorId, addr);
    delay(3000);
#if USE_6_AXIS
    if (!imu.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
#else
    if (!imu.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
#endif
    {
        m_Logger.fatal("Can't connect to BNO055 at address 0x%02x", addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    delay(1000);
    imu.setExtCrystalUse(true); //Adafruit BNO055's use external crystal. Enable it, otherwise it does not work.
    imu.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P0);
    imu.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P0);
    m_Logger.info("Connected to BNO055 at address 0x%02x", addr);

    working = true;
    configured = true;
}

void BNO055Sensor::motionLoop() {
#if ENABLE_INSPECTION
    {
        Vector3 gyro = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        Vector3 accel = imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        Vector3 mag = imu.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

        networkConnection.sendInspectionRawIMUData(sensorId, UNPACK_VECTOR(gyro), 255, UNPACK_VECTOR(accel), 255, UNPACK_VECTOR(mag), 255);
    }
#endif

    // TODO Optimize a bit with setting rawQuat directly
    Quat quat = imu.getQuat();
    fusedRotation.set(quat.x, quat.y, quat.z, quat.w);
    fusedRotation *= sensorOffset;
    setFusedRotationReady();

#if SEND_ACCELERATION
    {
        acceleration = this->imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        setAccelerationReady();
    }
#endif
}

void BNO055Sensor::startCalibration(int calibrationType) {

}

bool BNO055Sensor::hasMagnetometerEnabled() {
    return imu.getMode() == Adafruit_BNO055::OPERATION_MODE_NDOF;
}

bool BNO055Sensor::toggleMagnetometer(bool enabled) {
    imu.setMode(enabled ? Adafruit_BNO055::OPERATION_MODE_NDOF : Adafruit_BNO055::OPERATION_MODE_IMUPLUS);

    return true;
}
