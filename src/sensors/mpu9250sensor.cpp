/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain, S.J. Remington & SlimeVR contributors

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

#include "mpu9250sensor.h"

#include <i2cscan.h>

#include "GlobalVars.h"
#include "calibration.h"
#include "globals.h"
#include "helper_3dmath.h"
#include "magneto1.4.h"
#if MPU_USE_DMPMAG
#include "dmpmag.h"
#endif

// #if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
constexpr float gscale
	= (250. / 32768.0) * (PI / 180.0);  // gyro default 250 LSB per d/s -> rad/s
// #endif

#define ACCEL_SENSITIVITY_2G 16384.0f

// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE_2G
	= ((32768. / ACCEL_SENSITIVITY_2G) / 32768.) * CONST_EARTH_GRAVITY;

void MPU9250Sensor::motionSetup() {
	// initialize device
	imu.initialize(addr);
	if (!imu.testConnection()) {
		m_Logger.fatal(
			"Can't connect to MPU9250 (reported device ID 0x%02x) at address 0x%02x",
			imu.getDeviceID(),
			addr
		);
		return;
	}

	m_Logger.info(
		"Connected to MPU9250 (reported device ID 0x%02x) at address 0x%02x",
		imu.getDeviceID(),
		addr
	);

	int16_t ax, ay, az;

	// turn on while flip back to calibrate. then, flip again after 5 seconds.
	// TODO: Move calibration invoke after calibrate button on slimeVR server available
	imu.getAcceleration(&ax, &ay, &az);
	float g_az = (float)az / 16384;  // For 2G sensitivity
	if (g_az < -0.75f) {
		ledManager.on();
		m_Logger.info("Flip front to confirm start calibration");
		delay(5000);
		ledManager.off();

		imu.getAcceleration(&ax, &ay, &az);
		g_az = (float)az / 16384;
		if (g_az > 0.75f) {
			m_Logger.debug("Starting calibration...");
			startCalibration(0);
		}
	}

	// Initialize the configuration
	{
		SlimeVR::Configuration::SensorConfig sensorConfig
			= configuration.getSensor(sensorId);
		// If no compatible calibration data is found, the calibration data will just be
		// zero-ed out
		switch (sensorConfig.type) {
			case SlimeVR::Configuration::SensorConfigType::MPU9250:
				m_Config = sensorConfig.data.mpu9250;
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

#if MPU_USE_DMPMAG
	uint8_t devStatus = imu.dmpInitialize();
	if (devStatus == 0) {
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
#else
	// NOTE: could probably combine these into less total writes, but this should work,
	// and isn't time critical.
	imu.setAccelFIFOEnabled(true);
	imu.setXGyroFIFOEnabled(true);
	imu.setYGyroFIFOEnabled(true);
	imu.setZGyroFIFOEnabled(true);
	imu.setSlave0FIFOEnabled(true);

	// Set a rate we prefer
	imu.setRate(MPU9250_SAMPLE_DIV);  // 8khz / (1 + MPU9250_SAMPLE_DIV)

	imu.resetFIFO();
	imu.setFIFOEnabled(true);

	working = true;
#endif
}

void MPU9250Sensor::motionLoop() {
#if ENABLE_INSPECTION
	{
		int16_t rX, rY, rZ, aX, aY, aZ, mX, mY, mZ;
		imu.getRotation(&rX, &rY, &rZ);
		imu.getAcceleration(&aX, &aY, &aZ);
		imu.getMagnetometer(&mX, &mY, &mZ);

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
			mX,
			mY,
			mZ,
			255
		);
	}
#endif

#if MPU_USE_DMPMAG
	// Update quaternion
	if (!dmpReady) {
		return;
	}
	Quaternion rawQuat{};
	uint8_t dmpPacket[packetSize];
	if (!imu.GetCurrentFIFOPacket(dmpPacket, packetSize)) {
		return;
	}
	if (imu.dmpGetQuaternion(&rawQuat, dmpPacket)) {
		return;  // FIFO CORRUPTED
	}
	hadData = true;

	sfusion.updateQuaternion(rawQuat);

	int16_t temp[3];
	imu.getMagnetometer(&temp[0], &temp[1], &temp[2]);
	parseMagData(temp);

	sfusion.updateMag(Mxyz);
#if SEND_ACCELERATION
	{
		int16_t atemp[3];
		this->imu.dmpGetAccel(atemp, dmpPacket);
		parseAccelData(atemp);

		sfusion.updateAcc(Axyz);
	}
#endif
#else
	union fifo_sample_raw buf;
	uint16_t remaining_samples;
	// TODO: would it be faster to read multiple samples at once
	while (getNextSample(&buf, &remaining_samples)) {
		parseAccelData(buf.sample.accel);
		parseGyroData(buf.sample.gyro);
		parseMagData(buf.sample.mag);

		// TODO: monitor magnetometer status
		// buf.sample.mag_status;
		// TODO: monitor interrupts
		// imu.getIntStatus();
		// TODO: monitor remaining_samples to ensure that the number is going down, not
		// up. remaining_samples

		sfusion.update9D(Axyz, Gxyz, Mxyz);
	}
#endif
	setFusedRotation(sfusion.getQuaternionQuat());
#if SEND_ACCELERATION
	setAcceleration(sfusion.getLinearAccVec());
#endif
}

void MPU9250Sensor::startCalibration(int calibrationType) {
	ledManager.on();
#if MPU_USE_DMPMAG
	// with DMP, we just need mag data
	constexpr int calibrationSamples = 300;

	// Blink calibrating led before user should rotate the sensor
	m_Logger.info("Gently rotate the device while it's gathering magnetometer data");
	ledManager.pattern(15, 300, 3000 / 310);
	MagnetoCalibration* magneto = new MagnetoCalibration();
	for (int i = 0; i < calibrationSamples; i++) {
		ledManager.on();
		int16_t mx, my, mz;
		imu.getMagnetometer(&mx, &my, &mz);
		magneto->sample(my, mx, -mz);

		ledManager.off();
		delay(250);
	}
	m_Logger.debug("Calculating calibration data...");

	float M_BAinv[4][3];
	magneto->current_calibration(M_BAinv);
	delete magneto;

	m_Logger.debug("[INFO] Magnetometer calibration matrix:");
	m_Logger.debug("{");
	for (int i = 0; i < 3; i++) {
		m_Config.M_B[i] = M_BAinv[0][i];
		m_Config.M_Ainv[0][i] = M_BAinv[1][i];
		m_Config.M_Ainv[1][i] = M_BAinv[2][i];
		m_Config.M_Ainv[2][i] = M_BAinv[3][i];
		m_Logger.debug(
			"  %f, %f, %f, %f",
			M_BAinv[0][i],
			M_BAinv[1][i],
			M_BAinv[2][i],
			M_BAinv[3][i]
		);
	}
	m_Logger.debug("}");

#else

	m_Logger.debug("Gathering raw data for device calibration...");
	constexpr int calibrationSamples = 300;
	// Reset values
	Gxyz[0] = 0;
	Gxyz[1] = 0;
	Gxyz[2] = 0;

	// Wait for sensor to calm down before calibration
	m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
	delay(2000);

	union fifo_sample_raw buf;

	imu.resetFIFO(
	);  // fifo is sure to have filled up in the seconds of delay, don't try reading it.
	for (int i = 0; i < calibrationSamples; i++) {
		// wait for new sample
		while (!getNextSample(&buf, nullptr)) {
			;
		}

		Gxyz[0] += float(buf.sample.gyro[0]);
		Gxyz[1] += float(buf.sample.gyro[1]);
		Gxyz[2] += float(buf.sample.gyro[2]);
	}
	Gxyz[0] /= calibrationSamples;
	Gxyz[1] /= calibrationSamples;
	Gxyz[2] /= calibrationSamples;

#ifdef DEBUG_SENSOR
	m_Logger.trace("Gyro calibration results: %f %f %f", Gxyz[0], Gxyz[1], Gxyz[2]);
#endif

	// TODO: use offset registers?
	m_Config.G_off[0] = Gxyz[0];
	m_Config.G_off[1] = Gxyz[1];
	m_Config.G_off[2] = Gxyz[2];

	// Blink calibrating led before user should rotate the sensor
	m_Logger.info(
		"Gently rotate the device while it's gathering accelerometer and magnetometer "
		"data"
	);
	ledManager.pattern(15, 300, 3000 / 310);

	MagnetoCalibration* magneto_acc = new MagnetoCalibration();
	MagnetoCalibration* magneto_mag = new MagnetoCalibration();

	// NOTE: we don't use the FIFO here on *purpose*. This makes the difference between
	// a calibration that takes a second or three and a calibration that takes much
	// longer.
	for (int i = 0; i < calibrationSamples; i++) {
		ledManager.on();
		int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
		imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
		magneto_acc->sample(ax, ay, az);
		magneto_mag->sample(my, mx, -mz);

		ledManager.off();
		delay(250);
	}
	m_Logger.debug("Calculating calibration data...");

	float A_BAinv[4][3];
	magneto_acc->current_calibration(A_BAinv);
	delete magneto_acc;

	float M_BAinv[4][3];
	magneto_mag->current_calibration(M_BAinv);
	delete magneto_mag;

	m_Logger.debug("Finished Calculate Calibration data");
	m_Logger.debug("Accelerometer calibration matrix:");
	m_Logger.debug("{");
	for (int i = 0; i < 3; i++) {
		m_Config.A_B[i] = A_BAinv[0][i];
		m_Config.A_Ainv[0][i] = A_BAinv[1][i];
		m_Config.A_Ainv[1][i] = A_BAinv[2][i];
		m_Config.A_Ainv[2][i] = A_BAinv[3][i];
		m_Logger.debug(
			"  %f, %f, %f, %f",
			A_BAinv[0][i],
			A_BAinv[1][i],
			A_BAinv[2][i],
			A_BAinv[3][i]
		);
	}
	m_Logger.debug("}");
	m_Logger.debug("[INFO] Magnetometer calibration matrix:");
	m_Logger.debug("{");
	for (int i = 0; i < 3; i++) {
		m_Config.M_B[i] = M_BAinv[0][i];
		m_Config.M_Ainv[0][i] = M_BAinv[1][i];
		m_Config.M_Ainv[1][i] = M_BAinv[2][i];
		m_Config.M_Ainv[2][i] = M_BAinv[3][i];
		m_Logger.debug(
			"  %f, %f, %f, %f",
			M_BAinv[0][i],
			M_BAinv[1][i],
			M_BAinv[2][i],
			M_BAinv[3][i]
		);
	}
	m_Logger.debug("}");
#endif

	m_Logger.debug("Saving the calibration data");

	SlimeVR::Configuration::SensorConfig config;
	config.type = SlimeVR::Configuration::SensorConfigType::MPU9250;
	config.data.mpu9250 = m_Config;
	configuration.setSensor(sensorId, config);
	configuration.save();

	ledManager.off();
	m_Logger.debug("Saved the calibration data");

	m_Logger.info("Calibration data gathered");
	// fifo will certainly have overflown due to magnetometer calibration, reset it.
	imu.resetFIFO();
}

void MPU9250Sensor::parseMagData(int16_t data[3]) {
	// reading *little* endian int16
	Mxyz[0] = (float)data[1];  // my
	Mxyz[1] = (float)data[0];  // mx
	Mxyz[2] = -(float)data[2];  // mz

	float temp[3];

	// apply offsets and scale factors from Magneto
	for (unsigned i = 0; i < 3; i++) {
		temp[i] = (Mxyz[i] - m_Config.M_B[i]);
	}

	for (unsigned i = 0; i < 3; i++) {
#if useFullCalibrationMatrix == true
		Mxyz[i] = m_Config.M_Ainv[i][0] * temp[0] + m_Config.M_Ainv[i][1] * temp[1]
				+ m_Config.M_Ainv[i][2] * temp[2];
#else
		Mxyz[i] = temp[i];
#endif
	}
}

void MPU9250Sensor::parseAccelData(int16_t data[3]) {
	// reading big endian int16
	Axyz[0] = (float)data[0];
	Axyz[1] = (float)data[1];
	Axyz[2] = (float)data[2];

#if !MPU_USE_DMPMAG
	float temp[3];
#endif

	// apply offsets (bias) and scale factors from Magneto
	for (unsigned i = 0; i < 3; i++) {
#if !MPU_USE_DMPMAG
		temp[i] = (Axyz[i] - m_Config.A_B[i]);
	}

	for (unsigned i = 0; i < 3; i++) {
#if useFullCalibrationMatrix == true
		Axyz[i] = m_Config.A_Ainv[i][0] * temp[0] + m_Config.A_Ainv[i][1] * temp[1]
				+ m_Config.A_Ainv[i][2] * temp[2];
#else
		Axyz[i] = temp[i];
#endif
#endif
		Axyz[i] *= ASCALE_2G;
	}
}

// TODO: refactor so that calibration/conversion to float is only done in one place.
void MPU9250Sensor::parseGyroData(int16_t data[3]) {
	// reading big endian int16
	Gxyz[0] = ((float)data[0] - m_Config.G_off[0])
			* gscale;  // 250 LSB(d/s) default to radians/s
	Gxyz[1] = ((float)data[1] - m_Config.G_off[1]) * gscale;
	Gxyz[2] = ((float)data[2] - m_Config.G_off[2]) * gscale;
}

// really just an implementation detail of getNextSample...
void MPU9250Sensor::swapFifoData(union fifo_sample_raw* sample) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	// byteswap the big endian integers
	for (unsigned iii = 0; iii < 12; iii += 2) {
		uint8_t tmp = sample->raw[iii + 0];
		sample->raw[iii + 0] = sample->raw[iii + 1];
		sample->raw[iii + 1] = tmp;
	}
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	// byteswap the little endian integers
	for (unsigned iii = 12; iii < 18; iii += 2) {
		uint8_t tmp = sample->raw[iii + 0];
		sample->raw[iii + 0] = sample->raw[iii + 1];
		sample->raw[iii + 1] = tmp;
	}
#else
#error \
	"Endian isn't Little endian or big endian, are we not using GCC or is this a PDP?"
#endif

	// compiler hint for the union, should be optimized away for optimization >= -O1
	// according to compiler explorer
	memmove(&sample->sample, sample->raw, sensor_data_len);
}

// thought experiments:
// is a single burst i2c transaction faster than multiple? by how much?
// how does that compare to the performance of a memcpy?
// how does that compare to the performance of data fusion?
// if we read an extra byte from the magnetometer (or otherwise did something funky)
// we could read into a properly aligned array of fifo_samples (and not require a
// memcpy?)
// TODO: strict aliasing might not be violated if we just read directly into a
// fifo_sample*.
//  which means the union approach may be overcomplicated. *shrug*
bool MPU9250Sensor::getNextSample(
	union fifo_sample_raw* buffer,
	uint16_t* remaining_count
) {
	uint16_t count = imu.getFIFOCount();
	if (count < sensor_data_len) {
		// no samples to read
		remaining_count = 0;
		return false;
	}

	if (remaining_count) {
		*remaining_count = (count / sensor_data_len) - 1;
	}

	imu.getFIFOBytes(buffer->raw, sensor_data_len);
	swapFifoData(buffer);
	return true;
}
