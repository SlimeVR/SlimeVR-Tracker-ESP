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

#include "icm42688sensor.h"
#include "network/network.h"
#include "globals.h"
#include "helper_3dmath.h"
#include <i2cscan.h>
#include "calibration.h"
#include "magneto1.4.h"
#include "GlobalVars.h"
#include "mahony.h"
// #include "madgwick.h"

// >>>> Ascale = AFS_8G, Gscale = GFS_2000DPS, AODR = AODR_200Hz, GODR = GODR_1kHz, aMode = aMode_LN, gMode = gMode_LN;

//#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
constexpr float gscale = (2000. / 32768.0) * (PI / 180.0); //gyro 2000 LSB per d/s -> rad/s
//#endif

#define MAG_CORR_RATIO 0.02

//#define ACCEL_SENSITIVITY_2G 16384.0f
#define ACCEL_SENSITIVITY_8G 4096.0f

// Accel scale conversion steps: LSB/G -> G -> m/s^2
//constexpr float ASCALE_2G = ((32768. / ACCEL_SENSITIVITY_2G) / 32768.) * CONST_EARTH_GRAVITY;
constexpr float ASCALE_8G = (8. / 32768.) * CONST_EARTH_GRAVITY;

void ICM42688Sensor::motionSetup() {
    // initialize device
    uint8_t temp;
    I2Cdev::readByte(addr, ICM42688_WHO_AM_I, &temp);
    if(!(temp == 0x47 || temp == 0xDB)) {
        m_Logger.fatal("Can't connect to ICM42688 (reported device ID 0x%02x) at address 0x%02x", temp, addr);
        return;
    }

    m_Logger.info("Connected to ICM42688 (reported device ID 0x%02x) at address 0x%02x", temp, addr);

    I2Cdev::writeByte(addr, ICM42688_DEVICE_CONFIG, 1); // reset
    delay(2); // wait 1ms for reset
    I2Cdev::readByte(addr, ICM42688_INT_STATUS, &temp); // clear reset done int flag
    I2Cdev::writeByte(addr, ICM42688_INT_SOURCE0, 0); // disable ints
    I2Cdev::writeByte(addr, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
    I2Cdev::writeByte(addr, ICM42688_PWR_MGMT0, gMode_LN << 2 | aMode_LN); // set accel and gyro modes (low noise)
    delay(1); // wait >200us (datasheet 14.36)
    I2Cdev::writeByte(addr, ICM42688_ACCEL_CONFIG0, AFS_8G << 5 | AODR_200Hz); // set accel ODR and FS (200hz, 8g)
    I2Cdev::writeByte(addr, ICM42688_GYRO_CONFIG0, GFS_2000DPS << 5 | GODR_1kHz); // set gyro ODR and FS (1khz, 2000dps)
    I2Cdev::writeByte(addr, ICM42688_GYRO_ACCEL_CONFIG0, 0x44); // set gyro and accel bandwidth to ODR/10
	delay(50); // 10ms Accel, 30ms Gyro startup

    // turn on while flip back to calibrate. then, flip again after 5 seconds.
    // TODO: Move calibration invoke after calibrate button on slimeVR server available
    accel_read();
    if(Gxyz[2] < -0.75f) {
        ledManager.on();
        m_Logger.info("Flip front to confirm start calibration");
        delay(5000);
        ledManager.off();

        accel_read();
        if(Gxyz[2] > 0.75f) {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }
    }

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::ICM42688:
            m_Calibration = sensorCalibration.data.icm42688;
            break;

        case SlimeVR::Configuration::CalibrationConfigType::NONE:
            m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
            break;

        default:
            m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
        }
    }

    deltat = 1.0 / 1000.0; // gyro fifo 1khz

    I2Cdev::writeByte(addr, ICM42688_FIFO_CONFIG, 0x00); // FIFO bypass mode
    I2Cdev::writeByte(addr, ICM42688_FSYNC_CONFIG, 0x00); // disable FSYNC
    I2Cdev::readByte(addr, ICM42688_TMST_CONFIG, &temp); // disable FSYNC
    I2Cdev::writeByte(addr, ICM42688_TMST_CONFIG, temp & 0xfd); // disable FSYNC
    I2Cdev::writeByte(addr, ICM42688_FIFO_CONFIG1, 0x02); // enable FIFO gyro only
    I2Cdev::writeByte(addr, ICM42688_FIFO_CONFIG, 1<<6); // begin FIFO stream

    working = true;
    configured = true;
}

void ICM42688Sensor::motionLoop() {
	uint8_t rawCount[2];
	I2Cdev::readBytes(addr, ICM42688_FIFO_COUNTH, 2, &rawCount[0]);
	uint16_t count = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
	count += 32; // Add a few read buffer packets (4 ms)
	uint16_t packets = count / 8;								 // Packet size 8 bytes
	uint8_t rawData[2080];
	uint16_t stco = 0;
    I2Cdev::readBytes(addr, ICM42688_FIFO_DATA, count, &rawData[0]); // Read buffer

    accel_read();
    parseAccelData();

	for (uint16_t i = 0; i < packets; i++) {
		uint16_t index = i * 8; // Packet size 8 bytes
		if ((rawData[index] & 0x80) == 0x80) {
			continue; // Skip empty packets
		}
		// combine into 16 bit values
		float raw0 = (int16_t)((((int16_t)rawData[index + 1]) << 8) | rawData[index + 2]); // gx
		float raw1 = (int16_t)((((int16_t)rawData[index + 3]) << 8) | rawData[index + 4]); // gy
		float raw2 = (int16_t)((((int16_t)rawData[index + 5]) << 8) | rawData[index + 6]); // gz
		if (raw0 < -32766 || raw1 < -32766 || raw2 < -32766) {
			continue; // Skip invalid data
		}
		Gxyz[0] = raw0 * gscale; //gres
		Gxyz[1] = raw1 * gscale; //gres
		Gxyz[2] = raw2 * gscale; //gres
        parseGyroData();

        #if defined(_MAHONY_H_)
        mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
        #elif defined(_MADGWICK_H_)
        madgwickQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
        #endif
    }
    
    quaternion.set(-q[2], q[1], q[3], q[0]);

    quaternion *= sensorOffset;

    if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
    }
}

void ICM42688Sensor::accel_read() {
    uint8_t rawAccel[6];
    I2Cdev::readBytes(addr, ICM42688_ACCEL_DATA_X1, 6, &rawAccel[0]);
	float raw0 = (int16_t)((((int16_t)rawAccel[0]) << 8) | rawAccel[1]);
	float raw1 = (int16_t)((((int16_t)rawAccel[2]) << 8) | rawAccel[3]);
	float raw2 = (int16_t)((((int16_t)rawAccel[4]) << 8) | rawAccel[5]);
	Axyz[0] = raw0 * ASCALE_8G;
	Axyz[1] = raw1 * ASCALE_8G;
	Axyz[2] = raw2 * ASCALE_8G;
}

void ICM42688Sensor::gyro_read() {
    uint8_t rawGyro[6];
    I2Cdev::readBytes(addr, ICM42688_GYRO_DATA_X1, 6, &rawGyro[0]);
	float raw0 = (int16_t)((((int16_t)rawGyro[0]) << 8) | rawGyro[1]);
	float raw1 = (int16_t)((((int16_t)rawGyro[2]) << 8) | rawGyro[3]);
	float raw2 = (int16_t)((((int16_t)rawGyro[4]) << 8) | rawGyro[5]);
	Gxyz[0] = raw0 * gscale;
	Gxyz[1] = raw1 * gscale;
	Gxyz[2] = raw2 * gscale;
}

void ICM42688Sensor::startCalibration(int calibrationType) {
    ledManager.on();
    m_Logger.debug("Gathering raw data for device calibration...");
    constexpr int calibrationSamples = 500;
    double GxyzC[3] = {0, 0, 0};

    // Wait for sensor to calm down before calibration
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);

    for (int i = 0; i < calibrationSamples; i++) {
        delay(5);
        gyro_read();
        GxyzC[0] += Gxyz[0];
        GxyzC[1] += Gxyz[1];
        GxyzC[2] += Gxyz[2];
    }
    GxyzC[0] /= calibrationSamples;
    GxyzC[1] /= calibrationSamples;
    GxyzC[2] /= calibrationSamples;

#ifdef DEBUG_SENSOR
    m_Logger.trace("Gyro calibration results: %f %f %f", GxyzC[0], GxyzC[1], GxyzC[2]);
#endif

    // TODO: use offset registers?
    m_Calibration.G_off[0] = GxyzC[0];
    m_Calibration.G_off[1] = GxyzC[1];
    m_Calibration.G_off[2] = GxyzC[2];

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("Gently rotate the device while it's gathering accelerometer and magnetometer data");
    ledManager.pattern(15, 300, 3000/310);

    MagnetoCalibration *magneto_acc = new MagnetoCalibration();
    MagnetoCalibration *magneto_mag = new MagnetoCalibration();

    // NOTE: we don't use the FIFO here on *purpose*. This makes the difference between a calibration that takes a second or three and a calibration that takes much longer.
    for (int i = 0; i < calibrationSamples; i++) {
        ledManager.on();
        accel_read();
        magneto_acc->sample(Axyz[0], Axyz[1], Axyz[2]);

        ledManager.off();
        delay(50);
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
    for (int i = 0; i < 3; i++)
    {
        m_Calibration.A_B[i] = A_BAinv[0][i];
        m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
        m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
        m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    }
    m_Logger.debug("}");
    m_Logger.debug("[INFO] Magnetometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    m_Logger.debug("}");

    m_Logger.debug("Saving the calibration data");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::ICM42688;
    calibration.data.icm42688 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    ledManager.off();
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered");
}

void ICM42688Sensor::parseMagData() {
    float temp[3];

    //apply offsets and scale factors from Magneto
    for (unsigned i = 0; i < 3; i++) {
        temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
        #if useFullCalibrationMatrix == true
            Mxyz[i] = m_Calibration.M_Ainv[i][0] * temp[0] + m_Calibration.M_Ainv[i][1] * temp[1] + m_Calibration.M_Ainv[i][2] * temp[2];
        #else
            Mxyz[i] = temp[i];
        #endif
    }
}

void ICM42688Sensor::parseAccelData() {
    float temp[3];

    //apply offsets (bias) and scale factors from Magneto
    for (unsigned i = 0; i < 3; i++) {
        temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
        #if useFullCalibrationMatrix == true
            Axyz[i] = m_Calibration.A_Ainv[i][0] * temp[0] + m_Calibration.A_Ainv[i][1] * temp[1] + m_Calibration.A_Ainv[i][2] * temp[2];
        #else
            Axyz[i] = temp[i];
        #endif
    }
}

void ICM42688Sensor::parseGyroData() {
    // reading big endian int16
    Gxyz[0] = (Gxyz[0] - m_Calibration.G_off[0]);
    Gxyz[1] = (Gxyz[1] - m_Calibration.G_off[1]);
    Gxyz[2] = (Gxyz[2] - m_Calibration.G_off[2]);
}
