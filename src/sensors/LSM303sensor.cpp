/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 S.J. Remington & SlimeVR contributors

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

#include "LSM303sensor.h"
#include "network/network.h"
#include "GlobalVars.h"
#include "calibration.h"

#define FORCE_CALIBRATION false

void LSM303Sensor::motionSetup()
{
    imu.initialize(addr);
    int16_t output_buffer[3] = {0};
#if FORCE_CALIBRATION
    startCalibration(0);
#endif
    if (!imu.testConnection())
    {
        m_Logger.fatal("Can't connect to LSM303 at address 0x%02x", addr);
        return;
    }
    m_Logger.info("Connected to LSM303 at address 0x%02x", addr);
    imu.getAcceleration(output_buffer);
    float g_az = (float)(output_buffer[2]);
    if (g_az < -0.75f)
    {
        m_Logger.info("Flip front to confirm start calibration");
        delay(5000);
        imu.getAcceleration(output_buffer);
        g_az = (float)(output_buffer[2]);
        if (g_az > 0.75f)
        {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }
    }
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        switch (sensorCalibration.type)
        {
        case SlimeVR::Configuration::CalibrationConfigType::LSM303:
            m_Calibration = sensorCalibration.data.lsm303;
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
    working = true;
}

void LSM303Sensor::motionLoop()
{
#if ENABLE_INSPECTION
    {
        int16_t output_buffer[9] = {0};
        imu.getMotion9(output_buffer);
        Network::sendInspectionRawIMUData(sensorId, output_buffer[3], output_buffer[4], output_buffer[5], 255, output_buffer[0], output_buffer[1], output_buffer[2], 255, output_buffer[6], output_buffer[7], output_buffer[8], 255);
    }
#endif
    now = micros();
    deltat = now - last;
    last = now;
    float AGMxyz[9] = {0};
    getScaledValues(AGMxyz);
    mahonyQuaternionUpdate(q, AGMxyz[0], AGMxyz[1], AGMxyz[2], AGMxyz[3], AGMxyz[4], AGMxyz[5], AGMxyz[6], AGMxyz[7], AGMxyz[8], deltat * 1.0e-6f);
    quaternion.set(-q[2], q[1], q[3], q[0]);
    quaternion *= sensorOffset; // Network::sendTemperature((int)imu.getTemperatureC()>>2, sensorId);
    Network::sendTemperature(imu.getTemperatureC(), sensorId);
#if ENABLE_INSPECTION
    {
        Network::sendInspectionFusedIMUData(sensorId, quaternion);
    }
#endif

    if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
    {
        newData = true;
        lastQuatSent = quaternion;
    }
}

void LSM303Sensor::getScaledValues(float AGMxyz[9])
{
    int16_t output_buffer[9] = {0};
    imu.getMotion9(output_buffer);
    AGMxyz[0] = (float)output_buffer[0];
    AGMxyz[1] = (float)output_buffer[1];
    AGMxyz[2] = (float)output_buffer[2];

    AGMxyz[3] = ((float)output_buffer[3] - (m_Calibration.G_off[0])) * imu.gscale;
    AGMxyz[4] = ((float)output_buffer[4] - (m_Calibration.G_off[1])) * imu.gscale;
    AGMxyz[5] = ((float)output_buffer[5] - (m_Calibration.G_off[2])) * imu.gscale;

    AGMxyz[6] = (float)output_buffer[6];
    AGMxyz[7] = (float)output_buffer[7];
    AGMxyz[8] = -(float)output_buffer[8];
// apply offsets (bias) and scale factors from Magneto
#if useFullCalibrationMatrix == true
    float temp[3];
    for (uint8_t i = 0; i < 3; i++)
        temp[i] = (AGMxyz[i] - m_Calibration.A_B[i]);
    AGMxyz[0] = m_Calibration.A_Ainv[0][0] * temp[0] + m_Calibration.A_Ainv[0][1] * temp[1] + m_Calibration.A_Ainv[0][2] * temp[2];
    AGMxyz[1] = m_Calibration.A_Ainv[1][0] * temp[0] + m_Calibration.A_Ainv[1][1] * temp[1] + m_Calibration.A_Ainv[1][2] * temp[2];
    AGMxyz[2] = m_Calibration.A_Ainv[2][0] * temp[0] + m_Calibration.A_Ainv[2][1] * temp[1] + m_Calibration.A_Ainv[2][2] * temp[2];
#else
    for (uint8_t i = 0; i < 3; i++)
        AGMxyz[i] = (AGMxyz[i] - calibration->A_B[i]);
#endif
}

void LSM303Sensor::startCalibration(int calibrationType)
{
    ledManager.on();
    m_Logger.debug("Gathering raw data for device calibration...");
    constexpr int calibrationSamples = 200;
    // Reset values
    float Gxyz[3] = {0};
    // Wait for sensor to calm down before calibration
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        int16_t output_buffer[3] = {0};
        imu.getRotation(output_buffer);
        Gxyz[0] += float(output_buffer[0]) / calibrationSamples;
        Gxyz[1] += float(output_buffer[1]) / calibrationSamples;
        Gxyz[2] += float(output_buffer[2]) / calibrationSamples;
    }

#ifdef DEBUG_SENSOR
    m_Logger.trace("Gyro calibration results: %f %f %f", Gxyz[0], Gxyz[1], Gxyz[2]);
#endif

    Network::sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);
    m_Calibration.G_off[0] = Gxyz[0];
    m_Calibration.G_off[1] = Gxyz[1];
    m_Calibration.G_off[2] = Gxyz[2];

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("Gently rotate the device while it's gathering accelerometer and magnetometer data");
    ledManager.pattern(15, 300, 3000 / 310);
    float *calibrationDataAcc = (float *)malloc(calibrationSamples * 3 * sizeof(float));
    float *calibrationDataMag = (float *)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++)
    {
        ledManager.on();
        int16_t output_buffer[9] = {0};
        imu.getMotion9(output_buffer);
        calibrationDataAcc[i * 3 + 0] = output_buffer[0];
        calibrationDataAcc[i * 3 + 1] = output_buffer[1];
        calibrationDataAcc[i * 3 + 2] = output_buffer[2];
        calibrationDataMag[i * 3 + 0] = output_buffer[7];
        calibrationDataMag[i * 3 + 1] = output_buffer[6];
        calibrationDataMag[i * 3 + 2] = -output_buffer[8];
        Network::sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
        Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        ledManager.off();
        delay(250);
    }
    m_Logger.debug("Calculating calibration data...");
    float A_BAinv[4][3] = {0};
    float M_BAinv[4][3] = {0};
    CalculateCalibration(calibrationDataAcc, calibrationSamples, A_BAinv);
    free(calibrationDataAcc);
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataMag);
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
    for (int i = 0; i < 3; i++)
    {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    m_Logger.debug("}");
    m_Logger.debug("Saving the calibration data");
    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::LSM303;
    calibration.data.lsm303 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();
    ledManager.off();
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    m_Logger.debug("Saved the calibration data");
    m_Logger.info("Calibration data gathered");
}