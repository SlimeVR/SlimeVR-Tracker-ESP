/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 Rosdayle & SlimeVR

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
#include "sensors/AdafruitNXPsensor.h"
#include "network/network.h"
#include "utils.h"
#include "calibration.h"
#include "GlobalVars.h"

void ADANXPSensor::motionSetup()
{
#ifdef DEBUG_SENSOR
    imu.enableDebugging(Serial);
#endif
    if (!gyro.begin())
    {
        m_Logger.fatal("Can't connect to %s at address 0x%02x", "FXAS21002C", addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to FXAS21002C");
    gyro.setRange(GYRO_RANGE_2000DPS);

    if (!accmag.begin())
    {
        m_Logger.fatal("Can't connect to %s at address 0x%02x", "FXOS8700", addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to FXOS8700");
    accmag.setAccelRange(ACCEL_RANGE_8G);
    accmag.getEvent(&accel_event, &mag_event);
    float g_az = (float)accmag.accel_raw.z * ACCEL_SCALE_FACTOR;
    if (g_az < -0.75f)
    {
        ledManager.on();

        m_Logger.info("Flip front to confirm start calibration");
        delay(5000);
        accmag.getEvent(&accel_event, &mag_event);
        g_az = (float)accmag.accel_raw.z * ACCEL_SCALE_FACTOR;
        if (g_az > 0.75f)
        {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }

        ledManager.off();
    }

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type)
        {
        case SlimeVR::Configuration::CalibrationConfigType::NXP:
            m_Calibration = sensorCalibration.data.nxp;
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
    configured = true;
}

void ADANXPSensor::motionLoop()
{
    // code stolen from bmi160sensor.cpp
    now = micros();
    deltat = now - last; // seconds since last update
    if ((deltat * 1.0e-3f) >= samplingRateInMillis)
    {
        last = now;

        float Gxyz[3] = {0};
        float Axyz[3] = {0};
        float Mxyz[3] = {0};
        getScaledValues(Gxyz, Axyz, Mxyz);

#if !USE_6_AXIS
        mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6f);
#else
        mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat * 1.0e-6f);
#endif
        quaternion.set(-q[2], q[1], q[3], q[0]);
        quaternion *= sensorOffset;

        if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
        {
            newData = true;
            lastQuatSent = quaternion;
        }
    }
}

void ADANXPSensor::getScaledValues(float Gxyz[3], float Axyz[3], float Mxyz[3])
{
    gyro.getEvent(&gyro_event);
    accmag.getEvent(&accel_event, &mag_event);

    int16_t ax = accmag.accel_raw.x, ay = accmag.accel_raw.y, az = accmag.accel_raw.z;
    int16_t gx = gyro.raw.x, gy = gyro.raw.y, gz = gyro.raw.z;
    int16_t mx = accmag.mag_raw.x, my = accmag.mag_raw.y, mz = accmag.mag_raw.z;

    // gx *= 0.017453293F;
    // gy *= 0.017453293F;
    // gz *= 0.017453293F;

#if ENABLE_INSPECTION
    {
        Network::sendInspectionRawIMUData(sensorId, gx, gy, gz, 255, ax, ay, az, 255, mx, my, mz, 255);
    }
#endif

    Gxyz[0] = ((float)gx - m_Calibration.G_off[0]) * GYRO_SCALE_FACTOR * SENSORS_DPS_TO_RADS;
    Gxyz[1] = ((float)gy - m_Calibration.G_off[1]) * GYRO_SCALE_FACTOR * SENSORS_DPS_TO_RADS;
    Gxyz[2] = ((float)gz - m_Calibration.G_off[2]) * GYRO_SCALE_FACTOR * SENSORS_DPS_TO_RADS;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;
// apply offsets (bias) and scale factors from Magneto
#if useFullCalibrationMatrix == true
    float temp[3];
    for (uint8_t i = 0; i < 3; i++)
        temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
    Axyz[0] = (m_Calibration.A_Ainv[0][0] * temp[0] + m_Calibration.A_Ainv[0][1] * temp[1] + m_Calibration.A_Ainv[0][2] * temp[2]) * ACCEL_SCALE_FACTOR;
    Axyz[1] = (m_Calibration.A_Ainv[1][0] * temp[0] + m_Calibration.A_Ainv[1][1] * temp[1] + m_Calibration.A_Ainv[1][2] * temp[2]) * ACCEL_SCALE_FACTOR;
    Axyz[2] = (m_Calibration.A_Ainv[2][0] * temp[0] + m_Calibration.A_Ainv[2][1] * temp[1] + m_Calibration.A_Ainv[2][2] * temp[2]) * ACCEL_SCALE_FACTOR;
#else
    for (uint8_t i = 0; i < 3; i++)
        Axyz[i] = (Axyz[i] - calibration->A_B[i]);
#endif

#if !USE_6_AXIS
    Mxyz[0] = (float)mx;
    Mxyz[1] = (float)my;
    Mxyz[2] = (float)mz;
// apply offsets and scale factors from Magneto
#if useFullCalibrationMatrix == true
    for (uint8_t i = 0; i < 3; i++)
        temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
    Mxyz[0] = (m_Calibration.M_Ainv[0][0] * temp[0] + m_Calibration.M_Ainv[0][1] * temp[1] + m_Calibration.M_Ainv[0][2] * temp[2]) * MAG_UT_LSB;
    Mxyz[1] = (m_Calibration.M_Ainv[1][0] * temp[0] + m_Calibration.M_Ainv[1][1] * temp[1] + m_Calibration.M_Ainv[1][2] * temp[2]) * MAG_UT_LSB;
    Mxyz[2] = (m_Calibration.M_Ainv[2][0] * temp[0] + m_Calibration.M_Ainv[2][1] * temp[1] + m_Calibration.M_Ainv[2][2] * temp[2]) * MAG_UT_LSB;
#else
    for (i = 0; i < 3; i++)
        Mxyz[i] = (Mxyz[i] - m_Calibration.M_B[i]);
#endif
#endif
}

void ADANXPSensor::startCalibration(int calibrationType)
{
    ledManager.on();

    m_Logger.debug("Gathering raw data for device calibration...");
    constexpr int gyroSamples = 1000;
    constexpr int accMagSamples = 100;
    int gyroCounter = 0;
    int accMagCounter = 0;
    float Gxyz[3] = {0};
    // Wait for sensor to calm down before calibration
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < gyroSamples; i++)
    {
        gyro.getEvent(&gyro_event);
        int16_t gx = gyro.raw.x, gy = gyro.raw.y, gz = gyro.raw.z;
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
        delay(10);
        if (i % (gyroSamples / 10) == 0)
        {
            gyroCounter += 1;
            m_Logger.info("Gyro Cal at %i0%%", gyroCounter);
        }
    }
    Gxyz[0] /= gyroSamples;
    Gxyz[1] /= gyroSamples;
    Gxyz[2] /= gyroSamples;

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
    float *calibrationDataAcc = (float *)malloc(accMagSamples * 3 * sizeof(float));
    float *calibrationDataMag = (float *)malloc(accMagSamples * 3 * sizeof(float));
    for (int i = 0; i < accMagSamples; i++)
    {
        ledManager.on();
        accmag.getEvent(&accel_event, &mag_event);
        int16_t ax = accmag.accel_raw.x, ay = accmag.accel_raw.y, az = accmag.accel_raw.z;
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
        Network::sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
#if !USE_6_AXIS
        int16_t mx = accmag.mag_raw.x, my = accmag.mag_raw.y, mz = accmag.mag_raw.z;
        calibrationDataMag[i * 3 + 0] = my;
        calibrationDataMag[i * 3 + 1] = mx;
        calibrationDataMag[i * 3 + 2] = -mz;
        Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
#endif
        ledManager.off();
        delay(250);
        if (i % (accMagSamples / 10) == 0)
        {
            accMagCounter += 1;
            m_Logger.info("AccelMag Cal at %i0%%", accMagCounter);
        }
    }
    m_Logger.debug("Calculating calibration data...");

    float A_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, accMagSamples, A_BAinv);
    free(calibrationDataAcc);
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
#if !USE_6_AXIS
    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataMag, accMagSamples, M_BAinv);
    free(calibrationDataMag);
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
#endif

    m_Logger.debug("Saving the calibration data");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::NXP;
    calibration.data.nxp = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    ledManager.off();
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered");
}
