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

#include "bmi160sensor.h"
#include "network/network.h"
#include "GlobalVars.h"

// Typical sensitivity at 25C
// See p. 9 of https://www.mouser.com/datasheet/2/783/BST-BMI160-DS000-1509569.pdf
// 65.6 LSB/deg/s = 500 deg/s
#define TYPICAL_SENSITIVITY_LSB 65.6

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);

#define ACCEL_SENSITIVITY_4G 8192.0f

// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE_4G = ((32768. / ACCEL_SENSITIVITY_4G) / 32768.) * EARTH_GRAVITY;

// LSB change per temperature step map.
// These values were calculated for 500 deg/s sensitivity
// Step quantization - 5 degrees per step
const float LSB_COMP_PER_TEMP_X_MAP[13] = {
    0.77888f, 1.01376f, 0.83848f, 0.39416f,             // 15, 20, 25, 30
    -0.08792f, -0.01576f, -0.1018f, 0.22208f,           // 35, 40, 45, 50
    0.22208f, 0.22208f, 0.22208f, 0.2316f,              // 55, 60, 65, 70
    0.53416f                                            // 75
};
const float LSB_COMP_PER_TEMP_Y_MAP[13] = {
    0.10936f, 0.24392f, 0.28816f, 0.24096f,
    0.05376f, -0.1464f, -0.22664f, -0.23864f,
    -0.25064f, -0.26592f, -0.28064f, -0.30224f,
    -0.31608f
};
const float LSB_COMP_PER_TEMP_Z_MAP[13] = {
    0.15136f, 0.04472f, 0.02528f, -0.07056f,
    0.03184f, -0.002f, -0.03888f, -0.14f,
    -0.14488f, -0.14976f, -0.15656f, -0.16108f,
    -0.1656f
};

void BMI160Sensor::motionSetup() {
    // initialize device
    imu.initialize(addr);
    if(!imu.testConnection()) {
        m_Logger.fatal("Can't connect to BMI160 (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to BMI160 (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);

    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / 8192; // For 4G sensitivity
    if(g_az < -0.75f) {
        ledManager.on();

        m_Logger.info("Flip front to confirm start calibration");
        delay(5000);
        imu.getAcceleration(&ax, &ay, &az);
        g_az = (float)az / 8192;
        if(g_az > 0.75f)
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
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::BMI160:
            m_Calibration = sensorCalibration.data.bmi160;
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

void BMI160Sensor::motionLoop() {
#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ;
        imu.getRotation(&rX, &rY, &rZ);
        imu.getAcceleration(&aX, &aY, &aZ);

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, 0, 0, 0, 255);
    }
#endif

    now = micros();
    deltat = now - last; //seconds since last update
    last = now;

    float Gxyz[3] = {0};
    float Axyz[3] = {0};
    getScaledValues(Gxyz, Axyz);

    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat * 1.0e-6f);
    quaternion.set(-q[2], q[1], q[3], q[0]);

#if SEND_ACCELERATION
    {
        // Use the same mapping as in quaternion.set(-q[2], q[1], q[3], q[0]);
        this->acceleration[0] = -Axyz[1];
        this->acceleration[1] = Axyz[0];
        this->acceleration[2] = Axyz[2];

        // get the component of the acceleration that is gravity
        VectorFloat gravity;
        gravity.x = 2 * (this->quaternion.x * this->quaternion.z - this->quaternion.w * this->quaternion.y);
        gravity.y = 2 * (this->quaternion.w * this->quaternion.x + this->quaternion.y * this->quaternion.z);
        gravity.z = this->quaternion.w * this->quaternion.w - this->quaternion.x * this->quaternion.x - this->quaternion.y * this->quaternion.y + this->quaternion.z * this->quaternion.z;
        
        // subtract gravity from the acceleration vector
        this->acceleration[0] -= gravity.x * ACCEL_SENSITIVITY_4G;
        this->acceleration[1] -= gravity.y * ACCEL_SENSITIVITY_4G;
        this->acceleration[2] -= gravity.z * ACCEL_SENSITIVITY_4G;

        // finally scale the acceleration values to mps2
        this->acceleration[0] *= ASCALE_4G;
        this->acceleration[1] *= ASCALE_4G;
        this->acceleration[2] *= ASCALE_4G;
    }
#endif

    quaternion *= sensorOffset;

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

float BMI160Sensor::getTemperature()
{
    // Middle value is 23 degrees C (0x0000)
    #define TEMP_ZERO 23
    // Temperature per step from -41 + 1/2^9 degrees C (0x8001) to 87 - 1/2^9 degrees C (0x7FFF)
    constexpr float TEMP_STEP = 128. / 65535;
    return (imu.getTemperature() * TEMP_STEP) + TEMP_ZERO;
}

void BMI160Sensor::getScaledValues(float Gxyz[3], float Axyz[3])
{
#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ, mX, mY, mZ;
        imu.getRotation(&rX, &rY, &rZ);
        imu.getAcceleration(&aX, &aY, &aZ);

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, 0, 0, 0, 255);
    }
#endif

    float temperature = getTemperature();
    float tempDiff = temperature - m_Calibration.temperature;
    uint8_t quant = map(temperature, 15, 75, 0, 12);

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    // TODO: Read from FIFO?
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // TODO: Sensitivity over temp compensation?
    // TODO: Cross-axis sensitivity compensation?
    Gxyz[0] = ((float)gx - (m_Calibration.G_off[0] + (tempDiff * LSB_COMP_PER_TEMP_X_MAP[quant]))) * GSCALE;
    Gxyz[1] = ((float)gy - (m_Calibration.G_off[1] + (tempDiff * LSB_COMP_PER_TEMP_Y_MAP[quant]))) * GSCALE;
    Gxyz[2] = ((float)gz - (m_Calibration.G_off[2] + (tempDiff * LSB_COMP_PER_TEMP_Z_MAP[quant]))) * GSCALE;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;
    //apply offsets (bias) and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        float temp[3];
        for (uint8_t i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
        Axyz[0] = m_Calibration.A_Ainv[0][0] * temp[0] + m_Calibration.A_Ainv[0][1] * temp[1] + m_Calibration.A_Ainv[0][2] * temp[2];
        Axyz[1] = m_Calibration.A_Ainv[1][0] * temp[0] + m_Calibration.A_Ainv[1][1] * temp[1] + m_Calibration.A_Ainv[1][2] * temp[2];
        Axyz[2] = m_Calibration.A_Ainv[2][0] * temp[0] + m_Calibration.A_Ainv[2][1] * temp[1] + m_Calibration.A_Ainv[2][2] * temp[2];
    #else
        for (uint8_t i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - calibration->A_B[i]);
    #endif
}

void BMI160Sensor::startCalibration(int calibrationType) {
    ledManager.on();

    m_Logger.debug("Gathering raw data for device calibration...");

    // Wait for sensor to calm down before calibration
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    float temperature = getTemperature();
    m_Calibration.temperature = temperature;
    uint16_t gyroCalibrationSamples = 2500;
    float rawGxyz[3] = {0};

#ifdef DEBUG_SENSOR
    m_Logger.trace("Calibration temperature: %f", temperature);
#endif

    for (int i = 0; i < gyroCalibrationSamples; i++)
    {
        ledManager.on();

        int16_t gx, gy, gz;
        imu.getRotation(&gx, &gy, &gz);
        rawGxyz[0] += float(gx);
        rawGxyz[1] += float(gy);
        rawGxyz[2] += float(gz);

        ledManager.off();
    }
    m_Calibration.G_off[0] = rawGxyz[0] / gyroCalibrationSamples;
    m_Calibration.G_off[1] = rawGxyz[1] / gyroCalibrationSamples;
    m_Calibration.G_off[2] = rawGxyz[2] / gyroCalibrationSamples;

#ifdef DEBUG_SENSOR
    m_Logger.trace("Gyro calibration results: %f %f %f", UNPACK_VECTOR_ARRAY(m_Calibration.G_off));
#endif

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("After 3 seconds, Gently rotate the device while it's gathering accelerometer data");
    ledManager.on();
    delay(1500);
    ledManager.off();
    delay(1500);
    m_Logger.debug("Gathering accelerometer data...");

    MagnetoCalibration *magneto = new MagnetoCalibration();

    uint16_t accelCalibrationSamples = 300;
    for (int i = 0; i < accelCalibrationSamples; i++)
    {
        ledManager.on();

        int16_t ax, ay, az;
        imu.getAcceleration(&ax, &ay, &az);
        magneto->sample(ax, ay, az);

        ledManager.off();
        delay(100);
    }
    ledManager.off();
    m_Logger.debug("Calculating calibration data...");

    float A_BAinv[4][3];
    magneto->current_calibration(A_BAinv);
    delete magneto;

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

    m_Logger.debug("Saving the calibration data");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::BMI160;
    calibration.data.bmi160 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered");
    delay(5000);
}
