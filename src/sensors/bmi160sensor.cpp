/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 S.J. Remington, SlimeVR contributors

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

// Typical sensitivity at 25C
// See p. 9 of https://www.mouser.com/datasheet/2/783/BST-BMI160-DS000-1509569.pdf
// 65.6 LSB/deg/s = 500 deg/s
#define TYPICAL_SENSITIVITY_LSB 65.6

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);

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
        Serial.print("[ERR] Can't communicate with BMI160, response 0x");
        Serial.println(imu.getDeviceID(), HEX);
        LEDManager::signalAssert();
        return;
    }

    Serial.print("[OK] Connected to BMI160, ID 0x");
    Serial.println(imu.getDeviceID(), HEX);

    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);
    if(az < 0 && 10 * (ax * ax + ay * ay) < az * az) {
        LEDManager::off(CALIBRATING_LED);
        Serial.println("Calling Calibration... Flip front to confirm start calibration.");
        delay(5000);
        imu.getAcceleration(&ax, &ay, &az);
        if(az > 0 && 10 * (ax * ax + ay * ay) < az * az)
            startCalibration(0);
        LEDManager::on(CALIBRATING_LED);
    }

    DeviceConfig * const config = getConfigPtr();
    calibration = &config->calibration[sensorId];
    working = true;
}

void BMI160Sensor::motionLoop() {
    now = micros();
    deltat = now - last; //seconds since last update
    last = now;

    float Gxyz[3] = {0};
    float Axyz[3] = {0};
    getScaledValues(Gxyz, Axyz);

    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat * 1.0e-6f);
    quaternion.set(-q[2], q[1], q[3], q[0]);
    quaternion *= sensorOffset;
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
    float temperature = getTemperature();
    float tempDiff = temperature - calibration->temperature;
    uint8_t quant = map(temperature, 15, 75, 0, 12);

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    // TODO: Read from FIFO?
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // TODO: Sensitivity over temp compensation?
    // TODO: Cross-axis sensitivity compensation?
    Gxyz[0] = ((float)gx - (calibration->G_off[0] + (tempDiff * LSB_COMP_PER_TEMP_X_MAP[quant]))) * GSCALE;
    Gxyz[1] = ((float)gy - (calibration->G_off[1] + (tempDiff * LSB_COMP_PER_TEMP_Y_MAP[quant]))) * GSCALE;
    Gxyz[2] = ((float)gz - (calibration->G_off[2] + (tempDiff * LSB_COMP_PER_TEMP_Z_MAP[quant]))) * GSCALE;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;
    //apply offsets (bias) and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        float temp[3];
        for (uint8_t i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - calibration->A_B[i]);
        Axyz[0] = calibration->A_Ainv[0][0] * temp[0] + calibration->A_Ainv[0][1] * temp[1] + calibration->A_Ainv[0][2] * temp[2];
        Axyz[1] = calibration->A_Ainv[1][0] * temp[0] + calibration->A_Ainv[1][1] * temp[1] + calibration->A_Ainv[1][2] * temp[2];
        Axyz[2] = calibration->A_Ainv[2][0] * temp[0] + calibration->A_Ainv[2][1] * temp[1] + calibration->A_Ainv[2][2] * temp[2];
    #else
        for (uint8_t i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - calibration->A_B[i]);
    #endif
}

void BMI160Sensor::startCalibration(int calibrationType) {
    LEDManager::on(CALIBRATING_LED);
    Serial.println("[NOTICE] Gathering raw data for device calibration...");
    DeviceConfig * const config = getConfigPtr();

    // Wait for sensor to calm down before calibration
    Serial.println("[NOTICE] Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    float temperature = getTemperature();
    config->calibration[sensorId].temperature = temperature;
    uint16_t gyroCalibrationSamples = 2500;
    float rawGxyz[3] = {0};
    Serial.printf("[NOTICE] Calibration temperature: %f\n", temperature);
    for (int i = 0; i < gyroCalibrationSamples; i++)
    {
        LEDManager::on(CALIBRATING_LED);
        int16_t gx, gy, gz;
        imu.getRotation(&gx, &gy, &gz);
        rawGxyz[0] += float(gx);
        rawGxyz[1] += float(gy);
        rawGxyz[2] += float(gz);
        LEDManager::off(CALIBRATING_LED);
    }
    config->calibration[sensorId].G_off[0] = rawGxyz[0] / gyroCalibrationSamples;
    config->calibration[sensorId].G_off[1] = rawGxyz[1] / gyroCalibrationSamples;
    config->calibration[sensorId].G_off[2] = rawGxyz[2] / gyroCalibrationSamples;
    Serial.printf("[NOTICE] Gyro calibration results: %f %f %f\n", config->calibration[sensorId].G_off[0], config->calibration[sensorId].G_off[1], config->calibration[sensorId].G_off[2]);

    // Blink calibrating led before user should rotate the sensor
    Serial.println("[NOTICE] After 3seconds, Gently rotate the device while it's gathering accelerometer data");
    LEDManager::on(CALIBRATING_LED);
    delay(1500);
    LEDManager::off(CALIBRATING_LED);
    delay(1500);
    Serial.println("[NOTICE] Gathering accelerometer data start!");

    uint16_t accelCalibrationSamples = 300;
    float *calibrationDataAcc = (float*)malloc(accelCalibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < accelCalibrationSamples; i++)
    {
        LEDManager::on(CALIBRATING_LED);
        int16_t ax, ay, az;
        imu.getAcceleration(&ax, &ay, &az);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
        LEDManager::off(CALIBRATING_LED);
        delay(100);
    }
    Serial.println("[NOTICE] Calibration data gathered");
    LEDManager::off(CALIBRATING_LED);
    Serial.println("[NOTICE] Now Calculate Calibration data");

    float A_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, accelCalibrationSamples, A_BAinv);
    free(calibrationDataAcc);
    Serial.println("[NOTICE] Finished Calculate Calibration data");
    Serial.println("[NOTICE] Now Saving EEPROM");
    for (int i = 0; i < 3; i++)
    {
        config->calibration[sensorId].A_B[i] = A_BAinv[0][i];
        config->calibration[sensorId].A_Ainv[0][i] = A_BAinv[1][i];
        config->calibration[sensorId].A_Ainv[1][i] = A_BAinv[2][i];
        config->calibration[sensorId].A_Ainv[2][i] = A_BAinv[3][i];
    }

    setConfig(*config);
    Serial.println("[NOTICE] Finished Saving EEPROM");
    delay(5000);
}
