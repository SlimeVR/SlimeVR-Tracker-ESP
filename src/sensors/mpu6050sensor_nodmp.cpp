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
#include "mpu6050sensor_nodmp.h"

#include <i2cscan.h>
#include "calibration.h"
#include "GlobalVars.h"

#include "magneto1.4.h"
#include "../motionprocessing/RestDetection.h"


// 500 deg/s
#define GYRO_SENSITIVITY 65.5
// 2G
#define ACCEL_SENSITIVITY 16384.0


// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr sensor_real_t ASCALE = ((32768. / ACCEL_SENSITIVITY) / 32768.) * CONST_EARTH_GRAVITY;

// Gyro scale conversion steps: LSB/°/s -> °/s -> °/s / step -> rad/s / step
constexpr sensor_real_t GSCALE = ((32768. / GYRO_SENSITIVITY) / 32768.) * (PI / 180.0);

constexpr sensor_real_t SAMPLE_DELTA = 1. / MPU6050_SAMPLE_RATE;
constexpr int32_t SAMPLE_DELTA_MICROS = (int32_t)(1e6 / MPU6050_SAMPLE_RATE);


struct fifo_sample {
    // Apparently MPU6050 stores data in big-endian -w-
    uint8_t accel_x_h, accel_x_l;
    uint8_t accel_y_h, accel_y_l;
    uint8_t accel_z_h, accel_z_l;

    uint8_t temp_h, temp_l;

    uint8_t gyro_x_h, gyro_x_l;
    uint8_t gyro_y_h, gyro_y_l;
    uint8_t gyro_z_h, gyro_z_l;
};
#define INT16_FIFO_VALUE(fifo, name) (((int16_t)fifo.name##_h << 8) | ((int16_t)fifo.name##_l))

#define TEMP_TO_C(t) ((float)(t) / 340.0f + 36.53f)


void MPU6050NoDMPSensor::motionSetup()
{
    imu.initialize(addr);
    if (!imu.testConnection())
    {
        m_Logger.fatal("Can't connect to %s (reported device ID 0x%02x) at address 0x%02x", getIMUNameByType(sensorType), imu.getDeviceID(), addr);
        return;
    }

    m_Logger.info("Connected to %s (reported device ID 0x%02x) at address 0x%02x", getIMUNameByType(sensorType), imu.getDeviceID(), addr);

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::MPU6050:
            // Kinda pretty bad to do this, but meh
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

    // Do pretty much the same initialization as dmpInitialize
    // (but without the DMP specific stuff)
    // Not actually necessary since initialize() does all that anyway
    /*
    imu.reset();
    delay(100);
    imu.resetGyroscopePath();
    imu.resetAccelerometerPath();
    imu.resetTemperaturePath();
    delay(100);
    imu.setSleepEnabled(false);
    */

    // Same settings as dmpInitialize sets
    // imu.setClockSource(1); // PLL with X Gyro
    imu.setIntEnabled(0);                             // Disable all interrupts
    imu.setIntFIFOBufferOverflowEnabled(true);        // Only leave FIFO overflow
    imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // +-2G accel range
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  // +-500deg/sec
    
    #if HIGH_SPEED
    imu.setDLPFMode(MPU6050_DLPF_BW_256);             // don't low-pass gyro(it's bandwith is 256Hz now and sample rate is 8KHz)
    imu.setRate(9);                                  // Sample at 8KHz / (1 + 9) = 800Hz
    #else
    imu.setDLPFMode(MPU6050_DLPF_BW_98);             // low-pass gyro to 98Hz(and 94Hz for the accelerometer)
    imu.setRate(3);                                   // Sample at 1KHz / (1 + 3) = 250Hz
    #endif

    // Enable everything except DMP
    imu.setTempSensorEnabled(true);
    imu.setDMPEnabled(false);
    delay(200); // Just in case

    // The part that differs from dmpInitialize
    imu.setTempFIFOEnabled(true);
    imu.setAccelFIFOEnabled(true);
    imu.setXGyroFIFOEnabled(true);
    imu.setYGyroFIFOEnabled(true);
    imu.setZGyroFIFOEnabled(true);
    imu.setSlave0FIFOEnabled(false);
    imu.setSlave1FIFOEnabled(false);
    imu.setSlave2FIFOEnabled(false);
    imu.setSlave3FIFOEnabled(false);
    imu.resetFIFO();
    imu.setFIFOEnabled(true);

    // Each sample will take 14 bytes on FIFO
    // 1024 bytes FIFO capacity / 14 bytes = 73 (rounding down)
    // We get one sample every 800Hz = 1 / 1.25ms [or 400Hz = 1 / 2.5ms]
    // That means we have can have up to 91.25ms [or 182.5ms] stored in FIFO
    // On average, motionLoop should be called 100 times a second, or every 10ms
    // Meaning we have more than enough wiggle room with timings

    // What is of more concern now is the I2C speed(by default it's 400000 bites/sec)
    // but each sensor running this code will put 89600 bits/sec into FIFO
    // To keep everything smooth, we need to grab bytes out of FIFO at least as quickly as it gets filled up
    // So we can only have at most 3 MPUs running at this speed connected to one microcontroller

    initialized = true;
    working = true;
    configured = true;

    checkFlipCalibration();

    // Copy-pasted straight from BMI's code, again
#if MPU6050_USE_TEMPCAL
    auto samplesPerStep = 80u;
    gyroTempCalibrator = new GyroTemperatureCalibrator(
        SlimeVR::Configuration::CalibrationConfigType::MPU6050,
        sensorId,
        GYRO_SENSITIVITY,
        samplesPerStep
    );

    gyroTempCalibrator->loadConfig(GYRO_SENSITIVITY);
    if (gyroTempCalibrator->config.hasCoeffs) {
        float offsetAtCalibTemp[3];
        gyroTempCalibrator->approximateOffset(m_Calibration.temperature, offsetAtCalibTemp);
        for (uint32_t i = 0; i < 3; i++)
            tempCalOffset[i] = m_Calibration.G_off[i] - offsetAtCalibTemp[i];
    }
#endif
}

void MPU6050NoDMPSensor::motionLoop()
{
    if (!initialized)
        return;

#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ;
        imu.getRotation(&rX, &rY, &rZ);
        imu.getAcceleration(&aX, &aY, &aZ);

        networkConnection.sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, 0, 0, 0, 255);
    }
#endif

    if (imu.getIntFIFOBufferOverflowStatus()) {
        // Overflows make it so we lose track of which packet is which
        // This necessitates a reset
        imu.resetFIFO();
        m_Logger.debug("FIFO overflow!");
        return;
    }

    constexpr auto maxSamples = 1024 / sizeof(fifo_sample);
    fifo_sample samples[maxSamples];

    auto cnt = (int)imu.getFIFOCount();

    // auto t1 = micros();

    auto sampleCnt = min(maxSamples, cnt / sizeof(samples[0]));
    if (sampleCnt < 1)
        return;
    
    auto rem = sampleCnt * sizeof(samples[0]);
    auto ptr = (uint8_t*)&samples;
    while (rem > 0) {
        auto cnt = min(rem, (size_t)I2C_BUFFER_LENGTH);
        imu.getFIFOBytes(ptr, cnt);
        rem -= cnt;
        ptr += cnt;
    }

    // auto t2 = micros();
    
    for (auto i = 0u; i < sampleCnt; i++) {
        auto sample = samples[i];
        // Send temperature first, since temperature calibration(in onRawGyroSample) needs it
        onRawTempSample(
            INT16_FIFO_VALUE(sample, temp)
        );

        /*
        if (temperature > 100) {
            // Definitely invalid temperature value
            // FIFO probably desynced
            imu.resetFIFO();
            return;
        }
        */

        onRawAccelSample(
            INT16_FIFO_VALUE(sample, accel_x),
            INT16_FIFO_VALUE(sample, accel_y),
            INT16_FIFO_VALUE(sample, accel_z)
        );
        onRawGyroSample(
            INT16_FIFO_VALUE(sample, gyro_x),
            INT16_FIFO_VALUE(sample, gyro_y),
            INT16_FIFO_VALUE(sample, gyro_z)
        );
    }
    
    // auto t3 = micros();
    // auto _dt1 = t2 - t1;
    // auto _dt2 = t3 - t2;

    // m_Logger.debug("Sample count: %d, I2C time: %dus, Processing time: %luus", sampleCnt, _dt1, _dt2);

    fusedRotation = sfusion.getQuaternionQuat();
    fusedRotation *= sensorOffset;

#if SEND_ACCELERATION
    {
        sensor_real_t accel[3];
        sfusion.getLinearAcc(accel);
        this->acceleration.x = accel[0]; 
        this->acceleration.y = accel[1]; 
        this->acceleration.z = accel[2];
        this->newAcceleration = true;
    }
#endif

    if (!OPTIMIZE_UPDATES || !lastFusedRotationSent.equalsWithEpsilon(fusedRotation))
    {
        newFusedRotation = true;
        lastFusedRotationSent = fusedRotation;
    }
}


// These are pretty much copied from the BMI implementation
void MPU6050NoDMPSensor::onRawGyroSample(int16_t x, int16_t y, int16_t z) {
    // 1. Apply gyro calibration
    sensor_real_t Gxyz[3];

#if MPU6050_USE_TEMPCAL
    bool restDetected = sfusion.getRestDetected();
    gyroTempCalibrator->updateGyroTemperatureCalibration(temperature, restDetected, x, y, z);

    float GOxyz[3];
    if (gyroTempCalibrator->approximateOffset(temperature, GOxyz)) {
        Gxyz[0] = (sensor_real_t)x - GOxyz[0] - tempCalOffset[0];
        Gxyz[1] = (sensor_real_t)y - GOxyz[1] - tempCalOffset[1];
        Gxyz[2] = (sensor_real_t)z - GOxyz[2] - tempCalOffset[2];
    }
    else
#endif
    {
        Gxyz[0] = (float)x - m_Calibration.G_off[0];
        Gxyz[1] = (float)y - m_Calibration.G_off[1];
        Gxyz[2] = (float)z - m_Calibration.G_off[2];
    }

    // 2. Convert units to °/s
    for (auto i = 0; i < 3; i++)
        Gxyz[i] *= GSCALE;

    // 3. After a remap, feed data to sensor fusion
    remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), &Gxyz[0], &Gxyz[1], &Gxyz[2]);
    sfusion.updateGyro(Gxyz, -1);

    optimistic_yield(100);
}

void MPU6050NoDMPSensor::onRawAccelSample(int16_t x, int16_t y, int16_t z) {
    // Apply accel calibration and convert units to m/s^2
    float tmp[3];
    tmp[0] = ((float)x - m_Calibration.A_B[0]);
    tmp[1] = ((float)y - m_Calibration.A_B[1]);
    tmp[2] = ((float)z - m_Calibration.A_B[2]);

    sensor_real_t Axyz[3];
    Axyz[0] = m_Calibration.A_Ainv[0][0] * tmp[0] + m_Calibration.A_Ainv[0][1] * tmp[1] + m_Calibration.A_Ainv[0][2] * tmp[2];
    Axyz[1] = m_Calibration.A_Ainv[1][0] * tmp[0] + m_Calibration.A_Ainv[1][1] * tmp[1] + m_Calibration.A_Ainv[1][2] * tmp[2];
    Axyz[2] = m_Calibration.A_Ainv[2][0] * tmp[0] + m_Calibration.A_Ainv[2][1] * tmp[1] + m_Calibration.A_Ainv[2][2] * tmp[2];

    Axyz[0] *= ASCALE;
    Axyz[1] *= ASCALE;
    Axyz[2] *= ASCALE;

    remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), &Axyz[0], &Axyz[1], &Axyz[2]);

    // Feed data to sensor fusion
    sfusion.updateAcc(Axyz, -1);

    optimistic_yield(100);
}

void MPU6050NoDMPSensor::onRawTempSample(int16_t t) {
    newTemperature = true;
    temperature = TEMP_TO_C(t);
}


// Send temps because why not
// Weirdly, BMI's code sends temperature inside motionLoop(), which sounds more like a hack than anything
// Overriding sendData() seems more appropriate
void MPU6050NoDMPSensor::sendData() {
    if (newTemperature) {
        networkConnection.sendTemperature(sensorId, temperature);
    }

    Sensor::sendData();
}

// Calibration is also mostly copied from BMI
void MPU6050NoDMPSensor::checkFlipCalibration() {
    int16_t ax, ay, az;
    getRemappedAcceleration(&ax, &ay, &az);
    float g_az = (float)az * ASCALE / 9.81f;
    if (g_az < -0.75f) {
        ledManager.on();

        m_Logger.info("Flip front to confirm start calibration");
        delay(5000);
        getRemappedAcceleration(&ax, &ay, &az);
        g_az = (float)az * ASCALE / 9.81f;
        if (g_az > 0.75f) {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }

        ledManager.off();
    }
}

void MPU6050NoDMPSensor::startCalibration(int calibrationType) {
    auto fifoActive = imu.getFIFOEnabled();
    
    if (fifoActive) {
        imu.setFIFOEnabled(false);
        imu.resetFIFO();
    }

    ledManager.on();

    #if MPU6050_CALIBRATE_GYRO
        calibrateGyro();
    #else
        m_Logger.info("Skipping gyro calibration...");
        delay(500);
    #endif

    #if MPU6050_CALIBRATE_ACCEL
        calibrateAccel();
    #else
        m_Logger.info("Skipping accel calibration...");
        delay(500);
    #endif

    m_Logger.debug("Saving the calibration data");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::MPU6050;
    calibration.data.bmi160 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered, exiting calibration mode in...");
    constexpr uint8_t POST_CALIBRATION_DELAY_SEC = 3;
    ledManager.on();
    for (uint8_t i = POST_CALIBRATION_DELAY_SEC; i > 0; i--) {
        m_Logger.info("%i...", i);
        delay(1000);
    }

    if (fifoActive) {
        imu.setFIFOEnabled(true);
        imu.resetFIFO();
    }
}

void MPU6050NoDMPSensor::calibrateGyro() {
    // Wait for sensor to calm down before calibration
    constexpr uint8_t GYRO_CALIBRATION_DELAY_SEC = 3;

    constexpr float GYRO_CALIBRATION_DURATION_SEC = 8;

    m_Logger.info("Put down the device and wait for baseline gyro reading calibration (%.1f seconds)", GYRO_CALIBRATION_DURATION_SEC);

    ledManager.on();
    for (uint8_t i = GYRO_CALIBRATION_DELAY_SEC; i > 0; i--) {
        m_Logger.info("%i...", i);
        delay(1000);
    }
    ledManager.off();

    m_Calibration.temperature = TEMP_TO_C(imu.getTemperature());

    #ifdef DEBUG_SENSOR
        m_Logger.trace("Calibration temperature: %f", temperature);
    #endif

    ledManager.pattern(100, 100, 3);
    ledManager.on();
    m_Logger.info("Gyro calibration started...");

    constexpr uint16_t gyroCalibrationSamples =
        GYRO_CALIBRATION_DURATION_SEC * MPU6050_SAMPLE_RATE;
    
    int32_t rawGxyz[3] = {0};
    for (int i = 0; i < gyroCalibrationSamples; i++) {
        int16_t gx, gy, gz;
        imu.getRotation(&gx, &gy, &gz);
        rawGxyz[0] += gx;
        rawGxyz[1] += gy;
        rawGxyz[2] += gz;
        delayMicroseconds(SAMPLE_DELTA_MICROS);
    }
    ledManager.off();

    m_Calibration.G_off[0] = ((double)rawGxyz[0]) / gyroCalibrationSamples;
    m_Calibration.G_off[1] = ((double)rawGxyz[1]) / gyroCalibrationSamples;
    m_Calibration.G_off[2] = ((double)rawGxyz[2]) / gyroCalibrationSamples;

    #ifdef DEBUG_SENSOR
        m_Logger.info("Gyro calibration results: %f %f %f", UNPACK_VECTOR_ARRAY(m_Calibration.G_off));
    #endif
}

void MPU6050NoDMPSensor::calibrateAccel() {
    MagnetoCalibration* magneto = new MagnetoCalibration();

    // Blink calibrating led before user should rotate the sensor
    #if MPU6050_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_ROTATION
        m_Logger.info("After 3 seconds, Gently rotate the device while it's gathering data");
    #elif MPU6050_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_6POINT
        m_Logger.info("Put the device into 6 unique orientations (all sides), leave it still and do not hold/touch for 3 seconds each");
    #endif

    constexpr uint8_t ACCEL_CALIBRATION_DELAY_SEC = 3;
    ledManager.on();
    for (uint8_t i = ACCEL_CALIBRATION_DELAY_SEC; i > 0; i--) {
        m_Logger.info("%i...", i);
        delay(1000);
    }
    ledManager.off();


    #if MPU6050_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_ROTATION
        uint16_t accelCalibrationSamples = 400;
        ledManager.pattern(100, 100, 6);
        delay(100);

        ledManager.on();
        m_Logger.debug("Gathering accelerometer data...");
        for (int i = 0; i < accelCalibrationSamples; i++)
        {
            int16_t ax, ay, az;
            imu.getAcceleration(&ax, &ay, &az);
            magneto->sample(ax, ay, az);

            delay(100);
        }
        ledManager.off();
    #elif MPU6050_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_6POINT
        RestDetectionParams calibrationRestDetectionParams;
        calibrationRestDetectionParams.restMinTimeMicros = 3 * 1e6;
        calibrationRestDetectionParams.restThAcc = 0.7f;
        RestDetection calibrationRestDetection(
            calibrationRestDetectionParams,
            SAMPLE_DELTA, SAMPLE_DELTA
        );

        constexpr uint16_t expectedPositions = 6;
        constexpr uint16_t numSamplesPerPosition = 96;

        uint16_t numPositionsRecorded = 0;
        uint16_t numCurrentPositionSamples = 0;
        bool waitForMotion = true;

        float* accelCalibrationChunk = new float[numSamplesPerPosition * 3];
        ledManager.pattern(100, 100, 6);

        ledManager.on();
        m_Logger.info("Gathering accelerometer data...");
        m_Logger.info("Waiting for position %i, you can leave the device as is...", numPositionsRecorded + 1);
        while (true) {
            int16_t ax, ay, az;
            imu.getAcceleration(&ax, &ay, &az);

            sensor_real_t scaled[3];
            scaled[0] = ax * ASCALE_2G;
            scaled[1] = ay * ASCALE_2G;
            scaled[2] = az * ASCALE_2G;

            calibrationRestDetection.updateAcc(SAMPLE_DELTA_MICROS, scaled);

            if (waitForMotion) {
                if (!calibrationRestDetection.getRestDetected()) {
                    waitForMotion = false;
                }
                delayMicroseconds(SAMPLE_DELTA_MICROS);
                continue;
            }

            if (calibrationRestDetection.getRestDetected()) {
                const uint16_t i = numCurrentPositionSamples * 3;
                accelCalibrationChunk[i + 0] = ax;
                accelCalibrationChunk[i + 1] = ay;
                accelCalibrationChunk[i + 2] = az;
                numCurrentPositionSamples++;

                m_Logger.debug("Sample: %f %f %f", scaled[0], scaled[1], scaled[2]);

                if (numCurrentPositionSamples >= numSamplesPerPosition) {
                    for (int i = 0; i < numSamplesPerPosition; i++) {
                        magneto->sample(
                            accelCalibrationChunk[i * 3 + 0],
                            accelCalibrationChunk[i * 3 + 1],
                            accelCalibrationChunk[i * 3 + 2]
                        );
                    }
                    numPositionsRecorded++;
                    numCurrentPositionSamples = 0;
                    if (numPositionsRecorded < expectedPositions) {
                        ledManager.pattern(50, 50, 2);
                        ledManager.on();
                        m_Logger.info("Recorded, waiting for position %i...", numPositionsRecorded + 1);
                        waitForMotion = true;
                    }
                }
            } else {
                numCurrentPositionSamples = 0;
            }

            if (numPositionsRecorded >= expectedPositions) break;

            delayMicroseconds(SAMPLE_DELTA_MICROS);
        }
        ledManager.off();
        delete[] accelCalibrationChunk;
    #endif

    m_Logger.debug("Calculating accelerometer calibration data...");
    
    float A_BAinv[4][3];
    magneto->current_calibration(A_BAinv);
    delete magneto;

    m_Logger.debug("Finished calculating accelerometer calibration");
    m_Logger.debug("Accelerometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.A_B[i] = A_BAinv[0][i];
        m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
        m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
        m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    }
    m_Logger.debug("}");
}



void MPU6050NoDMPSensor::printTemperatureCalibrationState() {
    const auto degCtoF = [](float degC) { return (degC * 9.0f/5.0f) + 32.0f; };

    m_Logger.info("Sensor %i temperature calibration state:", sensorId);
    m_Logger.info("  current temp: %0.4f C (%0.4f F)", temperature, degCtoF(temperature));
    auto printTemperatureRange = [&](const char* label, float min, float max) {
        m_Logger.info("  %s: min %0.4f C max %0.4f C (min %0.4f F max %0.4f F)",
            label, min, max, degCtoF(min), degCtoF(max)
        );
    };
    printTemperatureRange("total range",
        TEMP_CALIBRATION_MIN,
        TEMP_CALIBRATION_MAX
    );
    printTemperatureRange("calibrated range",
        gyroTempCalibrator->config.minTemperatureRange,
        gyroTempCalibrator->config.maxTemperatureRange
    );
    m_Logger.info("  done: %0.1f%", gyroTempCalibrator->config.getCalibrationDonePercent());
}
void MPU6050NoDMPSensor::printDebugTemperatureCalibrationState() {
    m_Logger.info("Sensor %i gyro odr %f hz, sensitivity %f lsb",
        sensorId,
        MPU6050_SAMPLE_RATE,
        GYRO_SENSITIVITY
    );
    m_Logger.info("Sensor %i temperature calibration matrix (tempC x y z):", sensorId);
    m_Logger.info("BUF %i %i", sensorId, TEMP_CALIBRATION_BUFFER_SIZE);
    m_Logger.info("SENS %i %f", sensorId, GYRO_SENSITIVITY);
    m_Logger.info("DATA %i", sensorId);
    for (int i = 0; i < TEMP_CALIBRATION_BUFFER_SIZE; i++) {
        m_Logger.info("%f %f %f %f",
            gyroTempCalibrator->config.samples[i].t,
            gyroTempCalibrator->config.samples[i].x,
            gyroTempCalibrator->config.samples[i].y,
            gyroTempCalibrator->config.samples[i].z
        );
    }
    m_Logger.info("END %i", sensorId);
    m_Logger.info("y = %f + (%fx) + (%fxx) + (%fxxx)", UNPACK_VECTOR_ARRAY(gyroTempCalibrator->config.cx), gyroTempCalibrator->config.cx[3]);
    m_Logger.info("y = %f + (%fx) + (%fxx) + (%fxxx)", UNPACK_VECTOR_ARRAY(gyroTempCalibrator->config.cy), gyroTempCalibrator->config.cy[3]);
    m_Logger.info("y = %f + (%fx) + (%fxx) + (%fxxx)", UNPACK_VECTOR_ARRAY(gyroTempCalibrator->config.cz), gyroTempCalibrator->config.cz[3]);
}
void MPU6050NoDMPSensor::saveTemperatureCalibration() {
    gyroTempCalibrator->saveConfig();
}

