/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

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

#ifndef SENSORS_BMI323_H
#define SENSORS_BMI323_H

#include "sensor.h" // Base class Sensor
#include "bmi323_jojos38.h" // BMI323 custom library
#include "bmm350_jojos38.h" // BMM350 custom library
#include "Wire.h" // Used for I2C communication
#include "SensorFusionRestDetect.h" // Used for sensor fusion
#include "logging/Logger.h" // Used for logging
#include "GlobalVars.h" // Used for temperature networking
#include "magneto1.4.h" // For mag calibration

class BMI323Sensor : public Sensor
{
public:
    BMI323Sensor(uint8_t id, uint8_t address, float rotation, uint8_t sclPin, uint8_t sdaPin) :
        Sensor("BMI323Sensor", IMU_BMI323, id, address, rotation, sclPin, sdaPin),
        address(address),
        m_sfusion(0.0025, 0.005, 0.04), // Gyro, Accel, Mag // in seconds (1 / frequency)
        bmi323(i2cRead, i2cWrite, delayUs, &this->address),
        bmm350(i2cRead, i2cWrite, delayUs, &this->bmm350Address)
        {};
    ~BMI323Sensor(){};
    
    SensorStatus getSensorState() override final {
        return m_status;
    }

    /**
     * @brief This function is called once to setup the sensor (sensors settings, etc.)
    */
    void motionSetup() override final;

    /**
     * @brief This function is as often as possible, it should read the sensor data
     */
    void motionLoop() override final;

    /**
     * @brief This function is called every time data needs to be sent to the host
    */
    // void sendData() override final;

    /**
     * @brief This function is called once to calibrate the sensor
     * @param calibrationType The type of calibration to perform
     */
    void startCalibration(int calibrationType) override final;

    static int8_t i2cRead(uint8_t registerAddress, uint8_t *registerData, uint32_t length, void *interfacePointer);
    static int8_t i2cWrite(uint8_t registerAddress, const uint8_t *registerData, uint32_t length, void *interfacePointer);
    static void delayUs(uint32_t period, void *interfacePointer);

    uint8_t address;
    SensorStatus m_status = SensorStatus::SENSOR_OFFLINE;

private:
    /**
     * @brief Extracts one frame from the FIFO buffer
     * 
     * The FIFO buffer is an array of uint_8t divided into 16 bytes chunks (if everything is enabled)
     * The first 3 words (6 bytes) are the accelerometer data (X, Y, Z)
     * The next 3 words (6 bytes) are the gyroscope data (X, Y, Z)
     * The next word (2 bytes) is the temperature data
     * The last word (2 bytes) is the time data
     * 
     * The least significant byte is first
     * The most significant byte is last
     * 
     * Each data can be a dummy data, in which case the value of the first word
     * will be equal to BMI_FIFO_<SENSOR>_DUMMY_FRAME
     * 
     * All FIFO data buffer first two bytes are dummy bytes, they need to be skiped
     * It is done by adding bmi323.dummy_byte to the index after reseting them
     * 
     * If one of the data is disabled then the buffer will be shorter by the data length
     * In this case temperature is disabled so the buffer is 14 bytes long
     * 
     * Data example:
     * [<ACCEL_X_LSB>, <ACCEL_X_MSB>, <ACCEL_Y_LSB>, <ACCEL_Y_MSB>, <ACCEL_Z_LSB>,
     *  <ACCEL_Z_MSB>, <GYRO_X_LSB>, <GYRO_X_MSB>, <GYRO_Y_LSB>, <GYRO_Y_MSB>,
     *  <GYRO_Z_LSB>, <GYRO_Z_MSB>, <TEMP_LSB>, <TEMP_MSB>, <TIME_LSB>, <TIME_MSB>]
    */
    uint8_t extractFrame(uint8_t *data, uint8_t index, float *accelData, float *gyroData);

    /**
     * @brief Print calibration data
    */
    void printCalibrationData();

    /**
     * @brief Apply calibration data from Magnometer
    */
    void applyMagCalibrationAndScale(float Mxyz[3]);

    /**
     * @brief Get a single temperature reading
    */
    float getTemperature();

    /**
     * SlimeVR Stuff
    */
    SlimeVR::Sensors::SensorFusionRestDetect m_sfusion;
    SlimeVR::Configuration::BMI323CalibrationConfig m_calibration;
    SlimeVR::Configuration::Configuration m_configuration;

    /**
     * BMI323 FIFO reading
    */
    const uint8_t ACCEL_VALID = 0x01;   // Bit 0 represents accelerometer data validity
    const uint8_t GYRO_VALID = 0x02;    // Bit 1 represents gyroscope data validity

    uint8_t fifoData[I2C_BUFFER_LENGTH] = { 0 }; // 2048 is the maximum size of the FIFO
    uint8_t frameLength = 0;
    uint8_t defaultAccelIndex = 0;
    uint8_t defaultGyroIndex = 0;
    float accelData[3] = { 0 };
    float gyroData[3] = { 0 };
    float magData[3] = { 0 };

    /**
     * Networking stuff
    */
    Quat lastFusedRotation = Quat(0, 0, 0, 0);

    // Time between each fused rotation data send
    const uint32_t rotationSendInterval = 8333; // 120Hz (MICROS)
    // Time between each temperature data send
    const uint32_t tempSendInterval = 2000000; // 2Hz (MICROS)
    // Time between each magnetometer data fusion
    const uint32_t magFusionInterval = 40000; // 25Hz (MICROS)

    uint32_t lastRotationSendTime = 0;
    uint32_t lastTempSendTime = 0;
    uint32_t lastMagFusionTime = 0;
    uint32_t lastMagSensorNanoTime = 0;
    uint32_t lastMagSensorSecTime = 0;

    /**
     * Sensors
    */
    BMI323 bmi323;
    BMM350 bmm350;
    uint8_t bmm350Address = 0;
};

#endif