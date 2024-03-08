/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2024 jojos38

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

#ifndef BMI323_H
#define BMI323_H

#include <stdint.h>

typedef int8_t (*ReadFunction)(uint8_t, uint8_t*, uint32_t, void*);
typedef int8_t (*WriteFunction)(uint8_t, const uint8_t*, uint32_t, void *);
typedef void (*DelayFunction)(uint32_t, void *);

class BMI323 {
    
    public:
        /******************************************************************************************
         *                                   PUBLIC STRUCTS
        ******************************************************************************************/
        struct AccelConfig {
            /*! Output data rate in Hz */
            uint8_t odr;

            /*! Bandwidth parameter */
            uint8_t bandwidth;

            /*! Filter accel mode */
            uint8_t accelMode;

            /*! Gravity range */
            uint8_t range;

            /*! Defines the number of samples to be averaged */
            uint8_t avgNum;
        };

        /*!
        * @brief Structure to store self calibration result
        */
        struct SelfCalibResult {
            /*! Stores the self-calibration result */
            uint8_t gyroCalibrationResult;

            /*! Stores the self-calibration error codes result */
            uint8_t calibrationErrorResult;
        };

        /******************************************************************************************
         *                                   PUBLIC CONSTANTS
        ******************************************************************************************/
        // General
        static const uint16_t CHIP_ID               = UINT16_C(0x0043);
        static const uint8_t  ENABLE                = UINT8_C(1);
        static const uint8_t  DISABLE               = UINT8_C(0);
        static const uint8_t  ADDRESS_I2C_PRIMARY   = UINT8_C(0x68);
        static const uint8_t  ADDRESS_I2C_SECONDARY = UINT8_C(0x69);

        // Success
        static const int8_t SUCCESS                  = INT8_C(0);
        static const int8_t SUCCESS_INTERFACE_RESULT = INT8_C(0);

        // Errors
        static const int8_t ERR_NULL_PTR                 = INT8_C(-1);
        static const int8_t ERR_INTERFACE_RESULT = INT8_C(-2);
        static const int8_t ERR_DEVICE_NOT_FOUND         = INT8_C(-3);
        static const int8_t ERR_ACCEL_INVALID_CFG    = INT8_C(-4);
        static const int8_t ERR_GYRO_INVALID_CFG       = INT8_C(-5);

        // Warnings
        static const uint8_t WARN_FIFO_EMPTY = UINT8_C(1);

        // I²C
        static const uint8_t READ_BUFFER_LENGTH = UINT8_C(128);
        static const uint8_t DUMMY_BYTE         = UINT8_C(2);

        // FIFO
        static const uint16_t FIFO_TIME_EN = UINT16_C(0x0100);
        static const uint16_t FIFO_ACCEL_EN = UINT16_C(0x0200);
        static const uint16_t FIFO_GYRO_EN = UINT16_C(0x0400);
        static const uint16_t FIFO_TEMP_EN = UINT16_C(0x0800);
        static const uint16_t FIFO_ALL_EN = UINT16_C(0x0F00);

        // FIFO Dummy frames
        static const uint16_t FIFO_GYRO_DUMMY_FRAME  = UINT16_C(0x7F02);
        static const uint16_t FIFO_ACCEL_DUMMY_FRAME = UINT16_C(0x7F01);
        static const uint16_t FIFO_TEMP_DUMMY_FRAME  = UINT16_C(0x8000);

        // FIFO Lengths
        static const uint8_t LENGTH_FIFO_ACCEL      = UINT8_C(6);
        static const uint8_t LENGTH_FIFO_GYRO       = UINT8_C(6);
        static const uint8_t LENGTH_TEMPERATURE     = UINT8_C(2);
        static const uint8_t LENGTH_SENSOR_TIME     = UINT8_C(2);

        // Calibration
        static const uint8_t CALIBRATION_SENSITIVITY = UINT8_C(1);
        static const uint8_t CALIBRATION_OFFSET      = UINT8_C(2);
        static const uint8_t CALIBRATION_APPLY_FALSE = UINT8_C(0);
        static const uint8_t CALIBRATION_APPLY_TRUE  = UINT8_C(4);

        /**
         * Accelerometer power mode
         * ACCEL_MODE_DISABLE: Accelerometer is disabled
         * ACCEL_MODE_LOW_PWR: Data rate is limited from 12.5Hz to 400Hz
         */
        static const uint8_t ACCEL_MODE_DISABLE   = UINT8_C(0x00);
        static const uint8_t ACCEL_MODE_LOW_PWR   = UINT8_C(0x03);
        static const uint8_t ACCEL_MODE_NORMAL    = UINT8_C(0X04);
        static const uint8_t ACCEL_MODE_HIGH_PERF = UINT8_C(0x07);

        /**
         * Choose the number of values the accelerometer should average for the last reading.
         * Averaging can help reduce noise and provide a smoother accelerometer output.
         * ACCEL_AVG_1: No averaging (raw values)
         * ACCEL_AVG_X: Averae over X values
         */
        static const uint8_t ACCEL_AVG_1  = UINT8_C(0x00);
        static const uint8_t ACCEL_AVG_2  = UINT8_C(0x01);
        static const uint8_t ACCEL_AVG_4  = UINT8_C(0x02);
        static const uint8_t ACCEL_AVG_8  = UINT8_C(0x03);
        static const uint8_t ACCEL_AVG_16 = UINT8_C(0x04);
        static const uint8_t ACCEL_AVG_32 = UINT8_C(0x05);
        static const uint8_t ACCEL_AVG_64 = UINT8_C(0x06);

        /**
         * Accel output data rate bandwidth
         * 
         * ACCEL_BANDWIDTH_ODR_HALF: Moderate filtering, tolerates some high-frequency noise. 
         * Provides a balance between filtering and responsiveness.
         * Use when you need a reasonably smooth signal while preserving some high-frequency components.
         * 
         * ACCEL_BANDWIDTH_ODR_QUARTER: Heavily filtered, reduced high-frequency noise.
         * Offers higher noise reduction at the cost of reduced bandwidth.
         * Choose for applications where filtering out high-frequency noise is critical,
         * even if it means sacrificing some responsiveness.
         */
        static const uint8_t ACCEL_BANDWIDTH_ODR_HALF    = UINT8_C(0);
        static const uint8_t ACCEL_BANDWIDTH_ODR_QUARTER = UINT8_C(1);

        /**
         * Accelerometer data output rate in Hertz
         * Numbers with underscores should be interpreted as comma
         * ACCEL_ODR_0_78HZ: 0.78Hz
         * ACCEL_ODR_100HZ: 100Hz
         * etc...
         */
        static const uint8_t ACCEL_ODR_0_78HZ = UINT8_C(0x01);
        static const uint8_t ACCEL_ODR_1_56HZ = UINT8_C(0x02);
        static const uint8_t ACCEL_ODR_3_125HZ = UINT8_C(0x03);
        static const uint8_t ACCEL_ODR_6_25HZ = UINT8_C(0x04);
        static const uint8_t ACCEL_ODR_12_5HZ = UINT8_C(0x05);
        static const uint8_t ACCEL_ODR_25HZ = UINT8_C(0x06);
        static const uint8_t ACCEL_ODR_50HZ = UINT8_C(0x07);
        static const uint8_t ACCEL_ODR_100HZ = UINT8_C(0x08);
        static const uint8_t ACCEL_ODR_200HZ = UINT8_C(0x09);
        static const uint8_t ACCEL_ODR_400HZ = UINT8_C(0x0A);
        static const uint8_t ACCEL_ODR_800HZ = UINT8_C(0x0B);
        static const uint8_t ACCEL_ODR_1600HZ = UINT8_C(0x0C);
        static const uint8_t ACCEL_ODR_3200HZ = UINT8_C(0x0D);
        static const uint8_t ACCEL_ODR_6400HZ = UINT8_C(0x0E);

        /**
         * The maximum accelerometer range in G
         * Selecting a higher range allows the accelerometer to measure larger accelerations but may reduce sensitivity to smaller ones.
         * Conversely, selecting a lower range provides higher sensitivity to smaller accelerations but may saturate at higher values.
         */
        static const uint8_t ACCEL_RANGE_2G = UINT8_C(0x00);
        static const uint8_t ACCEL_RANGE_4G = UINT8_C(0x01);
        static const uint8_t ACCEL_RANGE_8G = UINT8_C(0x02);
        static const uint8_t ACCEL_RANGE_16G = UINT8_C(0x03);
    
        /**
         * Choose the number of values the gyroscope should average for the last reading.
         * Averaging can help reduce noise and provide a smoother accelerometer output.
         * GYRO_AVG_1: No averaging (raw values)
         * GYRO_AVG_X: Averae over X values
         */
        static const uint8_t GYRO_AVG_1 = UINT8_C(0x00);
        static const uint8_t GYRO_AVG_2 = UINT8_C(0x01);
        static const uint8_t GYRO_AVG_4 = UINT8_C(0x02);
        static const uint8_t GYRO_AVG_8 = UINT8_C(0x03);
        static const uint8_t GYRO_AVG_16 = UINT8_C(0x04);
        static const uint8_t GYRO_AVG_32 = UINT8_C(0x05);
        static const uint8_t GYRO_AVG_64 = UINT8_C(0x06);

        /**
         * Gyroscope data output rate in Hertz
         * Numbers with underscores should be interpreted as comma
         * GYRO_ODR_0_78HZ: 0.78Hz
         * GYRO_ODR_100HZ: 100Hz
         * etc...
         */  
        static const uint8_t GYRO_ODR_0_78HZ = UINT8_C(0x01);
        static const uint8_t GYRO_ODR_1_56HZ = UINT8_C(0x02);
        static const uint8_t GYRO_ODR_3_125HZ = UINT8_C(0x03);
        static const uint8_t GYRO_ODR_6_25HZ = UINT8_C(0x04);
        static const uint8_t GYRO_ODR_12_5HZ = UINT8_C(0x05);
        static const uint8_t GYRO_ODR_25HZ = UINT8_C(0x06);
        static const uint8_t GYRO_ODR_50HZ = UINT8_C(0x07);
        static const uint8_t GYRO_ODR_100HZ = UINT8_C(0x08);
        static const uint8_t GYRO_ODR_200HZ = UINT8_C(0x09);
        static const uint8_t GYRO_ODR_400HZ = UINT8_C(0x0A);
        static const uint8_t GYRO_ODR_800HZ = UINT8_C(0x0B);
        static const uint8_t GYRO_ODR_1600HZ = UINT8_C(0x0C);
        static const uint8_t GYRO_ODR_3200HZ = UINT8_C(0x0D);
        static const uint8_t GYRO_ODR_6400HZ = UINT8_C(0x0E);

        /**
         * Gyroscope Range Options (DPS - Degrees Per Second):
         * 
         * Set the maximum range of the gyroscope in degrees per second (DPS). The range
         * determines the maximum rate of angular rotation that the gyroscope can measure
         * accurately without saturation.
         * 
         * Selecting a higher range allows the gyroscope to measure faster rotations but may reduce sensitivity to smaller ones.
         * Conversely, selecting a lower range provides higher sensitivity to smaller rotations but may saturate at higher values.
         * 
         * GYRO_RANGE_125DPS:  Best for very slow rotations with no high frequencies. e.g track sun position.
         * GYRO_RANGE_250DPS:  Suitable for relatively slow or moderate rotational motion. e.g drone stabilization
         * GYRO_RANGE_500DPS:  Appropriate for applications involving faster rotational motion. e.g camera stabilizer.
         * GYRO_RANGE_1000DPS: Ideal for capturing faster and dynamic motion. e.g robotics performing agile maneuvers.
         * GYRO_RANGE_2000DPS: Recommended for rapid and high-frequency rotation. e.g ultra fast drones, ultra fast robots
         */
        static const uint8_t GYRO_RANGE_125DPS = UINT8_C(0x00);
        static const uint8_t GYRO_RANGE_250DPS = UINT8_C(0x01);
        static const uint8_t GYRO_RANGE_500DPS = UINT8_C(0x02);
        static const uint8_t GYRO_RANGE_1000DPS = UINT8_C(0x03);
        static const uint8_t GYRO_RANGE_2000DPS = UINT8_C(0x04);

        /**
         * Gyroscope power mode
         * GYRO_MODE_DISABLE: Gyroscope is disabled
        */
        static const uint8_t GYRO_MODE_DISABLE = UINT8_C(0x00);
        static const uint8_t GYRO_MODE_SUSPEND = UINT8_C(0x01);
        static const uint8_t GYRO_MODE_LOW_PWR = UINT8_C(0x03);
        static const uint8_t GYRO_MODE_NORMAL = UINT8_C(0x04);
        static const uint8_t GYRO_MODE_HIGH_PERF = UINT8_C(0x07);

        /**
         * Gyroscope output data rate bandwidth
         * 
         * GYRO_BANDWIDTH_ODR_HALF: Moderate filtering, tolerates some high-frequency noise. 
         * Provides a balance between filtering and responsiveness.
         * Use when you need a reasonably smooth signal while preserving some high-frequency components.
         * 
         * GYRO_BANDWIDTH_ODR_QUARTER: Heavily filtered, reduced high-frequency noise.
         * Offers higher noise reduction at the cost of reduced bandwidth.
         * Choose for applications where filtering out high-frequency noise is critical,
         * even if it means sacrificing some responsiveness.
         */
        static const uint8_t GYRO_BANDWIDTH_ODR_HALF = UINT8_C(0);
        static const uint8_t GYRO_BANDWIDTH_ODR_QUARTER = UINT8_C(1);

        /******************************************************************************************
         *                                   CONSTRUCTOR FUNCTIONS
        ******************************************************************************************/
        BMI323(ReadFunction readFunc, WriteFunction writeFunc, DelayFunction delayFunc, void* interfacePtr) :
            readFunction(readFunc),
            writeFunction(writeFunc),
            delayFunction(delayFunc),
            interfacePointer(interfacePtr)
            {};
        ~BMI323() {};

        int8_t read(uint8_t registerAddress, uint8_t* regData, uint32_t length) {
            return readFunction(registerAddress, regData, length, this->interfacePointer);
        }

        int8_t write(uint8_t registerAddress, const uint8_t* regData, uint32_t length) {
            return writeFunction(registerAddress, regData, length, this->interfacePointer);
        }

        void delayMicroseconds(uint32_t period) {
            delayFunction(period, this->interfacePointer);
        }

        /******************************************************************************************
         *                                   PUBLIC FUNCTIONS
        ******************************************************************************************/
        /**
         * Initialize the 
        */
        int8_t initI2C();
        int8_t softReset();
        int8_t setGyroOffsetGain(uint16_t *gyroOffset, uint8_t *gyroGain);
        int8_t setFifoConfig(uint16_t config, uint8_t enable);
        int8_t getAccelConfig(struct AccelConfig *config);
        int8_t performGyroCalibration(uint8_t calibrationType, uint8_t applyCalibration, SelfCalibResult *calibrationResult);
        int8_t getGyroOffsetGain(uint16_t *offset, uint8_t* gain);
        int8_t getFifoLength(uint16_t *fifoAvailableLength);
        int8_t readFifoData(uint8_t *data, const uint16_t length);
        int8_t getTemperatureCelcius(float *temperature);
        int8_t setCommandRegister(uint16_t command);
        int8_t setRegisters(uint8_t registerAddress, const uint8_t *data, uint16_t length);
        int8_t getRegisters(uint8_t registerAddress, uint8_t *data, uint16_t length);
        
        /**
         * @brief Set accelerometer configuration (Output data rate, bandwidth, mode, range and averaging)
         * @param odr Any ACCEL_ODR value. Output data rate in hertz
         * Defines how many accelerometer values you will be able to gather each second
         * @param bandwidth Any ACCEL_BANDWIDTH value.
         * For high-frequency vibrations, opt for "ACCEL_BANDWIDTH_ODR_HALF" to maintain responsiveness.
         * For low-frequency motion or noise reduction, "ACCEL_BANDWIDTH_ODR_QUARTER" is more suitable.
         * @param accelMode Any ACCEL_MODE value.
         * Defines accelerometer mode
         * @param range Any ACCEL_RANGE value.
         * Defines the range of the accelerometer in G
         * @param avgNum Any ACCEL_AVG value.
         * Defines how many values should be averaged
         */
        int8_t setAccelConfig(AccelConfig *config);
        int8_t setAccelConfig(uint8_t odr, uint8_t bandwidth, uint8_t accelMode, uint8_t range, uint8_t avgNum);
        int8_t setGyroConfig(uint8_t odr, uint8_t bandwidth, uint8_t gyroMode, uint8_t range, uint8_t avgNum);

        /******************************************************************************************
         *                                   PUBLIC VARIABLES
        ******************************************************************************************/
        // The chip id of the sensor
        int8_t chipId;
        // The result of the last I²C read or write operation (the return value of the user's function)
        int8_t interfaceResult;
        // The number of bits of the accelerometers, it's define by the chip revision id
        uint16_t accelBitWidth;

    private:
        /******************************************************************************************
         *                                   PRIVATE CONSTANTS
        ******************************************************************************************/  
        // Commands
        static const uint16_t COMMAND_SOFT_RESET         = UINT16_C(0xDEAF);
        static const uint16_t COMMAND_SELF_CALIB_TRIGGER = UINT16_C(0x0101);

        // Registers
        static const uint8_t REGISTER_COMMAND              = UINT8_C(0x7E);
        static const uint8_t REGISTER_FEATURE_IO1          = UINT8_C(0x11);
        static const uint8_t REGISTER_FEATURE_IO2          = UINT8_C(0x12);
        static const uint8_t REGISTER_FEATURE_IO_STATUS    = UINT8_C(0x14);
        static const uint8_t REGISTER_FEATURE_CTRL         = UINT8_C(0x40);
        static const uint8_t REGISTER_CHIP_ID              = UINT8_C(0x00);
        static const uint8_t REGISTER_GYRO_DP_OFF_X        = UINT8_C(0x66);
        static const uint8_t REGISTER_FIFO_CONF            = UINT8_C(0x36);
        static const uint8_t REGISTER_FIFO_CTRL            = UINT8_C(0x37);
        static const uint8_t REGISTER_ACCEL_CONF           = UINT8_C(0x20);
        static const uint8_t REGISTER_GYRO_CONF            = UINT8_C(0x21);
        static const uint8_t REGISTER_FEATURE_DATA_ADDRESS = UINT8_C(0x41);
        static const uint8_t REGISTER_FEATURE_DATA_TX      = UINT8_C(0x42);
        static const uint8_t REGISTER_FIFO_FILL_LEVEL      = UINT8_C(0x15);
        static const uint8_t REGISTER_FIFO_DATA            = UINT8_C(0x16);
        static const uint8_t REGISTER_TEMP_DATA            = UINT8_C(0x09);

        // Delays
        static const uint16_t DELAY_SOFT_RESET = UINT16_C(2000);

        // Feature engine
        static const int8_t   E_FEATURE_ENGINE_STATUS      = INT8_C(-14);
        static const uint16_t FEATURE_ENGINE_ENABLE_MASK   = UINT16_C(0X0001);

        // Revision id
        static const uint8_t REV_ID_POS  = UINT8_C(4);
        static const uint8_t REV_ID_MASK = UINT8_C(0xF0);

        // Set low and high byte
        static const uint16_t SET_LOW_BYTE  = UINT16_C(0x00FF);
        static const uint16_t SET_HIGH_BYTE = UINT16_C(0xFF00);

        // Accel
        static const uint16_t ACCEL_DP_OFF_XYZ_13_BIT_MASK = UINT16_C(0x1FFF);
        static const uint16_t ACCEL_DP_OFF_XYZ_14_BIT_MASK = UINT16_C(0x3FFF);
        static const uint16_t ACCEL_ODR_MASK               = UINT16_C(0x000F);
        static const uint16_t ACCEL_RANGE_MASK             = UINT16_C(0x0070);
        static const uint16_t ACCEL_RANGE_POS              = UINT8_C(4);
        static const uint16_t ACCEL_BW_MASK                = UINT16_C(0x0080);
        static const uint16_t ACCEL_BW_POS                 = UINT8_C(7);
        static const uint16_t ACCEL_AVG_NUM_MASK           = UINT16_C(0x0700);
        static const uint16_t ACCEL_AVG_NUM_POS            = UINT8_C(8);
        static const uint16_t ACCEL_MODE_MASK              = UINT16_C(0x7000);
        static const uint16_t ACCEL_MODE_POS               = UINT8_C(12);

        // Gyro
        static const uint16_t GYRO_DP_OFF_MASK   = UINT16_C(0x03FF);
        static const uint16_t GYRO_DP_DGAIN_MASK = UINT16_C(0x007F);
        static const uint16_t GYRO_ODR_MASK      = UINT16_C(0x000F);
        static const uint16_t GYRO_RANGE_MASK    = UINT16_C(0x0070);
        static const uint8_t GYRO_RANGE_POS      = UINT8_C(4);
        static const uint16_t GYRO_BW_MASK       = UINT16_C(0x0080);
        static const uint8_t GYRO_BW_POS         = UINT8_C(7);
        static const uint16_t GYRO_AVG_NUM_MASK  = UINT16_C(0x0700);
        static const uint8_t GYRO_AVG_NUM_POS    = UINT8_C(8);
        static const uint16_t GYRO_MODE_MASK     = UINT16_C(0x7000);
        static const uint8_t GYRO_MODE_POS       = UINT8_C(12);

        static const uint8_t BASE_ADDR_GYRO_CALIBRATION_SELECT = UINT8_C(0x26);

        // Fifo
        static const uint16_t FIFO_CONFIG_MASK     = UINT16_C(0x0F01);
        static const uint8_t  LENGTH_FIFO_DATA     = UINT8_C(2);
        static const uint16_t FIFO_FILL_LEVEL_MASK = UINT16_C(0x07FF);

        // Calibration
        static const uint8_t  CALIB_ST_STATUS_MASK   = UINT8_C(0x10);
        static const uint8_t  CALIB_ST_RESULT_MASK   = UINT8_C(0x40);
        static const uint16_t CALIB_ST_COMPLETE_MASK = UINT16_C(0x0010);
        static const uint8_t  CALIB_ST_COMPLETE_POS  = UINT8_C(4);
        static const uint16_t GYRO_CALIB_RESULT_MASK = UINT16_C(0x0020);
        static const uint8_t  GYRO_CALIB_RESULT_POS = UINT8_C(5);



        /******************************************************************************************
         *                                  PRIVATE FUNCTIONS
        ******************************************************************************************/
        static uint16_t setBitPos0(uint16_t registerData, uint16_t bitMask, uint16_t data);
        static uint16_t getBitPos0(uint16_t registerData, uint16_t bitMask);
        static uint16_t setBits(uint16_t registerData, uint16_t bitMask, uint16_t bitPos, uint16_t data);
        static uint16_t getBits(uint16_t registerData, uint16_t bitMask, uint16_t bitPos);
        static int8_t   checkBoundaryVal(uint8_t *val, uint8_t min, uint8_t max);
        
        static int8_t  validateBwAvgAccelMode(uint8_t *bandwidth, uint8_t *accelMode, uint8_t *avgNum);
        static int8_t  validateAccelOdrRange(uint8_t *odr, uint8_t *range);
        static int8_t  validateAccelOdrAvg(uint8_t accelOdr, uint8_t accelAvg);
        static int8_t  accelSkippedSamplesCheck(float odr, float avg);

        static int8_t  validateBwAvgGyroMode(uint8_t *bandwidth, uint8_t *gyroMode, uint8_t *avgNum);
        static int8_t  validateGyroOdrRange(uint8_t *odr, uint8_t *range);
        static int8_t  validateGyroOdrAvg(uint8_t gyr_odr, uint8_t gyr_avg);
        static int8_t  gyroSkippedSamplesCheck(float odr, float avg);

        int8_t getSetCalibrationDMA(uint8_t calibrationType, uint8_t applyCalibration);
        int8_t getFeatureEngineErrorStatus(uint8_t *featureEngineErrRegLsb, uint8_t *featureEngineErrRegMsb);
        int8_t getGyroCalibrationResult(struct SelfCalibResult *calibrationResult);

        /******************************************************************************************
         *                                  PRIVATE VARIABLES
        ******************************************************************************************/
        ReadFunction readFunction;
        WriteFunction writeFunction;
        DelayFunction delayFunction;
        void *interfacePointer;
};
#endif