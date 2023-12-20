#include "bmi323_jojos38.h"

/**
 * @brief Entry point for bmi323 sensor. It reads and validates the chip-id of the sensor.
 */
int8_t BMI323::initI2C() {
    this->chipId = 0;

    // Reset the sensor before initializing
    int8_t result = this->softReset();
    if (result != SUCCESS) return result;

    // Read chip-id of the BMI323
    uint8_t chipId[2] = { 0 };
    result = this->getRegisters(REGISTER_CHIP_ID, chipId, 2);
    if (result != SUCCESS) return result;

    // Validate the chip id
    if (chipId[0] != CHIP_ID) {
        return ERR_DEVICE_NOT_FOUND;
    }

    // Set the chip id
    this->chipId = chipId[0];

    // Set the accel bit width
    if (((chipId[1] & REV_ID_MASK) >> REV_ID_POS) == ENABLE) {
        this->accelBitWidth = ACCEL_DP_OFF_XYZ_14_BIT_MASK;
    }
    else {
        this->accelBitWidth = ACCEL_DP_OFF_XYZ_13_BIT_MASK;
    }
    
    return SUCCESS;
}

/**
 * @brief Writes the available sensor specific commands to the sensor.
 */
int8_t BMI323::setCommandRegister(uint16_t command) {
    // Define register data
    uint8_t registerData[2] = {
        (uint8_t)(command & SET_LOW_BYTE),
        (uint8_t)((command & SET_HIGH_BYTE) >> 8)
    };

    // Set the command in the command register
    int8_t result = this->setRegisters(REGISTER_COMMAND, registerData, 2);

    return result;
}

/**
 * @brief Writes data to the given register address of bmi323 sensor.
 */
int8_t BMI323::setRegisters(uint8_t registerAddress, const uint8_t *data, uint16_t length) {
    // Check that the data is not null
    if (data == nullptr) return ERR_NULL_PTR;

    // Write the data to the register
    this->interfaceResult = this->write(registerAddress, data, length);
    this->delayMicroseconds(2);

    // Check that the write was successful
    if (this->interfaceResult != SUCCESS_INTERFACE_RESULT) return ERR_INTERFACE_RESULT;

    return SUCCESS_INTERFACE_RESULT;
}

/**
 * @brief Reads the data from the given register address of bmi323
 * sensor.
 *
 * @note For most of the registers auto address increment applies, with the
 * exception of a few special registers, which trap the address. For e.g.,
 * Register address - 0x03.
 */
int8_t BMI323::getRegisters(uint8_t registerAddress, uint8_t *data, uint16_t length) {
    if (data == nullptr) return ERR_NULL_PTR;

    uint8_t tempBuffer[READ_BUFFER_LENGTH];

    // Read the data from the register
    this->interfaceResult = this->read(registerAddress, tempBuffer, length + DUMMY_BYTE);
    this->delayMicroseconds(2);

    // Check that the read was successful
    if (this->interfaceResult != SUCCESS_INTERFACE_RESULT) return ERR_INTERFACE_RESULT;

    // Copy the data to the data buffer
    uint16_t index = 0;
    while (index < length) {
        data[index] = tempBuffer[index + DUMMY_BYTE];
        index++;
    }

    return SUCCESS_INTERFACE_RESULT;
}

/**
 * @brief Resets bmi323 sensor. All registers are overwritten with their default values.
 */
int8_t BMI323::softReset() {
    int8_t result;

    // Reset bmi323
    result = this->setCommandRegister(COMMAND_SOFT_RESET);
    this->delayMicroseconds(DELAY_SOFT_RESET);
    if (result != SUCCESS) return result;
    
    // Enable feature engine
    uint8_t featureData[2] = { 0x2c, 0x01 };
    result = this->setRegisters(REGISTER_FEATURE_IO2, featureData, 2);
    if (result != SUCCESS) return result;

    // Enable feature status bit
    uint8_t featureIOStatus[2] = { ENABLE, 0 };
    result = this->setRegisters(REGISTER_FEATURE_IO_STATUS, featureIOStatus, 2);
    if (result != SUCCESS) return result;

    // Enable feature engine bit
    uint8_t featureEngineEnable[2] = { ENABLE, 0 };
    result = this->setRegisters(REGISTER_FEATURE_CTRL, featureEngineEnable, 2);
    if (result != SUCCESS) return result;


    // Checking the status bit for feature engine enable
    uint8_t registerData[2] = { 0 };
    uint8_t loop = 1;
    while (loop <= 10) {
        this->delayMicroseconds(100000);
        result = this->getRegisters(REGISTER_FEATURE_IO1, registerData, 2);
        if (result == SUCCESS) {
            if (registerData[0] & FEATURE_ENGINE_ENABLE_MASK) {
                result = SUCCESS;
                break;
            } else {
                result = E_FEATURE_ENGINE_STATUS;
            }
        }

        loop++;
    }

    return result;
}

/**
 * @brief Sets offset dgain for the sensor which stores self-calibrated values for gyro.
 */
int8_t BMI323::setGyroOffsetGain(uint16_t *gyroOffset, uint8_t *gyroGain) {
    if (gyroOffset == nullptr || gyroGain == nullptr) return ERR_NULL_PTR;

    // Gyroscope offset
    uint16_t gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    gyroOffsetX = BMI323::setBitPos0(0, GYRO_DP_OFF_MASK, gyroOffset[0]);
    gyroOffsetY = BMI323::setBitPos0(0, GYRO_DP_OFF_MASK, gyroOffset[1]);
    gyroOffsetZ = BMI323::setBitPos0(0, GYRO_DP_OFF_MASK, gyroOffset[2]);

    // Gyroscope gain
    uint8_t gyroGainX, gyroGainY, gyroGainZ;
    gyroGainX = (uint8_t)BMI323::setBitPos0(0, GYRO_DP_DGAIN_MASK, gyroGain[0]);
    gyroGainY = (uint8_t)BMI323::setBitPos0(0, GYRO_DP_DGAIN_MASK, gyroGain[1]);
    gyroGainZ = (uint8_t)BMI323::setBitPos0(0, GYRO_DP_DGAIN_MASK, gyroGain[2]);

    /// Prepare registers
    uint8_t gyroOffsetGain[12] = { 0 };
    // X
    gyroOffsetGain[0] = (uint8_t)(gyroOffsetX & SET_LOW_BYTE);
    gyroOffsetGain[1] = (gyroOffsetX & SET_HIGH_BYTE) >> 8;
    gyroOffsetGain[2] = gyroGainX;
    // Y
    gyroOffsetGain[4] = (uint8_t)(gyroOffsetY & SET_LOW_BYTE);
    gyroOffsetGain[5] = (gyroOffsetY & SET_HIGH_BYTE) >> 8;
    gyroOffsetGain[6] = gyroGainY;
    // Z
    gyroOffsetGain[8] = (uint8_t)(gyroOffsetZ & SET_LOW_BYTE);
    gyroOffsetGain[9] = (gyroOffsetZ & SET_HIGH_BYTE) >> 8;
    gyroOffsetGain[10] = gyroGainZ;

    // Set offset and gain
    uint8_t result = this->setRegisters(REGISTER_GYRO_DP_OFF_X, gyroOffsetGain, 12);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

uint16_t BMI323::setBitPos0(uint16_t registerData, uint16_t bitMask, uint16_t data) {
    return (registerData & ~(bitMask)) | (data & bitMask);
}

uint16_t BMI323::setBits(uint16_t registerData, uint16_t bitMask, uint16_t bitPos, uint16_t data) {
    return (registerData & ~(bitMask)) | ((data << bitPos) & bitMask);
}

/**
 * @brief Sets the FIFO configuration in the sensor.
 */
int8_t BMI323::setFifoConfig(uint16_t config, uint8_t enable) {
    // Get existing FIFO config
    uint8_t data[2] = { 0 };
    int8_t result = this->getRegisters(REGISTER_FIFO_CONF, data, 2);
    if (result != SUCCESS) return result;

    // Prepare the new confiuration
    uint16_t fifoConfig = config & FIFO_CONFIG_MASK;
    if (enable == ENABLE) {
        data[0] = data[0] | (uint8_t)fifoConfig;
        data[1] = data[1] | (uint8_t)(fifoConfig >> 8);
    } else {
        data[0] = data[0] & (~fifoConfig);
        data[1] = data[1] & (~(fifoConfig >> 8));
    }

    // Apply the new configuration
    result = this->setRegisters(REGISTER_FIFO_CONF, data, 2);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

int8_t BMI323::setAccelConfig(AccelConfig *config) {
    if (config == nullptr) return ERR_NULL_PTR;

    return this->setAccelConfig(
        config->odr,
        config->bandwidth,
        config->accelMode,
        config->range,
        config->avgNum
    );
}

/**
 * @brief Sets accelerometer configurations like ODR, accel mode, bandwidth, average samples and range.
 */
int8_t BMI323::setAccelConfig(uint8_t odr, uint8_t bandwidth, uint8_t accelMode, uint8_t range, uint8_t avgNum) {
    int8_t result;

    // Validate bandwidth and averaging samples
    result = this->validateBwAvgAccelMode(&bandwidth, &accelMode, &avgNum);
    if (result != SUCCESS) return ERR_ACCEL_INVALID_CFG;

    // Validate ODR and range
    result = this->validateAccelOdrRange(&odr, &range);
    if (result != SUCCESS) return ERR_ACCEL_INVALID_CFG;

    // Validate odr and avgNum in low power mode
    if (accelMode == ACCEL_MODE_LOW_PWR) {
        result = this->validateAccelOdrAvg(odr, avgNum);
        if (result != SUCCESS) return ERR_ACCEL_INVALID_CFG;
    }

    // Valide odr in normal and high power mode (Gyro must be in low power mode to go below 6.25Hz)
    if (accelMode == ACCEL_MODE_NORMAL || accelMode == ACCEL_MODE_HIGH_PERF) {
        // Accel cannot work at low rate when it's in normal or high power mode
        if (odr >= ACCEL_ODR_0_78HZ && odr <= ACCEL_ODR_6_25HZ) {
            return ERR_ACCEL_INVALID_CFG;
        }
    }

    // Prepare register data
    uint16_t odrRegister = BMI323::setBitPos0(0, ACCEL_ODR_MASK, odr);
    uint16_t rangeRegister = this->setBits(0, ACCEL_RANGE_MASK, ACCEL_RANGE_POS, range);
    uint16_t bandwidthRegister = this->setBits(0, ACCEL_BW_MASK, ACCEL_BW_POS, bandwidth);
    uint16_t avgNumRegister = this->setBits(0, ACCEL_AVG_NUM_MASK, ACCEL_AVG_NUM_POS, avgNum);
    uint16_t accelModeRegister = this->setBits(0, ACCEL_MODE_MASK, ACCEL_MODE_POS, accelMode);

    uint8_t registerData[2] = {
        (uint8_t)(odrRegister | rangeRegister | bandwidthRegister),
        (uint8_t)((avgNumRegister | accelModeRegister) >> 8)
    };

    // Set configurations for accel
    result = this->setRegisters(REGISTER_ACCEL_CONF, registerData, 2);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/**
 * @brief Used to validate the boundary conditions.
 */
int8_t BMI323::checkBoundaryVal(uint8_t *val, uint8_t min, uint8_t max) {
    if (val == nullptr) return ERR_NULL_PTR;

    // Check if value is below minimum value
    if (*val < min) {
        // Auto correct the invalid value to minimum value
        *val = min;
    }

    // Check if value is above maximum value
    if (*val > max) {
        // Auto correct the invalid value to maximum value
        *val = max;
    }

    return SUCCESS;
}

/**
 * @brief Validates bandwidth and accel mode of the accelerometer set by the user.
 */
int8_t BMI323::validateBwAvgAccelMode(uint8_t *bandwidth, uint8_t *accelMode, uint8_t *avgNum) {
    if (bandwidth == nullptr || accelMode == nullptr || avgNum == nullptr) return ERR_NULL_PTR;

    int8_t result;

    // Validate and auto-correct accel mode
    result = BMI323::checkBoundaryVal(accelMode, ACCEL_MODE_DISABLE, ACCEL_MODE_HIGH_PERF);
    if (result != SUCCESS) return result;

    // Validate for averaging number of samples
    result = BMI323::checkBoundaryVal(avgNum, ACCEL_AVG_1, ACCEL_AVG_64);
    if (result != SUCCESS) return result;

    // Validate bandwidth
    result = BMI323::checkBoundaryVal(bandwidth, ACCEL_BANDWIDTH_ODR_HALF, ACCEL_BANDWIDTH_ODR_QUARTER);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/**
 * @brief Validates ODR and range of the accelerometer set by the user.
 */
int8_t BMI323::validateAccelOdrRange(uint8_t *odr, uint8_t *range) {
    if (odr == nullptr || range == nullptr) return ERR_NULL_PTR;

    int8_t result;

    // Validate and auto correct ODR
    result = BMI323::checkBoundaryVal(odr, ACCEL_ODR_0_78HZ, ACCEL_ODR_6400HZ);
    if (result != SUCCESS) return result;

    // Validate and auto correct Range
    result = BMI323::checkBoundaryVal(range, ACCEL_RANGE_2G, ACCEL_RANGE_16G);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/**
 * Used to validate ODR and AVG combinations for accel
 */
int8_t BMI323::validateAccelOdrAvg(uint8_t accelOdr, uint8_t accelAvg) {
    float odr = 0.0, avg = 0.0;

    switch (accelOdr) {
        case ACCEL_ODR_0_78HZ:
            odr = 0.78125;
            break;
        case ACCEL_ODR_1_56HZ:
            odr = 1.5625;
            break;
        case ACCEL_ODR_3_125HZ:
            odr = 3.125;
            break;
        case ACCEL_ODR_6_25HZ:
            odr = 6.25;
            break;
        case ACCEL_ODR_12_5HZ:
            odr = 12.5;
            break;
        case ACCEL_ODR_25HZ:
            odr = 25.0;
            break;
        case ACCEL_ODR_50HZ:
            odr = 50.0;
            break;
        case ACCEL_ODR_100HZ:
            odr = 100.0;
            break;
        case ACCEL_ODR_200HZ:
            odr = 200.0;
            break;
        case ACCEL_ODR_400HZ:
            odr = 400.0;
            break;
        default:
            break;
    }

    switch (accelAvg) {
        case ACCEL_AVG_1:
            avg = 1.0;
            break;
        case ACCEL_AVG_2:
            avg = 2.0;
            break;
        case ACCEL_AVG_4:
            avg = 4.0;
            break;
        case ACCEL_AVG_8:
            avg = 8.0;
            break;
        case ACCEL_AVG_16:
            avg = 16.0;
            break;
        case ACCEL_AVG_32:
            avg = 32.0;
            break;
        case ACCEL_AVG_64:
            avg = 64.0;
            break;
        default:
            break;
    }

    return BMI323::accelSkippedSamplesCheck(odr, avg);
}

/*!
 * @brief Used to check skipped samples for accel
 */
int8_t BMI323::accelSkippedSamplesCheck(float odr, float avg) {
    // Check if odr and avg are valid
    if (odr <= 0.0 || avg <= 0.0) return ERR_ACCEL_INVALID_CFG;

    // Check if skipped samples is above 0
    float skippedSamples = (float)(6400.0 / odr) - avg;
    if (skippedSamples <= 0.0) return ERR_ACCEL_INVALID_CFG;

    return SUCCESS;
}

/*!
 * Sets gyroscope configurations like ODR, bandwidth, gyro mode, average samples and dps range.
 */
int8_t BMI323::setGyroConfig(uint8_t odr, uint8_t bandwidth, uint8_t gyroMode, uint8_t range, uint8_t avgNum) {
    int8_t result;

    // Validate bandwidth, average samples and mode */
    result = BMI323::validateBwAvgGyroMode(&bandwidth, &gyroMode, &avgNum);
    if (result != SUCCESS) return ERR_GYRO_INVALID_CFG;
    
    // Validate odr and range
    result = BMI323::validateGyroOdrRange(&odr, &range);
    if (result != SUCCESS) return ERR_GYRO_INVALID_CFG;

    // Validate odr and avgNum in low power mode
    if (gyroMode == GYRO_MODE_LOW_PWR) {
        result = BMI323::validateGyroOdrAvg(odr, avgNum);
        if (result != SUCCESS) return ERR_GYRO_INVALID_CFG;
    }

    // Prepare register data
    uint16_t odrRegister = BMI323::setBitPos0(0, GYRO_ODR_MASK, odr);
    uint16_t rangeRegister = BMI323::setBits(0, GYRO_RANGE_MASK, GYRO_RANGE_POS, range);
    uint16_t bandwidthRegister = BMI323::setBits(0, GYRO_BW_MASK, GYRO_BW_POS, bandwidth);
    uint16_t avgNumRegister = BMI323::setBits(0, GYRO_AVG_NUM_MASK, GYRO_AVG_NUM_POS, avgNum);
    uint16_t gyroModeRegister = BMI323::setBits(0, GYRO_MODE_MASK, GYRO_MODE_POS, gyroMode);

    uint8_t registerData[2] = {
        (uint8_t)(odrRegister | rangeRegister | bandwidthRegister),
        (uint8_t)((avgNumRegister | gyroModeRegister) >> 8)
    };

    // Set gyro configurations
    result = this->setRegisters(REGISTER_GYRO_CONF, registerData, 2);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/*!
 * @brief Validates bandwidth, average samples and gyr mode of the gyroscope set by the user.
 */
int8_t BMI323::validateBwAvgGyroMode(uint8_t *bandwidth, uint8_t *gyroMode, uint8_t *avgNum) {
    if (bandwidth == nullptr || gyroMode == nullptr || avgNum == nullptr) return ERR_NULL_PTR;

    int8_t result;

    // Validate and auto-correct gyro mode
    result = BMI323::checkBoundaryVal(gyroMode, GYRO_MODE_DISABLE, GYRO_MODE_HIGH_PERF);
    if (result != SUCCESS) return result;

    // Validate for averaging mode
    result = BMI323::checkBoundaryVal(avgNum, GYRO_AVG_1, GYRO_AVG_64);
    if (result != SUCCESS) return result;

    // Validate for bandwidth
    result = BMI323::checkBoundaryVal(bandwidth, GYRO_BANDWIDTH_ODR_HALF, GYRO_BANDWIDTH_ODR_QUARTER);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/*!
 * @brief Validates ODR and range of the gyroscope set by
 * the user.
 */
int8_t BMI323::validateGyroOdrRange(uint8_t *odr, uint8_t *range) {
    if (odr == nullptr || range == nullptr) return ERR_NULL_PTR;

    int8_t result;
    
    // Validate and auto correct ODR
    result = BMI323::checkBoundaryVal(odr, GYRO_ODR_0_78HZ, GYRO_ODR_6400HZ);
    if (result != SUCCESS) return result;

    // Validate and auto correct Range
    result = BMI323::checkBoundaryVal(range, GYRO_RANGE_125DPS, GYRO_RANGE_2000DPS);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/*!
 * @brief Used to validate ODR and AVG combinations for gyro
 */
int8_t BMI323::validateGyroOdrAvg(uint8_t gyroOdr, uint8_t gyroAvg) {

    float odr = 0.0, avg = 0.0;

    switch (gyroOdr)
    {
        case GYRO_ODR_0_78HZ:
            odr = 0.78125;
            break;
        case GYRO_ODR_1_56HZ:
            odr = 1.5625;
            break;
        case GYRO_ODR_3_125HZ:
            odr = 3.125;
            break;
        case GYRO_ODR_6_25HZ:
            odr = 6.25;
            break;
        case GYRO_ODR_12_5HZ:
            odr = 12.5;
            break;
        case GYRO_ODR_25HZ:
            odr = 25.0;
            break;
        case GYRO_ODR_50HZ:
            odr = 50.0;
            break;
        case GYRO_ODR_100HZ:
            odr = 100.0;
            break;
        case GYRO_ODR_200HZ:
            odr = 200.0;
            break;
        case GYRO_ODR_400HZ:
            odr = 400.0;
            break;
        default:
            break;
    }

    switch (gyroAvg)
    {
        case GYRO_AVG_1:
            avg = 1.0;
            break;
        case GYRO_AVG_2:
            avg = 2.0;
            break;
        case GYRO_AVG_4:
            avg = 4.0;
            break;
        case GYRO_AVG_8:
            avg = 8.0;
            break;
        case GYRO_AVG_16:
            avg = 16.0;
            break;
        case GYRO_AVG_32:
            avg = 32.0;
            break;
        case GYRO_AVG_64:
            avg = 64.0;
            break;
        default:
            break;
    }

    return BMI323::gyroSkippedSamplesCheck(odr, avg);
}

/*!
 * @brief Used to check skipped samples for gyro
 */
int8_t BMI323::gyroSkippedSamplesCheck(float odr, float avg) {
    // Check of odr and avg are valid
    if (odr <= 0.0 || avg <= 0.0) return ERR_GYRO_INVALID_CFG;

    // Check if skipped samples is above 0
    float skippedSamples = (float)(6400.0 / odr) - avg;
    if (skippedSamples <= 0.0) return ERR_GYRO_INVALID_CFG;

    return SUCCESS;
}

/*!
 * @brief This API is used to perform the self-calibration for either sensitivity or offset or both.
 */
int8_t BMI323::performGyroCalibration(uint8_t calibrationType, uint8_t applyCalibration, SelfCalibResult *calibrationResult) {
    if (calibrationResult == nullptr) return ERR_NULL_PTR;

    int8_t result;

    struct AccelConfig previousConfig;

    result = getAccelConfig(&previousConfig);
    if (result != SUCCESS) return result;

    result = setAccelConfig(
        ACCEL_ODR_100HZ,
        previousConfig.bandwidth,
        ACCEL_MODE_HIGH_PERF,
        ACCEL_RANGE_8G,
        previousConfig.avgNum
    );
    if (result != SUCCESS) return result;

    // Disable alternate accel and gyro mode
    // rslt = disable_alt_conf_acc_gyr_mode(dev); // TODO Too lazy to implement for now

    // Get and set the self-calibration mode given by the user in the self-calibration dma register.
    // result = this->getSetCalibrationDMA(calibrationType, applyCalibration);
    // if (result != SUCCESS) return result;

    // Trigger the self-calibration command
    result = this->setCommandRegister(COMMAND_SELF_CALIB_TRIGGER);
    if (result != SUCCESS) return result;

    // Get the self-calibration status and result 
    result = this->getGyroCalibrationResult(calibrationResult);
    if (result != SUCCESS) return result;

    // Restore accel configurations
    result = this->setAccelConfig(&previousConfig);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/*!
 * @brief This internal API gets accelerometer configurations like ODR,
 * bandwidth, accel mode, average samples and gravity range.
 */
int8_t BMI323::getAccelConfig(struct AccelConfig *config) {
    if (config == nullptr) return ERR_NULL_PTR;

    int8_t result;

    uint8_t dataArray[2] = { 0 };
    result = this->getRegisters(REGISTER_ACCEL_CONF, dataArray, 2);
    if (result != SUCCESS) return result;
      
    uint16_t registerData = dataArray[0];

    /* Get accelerometer ODR */
    config->odr = BMI323::getBitPos0(registerData, ACCEL_ODR_MASK);

    /* Get accelerometer range */
    config->range = BMI323::getBits(registerData, ACCEL_RANGE_MASK, ACCEL_RANGE_POS);

    /* Get accelerometer bandwidth */
    config->bandwidth = BMI323::getBits(registerData, ACCEL_BW_MASK, ACCEL_BW_POS);
    registerData = (uint16_t)dataArray[1] << 8;

    /* Get accelerometer average samples */
    config->avgNum = BMI323::getBits(registerData, ACCEL_AVG_NUM_MASK, ACCEL_AVG_NUM_POS);

    /* Get accel mode */
    config->accelMode = BMI323::getBits(registerData, ACCEL_MODE_MASK, ACCEL_MODE_POS);
    
    return SUCCESS;
}

uint16_t BMI323::getBitPos0(uint16_t registerData, uint16_t bitMask) {
    return registerData & bitMask;
}

uint16_t BMI323::getBits(uint16_t registerData, uint16_t bitMask, uint16_t bitPos) {
    return (registerData & bitMask) >> bitPos;
}

/**
 * @brief This internal API gets and sets the self-calibration mode given
 *  by the user in the self-calibration dma register.
 */
int8_t BMI323::getSetCalibrationDMA(uint8_t calibrationType, uint8_t applyCalibration) {
    int8_t result;

    // Array to set the base address of self-calibration feature
    uint8_t calibrationBaseAddress[2] = { BASE_ADDR_GYRO_CALIBRATION_SELECT, 0 };

    result = this->setRegisters(REGISTER_FEATURE_DATA_ADDRESS, calibrationBaseAddress, 2);
    if (result != SUCCESS) return result;

    uint8_t registerData[2];
    result = this->getRegisters(REGISTER_FEATURE_DATA_TX, registerData, 2);
    if (result != SUCCESS) return result;

    // The value of apply correction is appended with the selection given by the user
    registerData[0] = (applyCalibration | calibrationType);

    result = this->setRegisters(REGISTER_FEATURE_DATA_ADDRESS, calibrationBaseAddress, 2);
    if (result != SUCCESS) return result;

    result = this->setRegisters(REGISTER_FEATURE_DATA_TX, registerData, 2);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/**
 * @brief Used to get the status of gyro self-calibration and the result of the event
 */
int8_t BMI323::getGyroCalibrationResult(struct SelfCalibResult *calibrationResult) {
    int8_t result;

    calibrationResult->calibrationErrorResult = 0;

    uint8_t idx;
    uint8_t limit = 25;
    for (idx = 0; idx < limit; idx++) {
        /* A delay of 120ms is required to read the error status register */
        this->delayMicroseconds(120000);

        uint8_t data_array[2];
        result = this->getRegisters(REGISTER_FEATURE_IO1, data_array, 2);
        if (result != SUCCESS) return result;

        uint8_t calibrationStatus = (data_array[0] & CALIB_ST_STATUS_MASK) >> CALIB_ST_COMPLETE_POS;
        uint8_t featureEngineErrRegLsb, featureEngineErrRegMsb;
        if ((calibrationStatus == 1) && (result == SUCCESS)) {
            calibrationResult->gyroCalibrationResult = (data_array[0] & GYRO_CALIB_RESULT_MASK) >> GYRO_CALIB_RESULT_POS;
        }

        result = this->getFeatureEngineErrorStatus(&featureEngineErrRegLsb, &featureEngineErrRegMsb);
        if (result != SUCCESS) return result;
        calibrationResult->calibrationErrorResult = featureEngineErrRegLsb;
    }

    return SUCCESS;
}

/*!
 *  @brief Reads the feature engine error status from the sensor.
 */
int8_t BMI323::getFeatureEngineErrorStatus(uint8_t *featureEngineErrRegLsb, uint8_t *featureEngineErrRegMsb) {
    if (featureEngineErrRegLsb == nullptr || featureEngineErrRegMsb == nullptr) return ERR_NULL_PTR;

    int8_t result;

    // Array variable to get error status from register
    uint8_t data[2];
    
    // Read the feature engine error codes
    result = this->getRegisters(REGISTER_FEATURE_IO1, data, 2);
    if (result != SUCCESS) return result;

    *featureEngineErrRegLsb = data[0];
    *featureEngineErrRegMsb = data[1];

    return SUCCESS;
}

/*!
 * @brief Gets the length of FIFO data available in the sensor in bytes
 */
int8_t BMI323::getFifoLength(uint16_t *fifoAvailableLength) {
    if (fifoAvailableLength == nullptr) return ERR_NULL_PTR;

    int8_t result;

    // Array to store FIFO data length
    uint8_t data[LENGTH_FIFO_DATA] = { 0 };

    // Read FIFO length
    result = this->getRegisters(REGISTER_FIFO_FILL_LEVEL, data, 2);
    if (result != SUCCESS) return result;

    // Get the MSB byte of FIFO length
    data[0] = BMI323::getBitPos0(data[0], FIFO_FILL_LEVEL_MASK);
    uint16_t registerData;
    registerData = ((uint16_t)data[1] << 8);
    registerData = BMI323::getBitPos0(registerData, FIFO_FILL_LEVEL_MASK);

    // Get total FIFO length
    *fifoAvailableLength = (uint16_t)(registerData | data[0]) * 2;
    if (*fifoAvailableLength == 0) return WARN_FIFO_EMPTY;

    return SUCCESS;
}

/*!
 * @brief Reads the FIFO data
 */
int8_t BMI323::readFifoData(uint8_t *data, const uint16_t length) {
    if (data == nullptr) return ERR_NULL_PTR;
    if (length == 0) return ERR_INTERFACE_RESULT;

    uint8_t configData[2] = { 0 };
    this->getRegisters(REGISTER_FIFO_CONF, configData, 2);

    return this->read(REGISTER_FIFO_DATA, data, (uint32_t)length);
}

/*!
 * @brief This API gets offset dgain for the sensor which stores self-calibrated values for gyro.
 */
int8_t BMI323::getGyroOffsetGain(uint16_t *offset, uint8_t* gain) {
    if (offset == nullptr || gain == nullptr) return ERR_NULL_PTR;

    int8_t result;

    // Get temperature user offset for gyro
    uint8_t registerData[12];
    result = this->getRegisters(REGISTER_GYRO_DP_OFF_X, registerData, 12);
    if (result != SUCCESS) return result;

    // Offset register
    uint16_t offsetX = (uint16_t)(((uint16_t)registerData[1] << 8) | registerData[0]);
    uint16_t offsetY = (uint16_t)(((uint16_t)registerData[5] << 8) | registerData[4]);
    uint16_t offsetZ = (uint16_t)(((uint16_t)registerData[9] << 8) | registerData[8]);
    
    // Gain register
    uint8_t gainX = registerData[2];
    uint8_t gainY = registerData[6];
    uint8_t gainZ = registerData[10];

    // Offset
    offset[0] = BMI323::getBitPos0(offsetX, GYRO_DP_OFF_MASK);
    offset[1] = BMI323::getBitPos0(offsetY, GYRO_DP_OFF_MASK);
    offset[2] = BMI323::getBitPos0(offsetZ, GYRO_DP_OFF_MASK);

    // Gain
    gain[0] = BMI323::getBitPos0(gainX, GYRO_DP_DGAIN_MASK);
    gain[1] = BMI323::getBitPos0(gainY, GYRO_DP_DGAIN_MASK);
    gain[2] = BMI323::getBitPos0(gainZ, GYRO_DP_DGAIN_MASK);

    return SUCCESS;
}

/*!
 * @brief Returns the sensor temperature in Celcius
 */
int8_t BMI323::getTemperatureCelcius(float *temperature) {
    if (temperature == nullptr) return ERR_NULL_PTR;

    int8_t result;

    // Array to define data stored in register
    uint8_t registerData[2] = { 0 };

    // Read the sensor data
    result = this->getRegisters(REGISTER_TEMP_DATA, registerData, 2);
    if (result != SUCCESS) return result;

    // Parse register data
    uint16_t rawTemperature = (uint16_t)(registerData[0] | ((uint16_t)registerData[1] << 8));

    // Convert to Celcius
    *temperature = (float)(((float)((int16_t)rawTemperature)) / 512.0) + 23.0;

    return SUCCESS;
}