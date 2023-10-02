#include "bmm350_jojos38.h"

/**
 * @brief Entry point for bmm350 sensor. It reads and validates the chip-id of the sensor.
 */
int8_t BMM350::initI2C() {
    int8_t result;
    this->chipId = 0;

    // Assign axis_en with all axis enabled (BMM350_EN_XYZ_MSK)
    this->enabledAxes = EN_XYZ_MASK;

    this->delayMicroseconds(START_UP_TIME_FROM_POR);

    // Set the soft reset command in the command register
    uint8_t command = COMMAND_SOFTRESET;
    result = this->setRegisters(REGISTER_CMD, &command, 1);
    if (result != SUCCESS) return result;

    // Wait for the sensor to come back up
    this->delayMicroseconds(SOFT_RESET_DELAY);

    // Chip ID of the sensor is read
    uint8_t chipId = 0;
    result = this->getRegisters(REGISTER_CHIP_ID, &chipId, 1);
    if (result != SUCCESS) return result;
    this->chipId = chipId;

    // Check the chip id
    if (this->chipId != CHIP_ID) return ERR_DEVICE_NOT_FOUND;

    // Download OTP memory
    result = this->otpDumpAfterBoot();
    if (result != SUCCESS) return result;

    // Power off OTP
    command = OTP_CMD_PWR_OFF_OTP;
    result = this->setRegisters(REGISTER_OTP_CMD_REG, &command, 1);
    if (result != SUCCESS) return result;

    result = this->magneticResetAndWait();
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/**
 * @brief Writes data to the given register address of BMM350 sensor.
 */
int8_t BMM350::setRegisters(uint8_t registerAddress, const uint8_t *data, uint16_t length) {
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
 * @brief Reads the data from the given register address of BMM350 sensor.
 */
int8_t BMM350::getRegisters(uint8_t registerAddress, uint8_t *data, uint16_t length) {
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
 * @brief Used to read OTP data after boot in user mode.
 */
int8_t BMM350::otpDumpAfterBoot() {
    int8_t result;

    uint8_t index;
    uint16_t otpWord = 0;
    for (index = 0; index < OTP_DATA_LENGTH; index++) {
        result = readOtpWord(index, &otpWord);
        this->otpData[index] = otpWord;
    }

    this->variantId = (this->otpData[30] & 0x7f00) >> 9;

    // Update magnetometer offset and sensitivity data.
    this->updateMagOffSens();

    return result;
}

/**
 * @brief Used to read OTP word
 */
int8_t BMM350::readOtpWord(uint8_t address, uint16_t *lsbMsb) {
    if (lsbMsb == nullptr) return ERR_NULL_PTR;

    int8_t result;

    /* Set OTP command at specified address */
    uint8_t otpCommand = OTP_CMD_DIR_READ | (address & OTP_WORD_ADDR_MASK);
    result = this->setRegisters(REGISTER_OTP_CMD_REG, &otpCommand, 1);
    if (result != SUCCESS) return result;
    
    uint8_t otpStatus = 0;
    uint8_t otpError = OTP_STATUS_NO_ERROR;
    do {
        this->delayMicroseconds(300);

        // Get OTP status
        result = this->getRegisters(REGISTER_OTP_STATUS_REG, &otpStatus, 1);
        otpError = otpStatus & OTP_STATUS_ERROR_MASK;
        if (otpError != OTP_STATUS_NO_ERROR) {
            break;
        }
    } while (!(otpStatus & OTP_STATUS_CMD_DONE) && result == SUCCESS);

    if (otpError != OTP_STATUS_NO_ERROR) {
        switch (otpError) {
            case OTP_STATUS_BOOT_ERR:
                result = ERR_OTP_BOOT;
                break;
            case OTP_STATUS_PAGE_RD_ERR:
                result = ERR_OTP_PAGE_RD;
                break;
            case OTP_STATUS_PAGE_PRG_ERR:
                result =ERR_OTP_PAGE_PRG;
                break;
            case OTP_STATUS_SIGN_ERR:
                result = ERR_OTP_SIGN;
                break;
            case OTP_STATUS_INV_CMD_ERR:
                result = ERR_OTP_INV_CMD;
                break;
            default:
                result = ERR_OTP_UNDEFINED;
                break;
        }
    }


    if (result == SUCCESS) {
        // Get OTP MSB data
        uint8_t msb = 0;
        result = this->getRegisters(REGISTER_OTP_DATA_MSB_REG, &msb, 1);
        if (result != SUCCESS) return result;

        /* Get OTP LSB data */
        uint8_t lsb = 0;
        result = this->getRegisters(REGISTER_OTP_DATA_LSB_REG, &lsb, 1);
        *lsbMsb = ((uint16_t)(msb << 8) | lsb) & 0xFFFF;
    }

    return result;
}

/**
 * @brief Used to update magnetometer offset and sensitivity data.
 */
void BMM350::updateMagOffSens() {
    uint16_t offXLsbMsb, offYLsbMsb, offZLsbMsb, tOff;
    uint8_t sensX, sensY, sensZ, tSens;
    uint8_t tcoX, tcoY, tcoZ;
    uint8_t tcsX, tcsY, tcsZ;
    uint8_t crossXY, crossYX, crossZX, crossZY;

    offXLsbMsb = this->otpData[MAG_OFFSET_X] & 0x0FFF;
    offYLsbMsb = ((this->otpData[MAG_OFFSET_X] & 0xF000) >> 4) + (this->otpData[MAG_OFFSET_Y] & LSB_MASK);
    offZLsbMsb = (this->otpData[MAG_OFFSET_Y] & 0x0F00) + (this->otpData[MAG_OFFSET_Z] & LSB_MASK);
    tOff = this->otpData[TEMP_OFF_SENS] & LSB_MASK;

    this->magComp.dutOffsetCoef.offsetX = BMM350::fixSign(offXLsbMsb, SIGNED_12_BIT);
    this->magComp.dutOffsetCoef.offsetY = BMM350::fixSign(offYLsbMsb, SIGNED_12_BIT);
    this->magComp.dutOffsetCoef.offsetZ = BMM350::fixSign(offZLsbMsb, SIGNED_12_BIT);
    this->magComp.dutOffsetCoef.tOffs = BMM350::fixSign(tOff, SIGNED_8_BIT) / 5.0f;

    sensX = (this->otpData[MAG_SENS_X] & MSB_MASK) >> 8;
    sensY = (this->otpData[MAG_SENS_Y] & LSB_MASK);
    sensZ = (this->otpData[MAG_SENS_Z] & MSB_MASK) >> 8;
    tSens = (this->otpData[TEMP_OFF_SENS] & MSB_MASK) >> 8;

    this->magComp.dutSensitCoef.sensX = BMM350::fixSign(sensX, SIGNED_8_BIT) / 256.0f;
    this->magComp.dutSensitCoef.sensY = (BMM350::fixSign(sensY, SIGNED_8_BIT) / 256.0f) + SENS_CORR_Y;
    this->magComp.dutSensitCoef.sensZ = BMM350::fixSign(sensZ, SIGNED_8_BIT) / 256.0f;
    this->magComp.dutSensitCoef.tSens = BMM350::fixSign(tSens, SIGNED_8_BIT) / 512.0f;

    tcoX = (this->otpData[MAG_TCO_X] & LSB_MASK);
    tcoY = (this->otpData[MAG_TCO_Y] & LSB_MASK);
    tcoZ = (this->otpData[MAG_TCO_Z] & LSB_MASK);

    this->magComp.dutTco.tcoX = BMM350::fixSign(tcoX, SIGNED_8_BIT) / 32.0f;
    this->magComp.dutTco.tcoY = BMM350::fixSign(tcoY, SIGNED_8_BIT) / 32.0f;
    this->magComp.dutTco.tcoZ = BMM350::fixSign(tcoZ, SIGNED_8_BIT) / 32.0f;

    tcsX = (this->otpData[MAG_TCS_X] & MSB_MASK) >> 8;
    tcsY = (this->otpData[MAG_TCS_Y] & MSB_MASK) >> 8;
    tcsZ = (this->otpData[MAG_TCS_Z] & MSB_MASK) >> 8;

    this->magComp.dutTcs.tcsX = BMM350::fixSign(tcsX, SIGNED_8_BIT) / 16384.0f;
    this->magComp.dutTcs.tcsY = BMM350::fixSign(tcsY, SIGNED_8_BIT) / 16384.0f;
    this->magComp.dutTcs.tcsZ = (BMM350::fixSign(tcsZ, SIGNED_8_BIT) / 16384.0f) - TCS_CORR_Z;

    this->magComp.dutT0 = (BMM350::fixSign(this->otpData[MAG_DUT_T_0], SIGNED_16_BIT) / 512.0f) + 23.0f;

    crossXY = (this->otpData[CROSS_X_Y] & LSB_MASK);
    crossYX = (this->otpData[CROSS_Y_X] & MSB_MASK) >> 8;
    crossZX = (this->otpData[CROSS_Z_X] & LSB_MASK);
    crossZY = (this->otpData[CROSS_Z_Y] & MSB_MASK) >> 8;

    this->magComp.crossAxis.crossXY = BMM350::fixSign(crossXY, SIGNED_8_BIT) / 800.0f;
    this->magComp.crossAxis.crossYX = BMM350::fixSign(crossYX, SIGNED_8_BIT) / 800.0f;
    this->magComp.crossAxis.crossZX = BMM350::fixSign(crossZX, SIGNED_8_BIT) / 800.0f;
    this->magComp.crossAxis.crossZY = BMM350::fixSign(crossZY, SIGNED_8_BIT) / 800.0f;
}

/**
 *  @brief Converts the raw data from the IC data registers to signed integer
 */
int32_t BMM350::fixSign(uint32_t inval, int8_t bitsCount) {
    int32_t power = 0;
    int32_t retval;

    switch (bitsCount) {
        case SIGNED_8_BIT:
            power = 128; /* 2^7 */
            break;
        case SIGNED_12_BIT:
            power = 2048; /* 2^11 */
            break;
        case SIGNED_16_BIT:
            power = 32768; /* 2^15 */
            break;
        case SIGNED_21_BIT:
            power = 1048576; /* 2^20 */
            break;
        case SIGNED_24_BIT:
            power = 8388608; /* 2^23 */
            break;
        default:
            power = 0;
            break;
    }

    retval = (int32_t)inval;
    if (retval >= power) {
        retval = retval - (power * 2);
    }

    return retval;
}

/**
 * @brief Used to perform the magnetic reset of the sensor
 * which is necessary after a field shock (400mT field applied to sensor).
 * It sends flux guide or bit reset to the device in suspend mode.
 */
int8_t BMM350::magneticResetAndWait() {
    int8_t result;

    uint8_t pmuCmd = 0;
    PmuCmdStatus0 pmuCmdStat0 = { 0 };
    uint8_t restoreNormal = DISABLE;

    result = this->getPmuCmdStatus0(&pmuCmdStat0);
    if (result != SUCCESS) return result;

    // Check the power mode is normal before performing magnetic reset
    if (pmuCmdStat0.pwrModeIsNormal == ENABLE) {
        restoreNormal = ENABLE;

        // Reset can only be triggered in suspend
        result = this->setPowermode(SUSPEND_MODE);
        if (result != SUCCESS) return result;
    }

    // Set BR to PMU_CMD register
    pmuCmd = PMU_CMD_BR;

    result = this->setRegisters(REGISTER_PMU_CMD, &pmuCmd, 1);
    if (result != SUCCESS) return result;
    this->delayMicroseconds(BR_DELAY);

    // Verify if PMU_CMD_STATUS_0 register has BR set
    result = this->getPmuCmdStatus0(&pmuCmdStat0);
    if (result != SUCCESS) return result;
    if (pmuCmdStat0.pmuCmdValue != PMU_CMD_STATUS_0_BR) return ERR_PMU_CMD_VALUE;

    // Set FGR to PMU_CMD register
    pmuCmd = PMU_CMD_FGR;

    result = this->setRegisters(REGISTER_PMU_CMD, &pmuCmd, 1);
    if (result != SUCCESS) return result;
    this->delayMicroseconds(FGR_DELAY);

    // Verify if PMU_CMD_STATUS_0 register has FGR set
    result = this->getPmuCmdStatus0(&pmuCmdStat0);
    if (result != SUCCESS) return result;

    if (pmuCmdStat0.pmuCmdValue != PMU_CMD_STATUS_0_FGR) {
        return ERR_PMU_CMD_VALUE;
    }

    if (restoreNormal == ENABLE) {
        result = this->setPowermode(NORMAL_MODE);
    }

    return result;
}

/**
 * @brief Gets the PMU command status 0 value
 */
int8_t BMM350::getPmuCmdStatus0(struct PmuCmdStatus0 *pmuCmdStat0) {
    if (pmuCmdStat0 == nullptr) return ERR_NULL_PTR;

    int8_t result;

    // Get PMU command status 0 data
    uint8_t regData;
    result = this->getRegisters(REGISTER_PMU_CMD_STATUS_0, &regData, 1);
    if (result != SUCCESS) return result;

    pmuCmdStat0->pmuCmdBusy = this->getBitPos0(regData, PMU_CMD_BUSY_MASK);
    pmuCmdStat0->odrOvwr = this->getBits(regData, ODR_OVWR_MASK, ODR_OVWR_POS);
    pmuCmdStat0->avrOvwr = this->getBits(regData, AVG_OVWR_MASK, AVG_OVWR_POS);
    pmuCmdStat0->pwrModeIsNormal = this->getBits(regData, PWR_MODE_IS_NORMAL_MASK, PWR_MODE_IS_NORMAL_POS);
    pmuCmdStat0->cmdIsIllegal = this->getBits(regData, CMD_IS_ILLEGAL_MASK, CMD_IS_ILLEGAL_POS);
    pmuCmdStat0->pmuCmdValue = this->getBits(regData, PMU_CMD_VALUE_MASK, PMU_CMD_VALUE_POS);

    return SUCCESS;
}

uint16_t BMM350::getBitPos0(uint16_t registerData, uint16_t bitMask) {
    return registerData & bitMask;
}

uint16_t BMM350::getBits(uint16_t registerData, uint16_t bitMask, uint16_t bitPos) {
    return (registerData & bitMask) >> bitPos;
}

uint16_t BMM350::setBits(uint16_t registerData, uint16_t bitMask, uint16_t bitPos, uint16_t data) {
    return (registerData & ~(bitMask)) | ((data << bitPos) & bitMask);
}

/**
 * @brief This API is used to set the power mode of the sensor
 */
int8_t BMM350::setPowermode(enum Powermode powermode)  {    

    int8_t result;

    uint8_t last_pwr_mode;
    uint8_t reg_data;


    result = this->getRegisters(REGISTER_PMU_CMD, &last_pwr_mode, 1);
    if (result != SUCCESS) return result;

    if (last_pwr_mode > PMU_CMD_NM_TC) return ERR_INVALID_CONFIG;

    if (last_pwr_mode == PMU_CMD_NM || last_pwr_mode == PMU_CMD_UPD_OAE) {
        reg_data = PMU_CMD_SUS;

        // Set PMU command configuration
        result = this->setRegisters(REGISTER_PMU_CMD, &reg_data, 1);
        if (result != SUCCESS) return result;

        this->delayMicroseconds(GOTO_SUSPEND_DELAY);
    }

    result = this->switchPowermode(powermode);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/**
 * @brief This internal API is used to switch from suspend mode to normal mode or forced mode.
 */
int8_t BMM350::switchPowermode(enum Powermode powermode) {
    int8_t result;

    uint8_t regData = static_cast<uint8_t>(powermode);
    uint8_t getAvg;

    // Array to store suspend to forced mode delay
    uint32_t susToForcedMode[4] = {
        SUS_TO_FORCED_MODE_NO_AVG_DELAY,
        SUS_TO_FORCED_MODE_AVG_2_DELAY,
        SUS_TO_FORCED_MODE_AVG_4_DELAY,
        SUS_TO_FORCED_MODE_AVG_8_DELAY
    };

    // Array to store suspend to forced mode fast delay
    uint32_t susToForcedModeFast[4] = {
        SUS_TO_FORCED_MODE_FAST_NO_AVG_DELAY,
        SUS_TO_FORCED_MODE_FAST_AVG_2_DELAY,
        SUS_TO_FORCED_MODE_FAST_AVG_4_DELAY,
        SUS_TO_FORCED_MODE_FAST_AVG_8_DELAY
    };

    uint8_t avg = 0;
    uint32_t delayUs = 0;

    // Set PMU command configuration to desired power mode
    result = this->setRegisters(REGISTER_PMU_CMD, &regData, 1);
    if (result != SUCCESS) return result;


    // Get average configuration
    result = this->getRegisters(REGISTER_PMU_CMD_AGGR_SET, &getAvg, 1);
    if (result != SUCCESS) return result;

    // Mask the average value
    avg = ((getAvg & AVG_MASK) >> AVG_POS);
    
    // Check if desired power mode is normal mode
    if (powermode == NORMAL_MODE) {
        delayUs = SUSPEND_TO_NORMAL_DELAY;
    }

    // Check if desired power mode is forced mode
    if (powermode == FORCED_MODE) {
        // Store delay based on averaging mode
        delayUs = susToForcedMode[avg];
    }

    // Check if desired power mode is forced mode fast
    if (powermode == FORCED_MODE_FAST) {
        // Store delay based on averaging mode
        delayUs = susToForcedModeFast[avg];
    }

    // Perform delay based on power mode
    this->delayMicroseconds(delayUs);

    return result;
}

/**
 * @brief This API sets the ODR and averaging factor.
 */
int8_t BMM350::seOdrAvg(uint8_t odr, uint8_t performance) {
    int8_t result;

    uint8_t performance_fix = performance;

    // Reduce the performance setting when too high for the chosen ODR
    if (odr == ODR_400HZ && performance >= AVG_2) {
        performance_fix = AVG_NO_AVG;
    }
    else if (odr == ODR_200HZ && performance >= AVG_4) {
        performance_fix = AVG_2;
    }
    else if (odr == ODR_100HZ && performance >= AVG_8) {
        performance_fix = AVG_4;
    }

    // ODR is an enum taking the generated constants from the register map
    uint8_t regData = 0;
    regData = ((uint8_t)odr & ODR_MSK);

    // AVG / performance is an enum taking the generated constants from the register map
    regData = BMM350::setBits(regData, AVG_MASK, AVG_POS, (uint8_t)performance_fix);

    // Set PMU command configurations for ODR and performance
    result = this->setRegisters(REGISTER_PMU_CMD_AGGR_SET, &regData, 1);
    if (result != SUCCESS) return result;

    // Set PMU command configurations to update odr and average
    regData = PMU_CMD_UPD_OAE;

    // Set PMU command configuration
    result = this->setRegisters(REGISTER_PMU_CMD, &regData, 1);
    if (result != SUCCESS) return result;

    this->delayMicroseconds(UPD_OAE_DELAY);

    return result;
}

/*!
 * @brief Used to enable or disable the data ready interrupt
 */
int8_t BMM350::setInterruptEnabled(uint8_t enableDisable) {
    int8_t result;

    // Get interrupt control configuration
    uint8_t registerData = 0;
    result = this->setRegisters(REGISTER_INT_CTRL, &registerData, 1);
    if (result != SUCCESS) return result;

    registerData = BMM350::setBits(registerData, DRDY_DATA_REG_EN_MASK, DRDY_DATA_REG_EN_POS, (uint8_t)enableDisable);

    // Finally transfer the interrupt configurations
    result = this->setRegisters(REGISTER_INT_CTRL, &registerData, 1);
    if (result != SUCCESS) return result;

    return SUCCESS;
}

/*!
 * @brief This API is used to perform compensation for raw magnetometer and temperature data.
 */
int8_t BMM350::getCompensatedMagData(float *magData) {
    if (magData == nullptr) return ERR_NULL_PTR;

    int8_t result;

    float outData[4] = { 0.0f };
    float dutOffsetCoef[3], dutSensitCoef[3], dutTco[3], dutTcs[3];
    float crAxCompX, crAxCompY, crAxCompZ;

    /* Reads raw magnetic x,y and z axis along with temperature */
    result = this->readOutRawData(outData);
    if (result != SUCCESS) return result;

    /* Apply compensation to temperature reading */
    outData[3] = (1 + this->magComp.dutSensitCoef.tSens) * outData[3] +
                 this->magComp.dutOffsetCoef.tOffs;

    /* Store magnetic compensation structure to an array */
    dutOffsetCoef[0] = this->magComp.dutOffsetCoef.offsetX;
    dutOffsetCoef[1] = this->magComp.dutOffsetCoef.offsetY;
    dutOffsetCoef[2] = this->magComp.dutOffsetCoef.offsetZ;

    dutSensitCoef[0] = this->magComp.dutSensitCoef.sensX;
    dutSensitCoef[1] = this->magComp.dutSensitCoef.sensY;
    dutSensitCoef[2] = this->magComp.dutSensitCoef.sensZ;

    dutTco[0] = this->magComp.dutTco.tcoX;
    dutTco[1] = this->magComp.dutTco.tcoY;
    dutTco[2] = this->magComp.dutTco.tcoZ;

    dutTcs[0] = this->magComp.dutTcs.tcsX;
    dutTcs[1] = this->magComp.dutTcs.tcsY;
    dutTcs[2] = this->magComp.dutTcs.tcsZ;

    /* Compensate raw magnetic data */
    for (uint8_t index = 0; index < 3; index++) {
        outData[index] *= 1 + dutSensitCoef[index];
        outData[index] += dutOffsetCoef[index];
        outData[index] += dutTco[index] * (outData[3] - this->magComp.dutT0);
        outData[index] /= 1 + dutTcs[index] * (outData[3] - this->magComp.dutT0);
    }

    crAxCompX = (outData[0] - this->magComp.crossAxis.crossXY * outData[1]) /
                (1 - this->magComp.crossAxis.crossYX * this->magComp.crossAxis.crossXY);
    crAxCompY = (outData[1] - this->magComp.crossAxis.crossYX * outData[0]) /
                (1 - this->magComp.crossAxis.crossYX * this->magComp.crossAxis.crossXY);
    crAxCompZ =
        (outData[2] +
            (outData[0] *
            (this->magComp.crossAxis.crossYX * this->magComp.crossAxis.crossZY -
            this->magComp.crossAxis.crossZX) - outData[1] *
            (this->magComp.crossAxis.crossZY - this->magComp.crossAxis.crossYX *
            this->magComp.crossAxis.crossZX)) /
            (1 - this->magComp.crossAxis.crossYX * this->magComp.crossAxis.crossXY));

    outData[0] = crAxCompX;
    outData[1] = crAxCompY;
    outData[2] = crAxCompZ;

    if ((this->enabledAxes & EN_X_MASK) == DISABLE) {
        magData[0] = DISABLE;
    } else {
        magData[0] = outData[0];
    }

    if ((this->enabledAxes & EN_Y_MASK) == DISABLE) {
        magData[1] = DISABLE;
    } else {
        magData[1] = outData[1];
    }

    if ((this->enabledAxes & EN_Z_MASK) == DISABLE) {
        magData[2] = DISABLE;
    } else {
        magData[2] = outData[2];
    }

    magData[3] = outData[3];

    return result;
}


/*!
 * @brief This internal API is used to read raw magnetic x,y and z axis along with temperature
 */
int8_t BMM350::readOutRawData(float *outData) {
    if (outData == nullptr) return ERR_NULL_PTR;

    int8_t result;

    float rawData[4] = { 0 };
    result = this->readUncompMagData(rawData);
    if (result != SUCCESS) return result;

    outData[0] = (float)rawData[0] * 0.0070699787f;
    outData[1] = (float)rawData[1] * 0.0070699787f;
    outData[2] = (float)rawData[2] * 0.007174964082f;
    outData[3] = (float)rawData[3] * 0.000981281852f;

    float temp = 0.0;
    if (outData[3] > 0.0) {
        temp = (float)(outData[3] - (1 * 25.49));
    } else if (outData[3] < 0.0) {
        temp = (float)(outData[3] - (-1 * 25.49));
    } else {
        temp = (float)(outData[3]);
    }

    outData[3] = temp;

    return SUCCESS;
}

/*!
 * @brief This API is used to read uncompensated mag and temperature data.
 */
int8_t BMM350::readUncompMagData(float *rawData) {
    if (rawData == nullptr) return ERR_NULL_PTR;

    int8_t result;

    uint8_t magData[12] = { 0 };

    uint32_t rawMagX, rawMagY, rawMagZ, rawTemp;

    // Get uncompensated mag data
    result = this->getRegisters(REGISTER_MAG_X_XLSB, magData, MAG_TEMP_DATA_LEN);
    if (result != SUCCESS) return result;

    rawMagX = magData[0] + ((uint32_t)magData[1] << 8) + ((uint32_t)magData[2] << 16);
    rawMagY = magData[3] + ((uint32_t)magData[4] << 8) + ((uint32_t)magData[5] << 16);
    rawMagZ = magData[6] + ((uint32_t)magData[7] << 8) + ((uint32_t)magData[8] << 16);
    rawTemp = magData[9] + ((uint32_t)magData[10] << 8) + ((uint32_t)magData[11] << 16);

    if ((this->enabledAxes & EN_X_MASK) == DISABLE) {
        rawData[0] = DISABLE;
    } else {
        rawData[0] = BMM350::fixSign(rawMagX, SIGNED_24_BIT);
    }

    if ((this->enabledAxes & EN_Y_MASK) == DISABLE) {
        rawData[1] = DISABLE;
    } else {
        rawData[1] = BMM350::fixSign(rawMagY, SIGNED_24_BIT);
    }

    if ((this->enabledAxes & EN_Z_MASK) == DISABLE) {
        rawData[2] = DISABLE;
    } else {
        rawData[2] = BMM350::fixSign(rawMagZ, SIGNED_24_BIT);
    }

    rawData[3] = BMM350::fixSign(rawTemp, SIGNED_24_BIT);

    return SUCCESS;
}