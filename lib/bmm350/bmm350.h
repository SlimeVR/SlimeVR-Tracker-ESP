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

#ifndef BMM350_H
#define BMM350_H

#include <stdint.h>

typedef int8_t (*ReadFunction)(uint8_t, uint8_t*, uint32_t, void*);
typedef int8_t (*WriteFunction)(uint8_t, const uint8_t*, uint32_t, void *);
typedef void (*DelayFunction)(uint32_t, void *);

class BMM350 {
    
    public:
        /******************************************************************************************
         *                                   PUBLIC STRUCTS
        ******************************************************************************************/
        /**
        * @brief Magnetometer dut offset coefficient structure
        */
        struct DutOffsetCoef {
            // Temperature offset
            float tOffs;

            // Offset x-axis
            float offsetX;

            // Offset y-axis
            float offsetY;

            // Offset z-axis
            float offsetZ;
        };

        /**
        * @brief Magnetometer dut sensitivity coefficient structure
        */
        struct DutSensitCoef {
            // Temperature sensitivity
            float tSens;

            // Sensitivity x-axis
            float sensX;

            // Sensitivity y-axis
            float sensY;

            // Sensitivity z-axis
            float sensZ;
        };

        /**
        * @brief Magnetometer dut tco structure
        */
        struct DutTco {
            float tcoX;
            float tcoY;
            float tcoZ;
        };

        /**
        * @brief Magnetometer dut tcs structure
        */
        struct DutTcs {
            float tcsX;
            float tcsY;
            float tcsZ;
        };

        /**
        * @brief Magnetometer cross-axis compensation structure
        */
        struct CrossAxis {
            float crossXY;
            float crossYX;
            float crossZX;
            float crossZY;
        };

        /**
        * @brief Magnetometer compensation structure
        */
        struct MagCompensate {
            // Structure to store dut offset coefficient
            DutOffsetCoef dutOffsetCoef;

            // Structure to store dut sensitivity coefficient
            DutSensitCoef dutSensitCoef;

            // Structure to store dut tco
            DutTco dutTco;

            // Structure to store dut tcs
            DutTcs dutTcs;

            // Initialize T0_reading parameter
            float dutT0;

            // Structure to define cross-axis compensation
            CrossAxis crossAxis;
        };

        /******************************************************************************************
         *                                   PUBLIC ENUMS
        ******************************************************************************************/
        enum Powermode {
            SUSPEND_MODE     = 0x00, // Replace with the actual values
            NORMAL_MODE      = 0x01,
            FORCED_MODE      = 0x03,
            FORCED_MODE_FAST = 0x04
        };

        /******************************************************************************************
         *                                   PUBLIC CONSTANTS
        ******************************************************************************************/
        // General
        static const uint8_t  CHIP_ID               = UINT8_C(0x33);
        static const uint8_t  ENABLE                = UINT8_C(1);
        static const uint8_t  DISABLE               = UINT8_C(0);
        static const uint8_t  ADDRESS_I2C_PRIMARY   = UINT8_C(0x14);
        static const uint8_t  ADDRESS_I2C_SECONDARY = UINT8_C(0x15);

        // Success
        static const int8_t SUCCESS                  = INT8_C(0);
        static const int8_t SUCCESS_INTERFACE_RESULT = INT8_C(0);

        // Errors
        static const int8_t ERR_NULL_PTR                 = INT8_C(-1);
        static const int8_t ERR_INTERFACE_RESULT         = INT8_C(-2);
        static const int8_t ERR_DEVICE_NOT_FOUND         = INT8_C(-3);
        static const int8_t ERR_INVALID_CONFIG           = INT8_C(-4);
        static const int8_t ERR_PMU_CMD_VALUE            = INT8_C(-16);
        static const int8_t ERR_OTP_BOOT                 = INT8_C(-9);
        static const int8_t ERR_OTP_PAGE_RD              = INT8_C(-10);
        static const int8_t ERR_OTP_PAGE_PRG             = INT8_C(-11);
        static const int8_t ERR_OTP_SIGN                 = INT8_C(-12);
        static const int8_t ERR_OTP_INV_CMD              = INT8_C(-13);
        static const int8_t ERR_OTP_UNDEFINED            = INT8_C(-14);

        /**
         * Choose the number of values the magnometer should average for the last reading.
         * Averaging can help reduce noise and provide a smoother magnometer output.
         * AVG_NO_AVG: No averaging (raw values)
         * AVG_X: Averae over X values
         */
        static const uint8_t AVG_NO_AVG = UINT8_C(0x0);
        static const uint8_t AVG_2 = UINT8_C(0x1);
        static const uint8_t AVG_4 = UINT8_C(0x2);
        static const uint8_t AVG_8 = UINT8_C(0x3);

        /**
         * Magnometer data output rate in Hertz
         * Numbers with underscores should be interpreted as comma
         * ODR_1_5625HZ: 1.5625Hz
         * ODR_100HZ: 100Hz
         * etc...
         */
        static const uint8_t ODR_400HZ = UINT8_C(0x2);
        static const uint8_t ODR_200HZ = UINT8_C(0x3);
        static const uint8_t ODR_100HZ = UINT8_C(0x4);
        static const uint8_t ODR_50HZ = UINT8_C(0x5);
        static const uint8_t ODR_25HZ = UINT8_C(0x6);
        static const uint8_t ODR_12_5HZ = UINT8_C(0x7);
        static const uint8_t ODR_6_25HZ = UINT8_C(0x8);
        static const uint8_t ODR_3_125HZ = UINT8_C(0x9);
        static const uint8_t ODR_1_5625HZ = UINT8_C(0xA);

        // I²2
        static const uint8_t READ_BUFFER_LENGTH = UINT8_C(127);
        static const uint8_t DUMMY_BYTE         = UINT8_C(2);
        static const uint8_t OTP_DATA_LENGTH    = UINT8_C(32);

        /******************************************************************************************
         *                                   CONSTRUCTOR FUNCTIONS
        ******************************************************************************************/
        BMM350(ReadFunction readFunc, WriteFunction writeFunc, DelayFunction delayFunc, void* interfacePtr) :
            readFunction(readFunc),
            writeFunction(writeFunc),
            delayFunction(delayFunc),
            interfacePointer(interfacePtr)
            {};
        ~BMM350() {};

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
        int8_t initI2C();
        int8_t magneticResetAndWait();
        int8_t setPowermode(enum Powermode powermode);
        int8_t seOdrAvg(uint8_t odr, uint8_t avg);
        int8_t setInterruptEnabled(uint8_t enableDisable);
        int8_t getCompensatedMagData(float *magData);

        /******************************************************************************************
         *                                   PUBLIC VARIABLES
        ******************************************************************************************/
        // The chip id of the sensor
        int8_t chipId;
        // Variant id
        uint8_t variantId;
        // The result of the last I²C read or write operation (the return value of the user's function)
        int8_t interfaceResult;
        // The enabled axes
        uint8_t enabledAxes;
        // OTP Data
        uint16_t otpData[OTP_DATA_LENGTH];
        // Compensation parameters
        MagCompensate magComp;

    private:
        /******************************************************************************************
         *                                   PUBLIC STRUCTS
        ******************************************************************************************/
        /**
        * @brief BMM350 PMU command status 0 structure
        */
        struct PmuCmdStatus0 {
            // The previous PMU CMD is still in processing
            uint8_t pmuCmdBusy;

            // The previous PMU_CMD_AGGR_SET.odr has been overwritten
            uint8_t odrOvwr;

            // The previous PMU_CMD_AGGR_SET.avg has been overwritten
            uint8_t avrOvwr;

            // The chip is in normal power mode
            uint8_t pwrModeIsNormal;

            // CMD value is not allowed
            uint8_t cmdIsIllegal;

            // Stores the latest PMU_CMD code processed
            uint8_t pmuCmdValue;
        };

        /******************************************************************************************
         *                                   PRIVATE CONSTANTS
        ******************************************************************************************/  
        // Commands
        static const uint8_t OTP_CMD_PWR_OFF_OTP = UINT8_C(0x80);
        static const uint8_t COMMAND_SOFTRESET   = UINT8_C(0xB6);

        // Data length
        static const uint8_t MAG_TEMP_DATA_LEN = UINT8_C(12);

        // Registers
        static const uint8_t REGISTER_CHIP_ID          = UINT8_C(0x00);
        static const uint8_t REGISTER_REV_ID           = UINT8_C(0x01);
        static const uint8_t REGISTER_OTP_CMD_REG      = UINT8_C(0x50);
        static const uint8_t REGISTER_OTP_STATUS_REG   = UINT8_C(0x55);
        static const uint8_t REGISTER_CMD              = UINT8_C(0x7E);
        static const uint8_t REGISTER_OTP_DATA_MSB_REG = UINT8_C(0x52);
        static const uint8_t REGISTER_OTP_DATA_LSB_REG = UINT8_C(0x53);
        static const uint8_t REGISTER_PMU_CMD_STATUS_0 = UINT8_C(0x07);
        static const uint8_t REGISTER_PMU_CMD          = UINT8_C(0x06);
        static const uint8_t REGISTER_PMU_CMD_AGGR_SET = UINT8_C(0x04);
        static const uint8_t REGISTER_INT_CTRL         = UINT8_C(0x2E);
        static const uint8_t REGISTER_MAG_X_XLSB       = UINT8_C(0x31);

        // Delays
        static const uint32_t SOFT_RESET_DELAY                     = UINT32_C(24000);
        static const uint32_t MAGNETIC_RESET_DELAY                 = UINT32_C(40000);
        static const uint32_t START_UP_TIME_FROM_POR               = UINT32_C(3000);
        static const uint32_t GOTO_SUSPEND_DELAY                   = UINT32_C(6000);
        static const uint32_t SUSPEND_TO_NORMAL_DELAY              = UINT32_C(38000);
        static const uint32_t SUS_TO_FORCED_MODE_NO_AVG_DELAY      = UINT32_C(15000);
        static const uint32_t SUS_TO_FORCED_MODE_AVG_2_DELAY       = UINT32_C(17000);
        static const uint32_t SUS_TO_FORCED_MODE_AVG_4_DELAY       = UINT32_C(20000);
        static const uint32_t SUS_TO_FORCED_MODE_AVG_8_DELAY       = UINT32_C(28000);
        static const uint32_t SUS_TO_FORCED_MODE_FAST_NO_AVG_DELAY = UINT32_C(4000);
        static const uint32_t SUS_TO_FORCED_MODE_FAST_AVG_2_DELAY  = UINT32_C(5000);
        static const uint32_t SUS_TO_FORCED_MODE_FAST_AVG_4_DELAY  = UINT32_C(9000);
        static const uint32_t SUS_TO_FORCED_MODE_FAST_AVG_8_DELAY  = UINT32_C(16000);
        static const uint16_t BR_DELAY                             = UINT16_C(14000);
        static const uint16_t FGR_DELAY                            = UINT16_C(18000);
        static const uint16_t UPD_OAE_DELAY                        = UINT16_C(1000);

        // OTP
        static const uint8_t OTP_CMD_DIR_READ      = UINT8_C(0x20);
        static const uint8_t OTP_WORD_ADDR_MASK    = UINT8_C(0x1F);
        static const uint8_t OTP_STATUS_ERROR_MASK = UINT8_C(0xE0);

        static const uint8_t OTP_STATUS_NO_ERROR     = UINT8_C(0x00);
        static const uint8_t OTP_STATUS_BOOT_ERR     = UINT8_C(0x20);
        static const uint8_t OTP_STATUS_PAGE_RD_ERR  = UINT8_C(0x40);
        static const uint8_t OTP_STATUS_PAGE_PRG_ERR = UINT8_C(0x60);
        static const uint8_t OTP_STATUS_SIGN_ERR     = UINT8_C(0x80);
        static const uint8_t OTP_STATUS_INV_CMD_ERR  = UINT8_C(0xA0);
        static const uint8_t OTP_STATUS_CMD_DONE     = UINT8_C(0x01);

        static const uint8_t PMU_CMD_BUSY_MASK       = UINT8_C(0x1);
        static const uint8_t PMU_CMD_BUSY_POS        = UINT8_C(0x0);
        static const uint8_t ODR_OVWR_MASK           = UINT8_C(0x2);
        static const uint8_t ODR_OVWR_POS            = UINT8_C(0x1);
        static const uint8_t AVG_OVWR_MASK           = UINT8_C(0x4);
        static const uint8_t AVG_OVWR_POS            = UINT8_C(0x2);
        static const uint8_t PWR_MODE_IS_NORMAL_MASK = UINT8_C(0x8);
        static const uint8_t PWR_MODE_IS_NORMAL_POS  = UINT8_C(0x3);
        static const uint8_t CMD_IS_ILLEGAL_MASK     = UINT8_C(0x10);
        static const uint8_t CMD_IS_ILLEGAL_POS      = UINT8_C(0x4);
        static const uint8_t PMU_CMD_VALUE_MASK      = UINT8_C(0xE0);
        static const uint8_t PMU_CMD_VALUE_POS       = UINT8_C(0x5);
        static const uint8_t DRDY_DATA_REG_EN_MASK    = UINT8_C(0x80);
        static const uint8_t DRDY_DATA_REG_EN_POS    = UINT8_C(0x7);

        // Axes
        static const uint8_t EN_XYZ_MASK = UINT8_C(0x7);
        static const uint8_t EN_XYZ_POS  = UINT8_C(0x0);
        static const uint8_t EN_X_MASK   = UINT8_C(0x01);
        static const uint8_t EN_X_POS    = UINT8_C(0x0);
        static const uint8_t EN_Y_MASK   = UINT8_C(0x02);
        static const uint8_t EN_Y_POS    = UINT8_C(0x1);
        static const uint8_t EN_Z_MASK   = UINT8_C(0x04);
        static const uint8_t EN_Z_POS    = UINT8_C(0x2);

        // OTP indices
        static const uint8_t TEMP_OFF_SENS = UINT8_C(0x0D);

        static const uint8_t MAG_OFFSET_X = UINT8_C(0x0E);
        static const uint8_t MAG_OFFSET_Y = UINT8_C(0x0F);
        static const uint8_t MAG_OFFSET_Z = UINT8_C(0x10);

        static const uint8_t MAG_SENS_X = UINT8_C(0x10);
        static const uint8_t MAG_SENS_Y = UINT8_C(0x11);
        static const uint8_t MAG_SENS_Z = UINT8_C(0x11);

        static const uint8_t MAG_TCO_X = UINT8_C(0x12);
        static const uint8_t MAG_TCO_Y = UINT8_C(0x13);
        static const uint8_t MAG_TCO_Z = UINT8_C(0x14);

        static const uint8_t MAG_TCS_X = UINT8_C(0x12);
        static const uint8_t MAG_TCS_Y = UINT8_C(0x13);
        static const uint8_t MAG_TCS_Z = UINT8_C(0x14);

        static const uint8_t MAG_DUT_T_0 = UINT8_C(0x18);

        static const uint8_t CROSS_X_Y = UINT8_C(0x15);
        static const uint8_t CROSS_Y_X = UINT8_C(0x15);
        static const uint8_t CROSS_Z_X = UINT8_C(0x16);
        static const uint8_t CROSS_Z_Y = UINT8_C(0x16);

        static constexpr float SENS_CORR_Y = 0.01f;
        static constexpr float TCS_CORR_Z  = 0.0001f;

        // Other
        static const uint16_t LSB_MASK = UINT16_C(0x00FF);
        static const uint16_t MSB_MASK = UINT16_C(0xFF00);

        static const uint8_t AVG_MASK = UINT8_C(0x30);
        static const uint8_t AVG_POS = UINT8_C(0x4);
        static const uint8_t ODR_MSK = UINT8_C(0xf);
        static const uint8_t ODR_POS = UINT8_C(0x0);

        static const uint8_t SIGNED_8_BIT = UINT8_C(8);
        static const uint8_t SIGNED_12_BIT = UINT8_C(12);
        static const uint8_t SIGNED_16_BIT = UINT8_C(16);
        static const uint8_t SIGNED_21_BIT = UINT8_C(21);
        static const uint8_t SIGNED_24_BIT = UINT8_C(24);

        // PMU
        static const uint8_t PMU_CMD_SUS      = UINT8_C(0x00);
        static const uint8_t PMU_CMD_NM       = UINT8_C(0x01);
        static const uint8_t PMU_CMD_UPD_OAE  = UINT8_C(0x02);
        static const uint8_t PMU_CMD_FM       = UINT8_C(0x03);
        static const uint8_t PMU_CMD_FM_FAST  = UINT8_C(0x04);
        static const uint8_t PMU_CMD_FGR      = UINT8_C(0x05);
        static const uint8_t PMU_CMD_FGR_FAST = UINT8_C(0x06);
        static const uint8_t PMU_CMD_BR       = UINT8_C(0x07);
        static const uint8_t PMU_CMD_BR_FAST  = UINT8_C(0x08);
        static const uint8_t PMU_CMD_NM_TC    = UINT8_C(0x09);

        static const uint8_t PMU_CMD_STATUS_0_SUS = UINT8_C(0x00);
        static const uint8_t PMU_CMD_STATUS_0_NM = UINT8_C(0x01);
        static const uint8_t PMU_CMD_STATUS_0_UPD_OAE = UINT8_C(0x02);
        static const uint8_t PMU_CMD_STATUS_0_FM = UINT8_C(0x03);
        static const uint8_t PMU_CMD_STATUS_0_FM_FAST = UINT8_C(0x04);
        static const uint8_t PMU_CMD_STATUS_0_FGR = UINT8_C(0x05);
        static const uint8_t PMU_CMD_STATUS_0_FGR_FAST = UINT8_C(0x06);
        static const uint8_t PMU_CMD_STATUS_0_BR = UINT8_C(0x07);
        static const uint8_t PMU_CMD_STATUS_0_BR_FAST = UINT8_C(0x07);

        /******************************************************************************************
         *                                  PRIVATE FUNCTIONS
        ******************************************************************************************/
        static int32_t  fixSign(uint32_t inval, int8_t bitsCount);
        static uint16_t getBits(uint16_t registerData, uint16_t bitMask, uint16_t bitPos);
        static uint16_t getBitPos0(uint16_t registerData, uint16_t bitMask);
        static uint16_t setBits(uint16_t registerData, uint16_t bitMask, uint16_t bitPos, uint16_t data);

        int8_t setRegisters(uint8_t registerAddress, const uint8_t *data, uint16_t length);
        int8_t getRegisters(uint8_t registerAddress, uint8_t *data, uint16_t length);
        int8_t otpDumpAfterBoot();
        int8_t readOtpWord(uint8_t address, uint16_t *lsbMsb);
        int8_t getPmuCmdStatus0(struct PmuCmdStatus0 *pmuCmdStat0);
        int8_t switchPowermode(enum Powermode PowerMode);
        int8_t readOutRawData(float *outData);
        int8_t readUncompMagData(float *rawData);
        void   updateMagOffSens();

        /******************************************************************************************
         *                                  PRIVATE VARIABLES
        ******************************************************************************************/
        ReadFunction readFunction;
        WriteFunction writeFunction;
        DelayFunction delayFunction;
        void *interfacePointer;
};
#endif