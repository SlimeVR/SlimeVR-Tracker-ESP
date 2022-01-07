/*

This file contains a useful c translation of the datasheet register map

*/

#ifndef _ICM_20948_REGISTERS_H_
#define _ICM_20948_REGISTERS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

  typedef enum
  {
    // Generalized
    REG_BANK_SEL = 0x7F,

    // Gyroscope and Accelerometer
    // User Bank 0
    AGB0_REG_WHO_AM_I = 0x00,
    AGB0_REG_LPF,
    // Break
    AGB0_REG_USER_CTRL = 0x03,
    // Break
    AGB0_REG_LP_CONFIG = 0x05,
    AGB0_REG_PWR_MGMT_1,
    AGB0_REG_PWR_MGMT_2,
    // Break
    AGB0_REG_INT_PIN_CONFIG = 0x0F,
    AGB0_REG_INT_ENABLE,
    AGB0_REG_INT_ENABLE_1,
    AGB0_REG_INT_ENABLE_2,
    AGB0_REG_INT_ENABLE_3,
    // Break
    AGB0_REG_I2C_MST_STATUS = 0x17,
    AGB0_REG_DMP_INT_STATUS,
    AGB0_REG_INT_STATUS,
    AGB0_REG_INT_STATUS_1,
    AGB0_REG_INT_STATUS_2,
    AGB0_REG_INT_STATUS_3,
    // Break
    AGB0_REG_SINGLE_FIFO_PRIORITY_SEL = 0x26,
    // Break
    AGB0_REG_DELAY_TIMEH = 0x28,
    AGB0_REG_DELAY_TIMEL,
    // Break
    AGB0_REG_ACCEL_XOUT_H = 0x2D,
    AGB0_REG_ACCEL_XOUT_L,
    AGB0_REG_ACCEL_YOUT_H,
    AGB0_REG_ACCEL_YOUT_L,
    AGB0_REG_ACCEL_ZOUT_H,
    AGB0_REG_ACCEL_ZOUT_L,
    AGB0_REG_GYRO_XOUT_H,
    AGB0_REG_GYRO_XOUT_L,
    AGB0_REG_GYRO_YOUT_H,
    AGB0_REG_GYRO_YOUT_L,
    AGB0_REG_GYRO_ZOUT_H,
    AGB0_REG_GYRO_ZOUT_L,
    AGB0_REG_TEMP_OUT_H,
    AGB0_REG_TEMP_OUT_L,
    AGB0_REG_EXT_PERIPH_SENS_DATA_00,
    AGB0_REG_EXT_PERIPH_SENS_DATA_01,
    AGB0_REG_EXT_PERIPH_SENS_DATA_02,
    AGB0_REG_EXT_PERIPH_SENS_DATA_03,
    AGB0_REG_EXT_PERIPH_SENS_DATA_04,
    AGB0_REG_EXT_PERIPH_SENS_DATA_05,
    AGB0_REG_EXT_PERIPH_SENS_DATA_06,
    AGB0_REG_EXT_PERIPH_SENS_DATA_07,
    AGB0_REG_EXT_PERIPH_SENS_DATA_08,
    AGB0_REG_EXT_PERIPH_SENS_DATA_09,
    AGB0_REG_EXT_PERIPH_SENS_DATA_10,
    AGB0_REG_EXT_PERIPH_SENS_DATA_11,
    AGB0_REG_EXT_PERIPH_SENS_DATA_12,
    AGB0_REG_EXT_PERIPH_SENS_DATA_13,
    AGB0_REG_EXT_PERIPH_SENS_DATA_14,
    AGB0_REG_EXT_PERIPH_SENS_DATA_15,
    AGB0_REG_EXT_PERIPH_SENS_DATA_16,
    AGB0_REG_EXT_PERIPH_SENS_DATA_17,
    AGB0_REG_EXT_PERIPH_SENS_DATA_18,
    AGB0_REG_EXT_PERIPH_SENS_DATA_19,
    AGB0_REG_EXT_PERIPH_SENS_DATA_20,
    AGB0_REG_EXT_PERIPH_SENS_DATA_21,
    AGB0_REG_EXT_PERIPH_SENS_DATA_22,
    AGB0_REG_EXT_PERIPH_SENS_DATA_23,
    // Break
    AGB0_REG_TEMP_CONFIG = 0x53,
    // Break
    AGB0_REG_FIFO_EN_1 = 0x66,
    AGB0_REG_FIFO_EN_2,
    AGB0_REG_FIFO_RST,
    AGB0_REG_FIFO_MODE,
    // Break
    AGB0_REG_FIFO_COUNT_H = 0x70,
    AGB0_REG_FIFO_COUNT_L,
    AGB0_REG_FIFO_R_W,
    // Break
    AGB0_REG_DATA_RDY_STATUS = 0x74,
    AGB0_REG_HW_FIX_DISABLE,
    AGB0_REG_FIFO_CFG,
    // Break
    AGB0_REG_MEM_START_ADDR = 0x7C, // Hmm, Invensense thought they were sneaky not listing these locations on the datasheet...
    AGB0_REG_MEM_R_W = 0x7D,        // These three locations seem to be able to access some memory within the device
    AGB0_REG_MEM_BANK_SEL = 0x7E,   // And that location is also where the DMP image gets loaded
    AGB0_REG_REG_BANK_SEL = 0x7F,

    // Bank 1
    AGB1_REG_SELF_TEST_X_GYRO = 0x02,
    AGB1_REG_SELF_TEST_Y_GYRO,
    AGB1_REG_SELF_TEST_Z_GYRO,
    // Break
    AGB1_REG_SELF_TEST_X_ACCEL = 0x0E,
    AGB1_REG_SELF_TEST_Y_ACCEL,
    AGB1_REG_SELF_TEST_Z_ACCEL,
    // Break
    AGB1_REG_XA_OFFS_H = 0x14,
    AGB1_REG_XA_OFFS_L,
    // Break
    AGB1_REG_YA_OFFS_H = 0x17,
    AGB1_REG_YA_OFFS_L,
    // Break
    AGB1_REG_ZA_OFFS_H = 0x1A,
    AGB1_REG_ZA_OFFS_L,
    // Break
    AGB1_REG_TIMEBASE_CORRECTION_PLL = 0x28,
    // Break
    AGB1_REG_REG_BANK_SEL = 0x7F,

    // Bank 2
    AGB2_REG_GYRO_SMPLRT_DIV = 0x00,
    AGB2_REG_GYRO_CONFIG_1,
    AGB2_REG_GYRO_CONFIG_2,
    AGB2_REG_XG_OFFS_USRH,
    AGB2_REG_XG_OFFS_USRL,
    AGB2_REG_YG_OFFS_USRH,
    AGB2_REG_YG_OFFS_USRL,
    AGB2_REG_ZG_OFFS_USRH,
    AGB2_REG_ZG_OFFS_USRL,
    AGB2_REG_ODR_ALIGN_EN,
    // Break
    AGB2_REG_ACCEL_SMPLRT_DIV_1 = 0x10,
    AGB2_REG_ACCEL_SMPLRT_DIV_2,
    AGB2_REG_ACCEL_INTEL_CTRL,
    AGB2_REG_ACCEL_WOM_THR,
    AGB2_REG_ACCEL_CONFIG,
    AGB2_REG_ACCEL_CONFIG_2,
    // Break
    AGB2_REG_PRS_ODR_CONFIG = 0x20,
    // Break
    AGB2_REG_PRGM_START_ADDRH = 0x50,
    AGB2_REG_PRGM_START_ADDRL,
    AGB2_REG_FSYNC_CONFIG,
    AGB2_REG_TEMP_CONFIG,
    AGB2_REG_MOD_CTRL_USR,
    // Break
    AGB2_REG_REG_BANK_SEL = 0x7F,

    // Bank 3
    AGB3_REG_I2C_MST_ODR_CONFIG = 0x00,
    AGB3_REG_I2C_MST_CTRL,
    AGB3_REG_I2C_MST_DELAY_CTRL,
    AGB3_REG_I2C_PERIPH0_ADDR,
    AGB3_REG_I2C_PERIPH0_REG,
    AGB3_REG_I2C_PERIPH0_CTRL,
    AGB3_REG_I2C_PERIPH0_DO,
    AGB3_REG_I2C_PERIPH1_ADDR,
    AGB3_REG_I2C_PERIPH1_REG,
    AGB3_REG_I2C_PERIPH1_CTRL,
    AGB3_REG_I2C_PERIPH1_DO,
    AGB3_REG_I2C_PERIPH2_ADDR,
    AGB3_REG_I2C_PERIPH2_REG,
    AGB3_REG_I2C_PERIPH2_CTRL,
    AGB3_REG_I2C_PERIPH2_DO,
    AGB3_REG_I2C_PERIPH3_ADDR,
    AGB3_REG_I2C_PERIPH3_REG,
    AGB3_REG_I2C_PERIPH3_CTRL,
    AGB3_REG_I2C_PERIPH3_DO,
    AGB3_REG_I2C_PERIPH4_ADDR,
    AGB3_REG_I2C_PERIPH4_REG,
    AGB3_REG_I2C_PERIPH4_CTRL,
    AGB3_REG_I2C_PERIPH4_DO,
    AGB3_REG_I2C_PERIPH4_DI,
    // Break
    AGB3_REG_REG_BANK_SEL = 0x7F,

    // Magnetometer
    M_REG_WIA2 = 0x01,
    // Break
    M_REG_ST1 = 0x10,
    M_REG_HXL,
    M_REG_HXH,
    M_REG_HYL,
    M_REG_HYH,
    M_REG_HZL,
    M_REG_HZH,
    M_REG_ST2,
    // Break
    M_REG_CNTL2 = 0x31,
    M_REG_CNTL3,
    M_REG_TS1,
    M_REG_TS2,
  } ICM_20948_Reg_Addr_e; // These enums are not needed for the user, so they stay in this scope (simplifies naming among other things)

  // Type definitions for the registers
  typedef struct
  {
    uint8_t WHO_AM_I;
  } ICM_20948_WHO_AM_I_t;

  typedef struct
  {
    uint8_t reserved_0 : 1;
    uint8_t I2C_MST_RST : 1;
    uint8_t SRAM_RST : 1;
    uint8_t DMP_RST : 1;
    uint8_t I2C_IF_DIS : 1;
    uint8_t I2C_MST_EN : 1;
    uint8_t FIFO_EN : 1;
    uint8_t DMP_EN : 1;
  } ICM_20948_USER_CTRL_t;

  typedef struct
  {
    uint8_t reserved_0 : 4;
    uint8_t GYRO_CYCLE : 1;
    uint8_t ACCEL_CYCLE : 1;
    uint8_t I2C_MST_CYCLE : 1;
    uint8_t reserved_1 : 1;
  } ICM_20948_LP_CONFIG_t;

  typedef struct
  {
    uint8_t CLKSEL : 3;
    uint8_t TEMP_DIS : 1;
    uint8_t reserved_0 : 1;
    uint8_t LP_EN : 1;
    uint8_t SLEEP : 1;
    uint8_t DEVICE_RESET : 1;
  } ICM_20948_PWR_MGMT_1_t;

  typedef struct
  {
    uint8_t DISABLE_GYRO : 3;
    uint8_t DIABLE_ACCEL : 3;
    uint8_t reserved_0 : 2;
  } ICM_20948_PWR_MGMT_2_t;

  typedef struct
  {
    uint8_t reserved_0 : 1;
    uint8_t BYPASS_EN : 1;
    uint8_t FSYNC_INT_MODE_EN : 1;
    uint8_t ACTL_FSYNC : 1;
    uint8_t INT_ANYRD_2CLEAR : 1;
    uint8_t INT1_LATCH_EN : 1;
    uint8_t INT1_OPEN : 1;
    uint8_t INT1_ACTL : 1;
  } ICM_20948_INT_PIN_CFG_t;

  typedef struct
  {
    uint8_t I2C_MST_INT_EN : 1;
    uint8_t DMP_INT1_EN : 1;
    uint8_t PLL_READY_EN : 1;
    uint8_t WOM_INT_EN : 1;
    uint8_t reserved_0 : 3;
    uint8_t REG_WOF_EN : 1;
  } ICM_20948_INT_ENABLE_t;

  typedef struct
  {
    uint8_t RAW_DATA_0_RDY_EN : 1;
    uint8_t reserved_0 : 7;
  } ICM_20948_INT_ENABLE_1_t;

  typedef union
  {
    struct
    {
      uint8_t FIFO_OVERFLOW_EN_40 : 5;
      uint8_t reserved_0 : 3;
    } grouped;
    struct
    {
      uint8_t FIFO_OVERFLOW_EN_0 : 1;
      uint8_t FIFO_OVERFLOW_EN_1 : 1;
      uint8_t FIFO_OVERFLOW_EN_2 : 1;
      uint8_t FIFO_OVERFLOW_EN_3 : 1;
      uint8_t FIFO_OVERFLOW_EN_4 : 1;
      uint8_t reserved_0 : 3;
    } individual;
  } ICM_20948_INT_ENABLE_2_t;

  // typedef struct{
  // 	uint8_t FIFO_OVERFLOW_EN_40		: 5;
  // 	uint8_t reserved_0				: 3;
  // }ICM_20948_INT_ENABLE_2_t;

  typedef union
  {
    struct
    {
      uint8_t FIFO_WM_EN_40 : 5;
      uint8_t reserved_0 : 3;
    } grouped;
    struct
    {
      uint8_t FIFO_WM_EN_0 : 1;
      uint8_t FIFO_WM_EN_1 : 1;
      uint8_t FIFO_WM_EN_2 : 1;
      uint8_t FIFO_WM_EN_3 : 1;
      uint8_t FIFO_WM_EN_4 : 1;
      uint8_t reserved_0 : 3;
    } individual;
  } ICM_20948_INT_ENABLE_3_t;

  // typedef struct{
  // 	uint8_t FIFO_WM_EN_40			: 5;
  // 	uint8_t reserved_0				: 3;
  // }ICM_20948_INT_ENABLE_3_t;

  typedef struct
  {
    uint8_t I2C_PERIPH0_NACK : 1;
    uint8_t I2C_PERIPH1_NACK : 1;
    uint8_t I2C_PERIPH2_NACK : 1;
    uint8_t I2C_PERIPH3_NACK : 1;
    uint8_t I2C_PERIPH4_NACK : 1;
    uint8_t I2C_LOST_ARB : 1;
    uint8_t I2C_PERIPH4_DONE : 1;
    uint8_t PASS_THROUGH : 1;
  } ICM_20948_I2C_MST_STATUS_t;

  typedef struct
  {
    uint8_t reserved0 : 1;
    uint8_t DMP_INT_Motion_Detect_SMD : 1;
    uint8_t reserved1 : 1;
    uint8_t DMP_INT_Tilt_Event : 1;
    uint8_t reserved2 : 4;
  } ICM_20948_DMP_INT_STATUS_t; // Mostly guesswork from InvenSense App Note

  typedef struct
  {
    uint8_t I2C_MST_INT : 1;
    uint8_t DMP_INT1 : 1;
    uint8_t PLL_RDY_INT : 1;
    uint8_t WOM_INT : 1;
    uint8_t reserved_0 : 4;
  } ICM_20948_INT_STATUS_t;

  typedef struct
  {
    uint8_t RAW_DATA_0_RDY_INT : 1;
    uint8_t reserved_0 : 7;
  } ICM_20948_INT_STATUS_1_t;

  // typedef union{
  // 	struct{
  // 		uint8_t FIFO_OVERFLOW_INT_40	: 5;
  // 		uint8_t reserved_0				: 3;
  // 	}grouped;
  // 	struct{
  // 		uint8_t FIFO_OVERFLOW_INT_0 	: 1;
  // 		uint8_t FIFO_OVERFLOW_INT_1 	: 1;
  // 		uint8_t FIFO_OVERFLOW_INT_2 	: 1;
  // 		uint8_t FIFO_OVERFLOW_INT_3 	: 1;
  // 		uint8_t FIFO_OVERFLOW_INT_4 	: 1;
  // 		uint8_t reserved_0				: 3;
  // 	}individual;
  // }ICM_20948_INT_STATUS_2_t;

  typedef struct
  {
    uint8_t FIFO_OVERFLOW_INT_40 : 5;
    uint8_t reserved_0 : 3;
  } ICM_20948_INT_STATUS_2_t;

  // typedef union{
  // 	struct{
  // 		uint8_t FIFO_WM_INT_40	: 5;
  // 		uint8_t reserved_0				: 3;
  // 	}grouped;
  // 	struct{
  // 		uint8_t FIFO_WM_INT_0 	: 1;
  // 		uint8_t FIFO_WM_INT_1 	: 1;
  // 		uint8_t FIFO_WM_INT_2 	: 1;
  // 		uint8_t FIFO_WM_INT_3 	: 1;
  // 		uint8_t FIFO_WM_INT_4 	: 1;
  // 		uint8_t reserved_0				: 3;
  // 	}individual;
  // }ICM_20948_INT_STATUS_3_t;

  typedef struct
  {
    uint8_t FIFO_WM_INT40 : 5;
    uint8_t reserved_0 : 3;
  } ICM_20948_INT_STATUS_3_t;

  typedef struct
  {
    uint8_t DELAY_TIMEH;
  } ICM_20948_DELAY_TIMEH_t;

  typedef struct
  {
    uint8_t DELAY_TIMEL;
  } ICM_20948_DELAY_TIMEL_t;

  typedef struct
  {
    uint8_t ACCEL_XOUT_H;
  } ICM_20948_ACCEL_XOUT_H_t;

  typedef struct
  {
    uint8_t ACCEL_XOUT_L;
  } ICM_20948_ACCEL_XOUT_L_t;

  typedef struct
  {
    uint8_t ACCEL_YOUT_H;
  } ICM_20948_ACCEL_YOUT_H_t;

  typedef struct
  {
    uint8_t ACCEL_YOUT_L;
  } ICM_20948_ACCEL_YOUT_L_t;

  typedef struct
  {
    uint8_t ACCEL_ZOUT_H;
  } ICM_20948_ACCEL_ZOUT_H_t;

  typedef struct
  {
    uint8_t ACCEL_ZOUT_L;
  } ICM_20948_ACCEL_ZOUT_L_t;

  typedef struct
  {
    uint8_t GYRO_XOUT_H;
  } ICM_20948_GYRO_XOUT_H_t;

  typedef struct
  {
    uint8_t GYRO_XOUT_L;
  } ICM_20948_GYRO_XOUT_L_t;

  typedef struct
  {
    uint8_t GYRO_YOUT_H;
  } ICM_20948_GYRO_YOUT_H_t;

  typedef struct
  {
    uint8_t GYRO_YOUT_L;
  } ICM_20948_GYRO_YOUT_L_t;

  typedef struct
  {
    uint8_t GYRO_ZOUT_H;
  } ICM_20948_GYRO_ZOUT_H_t;

  typedef struct
  {
    uint8_t GYRO_ZOUT_L;
  } ICM_20948_GYRO_ZOUT_L_t;

  typedef struct
  {
    uint8_t TEMP_OUT_H;
  } ICM_20948_TEMP_OUT_H_t;

  typedef struct
  {
    uint8_t TEMP_OUT_L;
  } ICM_20948_TEMP_OUT_L_t;

  typedef struct
  {
    uint8_t DATA; // Note: this is not worth copying 24 times, despite there being 24 registers like this one
  } ICM_20948_EXT_PERIPH_SENS_DATA_t;

  typedef struct
  {
    uint8_t PERIPH_0_FIFO_EN : 1;
    uint8_t PERIPH_1_FIFO_EN : 1;
    uint8_t PERIPH_2_FIFO_EN : 1;
    uint8_t PERIPH_3_FIFO_EN : 1;
    uint8_t reserved_0 : 4;
  } ICM_20948_FIFO_EN_1_t;

  typedef struct
  {
    uint8_t TEMP_FIFO_EN : 1;
    uint8_t GYRO_X_FIFO_EN : 1;
    uint8_t GYRO_Y_FIFO_EN : 1;
    uint8_t GYRO_Z_FIFO_EN : 1;
    uint8_t ACCEL_FIFO_EN : 1;
    uint8_t reserved_0 : 3;
  } ICM_20948_FIFO_EN_2_t;

  typedef struct
  {
    uint8_t FIFO_RESET : 5;
    uint8_t reserved_0 : 3;
  } ICM_20948_FIFO_RST_t;

  typedef struct
  {
    uint8_t FIFO_MODE : 5;
    uint8_t reserved_0 : 3;
  } ICM_20948_FIFO_MODE_t;

  typedef struct
  {
    uint8_t FIFO_COUNTH;
  } ICM_20948_FIFO_COUNTH_t;

  typedef struct
  {
    uint8_t FIFO_COUNTL;
  } ICM_20948_FIFO_COUNTL_t;

  typedef struct
  {
    uint8_t RAW_DATA_RDY : 4;
    uint8_t reserved_0 : 3;
    uint8_t WOF_STATUS : 1;
  } ICM_20948_DATA_RDY_STATUS_t;

  typedef struct
  {
    uint8_t FIFO_CFG : 1;
    uint8_t reserved_0 : 7;
  } ICM_20948_FIFO_CFG_t;

  // User bank 1 Types

  typedef struct
  {
    uint8_t XG_ST_DATA;
  } ICM_20948_SELF_TEST_X_GYRO_t;

  typedef struct
  {
    uint8_t YG_ST_DATA;
  } ICM_20948_SELF_TEST_Y_GYRO_t;

  typedef struct
  {
    uint8_t ZG_ST_DATA;
  } ICM_20948_SELF_TEST_Z_GYRO_t;

  typedef struct
  {
    uint8_t XA_ST_DATA;
  } ICM_20948_SELF_TEST_X_ACCEL_t;

  typedef struct
  {
    uint8_t YA_ST_DATA;
  } ICM_20948_SELF_TEST_Y_ACCEL_t;

  typedef struct
  {
    uint8_t ZA_ST_DATA;
  } ICM_20948_SELF_TEST_Z_ACCEL_t;

  typedef struct
  {
    uint8_t XA_OFFS_14_7;
  } ICM_20948_XA_OFFS_H_t;

  typedef struct
  {
    uint8_t reserved_0 : 1;
    uint8_t XA_OFFS_6_0 : 7;
  } ICM_20948_XA_OFFS_L_t;

  typedef struct
  {
    uint8_t YA_OFFS_14_7;
  } ICM_20948_YA_OFFS_H_t;

  typedef struct
  {
    uint8_t reserved_0 : 1;
    uint8_t YA_OFFS_6_0 : 7;
  } ICM_20948_YA_OFFS_L_t;

  typedef struct
  {
    uint8_t ZA_OFFS_14_7;
  } ICM_20948_ZA_OFFS_H_t;

  typedef struct
  {
    uint8_t reserved_0 : 1;
    uint8_t ZA_OFFS_6_0 : 7;
  } ICM_20948_ZA_OFFS_L_t;

  typedef struct
  {
    uint8_t TBC_PLL;
  } ICM_20948_TIMEBASE_CORRECTION_PLL_t;

  // User Bank 2 Types
  typedef struct
  {
    uint8_t GYRO_SMPLRT_DIV;
  } ICM_20948_GYRO_SMPLRT_DIV_t;

  typedef struct
  {
    uint8_t GYRO_FCHOICE : 1;
    uint8_t GYRO_FS_SEL : 2;
    uint8_t GYRO_DLPFCFG : 3;
    uint8_t reserved_0 : 2;
  } ICM_20948_GYRO_CONFIG_1_t;

  typedef struct
  {
    uint8_t GYRO_AVGCFG : 3;
    uint8_t ZGYRO_CTEN : 1;
    uint8_t YGYRO_CTEN : 1;
    uint8_t XGYRO_CTEN : 1;
    uint8_t reserved_0 : 2;
  } ICM_20948_GYRO_CONFIG_2_t;

  typedef struct
  {
    uint8_t XG_OFFS_USER_H;
  } ICM_20948_XG_OFFS_USRH_t;

  typedef struct
  {
    uint8_t XG_OFFS_USER_L;
  } ICM_20948_XG_OFFS_USRL_t;

  typedef struct
  {
    uint8_t YG_OFFS_USER_H;
  } ICM_20948_YG_OFFS_USRH_t;

  typedef struct
  {
    uint8_t YG_OFFS_USER_L;
  } ICM_20948_YG_OFFS_USRL_t;

  typedef struct
  {
    uint8_t ZG_OFFS_USER_H;
  } ICM_20948_ZG_OFFS_USRH_t;

  typedef struct
  {
    uint8_t ZG_OFFS_USER_L;
  } ICM_20948_ZG_OFFS_USRL_t;

  typedef struct
  {
    uint8_t ODR_ALIGN_EN : 1;
    uint8_t reserved_0 : 7;
  } ICM_20948_ODR_ALIGN_EN_t;

  typedef struct
  {
    uint8_t ACCEL_SMPLRT_DIV_11_8 : 4;
    uint8_t reserved_0 : 4;
  } ICM_20948_ACCEL_SMPLRT_DIV_1_t;

  typedef struct
  {
    uint8_t ACCEL_SMPLRT_DIV_7_0;
  } ICM_20948_ACCEL_SMPLRT_DIV_2_t;

  typedef struct
  {
    uint8_t ACCEL_INTEL_MODE_INT : 1;
    uint8_t ACCEL_INTEL_EN : 1;
    uint8_t reserved_0 : 6;
  } ICM_20948_ACCEL_INTEL_CTRL_t;

  typedef struct
  {
    uint8_t WOM_THRESHOLD;
  } ICM_20948_ACCEL_WOM_THR_t;

  typedef struct
  {
    uint8_t ACCEL_FCHOICE : 1;
    uint8_t ACCEL_FS_SEL : 2;
    uint8_t ACCEL_DLPFCFG : 3;
    uint8_t reserved_0 : 2;
  } ICM_20948_ACCEL_CONFIG_t;

  typedef struct
  {
    uint8_t DEC3_CFG : 2;
    uint8_t AZ_ST_EN : 1;
    uint8_t AY_ST_EN : 1;
    uint8_t AX_ST_EN : 1;
    uint8_t reserved_0 : 3;
  } ICM_20948_ACCEL_CONFIG_2_t;

  typedef struct
  {
    uint8_t EXT_SYNC_SET : 4;
    uint8_t WOF_EDGE_INT : 1;
    uint8_t WOF_DEGLITCH_EN : 1;
    uint8_t reserved_0 : 1;
    uint8_t DELAY_TIME_EN : 1;
  } ICM_20948_FSYNC_CONFIG_t;

  typedef struct
  {
    uint8_t TEMP_DLPFCFG : 3;
    uint8_t reserved_0 : 5;
  } ICM_20948_TEMP_CONFIG_t;

  typedef struct
  {
    uint8_t REG_LP_DMP_EN : 1;
    uint8_t reserved_0 : 7;
  } ICM_20948_MOD_CTRL_USR_t;

  // Bank 3 Types

  typedef struct
  {
    uint8_t I2C_MST_ODR_CONFIG : 4;
    uint8_t reserved_0 : 4;
  } ICM_20948_I2C_MST_ODR_CONFIG_t;

  typedef struct
  {
    uint8_t I2C_MST_CLK : 4;
    uint8_t I2C_MST_P_NSR : 1;
    uint8_t reserved_0 : 2;
    uint8_t MULT_MST_EN : 1;
  } ICM_20948_I2C_MST_CTRL_t;

  typedef struct
  {
    uint8_t I2C_PERIPH0_DELAY_EN : 1;
    uint8_t I2C_PERIPH1_DELAY_EN : 1;
    uint8_t I2C_PERIPH2_DELAY_EN : 1;
    uint8_t I2C_PERIPH3_DELAY_EN : 1;
    uint8_t I2C_PERIPH4_DELAY_EN : 1;
    uint8_t reserved_0 : 2;
    uint8_t DELAY_ES_SHADOW : 1;
  } ICM_20948_I2C_MST_DELAY_CTRL_t;

  typedef struct
  {
    uint8_t ID : 7;
    uint8_t RNW : 1;
  } ICM_20948_I2C_PERIPHX_ADDR_t;

  typedef struct
  {
    uint8_t REG;
  } ICM_20948_I2C_PERIPHX_REG_t;

  typedef struct
  {
    uint8_t LENG : 4;
    uint8_t GRP : 1;
    uint8_t REG_DIS : 1;
    uint8_t BYTE_SW : 1;
    uint8_t EN : 1;
  } ICM_20948_I2C_PERIPHX_CTRL_t;

  typedef struct
  {
    uint8_t DO;
  } ICM_20948_I2C_PERIPHX_DO_t;

  typedef struct
  {
    uint8_t DLY : 5;
    uint8_t REG_DIS : 1;
    uint8_t INT_EN : 1;
    uint8_t EN : 1;
  } ICM_20948_I2C_PERIPH4_CTRL_t;

  typedef struct
  {
    uint8_t DI;
  } ICM_20948_I2C_PERIPH4_DI_t;

  // Bank select register!

  typedef struct
  {
    uint8_t reserved_0 : 4;
    uint8_t USER_BANK : 2;
    uint8_t reserved_1 : 2;
  } ICM_20948_REG_BANK_SEL_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ICM_20948_REGISTERS_H_ */
