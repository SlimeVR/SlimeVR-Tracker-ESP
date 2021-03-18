/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bno055.h
* @date 10/01/2020
* @version  2.0.6
*
*/

/*! \file bno055.h
 * \brief BNO055 Sensor Driver Support Header File */

#ifndef __BNO055_H__
#define __BNO055_H__

/****************************************************************/
/**\name    DATA TYPES INCLUDES     */
/************************************************************/

/*!
 * @brief The following definition uses for define the data types
 *
 * @note While porting the API please consider the following
 * @note Please check the version of C standard
 * @note Are you using Linux platform
 */

/*!
 * @brief For the Linux platform support
 * Please use the types.h for your data types definitions
 */
#ifdef  __KERNEL__

#include <linux/types.h>

/* singed integer type*/
typedef int8_t s8; /**< used for signed 8bit */
typedef int16_t s16; /**< used for signed 16bit */
typedef int32_t s32; /**< used for signed 32bit */
typedef int64_t s64; /**< used for signed 64bit */
typedef u_int8_t u8; /**< used for unsigned 8bit */
typedef u_int16_t u16; /**< used for unsigned 16bit */
typedef u_int32_t u32; /**< used for unsigned 32bit */
typedef u_int64_t u64; /**< used for unsigned 64bit */

#else /* ! __KERNEL__ */

/**********************************************************
 * These definition uses for define the C
 * standard version data types
 ***********************************************************/
# if !defined(__STDC_VERSION__)

/************************************************
* compiler is C11 C standard
************************************************/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>

/************************************************/
/*unsigned integer types*/
typedef uint8_t u8; /**< used for unsigned 8bit */
typedef uint16_t u16; /**< used for unsigned 16bit */
typedef uint32_t u32; /**< used for unsigned 32bit */
typedef uint64_t u64; /**< used for unsigned 64bit */
/*signed integer types*/
typedef int8_t s8; /**< used for signed 8bit */
typedef int16_t s16; /**< used for signed 16bit */
typedef int32_t s32; /**< used for signed 32bit */
typedef int64_t s64; /**< used for signed 64bit */

/************************************************
* compiler is C99 C standard
************************************************/

#elif (__STDC_VERSION__ == 199901L)

/* stdint.h is a C99 supported c library.
 *  which is used to fixed the integer size*/

/************************************************/
#include <stdint.h>

/************************************************/
/*unsigned integer types*/
typedef uint8_t u8; /**< used for unsigned 8bit */
typedef uint16_t u16; /**< used for unsigned 16bit */
typedef uint32_t u32; /**< used for unsigned 32bit */
typedef uint64_t u64; /**< used for unsigned 64bit */
/*signed integer types*/
typedef int8_t s8; /**< used for signed 8bit */
typedef int16_t s16; /**< used for signed 16bit */
typedef int32_t s32; /**< used for signed 32bit */
typedef int64_t s64; /**< used for signed 64bit */

/************************************************
* compiler is C89 or other C standard
************************************************/

#else /*  !defined(__STDC_VERSION__) */

/*!
 * @brief By default it is defined as 32 bit machine configuration
 *  define your data types based on your
 *  machine/compiler/controller configuration
 */
#define  MACHINE_32_BIT

/*! @brief
 *  If your machine support 16 bit
 *  define the MACHINE_16_BIT
 */
#ifdef MACHINE_16_BIT
#include <limits.h>

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed long int s32; /**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int s64; /**< used for signed 64bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int s64; /**< used for signed 64bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
    could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
    please set s64 manually.
#endif

/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned long int u32; /**< used for unsigned 32bit */

/* If your machine support 32 bit
 * define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32; /**< used for signed 32bit */
typedef signed long long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32; /**< used for unsigned 32bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */

/* If your machine support 64 bit
 * define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32; /**< used for signed 32bit */
typedef signed long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32; /**< used for unsigned 32bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */

#else
#warning The data types defined above which not supported \
    define the data types manually
#endif
#endif

/*** This else will execute for the compilers
 *  which are not supported the C standards
 *  Like C89/C99/C11***/
#else

/*!
 * @brief By default it is defined as 32 bit machine configuration
 *  define your data types based on your
 *  machine/compiler/controller configuration
 */
#define  MACHINE_32_BIT

/* If your machine support 16 bit
 * define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed long int s32; /**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int s64; /**< used for signed 64bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int s64; /**< used for signed 64bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
    could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
    please set s64 manually.
#endif

/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned long int u32; /**< used for unsigned 32bit */

/*! @brief If your machine support 32 bit
 * define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32; /**< used for signed 32bit */
typedef signed long long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32; /**< used for unsigned 32bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */

/* If your machine support 64 bit
 * define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32; /**< used for signed 32bit */
typedef signed long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32; /**< used for unsigned 32bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */

#else
#warning The data types defined above which not supported \
    define the data types manually
#endif
#endif
#endif

/***************************************************************/
/**\name    BUS READ AND WRITE FUNCTIONS           */
/***************************************************************/
#define BNO055_WR_FUNC_PTR       s8 (*bus_write) \
        (u8, u8, u8 *, u8)

#define BNO055_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len) \
    bus_write(dev_addr, reg_addr, reg_data, wr_len)

#define BNO055_RD_FUNC_PTR       s8 \
    (*bus_read)(u8, u8, u8 *, u8)

#define BNO055_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len) \
    bus_read(dev_addr, reg_addr, reg_data, r_len)

#define BNO055_DELAY_RETURN_TYPE void

#define BNO055_DELAY_PARAM_TYPES u32

#define BNO055_DELAY_FUNC(delay_in_msec) \
    delay_func(delay_in_msec)

/********************************************************/
/**\name    I2C ADDRESS DEFINITION FOR BNO055           */
/********************************************************/
/* bno055 I2C Address */
#define BNO055_I2C_ADDR1 (0x28)
#define BNO055_I2C_ADDR2 (0x29)

/***************************************************/
/**\name    REGISTER ADDRESS DEFINITION  */
/***************************************************/
/* Page id register definition*/
#define BNO055_PAGE_ID_ADDR                 (0X07)

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                 (0x00)
#define BNO055_ACCEL_REV_ID_ADDR            (0x01)
#define BNO055_MAG_REV_ID_ADDR              (0x02)
#define BNO055_GYRO_REV_ID_ADDR             (0x03)
#define BNO055_SW_REV_ID_LSB_ADDR           (0x04)
#define BNO055_SW_REV_ID_MSB_ADDR           (0x05)
#define BNO055_BL_REV_ID_ADDR               (0X06)

/* Accel data register*/
#define BNO055_ACCEL_DATA_X_LSB_ADDR        (0X08)
#define BNO055_ACCEL_DATA_X_MSB_ADDR        (0X09)
#define BNO055_ACCEL_DATA_Y_LSB_ADDR        (0X0A)
#define BNO055_ACCEL_DATA_Y_MSB_ADDR        (0X0B)
#define BNO055_ACCEL_DATA_Z_LSB_ADDR        (0X0C)
#define BNO055_ACCEL_DATA_Z_MSB_ADDR        (0X0D)

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR          (0X0E)
#define BNO055_MAG_DATA_X_MSB_ADDR          (0X0F)
#define BNO055_MAG_DATA_Y_LSB_ADDR          (0X10)
#define BNO055_MAG_DATA_Y_MSB_ADDR          (0X11)
#define BNO055_MAG_DATA_Z_LSB_ADDR          (0X12)
#define BNO055_MAG_DATA_Z_MSB_ADDR          (0X13)

/*Gyro data registers*/
#define BNO055_GYRO_DATA_X_LSB_ADDR         (0X14)
#define BNO055_GYRO_DATA_X_MSB_ADDR         (0X15)
#define BNO055_GYRO_DATA_Y_LSB_ADDR         (0X16)
#define BNO055_GYRO_DATA_Y_MSB_ADDR         (0X17)
#define BNO055_GYRO_DATA_Z_LSB_ADDR         (0X18)
#define BNO055_GYRO_DATA_Z_MSB_ADDR         (0X19)

/*Euler data registers*/
#define BNO055_EULER_H_LSB_ADDR             (0X1A)
#define BNO055_EULER_H_MSB_ADDR             (0X1B)

#define BNO055_EULER_R_LSB_ADDR             (0X1C)
#define BNO055_EULER_R_MSB_ADDR             (0X1D)

#define BNO055_EULER_P_LSB_ADDR             (0X1E)
#define BNO055_EULER_P_MSB_ADDR             (0X1F)

/*Quaternion data registers*/
#define BNO055_QUATERNION_DATA_W_LSB_ADDR   (0X20)
#define BNO055_QUATERNION_DATA_W_MSB_ADDR   (0X21)
#define BNO055_QUATERNION_DATA_X_LSB_ADDR   (0X22)
#define BNO055_QUATERNION_DATA_X_MSB_ADDR   (0X23)
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR   (0X24)
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR   (0X25)
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR   (0X26)
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR   (0X27)

/* Linear acceleration data registers*/
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR (0X28)
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR (0X29)
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR (0X2A)
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR (0X2B)
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR (0X2C)
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR (0X2D)

/*Gravity data registers*/
#define BNO055_GRAVITY_DATA_X_LSB_ADDR      (0X2E)
#define BNO055_GRAVITY_DATA_X_MSB_ADDR      (0X2F)
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR      (0X30)
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR      (0X31)
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR      (0X32)
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR      (0X33)

/* Temperature data register*/
#define BNO055_TEMP_ADDR                    (0X34)

/* Status registers*/
#define BNO055_CALIB_STAT_ADDR              (0X35)
#define BNO055_SELFTEST_RESULT_ADDR         (0X36)
#define BNO055_INTR_STAT_ADDR               (0X37)
#define BNO055_SYS_CLK_STAT_ADDR            (0X38)
#define BNO055_SYS_STAT_ADDR                (0X39)
#define BNO055_SYS_ERR_ADDR                 (0X3A)

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR                (0X3B)
#define BNO055_DATA_SELECT_ADDR             (0X3C)

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR                (0X3D)
#define BNO055_PWR_MODE_ADDR                (0X3E)

#define BNO055_SYS_TRIGGER_ADDR             (0X3F)
#define BNO055_TEMP_SOURCE_ADDR             (0X40)

/* Axis remap registers*/
#define BNO055_AXIS_MAP_CONFIG_ADDR         (0X41)
#define BNO055_AXIS_MAP_SIGN_ADDR           (0X42)

/* SIC registers*/
#define BNO055_SIC_MATRIX_0_LSB_ADDR        (0X43)
#define BNO055_SIC_MATRIX_0_MSB_ADDR        (0X44)
#define BNO055_SIC_MATRIX_1_LSB_ADDR        (0X45)
#define BNO055_SIC_MATRIX_1_MSB_ADDR        (0X46)
#define BNO055_SIC_MATRIX_2_LSB_ADDR        (0X47)
#define BNO055_SIC_MATRIX_2_MSB_ADDR        (0X48)
#define BNO055_SIC_MATRIX_3_LSB_ADDR        (0X49)
#define BNO055_SIC_MATRIX_3_MSB_ADDR        (0X4A)
#define BNO055_SIC_MATRIX_4_LSB_ADDR        (0X4B)
#define BNO055_SIC_MATRIX_4_MSB_ADDR        (0X4C)
#define BNO055_SIC_MATRIX_5_LSB_ADDR        (0X4D)
#define BNO055_SIC_MATRIX_5_MSB_ADDR        (0X4E)
#define BNO055_SIC_MATRIX_6_LSB_ADDR        (0X4F)
#define BNO055_SIC_MATRIX_6_MSB_ADDR        (0X50)
#define BNO055_SIC_MATRIX_7_LSB_ADDR        (0X51)
#define BNO055_SIC_MATRIX_7_MSB_ADDR        (0X52)
#define BNO055_SIC_MATRIX_8_LSB_ADDR        (0X53)
#define BNO055_SIC_MATRIX_8_MSB_ADDR        (0X54)

/* Accelerometer Offset registers*/
#define BNO055_ACCEL_OFFSET_X_LSB_ADDR      (0X55)
#define BNO055_ACCEL_OFFSET_X_MSB_ADDR      (0X56)
#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR      (0X57)
#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR      (0X58)
#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR      (0X59)
#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR      (0X5A)

/* Magnetometer Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB_ADDR        (0X5B)
#define BNO055_MAG_OFFSET_X_MSB_ADDR        (0X5C)
#define BNO055_MAG_OFFSET_Y_LSB_ADDR        (0X5D)
#define BNO055_MAG_OFFSET_Y_MSB_ADDR        (0X5E)
#define BNO055_MAG_OFFSET_Z_LSB_ADDR        (0X5F)
#define BNO055_MAG_OFFSET_Z_MSB_ADDR        (0X60)

/* Gyroscope Offset registers*/
#define BNO055_GYRO_OFFSET_X_LSB_ADDR       (0X61)
#define BNO055_GYRO_OFFSET_X_MSB_ADDR       (0X62)
#define BNO055_GYRO_OFFSET_Y_LSB_ADDR       (0X63)
#define BNO055_GYRO_OFFSET_Y_MSB_ADDR       (0X64)
#define BNO055_GYRO_OFFSET_Z_LSB_ADDR       (0X65)
#define BNO055_GYRO_OFFSET_Z_MSB_ADDR       (0X66)

/* Radius registers*/
#define BNO055_ACCEL_RADIUS_LSB_ADDR        (0X67)
#define BNO055_ACCEL_RADIUS_MSB_ADDR        (0X68)
#define BNO055_MAG_RADIUS_LSB_ADDR          (0X69)
#define BNO055_MAG_RADIUS_MSB_ADDR          (0X6A)

/* PAGE0 REGISTERS DEFINITION END*/
/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define BNO055_ACCEL_CONFIG_ADDR            (0X08)
#define BNO055_MAG_CONFIG_ADDR              (0X09)
#define BNO055_GYRO_CONFIG_ADDR             (0X0A)
#define BNO055_GYRO_MODE_CONFIG_ADDR        (0X0B)
#define BNO055_ACCEL_SLEEP_CONFIG_ADDR      (0X0C)
#define BNO055_GYRO_SLEEP_CONFIG_ADDR       (0X0D)
#define BNO055_MAG_SLEEP_CONFIG_ADDR        (0x0E)

/* Interrupt registers*/
#define BNO055_INT_MASK_ADDR                (0X0F)
#define BNO055_INT_ADDR                     (0X10)
#define BNO055_ACCEL_ANY_MOTION_THRES_ADDR  (0X11)
#define BNO055_ACCEL_INTR_SETTINGS_ADDR     (0X12)
#define BNO055_ACCEL_HIGH_G_DURN_ADDR       (0X13)
#define BNO055_ACCEL_HIGH_G_THRES_ADDR      (0X14)
#define BNO055_ACCEL_NO_MOTION_THRES_ADDR   (0X15)
#define BNO055_ACCEL_NO_MOTION_SET_ADDR     (0X16)
#define BNO055_GYRO_INTR_SETING_ADDR        (0X17)
#define BNO055_GYRO_HIGHRATE_X_SET_ADDR     (0X18)
#define BNO055_GYRO_DURN_X_ADDR             (0X19)
#define BNO055_GYRO_HIGHRATE_Y_SET_ADDR     (0X1A)
#define BNO055_GYRO_DURN_Y_ADDR             (0X1B)
#define BNO055_GYRO_HIGHRATE_Z_SET_ADDR     (0X1C)
#define BNO055_GYRO_DURN_Z_ADDR             (0X1D)
#define BNO055_GYRO_ANY_MOTION_THRES_ADDR   (0X1E)
#define BNO055_GYRO_ANY_MOTION_SET_ADDR     (0X1F)

/* PAGE1 REGISTERS DEFINITION END*/

#define BNO055_MDELAY_DATA_TYPE             u32

/*< This refers BNO055 return type as s8 */
#define BNO055_RETURN_FUNCTION_TYPE         s8

/* Compile switch definition for Float and double*/
#define BNO055_FLOAT_ENABLE
#define BNO055_DOUBLE_ENABLE

/**************************************************************/
/**\name    STRUCTURE DEFINITIONS                         */
/**************************************************************/

/*!
 *  @brief bno055 struct
 */
struct bno055_t
{
    u8 chip_id; /**< chip_id of bno055 */
    u16 sw_rev_id; /**< software revision id of bno055 */
    u8 page_id; /**< page_id of bno055 */
    u8 accel_rev_id; /**< accel revision id of bno055 */
    u8 mag_rev_id; /**< mag revision id of bno055 */
    u8 gyro_rev_id; /**< gyro revision id of bno055 */
    u8 bl_rev_id; /**< boot loader revision id of bno055 */
    u8 dev_addr; /**< i2c device address of bno055 */
    BNO055_WR_FUNC_PTR; /**< bus write function pointer */
    BNO055_RD_FUNC_PTR; /**<bus read function pointer */
    void (*delay_msec)(BNO055_MDELAY_DATA_TYPE); /**< delay function pointer */
};

/*!
 * @brief struct for accel data read from registers
 */
struct bno055_accel_t
{
    s16 x; /**< accel x data */
    s16 y; /**< accel y data */
    s16 z; /**< accel z data */
};

/*!
 * @brief struct for Mag data read from registers
 */
struct bno055_mag_t
{
    s16 x; /**< mag x data */
    s16 y; /**< mag y data */
    s16 z; /**< mag z data */
};

/*!
 * @brief struct for Gyro data read from registers
 */
struct bno055_gyro_t
{
    s16 x; /**< gyro x data */
    s16 y; /**< gyro y data */
    s16 z; /**< gyro z data */
};

/*!
 * @brief struct for Euler data read from registers
 */
struct bno055_euler_t
{
    s16 h; /**< Euler h data */
    s16 r; /**< Euler r data */
    s16 p; /**< Euler p data */
};

/*!
 * @brief struct for Quaternion data read from registers
 */
struct bno055_quaternion_t
{
    s16 w; /**< Quaternion w data */
    s16 x; /**< Quaternion x data */
    s16 y; /**< Quaternion y data */
    s16 z; /**< Quaternion z data */
};

/*!
 * @brief struct for Linear Accel data read from registers
 */
struct bno055_linear_accel_t
{
    s16 x; /**< Linear Accel x data */
    s16 y; /**< Linear Accel y data */
    s16 z; /**< Linear Accel z data */
};

/*!
 * @brief struct for Gravity data read from registers
 */
struct bno055_gravity_t
{
    s16 x; /**< Gravity x data */
    s16 y; /**< Gravity y data */
    s16 z; /**< Gravity z data */
};
#ifdef  BNO055_DOUBLE_ENABLE

/*!
 * @brief struct for Accel-output data of precision double
 */
struct bno055_accel_double_t
{
    double x; /**< Accel x double data */
    double y; /**< Accel y double data */
    double z; /**< Accel z double data */
};

/*!
 * @brief struct for Mag-output data of precision double
 */
struct bno055_mag_double_t
{
    double x; /**< Mag x double data */
    double y; /**< Mag y double data */
    double z; /**< Mag z double data */
};

/*!
 * @brief struct for Gyro-output data of precision double
 */
struct bno055_gyro_double_t
{
    double x; /**< Gyro x double data */
    double y; /**< Gyro y double data */
    double z; /**< Gyro z double data */
};

/*!
 * @brief struct for Euler-output data of precision double
 */
struct bno055_euler_double_t
{
    double h; /**< Euler h double data */
    double r; /**< Euler r double data */
    double p; /**< Euler p double data */
};

/*!
 * @brief struct for Linear Accel-output data of precision double
 */
struct bno055_linear_accel_double_t
{
    double x; /**< linear accel x double data */
    double y; /**< linear accel y double data */
    double z; /**< linear accel z double data */
};

/*!
 * @brief struct for Gravity-output data of precision double
 */
struct bno055_gravity_double_t
{
    double x; /**< Gravity x double data */
    double y; /**< Gravity y double data */
    double z; /**< Gravity z double data */
};
#endif
#ifdef  BNO055_FLOAT_ENABLE

/*!
 * @brief struct for Accel-output data of precision float
 */
struct bno055_accel_float_t
{
    float x; /**< accel x float data */
    float y; /**< accel y float data */
    float z; /**< accel z float data */
};

/*!
 * @brief struct for Mag-output data of precision float
 */
struct bno055_mag_float_t
{
    float x; /**< Mag x float data */
    float y; /**< Mag y float data */
    float z; /**< Mag z float data */
};

/*!
 * @brief struct for Gyro-output data of precision float
 */
struct bno055_gyro_float_t
{
    float x; /**< Gyro x float data */
    float y; /**< Gyro y float data */
    float z; /**< Gyro z float data */
};

/*!
 * @brief struct for Euler-output data of precision float
 */
struct bno055_euler_float_t
{
    float h; /**< Euler h float data */
    float r; /**< Euler r float data */
    float p; /**< Euler p float data */
};

/*!
 * @brief struct for Linear accel-output data of precision float
 */
struct bno055_linear_accel_float_t
{
    float x; /**< Linear accel x float data */
    float y; /**< Linear accel y float data */
    float z; /**< Linear accel z float data */
};

/*!
 * @brief struct for Gravity-output data of precision float
 */
struct bno055_gravity_float_t
{
    float x; /**< Gravity x float data */
    float y; /**< Gravity y float data */
    float z; /**< Gravity z float data */
};
#endif

/*!
 * @brief struct for Accel offset
 */
struct bno055_accel_offset_t
{
    s16 x; /**< Accel offset x data */
    s16 y; /**< Accel offset y data */
    s16 z; /**< Accel offset z data */
    s16 r; /**< Accel radius r data */
};

/*!
 * @brief struct for Gyro offset
 */
struct bno055_gyro_offset_t
{
    s16 x; /**< Gyro offset x data */
    s16 y; /**< Gyro offset y data */
    s16 z; /**< Gyro offset z data */
};

/*!
 * @brief struct for Mag offset
 */
struct bno055_mag_offset_t
{
    s16 x; /**< Mag offset x data */
    s16 y; /**< Mag offset y data */
    s16 z; /**< Mag offset z data */
    s16 r; /**< Mag radius x data */
};

/*!
 * @brief struct for soft iron calibration matrix
 */
struct bno055_sic_matrix_t
{
    s16 sic_0; /**< soft iron calibration matrix 0 data */
    s16 sic_1; /**< soft iron calibration matrix 1 data */
    s16 sic_2; /**< soft iron calibration matrix 2 data */
    s16 sic_3; /**< soft iron calibration matrix 3 data */
    s16 sic_4; /**< soft iron calibration matrix 4 data */
    s16 sic_5; /**< soft iron calibration matrix 5 data */
    s16 sic_6; /**< soft iron calibration matrix 6 data */
    s16 sic_7; /**< soft iron calibration matrix 7 data */
    s16 sic_8; /**< soft iron calibration matrix 8 data */
};

/***************************************************/
/**\name    CONSTANT DEFINITIONS                   */
/***************************************************/
#define  BNO055_INIT_VALUE                         ((u8)0)
#define  BNO055_GEN_READ_WRITE_LENGTH              ((u8)1)
#define  BNO055_LSB_MSB_READ_LENGTH                ((u8)2)
#define  BNO055_MAG_POWER_MODE_RANGE               ((u8)4)
#define  BNO055_MAG_OPR_MODE_RANGE                 ((u8)5)
#define  BNO055_ACCEL_POWER_MODE_RANGE             ((u8)6)
#define  BNO055_ACCEL_SLEEP_DURATION_RANGE         ((u8)16)
#define  BNO055_GYRO_AUTO_SLEEP_DURATION_RANGE     ((u8)8)
#define  BNO055_ACCEL_GYRO_BW_RANGE                ((u8)8)
#define  BNO055_MAG_OUTPUT_RANGE                   ((u8)8)
#define  BNO055_ACCEL_RANGE                        ((u8)5)
#define  BNO055_SHIFT_EIGHT_BITS                   ((u8)8)
#define  BNO055_GYRO_RANGE                         ((u8)5)
#define  BNO055_ACCEL_SLEEP_MODE_RANGE             ((u8)2)

/*  BNO055 API BNO055_ERROR codes */
#define BNO055_E_NULL_PTR                          ((s8) - 127)
#define BNO055_OUT_OF_RANGE                        ((s8) - 2)
#define BNO055_SUCCESS                             ((u8)0)
#define BNO055_ERROR                               ((s8) - 1)

/* Selection for bit enable and disable */
#define BNO055_BIT_ENABLE                          (0x01)
#define BNO055_BIT_DISABLE                         (0x00)

/* Page ID */
#define BNO055_PAGE_ZERO                           (0X00)
#define BNO055_PAGE_ONE                            (0X01)

/* Enable the temperature source */
#define BNO055_ACCEL_TEMP_EN                       (0x00)
#define BNO055_GYRO_TEMP_EN                        (0x01)
#define BNO055_MCU_TEMP_EN                         (0x03)

/*Accel unit*/
#define BNO055_ACCEL_UNIT_MSQ                      (0x00)
#define BNO055_ACCEL_UNIT_MG                       (0x01)

/*Gyro unit*/
#define BNO055_GYRO_UNIT_DPS                       (0x00)
#define BNO055_GYRO_UNIT_RPS                       (0x01)

/* Euler unit*/
#define BNO055_EULER_UNIT_DEG                      (0x00)
#define BNO055_EULER_UNIT_RAD                      (0x01)

/*Temperature unit*/
#define BNO055_TEMP_UNIT_CELSIUS                   (0x00)
#define BNO055_TEMP_UNIT_FAHRENHEIT                (0x01)

/*Accel division factor*/
#define BNO055_ACCEL_DIV_MSQ                       (100.0)
#define BNO055_ACCEL_DIV_MG                        (1.0)

/*Mag division factor*/
#define BNO055_MAG_DIV_UT                          (16.0)

/*Gyro division factor*/
#define BNO055_GYRO_DIV_DPS                        (16.0)
#define BNO055_GYRO_DIV_RPS                        (900.0)

/*Euler division factor*/
#define BNO055_EULER_DIV_DEG                       (16.0)
#define BNO055_EULER_DIV_RAD                       (900.0)

/*Linear accel division factor*/
#define BNO055_LINEAR_ACCEL_DIV_MSQ                (100.0)

/*Gravity accel division factor*/
#define BNO055_GRAVITY_DIV_MSQ                     (100.0)

/* Temperature division factor*/
#define BNO055_TEMP_DIV_FAHRENHEIT                 (0.5)
#define BNO055_TEMP_DIV_CELSIUS                    (1.0)

#define BNO055_MODE_SWITCHING_DELAY                (600)
#define BNO055_CONFIG_MODE_SWITCHING_DELAY         ((u8)20)

/* Operation mode settings*/
#define BNO055_OPERATION_MODE_CONFIG               (0X00)
#define BNO055_OPERATION_MODE_ACCONLY              (0X01)
#define BNO055_OPERATION_MODE_MAGONLY              (0X02)
#define BNO055_OPERATION_MODE_GYRONLY              (0X03)
#define BNO055_OPERATION_MODE_ACCMAG               (0X04)
#define BNO055_OPERATION_MODE_ACCGYRO              (0X05)
#define BNO055_OPERATION_MODE_MAGGYRO              (0X06)
#define BNO055_OPERATION_MODE_AMG                  (0X07)
#define BNO055_OPERATION_MODE_IMUPLUS              (0X08)
#define BNO055_OPERATION_MODE_COMPASS              (0X09)
#define BNO055_OPERATION_MODE_M4G                  (0X0A)
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF         (0X0B)
#define BNO055_OPERATION_MODE_NDOF                 (0X0C)

/* Power mode*/
#define BNO055_POWER_MODE_NORMAL                   (0X00)
#define BNO055_POWER_MODE_LOWPOWER                 (0X01)
#define BNO055_POWER_MODE_SUSPEND                  (0X02)

/* PAGE-1 definitions*/
/* Accel Range */

#define BNO055_ACCEL_RANGE_2G                      (0X00)
#define BNO055_ACCEL_RANGE_4G                      (0X01)
#define BNO055_ACCEL_RANGE_8G                      (0X02)
#define BNO055_ACCEL_RANGE_16G                     (0X03)

/* Accel Bandwidth*/
#define BNO055_ACCEL_BW_7_81HZ                     (0x00)
#define BNO055_ACCEL_BW_15_63HZ                    (0x01)
#define BNO055_ACCEL_BW_31_25HZ                    (0x02)
#define BNO055_ACCEL_BW_62_5HZ                     (0X03)
#define BNO055_ACCEL_BW_125HZ                      (0X04)
#define BNO055_ACCEL_BW_250HZ                      (0X05)
#define BNO055_ACCEL_BW_500HZ                      (0X06)
#define BNO055_ACCEL_BW_1000HZ                     (0X07)

/* Accel Power mode*/
#define BNO055_ACCEL_NORMAL                        (0X00)
#define BNO055_ACCEL_SUSPEND                       (0X01)
#define BNO055_ACCEL_LOWPOWER_1                    (0X02)
#define BNO055_ACCEL_STANDBY                       (0X03)
#define BNO055_ACCEL_LOWPOWER_2                    (0X04)
#define BNO055_ACCEL_DEEPSUSPEND                   (0X05)

/* Mag data output rate*/
#define BNO055_MAG_DATA_OUTRATE_2HZ                (0X00)
#define BNO055_MAG_DATA_OUTRATE_6HZ                (0X01)
#define BNO055_MAG_DATA_OUTRATE_8HZ                (0X02)
#define BNO055_MAG_DATA_OUTRATE_10HZ               (0X03)
#define BNO055_MAG_DATA_OUTRATE_15HZ               (0X04)
#define BNO055_MAG_DATA_OUTRATE_20HZ               (0X05)
#define BNO055_MAG_DATA_OUTRATE_25HZ               (0X06)
#define BNO055_MAG_DATA_OUTRATE_30HZ               (0X07)

/* Mag Operation mode*/
#define BNO055_MAG_OPERATION_MODE_LOWPOWER         (0X00)
#define BNO055_MAG_OPERATION_MODE_REGULAR          (0X01)
#define BNO055_MAG_OPERATION_MODE_ENHANCED_REGULAR (0X02)
#define BNO055_MAG_OPERATION_MODE_HIGH_ACCURACY    (0X03)

/* Mag power mode*/
#define BNO055_MAG_POWER_MODE_NORMAL               (0X00)
#define BNO055_MAG_POWER_MODE_SLEEP                (0X01)
#define BNO055_MAG_POWER_MODE_SUSPEND              (0X02)
#define BNO055_MAG_POWER_MODE_FORCE_MODE           (0X03)

/* Gyro range*/
#define BNO055_GYRO_RANGE_2000DPS                  (0x00)
#define BNO055_GYRO_RANGE_1000DPS                  (0x01)
#define BNO055_GYRO_RANGE_500DPS                   (0x02)
#define BNO055_GYRO_RANGE_250DPS                   (0x03)
#define BNO055_GYRO_RANGE_125DPS                   (0x04)

/* Gyro Bandwidth*/
#define BNO055_GYRO_BW_523HZ                       (0x00)
#define BNO055_GYRO_BW_230HZ                       (0x01)
#define BNO055_GYRO_BW_116HZ                       (0x02)
#define BNO055_GYRO_BW_47HZ                        (0x03)
#define BNO055_GYRO_BW_23HZ                        (0x04)
#define BNO055_GYRO_BW_12HZ                        (0x05)
#define BNO055_GYRO_BW_64HZ                        (0x06)
#define BNO055_GYRO_BW_32HZ                        (0x07)

/* Gyro power mode*/
#define BNO055_GYRO_POWER_MODE_NORMAL              (0X00)
#define BNO055_GYRO_POWER_MODE_FASTPOWERUP         (0X01)
#define BNO055_GYRO_POWER_MODE_DEEPSUSPEND         (0X02)
#define BNO055_GYRO_POWER_MODE_SUSPEND             (0X03)
#define BNO055_GYRO_POWER_MODE_ADVANCE_POWERSAVE   (0X04)

/* Accel Sleep Duration */
#define BNO055_ACCEL_SLEEP_DURN_0_5MS              (0x05)

/* sets sleep duration to 0.5 ms  */
#define BNO055_ACCEL_SLEEP_DURN_1MS                (0x06)

/* sets sleep duration to 1 ms */
#define BNO055_ACCEL_SLEEP_DURN_2MS                (0x07)

/* sets sleep duration to 2 ms */
#define BNO055_ACCEL_SLEEP_DURN_4MS                (0x08)

/* sets sleep duration to 4 ms */
#define BNO055_ACCEL_SLEEP_DURN_6MS                (0x09)

/* sets sleep duration to 6 ms*/
#define BNO055_ACCEL_SLEEP_DURN_10MS               (0x0A)

/* sets sleep duration to 10 ms */
#define BNO055_ACCEL_SLEEP_DURN_25MS               (0x0B)

/* sets sleep duration to 25 ms */
#define BNO055_ACCEL_SLEEP_DURN_50MS               (0x0C)

/* sets sleep duration to 50 ms */
#define BNO055_ACCEL_SLEEP_DURN_100MS              (0x0D)

/* sets sleep duration to 100 ms */
#define BNO055_ACCEL_SLEEP_DURN_500MS              (0x0E)

/* sets sleep duration to 500 ms */
#define BNO055_ACCEL_SLEEP_DURN_1S                 (0x0F)

/* sets sleep duration to 1 s */
/* Gyro Auto sleep duration*/
#define BNO055_GYRO_No_AUTOSLPDUR                  (0x00)
#define BNO055_GYRO_4MS_AUTOSLPDUR                 (0x01)
#define BNO055_GYRO_5MS_AUTOSLPDUR                 (0x02)
#define BNO055_GYRO_8MS_AUTOSLPDUR                 (0x03)
#define BNO055_GYRO_10MS_AUTOSLPDUR                (0x04)
#define BNO055_GYRO_15MS_AUTOSLPDUR                (0x05)
#define BNO055_GYRO_20MS_AUTOSLPDUR                (0x06)
#define BNO055_GYRO_40MS_AUTOSLPDUR                (0x07)

/* Accel Any/No motion axis selection*/
#define BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS   (0)
#define BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS   (1)
#define BNO055_ACCEL_ANY_MOTION_NO_MOTION_Z_AXIS   (2)

/* Accel High g axis selection*/
#define BNO055_ACCEL_HIGH_G_X_AXIS                 (0)
#define BNO055_ACCEL_HIGH_G_Y_AXIS                 (1)
#define BNO055_ACCEL_HIGH_G_Z_AXIS                 (2)

/* Gyro Any motion axis selection*/
#define BNO055_GYRO_ANY_MOTION_X_AXIS              (0)
#define BNO055_GYRO_ANY_MOTION_Y_AXIS              (1)
#define BNO055_GYRO_ANY_MOTION_Z_AXIS              (2)

/* Gyro High rate axis selection*/
#define BNO055_GYRO_HIGHRATE_X_AXIS                (0)
#define BNO055_GYRO_HIGHRATE_Y_AXIS                (1)
#define BNO055_GYRO_HIGHRATE_Z_AXIS                (2)

/* Axis remap values*/
#define BNO055_REMAP_X_Y                           (0X21)
#define BNO055_REMAP_Y_Z                           (0X18)
#define BNO055_REMAP_Z_X                           (0X06)
#define BNO055_REMAP_X_Y_Z_TYPE0                   (0X12)
#define BNO055_REMAP_X_Y_Z_TYPE1                   (0X09)
#define BNO055_DEFAULT_AXIS                        (0X24)

/* Axis remap sign */
#define BNO055_REMAP_AXIS_POSITIVE                 (0X00)
#define BNO055_REMAP_AXIS_NEGATIVE                 (0X01)

/* Gyro anymotion and high rate filter configuration */
#define BNO055_GYRO_FILTERED_CONFIG                (0x00)
#define BNO055_GYRO_UNFILTERED_CONFIG              (0x01)

/* mask definitions*/
#define BNO055_SIC_HEX_0_0_F_F_DATA                (0x00FF)

/****************************************************/
/**\name    ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define BNO055_REV_ID_SIZE                         (2)
#define BNO055_ACCEL_DATA_SIZE                     (2)
#define BNO055_ACCEL_XYZ_DATA_SIZE                 (6)
#define BNO055_MAG_DATA_SIZE                       (2)
#define BNO055_MAG_XYZ_DATA_SIZE                   (6)
#define BNO055_GYRO_DATA_SIZE                      (2)
#define BNO055_GYRO_XYZ_DATA_SIZE                  (6)
#define BNO055_EULER_DATA_SIZE                     (2)
#define BNO055_EULER_HRP_DATA_SIZE                 (6)
#define BNO055_QUATERNION_DATA_SIZE                (2)
#define BNO055_QUATERNION_WXYZ_DATA_SIZE           (8)
#define BNO055_GRAVITY_DATA_SIZE                   (2)
#define BNO055_GRAVITY_XYZ_DATA_SIZE               (6)
#define BNO055_ACCEL_OFFSET_ARRAY                  (6)
#define BNO055_MAG_OFFSET_ARRAY                    (6)
#define BNO055_GYRO_OFFSET_ARRAY                   (6)
#define BNO055_SOFT_IRON_CALIBRATION_MATRIX_SIZE   (18)

/*ARRAY INDEX DEFINITIONS*/
#define BNO055_SW_ID_LSB                           (0)
#define BNO055_SW_ID_MSB                           (1)
#define BNO055_SENSOR_DATA_LSB                     (0)
#define BNO055_SENSOR_DATA_MSB                     (1)
#define BNO055_SENSOR_DATA_EULER_LSB               (0)
#define BNO055_SENSOR_DATA_EULER_MSB               (1)
#define BNO055_SENSOR_DATA_QUATERNION_LSB          (0)
#define BNO055_SENSOR_DATA_QUATERNION_MSB          (1)

#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB   (0)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB   (1)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB   (2)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB   (3)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB   (4)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB   (5)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB   (6)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB   (7)

#define BNO055_SENSOR_DATA_XYZ_X_LSB               (0)
#define BNO055_SENSOR_DATA_XYZ_X_MSB               (1)
#define BNO055_SENSOR_DATA_XYZ_Y_LSB               (2)
#define BNO055_SENSOR_DATA_XYZ_Y_MSB               (3)
#define BNO055_SENSOR_DATA_XYZ_Z_LSB               (4)
#define BNO055_SENSOR_DATA_XYZ_Z_MSB               (5)

#define BNO055_SENSOR_DATA_EULER_HRP_H_LSB         (0)
#define BNO055_SENSOR_DATA_EULER_HRP_H_MSB         (1)
#define BNO055_SENSOR_DATA_EULER_HRP_R_LSB         (2)
#define BNO055_SENSOR_DATA_EULER_HRP_R_MSB         (3)
#define BNO055_SENSOR_DATA_EULER_HRP_P_LSB         (4)
#define BNO055_SENSOR_DATA_EULER_HRP_P_MSB         (5)

#define BNO055_SOFT_IRON_CALIB_0_LSB               (0)
#define BNO055_SOFT_IRON_CALIB_0_MSB               (1)
#define BNO055_SOFT_IRON_CALIB_1_LSB               (2)
#define BNO055_SOFT_IRON_CALIB_1_MSB               (3)
#define BNO055_SOFT_IRON_CALIB_2_LSB               (4)
#define BNO055_SOFT_IRON_CALIB_2_MSB               (5)
#define BNO055_SOFT_IRON_CALIB_3_LSB               (6)
#define BNO055_SOFT_IRON_CALIB_3_MSB               (7)
#define BNO055_SOFT_IRON_CALIB_4_LSB               (8)
#define BNO055_SOFT_IRON_CALIB_4_MSB               (9)
#define BNO055_SOFT_IRON_CALIB_5_LSB               (10)
#define BNO055_SOFT_IRON_CALIB_5_MSB               (11)
#define BNO055_SOFT_IRON_CALIB_6_LSB               (12)
#define BNO055_SOFT_IRON_CALIB_6_MSB               (13)
#define BNO055_SOFT_IRON_CALIB_7_LSB               (14)
#define BNO055_SOFT_IRON_CALIB_7_MSB               (15)
#define BNO055_SOFT_IRON_CALIB_8_LSB               (16)
#define BNO055_SOFT_IRON_CALIB_8_MSB               (17)

#define BNO055_SENSOR_OFFSET_DATA_X_LSB            (0)
#define BNO055_SENSOR_OFFSET_DATA_X_MSB            (1)
#define BNO055_SENSOR_OFFSET_DATA_Y_LSB            (2)
#define BNO055_SENSOR_OFFSET_DATA_Y_MSB            (3)
#define BNO055_SENSOR_OFFSET_DATA_Z_LSB            (4)
#define BNO055_SENSOR_OFFSET_DATA_Z_MSB            (5)

#define BNO055_OFFSET_RADIUS_LSB                   (0)
#define BNO055_OFFSET_RADIUS_MSB                   (1)

/*********************************************************/
/**\name PAGE0 DATA REGISTERS DEFINITION */
/*********************************************************/
/* Chip ID */
#define BNO055_CHIP_ID_POS                        (0)
#define BNO055_CHIP_ID_MSK                        (0xFF)
#define BNO055_CHIP_ID_LEN                        (8)
#define BNO055_CHIP_ID_REG                        BNO055_CHIP_ID_ADDR

/* Accel revision id*/
#define BNO055_ACCEL_REV_ID_POS                   (0)
#define BNO055_ACCEL_REV_ID_MSK                   (0xFF)
#define BNO055_ACCEL_REV_ID_LEN                   (8)
#define BNO055_ACCEL_REV_ID_REG                   BNO055_ACCEL_REV_ID_ADDR

/* Mag revision id*/
#define BNO055_MAG_REV_ID_POS                     (0)
#define BNO055_MAG_REV_ID_MSK                     (0xFF)
#define BNO055_MAG_REV_ID_LEN                     (8)
#define BNO055_MAG_REV_ID_REG                     BNO055_MAG_REV_ID_ADDR

/* Gyro revision id*/
#define BNO055_GYRO_REV_ID_POS                    (0)
#define BNO055_GYRO_REV_ID_MSK                    (0xFF)
#define BNO055_GYRO_REV_ID_LEN                    (8)
#define BNO055_GYRO_REV_ID_REG                    BNO055_GYRO_REV_ID_ADDR

/*Software revision id LSB*/
#define BNO055_SW_REV_ID_LSB_POS                  (0)
#define BNO055_SW_REV_ID_LSB_MSK                  (0xFF)
#define BNO055_SW_REV_ID_LSB_LEN                  (8)
#define BNO055_SW_REV_ID_LSB_REG                  BNO055_SW_REV_ID_LSB_ADDR

/*Software revision id MSB*/
#define BNO055_SW_REV_ID_MSB_POS                  (0)
#define BNO055_SW_REV_ID_MSB_MSK                  (0xFF)
#define BNO055_SW_REV_ID_MSB_LEN                  (8)
#define BNO055_SW_REV_ID_MSB_REG                  BNO055_SW_REV_ID_MSB_ADDR

/* BOOTLODER revision id*/
#define BNO055_BL_REV_ID_POS                      (0)
#define BNO055_BL_REV_ID_MSK                      (0xFF)
#define BNO055_BL_REV_ID_LEN                      (8)
#define BNO055_BL_REV_ID_REG                      BNO055_BL_REV_ID_ADDR

/*Page id*/
#define BNO055_PAGE_ID_POS                        (0)
#define BNO055_PAGE_ID_MSK                        (0xFF)
#define BNO055_PAGE_ID_LEN                        (8)
#define BNO055_PAGE_ID_REG                        BNO055_PAGE_ID_ADDR

/* Accel data X-LSB register*/
#define BNO055_ACCEL_DATA_X_LSB_VALUEX_POS        (0)
#define BNO055_ACCEL_DATA_X_LSB_VALUEX_MSK        (0xFF)
#define BNO055_ACCEL_DATA_X_LSB_VALUEX_LEN        (8)
#define BNO055_ACCEL_DATA_X_LSB_VALUEX_REG             \
    BNO055_ACCEL_DATA_X_LSB_ADDR

/* Accel data X-MSB register*/
#define BNO055_ACCEL_DATA_X_MSB_VALUEX_POS        (0)
#define BNO055_ACCEL_DATA_X_MSB_VALUEX_MSK        (0xFF)
#define BNO055_ACCEL_DATA_X_MSB_VALUEX_LEN        (8)
#define BNO055_ACCEL_DATA_X_MSB_VALUEX_REG             \
    BNO055_ACCEL_DATA_X_MSB_ADDR

/* Accel data Y-LSB register*/
#define BNO055_ACCEL_DATA_Y_LSB_VALUEY_POS        (0)
#define BNO055_ACCEL_DATA_Y_LSB_VALUEY_MSK        (0xFF)
#define BNO055_ACCEL_DATA_Y_LSB_VALUEY_LEN        (8)
#define BNO055_ACCEL_DATA_Y_LSB_VALUEY_REG             \
    BNO055_ACCEL_DATA_Y_LSB_ADDR

/* Accel data Y-MSB register*/
#define BNO055_ACCEL_DATA_Y_MSB_VALUEY_POS        (0)
#define BNO055_ACCEL_DATA_Y_MSB_VALUEY_MSK        (0xFF)
#define BNO055_ACCEL_DATA_Y_MSB_VALUEY_LEN        (8)
#define BNO055_ACCEL_DATA_Y_MSB_VALUEY_REG             \
    BNO055_ACCEL_DATA_Y_MSB_ADDR

/* Accel data Z-LSB register*/
#define BNO055_ACCEL_DATA_Z_LSB_VALUEZ_POS        (0)
#define BNO055_ACCEL_DATA_Z_LSB_VALUEZ_MSK        (0xFF)
#define BNO055_ACCEL_DATA_Z_LSB_VALUEZ_LEN        (8)
#define BNO055_ACCEL_DATA_Z_LSB_VALUEZ_REG     \
    BNO055_ACCEL_DATA_Z_LSB_ADDR

/* Accel data Z-MSB register*/
#define BNO055_ACCEL_DATA_Z_MSB_VALUEZ_POS        (0)
#define BNO055_ACCEL_DATA_Z_MSB_VALUEZ_MSK        (0xFF)
#define BNO055_ACCEL_DATA_Z_MSB_VALUEZ_LEN        (8)
#define BNO055_ACCEL_DATA_Z_MSB_VALUEZ_REG     \
    BNO055_ACCEL_DATA_Z_MSB_ADDR

/* Mag data X-LSB register*/
#define BNO055_MAG_DATA_X_LSB_VALUEX_POS          (0)
#define BNO055_MAG_DATA_X_LSB_VALUEX_MSK          (0xFF)
#define BNO055_MAG_DATA_X_LSB_VALUEX_LEN          (8)
#define BNO055_MAG_DATA_X_LSB_VALUEX_REG             \
    BNO055_MAG_DATA_X_LSB_ADDR

/* Mag data X-MSB register*/
#define BNO055_MAG_DATA_X_MSB_VALUEX_POS          (0)
#define BNO055_MAG_DATA_X_MSB_VALUEX_MSK          (0xFF)
#define BNO055_MAG_DATA_X_MSB_VALUEX_LEN          (8)
#define BNO055_MAG_DATA_X_MSB_VALUEX_REG          BNO055_MAG_DATA_X_MSB_ADDR

/* Mag data Y-LSB register*/
#define BNO055_MAG_DATA_Y_LSB_VALUEY_POS          (0)
#define BNO055_MAG_DATA_Y_LSB_VALUEY_MSK          (0xFF)
#define BNO055_MAG_DATA_Y_LSB_VALUEY_LEN          (8)
#define BNO055_MAG_DATA_Y_LSB_VALUEY_REG          BNO055_MAG_DATA_Y_LSB_ADDR

/* Mag data Y-MSB register*/
#define BNO055_MAG_DATA_Y_MSB_VALUEY_POS          (0)
#define BNO055_MAG_DATA_Y_MSB_VALUEY_MSK          (0xFF)
#define BNO055_MAG_DATA_Y_MSB_VALUEY_LEN          (8)
#define BNO055_MAG_DATA_Y_MSB_VALUEY_REG          BNO055_MAG_DATA_Y_MSB_ADDR

/* Mag data Z-LSB register*/
#define BNO055_MAG_DATA_Z_LSB_VALUEZ_POS          (0)
#define BNO055_MAG_DATA_Z_LSB_VALUEZ_MSK          (0xFF)
#define BNO055_MAG_DATA_Z_LSB_VALUEZ_LEN          (8)
#define BNO055_MAG_DATA_Z_LSB_VALUEZ_REG          BNO055_MAG_DATA_Z_LSB_ADDR

/* Mag data Z-MSB register*/
#define BNO055_MAG_DATA_Z_MSB_VALUEZ_POS          (0)
#define BNO055_MAG_DATA_Z_MSB_VALUEZ_MSK          (0xFF)
#define BNO055_MAG_DATA_Z_MSB_VALUEZ_LEN          (8)
#define BNO055_MAG_DATA_Z_MSB_VALUEZ_REG          BNO055_MAG_DATA_Z_MSB_ADDR

/* Gyro data X-LSB register*/
#define BNO055_GYRO_DATA_X_LSB_VALUEX_POS         (0)
#define BNO055_GYRO_DATA_X_LSB_VALUEX_MSK         (0xFF)
#define BNO055_GYRO_DATA_X_LSB_VALUEX_LEN         (8)
#define BNO055_GYRO_DATA_X_LSB_VALUEX_REG         BNO055_GYRO_DATA_X_LSB_ADDR

/* Gyro data X-MSB register*/
#define BNO055_GYRO_DATA_X_MSB_VALUEX_POS         (0)
#define BNO055_GYRO_DATA_X_MSB_VALUEX_MSK         (0xFF)
#define BNO055_GYRO_DATA_X_MSB_VALUEX_LEN         (8)
#define BNO055_GYRO_DATA_X_MSB_VALUEX_REG         BNO055_GYRO_DATA_X_MSB_ADDR

/* Gyro data Y-LSB register*/
#define BNO055_GYRO_DATA_Y_LSB_VALUEY_POS         (0)
#define BNO055_GYRO_DATA_Y_LSB_VALUEY_MSK         (0xFF)
#define BNO055_GYRO_DATA_Y_LSB_VALUEY_LEN         (8)
#define BNO055_GYRO_DATA_Y_LSB_VALUEY_REG         BNO055_GYRO_DATA_Y_LSB_ADDR

/* Gyro data Y-MSB register*/
#define BNO055_GYRO_DATA_Y_MSB_VALUEY_POS         (0)
#define BNO055_GYRO_DATA_Y_MSB_VALUEY_MSK         (0xFF)
#define BNO055_GYRO_DATA_Y_MSB_VALUEY_LEN         (8)
#define BNO055_GYRO_DATA_Y_MSB_VALUEY_REG         BNO055_GYRO_DATA_Y_MSB_ADDR

/* Gyro data Z-LSB register*/
#define BNO055_GYRO_DATA_Z_LSB_VALUEZ_POS         (0)
#define BNO055_GYRO_DATA_Z_LSB_VALUEZ_MSK         (0xFF)
#define BNO055_GYRO_DATA_Z_LSB_VALUEZ_LEN         (8)
#define BNO055_GYRO_DATA_Z_LSB_VALUEZ_REG         BNO055_GYRO_DATA_Z_LSB_ADDR

/* Gyro data Z-MSB register*/
#define BNO055_GYRO_DATA_Z_MSB_VALUEZ_POS         (0)
#define BNO055_GYRO_DATA_Z_MSB_VALUEZ_MSK         (0xFF)
#define BNO055_GYRO_DATA_Z_MSB_VALUEZ_LEN         (8)
#define BNO055_GYRO_DATA_Z_MSB_VALUEZ_REG         BNO055_GYRO_DATA_Z_MSB_ADDR

/* Euler data HEADING-LSB register*/
#define BNO055_EULER_H_LSB_VALUEH_POS             (0)
#define BNO055_EULER_H_LSB_VALUEH_MSK             (0xFF)
#define BNO055_EULER_H_LSB_VALUEH_LEN             (8)
#define BNO055_EULER_H_LSB_VALUEH_REG             BNO055_EULER_H_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BNO055_EULER_H_MSB_VALUEH_POS             (0)
#define BNO055_EULER_H_MSB_VALUEH_MSK             (0xFF)
#define BNO055_EULER_H_MSB_VALUEH_LEN             (8)
#define BNO055_EULER_H_MSB_VALUEH_REG             BNO055_EULER_H_MSB_ADDR

/* Euler data ROLL-LSB register*/
#define BNO055_EULER_R_LSB_VALUER_POS             (0)
#define BNO055_EULER_R_LSB_VALUER_MSK             (0xFF)
#define BNO055_EULER_R_LSB_VALUER_LEN             (8)
#define BNO055_EULER_R_LSB_VALUER_REG             BNO055_EULER_R_LSB_ADDR

/* Euler data ROLL-MSB register*/
#define BNO055_EULER_R_MSB_VALUER_POS             (0)
#define BNO055_EULER_R_MSB_VALUER_MSK             (0xFF)
#define BNO055_EULER_R_MSB_VALUER_LEN             (8)
#define BNO055_EULER_R_MSB_VALUER_REG             BNO055_EULER_R_MSB_ADDR

/* Euler data PITCH-LSB register*/
#define BNO055_EULER_P_LSB_VALUEP_POS             (0)
#define BNO055_EULER_P_LSB_VALUEP_MSK             (0xFF)
#define BNO055_EULER_P_LSB_VALUEP_LEN             (8)
#define BNO055_EULER_P_LSB_VALUEP_REG             BNO055_EULER_P_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BNO055_EULER_P_MSB_VALUEP_POS             (0)
#define BNO055_EULER_P_MSB_VALUEP_MSK             (0xFF)
#define BNO055_EULER_P_MSB_VALUEP_LEN             (8)
#define BNO055_EULER_P_MSB_VALUEP_REG             BNO055_EULER_P_MSB_ADDR

/* Quaternion data W-LSB register*/
#define BNO055_QUATERNION_DATA_W_LSB_VALUEW_POS   (0)
#define BNO055_QUATERNION_DATA_W_LSB_VALUEW_MSK   (0xFF)
#define BNO055_QUATERNION_DATA_W_LSB_VALUEW_LEN   (8)
#define BNO055_QUATERNION_DATA_W_LSB_VALUEW_REG  \
    BNO055_QUATERNION_DATA_W_LSB_ADDR

/* Quaternion data W-MSB register*/
#define BNO055_QUATERNION_DATA_W_MSB_VALUEW_POS   (0)
#define BNO055_QUATERNION_DATA_W_MSB_VALUEW_MSK   (0xFF)
#define BNO055_QUATERNION_DATA_W_MSB_VALUEW_LEN   (8)
#define BNO055_QUATERNION_DATA_W_MSB_VALUEW_REG  \
    BNO055_QUATERNION_DATA_W_MSB_ADDR

/* Quaternion data X-LSB register*/
#define BNO055_QUATERNION_DATA_X_LSB_VALUEX_POS   (0)
#define BNO055_QUATERNION_DATA_X_LSB_VALUEX_MSK   (0xFF)
#define BNO055_QUATERNION_DATA_X_LSB_VALUEX_LEN   (8)
#define BNO055_QUATERNION_DATA_X_LSB_VALUEX_REG  \
    BNO055_QUATERNION_DATA_X_LSB_ADDR

/* Quaternion data X-MSB register*/
#define BNO055_QUATERNION_DATA_X_MSB_VALUEX_POS   (0)
#define BNO055_QUATERNION_DATA_X_MSB_VALUEX_MSK   (0xFF)
#define BNO055_QUATERNION_DATA_X_MSB_VALUEX_LEN   (8)
#define BNO055_QUATERNION_DATA_X_MSB_VALUEX_REG \
    BNO055_QUATERNION_DATA_X_MSB_ADDR

/* Quaternion data Y-LSB register*/
#define BNO055_QUATERNION_DATA_Y_LSB_VALUEY_POS   (0)
#define BNO055_QUATERNION_DATA_Y_LSB_VALUEY_MSK   (0xFF)
#define BNO055_QUATERNION_DATA_Y_LSB_VALUEY_LEN   (8)
#define BNO055_QUATERNION_DATA_Y_LSB_VALUEY_REG \
    BNO055_QUATERNION_DATA_Y_LSB_ADDR

/* Quaternion data Y-MSB register*/
#define BNO055_QUATERNION_DATA_Y_MSB_VALUEY_POS   (0)
#define BNO055_QUATERNION_DATA_Y_MSB_VALUEY_MSK   (0xFF)
#define BNO055_QUATERNION_DATA_Y_MSB_VALUEY_LEN   (8)
#define BNO055_QUATERNION_DATA_Y_MSB_VALUEY_REG  \
    BNO055_QUATERNION_DATA_Y_MSB_ADDR

/* Quaternion data Z-LSB register*/
#define BNO055_QUATERNION_DATA_Z_LSB_VALUEZ_POS   (0)
#define BNO055_QUATERNION_DATA_Z_LSB_VALUEZ_MSK   (0xFF)
#define BNO055_QUATERNION_DATA_Z_LSB_VALUEZ_LEN   (8)
#define BNO055_QUATERNION_DATA_Z_LSB_VALUEZ_REG \
    BNO055_QUATERNION_DATA_Z_LSB_ADDR

/* Quaternion data Z-MSB register*/
#define BNO055_QUATERNION_DATA_Z_MSB_VALUEZ_POS   (0)
#define BNO055_QUATERNION_DATA_Z_MSB_VALUEZ_MSK   (0xFF)
#define BNO055_QUATERNION_DATA_Z_MSB_VALUEZ_LEN   (8)
#define BNO055_QUATERNION_DATA_Z_MSB_VALUEZ_REG  \
    BNO055_QUATERNION_DATA_Z_MSB_ADDR

/* Linear acceleration data X-LSB register*/
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX_POS (0)
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX_MSK (0xFF)
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX_LEN (8)
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX_REG  \
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR

/* Linear acceleration data X-MSB register*/
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX_POS (0)
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX_MSK (0xFF)
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX_LEN (8)
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX_REG  \
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR

/* Linear acceleration data Y-LSB register*/
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY_POS (0)
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY_MSK (0xFF)
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY_LEN (8)
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY_REG  \
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR

/* Linear acceleration data Y-MSB register*/
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY_POS (0)
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY_MSK (0xFF)
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY_LEN (8)
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY_REG  \
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR

/* Linear acceleration data Z-LSB register*/
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ_POS (0)
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ_MSK (0xFF)
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ_LEN (8)
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ_REG \
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR

/* Linear acceleration data Z-MSB register*/
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ_POS (0)
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ_MSK (0xFF)
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ_LEN (8)
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ_REG  \
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR

/* Gravity data X-LSB register*/
#define BNO055_GRAVITY_DATA_X_LSB_VALUEX_POS      (0)
#define BNO055_GRAVITY_DATA_X_LSB_VALUEX_MSK      (0xFF)
#define BNO055_GRAVITY_DATA_X_LSB_VALUEX_LEN      (8)
#define BNO055_GRAVITY_DATA_X_LSB_VALUEX_REG  \
    BNO055_GRAVITY_DATA_X_LSB_ADDR

/* Gravity data X-MSB register*/
#define BNO055_GRAVITY_DATA_X_MSB_VALUEX_POS      (0)
#define BNO055_GRAVITY_DATA_X_MSB_VALUEX_MSK      (0xFF)
#define BNO055_GRAVITY_DATA_X_MSB_VALUEX_LEN      (8)
#define BNO055_GRAVITY_DATA_X_MSB_VALUEX_REG  \
    BNO055_GRAVITY_DATA_X_MSB_ADDR

/* Gravity data Y-LSB register*/
#define BNO055_GRAVITY_DATA_Y_LSB_VALUEY_POS      (0)
#define BNO055_GRAVITY_DATA_Y_LSB_VALUEY_MSK      (0xFF)
#define BNO055_GRAVITY_DATA_Y_LSB_VALUEY_LEN      (8)
#define BNO055_GRAVITY_DATA_Y_LSB_VALUEY_REG  \
    BNO055_GRAVITY_DATA_Y_LSB_ADDR

/* Gravity data Y-MSB register*/
#define BNO055_GRAVITY_DATA_Y_MSB_VALUEY_POS      (0)
#define BNO055_GRAVITY_DATA_Y_MSB_VALUEY_MSK      (0xFF)
#define BNO055_GRAVITY_DATA_Y_MSB_VALUEY_LEN      (8)
#define BNO055_GRAVITY_DATA_Y_MSB_VALUEY_REG  \
    BNO055_GRAVITY_DATA_Y_MSB_ADDR

/* Gravity data Z-LSB register*/
#define BNO055_GRAVITY_DATA_Z_LSB_VALUEZ_POS      (0)
#define BNO055_GRAVITY_DATA_Z_LSB_VALUEZ_MSK      (0xFF)
#define BNO055_GRAVITY_DATA_Z_LSB_VALUEZ_LEN      (8)
#define BNO055_GRAVITY_DATA_Z_LSB_VALUEZ_REG  \
    BNO055_GRAVITY_DATA_Z_LSB_ADDR

/* Gravity data Z-MSB register*/
#define BNO055_GRAVITY_DATA_Z_MSB_VALUEZ_POS      (0)
#define BNO055_GRAVITY_DATA_Z_MSB_VALUEZ_MSK      (0xFF)
#define BNO055_GRAVITY_DATA_Z_MSB_VALUEZ_LEN      (8)
#define BNO055_GRAVITY_DATA_Z_MSB_VALUEZ_REG  \
    BNO055_GRAVITY_DATA_Z_MSB_ADDR

/* Temperature register*/
#define BNO055_TEMP_POS                           (0)
#define BNO055_TEMP_MSK                           (0xFF)
#define BNO055_TEMP_LEN                           (8)
#define BNO055_TEMP_REG                           BNO055_TEMP_ADDR

/*Mag_Calib status register*/
#define BNO055_MAG_CALIB_STAT_POS                 (0)
#define BNO055_MAG_CALIB_STAT_MSK                 (0X03)
#define BNO055_MAG_CALIB_STAT_LEN                 (2)
#define BNO055_MAG_CALIB_STAT_REG                 BNO055_CALIB_STAT_ADDR

/*Acc_Calib status register*/
#define BNO055_ACCEL_CALIB_STAT_POS               (2)
#define BNO055_ACCEL_CALIB_STAT_MSK               (0X0C)
#define BNO055_ACCEL_CALIB_STAT_LEN               (2)
#define BNO055_ACCEL_CALIB_STAT_REG               BNO055_CALIB_STAT_ADDR

/*Gyro_Calib status register*/
#define BNO055_GYRO_CALIB_STAT_POS                (4)
#define BNO055_GYRO_CALIB_STAT_MSK                (0X30)
#define BNO055_GYRO_CALIB_STAT_LEN                (2)
#define BNO055_GYRO_CALIB_STAT_REG                BNO055_CALIB_STAT_ADDR

/*Sys_Calib status register*/
#define BNO055_SYS_CALIB_STAT_POS                 (6)
#define BNO055_SYS_CALIB_STAT_MSK                 (0XC0)
#define BNO055_SYS_CALIB_STAT_LEN                 (2)
#define BNO055_SYS_CALIB_STAT_REG                 BNO055_CALIB_STAT_ADDR

/*ST_ACCEL register*/
#define BNO055_SELFTEST_ACCEL_POS                 (0)
#define BNO055_SELFTEST_ACCEL_MSK                 (0X01)
#define BNO055_SELFTEST_ACCEL_LEN                 (1)
#define BNO055_SELFTEST_ACCEL_REG                 BNO055_SELFTEST_RESULT_ADDR

/*ST_MAG register*/
#define BNO055_SELFTEST_MAG_POS                   (1)
#define BNO055_SELFTEST_MAG_MSK                   (0X02)
#define BNO055_SELFTEST_MAG_LEN                   (1)
#define BNO055_SELFTEST_MAG_REG                   BNO055_SELFTEST_RESULT_ADDR

/*ST_GYRO register*/
#define BNO055_SELFTEST_GYRO_POS                  (2)
#define BNO055_SELFTEST_GYRO_MSK                  (0X04)
#define BNO055_SELFTEST_GYRO_LEN                  (1)
#define BNO055_SELFTEST_GYRO_REG                  BNO055_SELFTEST_RESULT_ADDR

/*ST_MCU register*/
#define BNO055_SELFTEST_MCU_POS                   (3)
#define BNO055_SELFTEST_MCU_MSK                   (0X08)
#define BNO055_SELFTEST_MCU_LEN                   (1)
#define BNO055_SELFTEST_MCU_REG                   BNO055_SELFTEST_RESULT_ADDR

/*Interrupt status registers*/
#define BNO055_INTR_STAT_GYRO_ANY_MOTION_POS      (2)
#define BNO055_INTR_STAT_GYRO_ANY_MOTION_MSK      (0X04)
#define BNO055_INTR_STAT_GYRO_ANY_MOTION_LEN      (1)
#define BNO055_INTR_STAT_GYRO_ANY_MOTION_REG      BNO055_INTR_STAT_ADDR

#define BNO055_INTR_STAT_GYRO_HIGHRATE_POS        (3)
#define BNO055_INTR_STAT_GYRO_HIGHRATE_MSK        (0X08)
#define BNO055_INTR_STAT_GYRO_HIGHRATE_LEN        (1)
#define BNO055_INTR_STAT_GYRO_HIGHRATE_REG        BNO055_INTR_STAT_ADDR

#define BNO055_INTR_STAT_ACCEL_HIGH_G_POS         (5)
#define BNO055_INTR_STAT_ACCEL_HIGH_G_MSK         (0X20)
#define BNO055_INTR_STAT_ACCEL_HIGH_G_LEN         (1)
#define BNO055_INTR_STAT_ACCEL_HIGH_G_REG         BNO055_INTR_STAT_ADDR

#define BNO055_INTR_STAT_ACCEL_ANY_MOTION_POS     (6)
#define BNO055_INTR_STAT_ACCEL_ANY_MOTION_MSK     (0X40)
#define BNO055_INTR_STAT_ACCEL_ANY_MOTION_LEN     (1)
#define BNO055_INTR_STAT_ACCEL_ANY_MOTION_REG     BNO055_INTR_STAT_ADDR

#define BNO055_INTR_STAT_ACCEL_NO_MOTION_POS      (7)
#define BNO055_INTR_STAT_ACCEL_NO_MOTION_MSK      (0X80)
#define BNO055_INTR_STAT_ACCEL_NO_MOTION_LEN      (1)
#define BNO055_INTR_STAT_ACCEL_NO_MOTION_REG      BNO055_INTR_STAT_ADDR

/* system clock status register*/
#define BNO055_SYS_MAIN_CLK_POS                   (0)
#define BNO055_SYS_MAIN_CLK_MSK                   (0X10)
#define BNO055_SYS_MAIN_CLK_LEN                   (1)
#define BNO055_SYS_MAIN_CLK_REG                   BNO055_SYS_CLK_STAT_ADDR

/* System registers*/
#define BNO055_SYS_STAT_CODE_POS                  (0)
#define BNO055_SYS_STAT_CODE_MSK                  (0XFF)
#define BNO055_SYS_STAT_CODE_LEN                  (8)
#define BNO055_SYS_STAT_CODE_REG                  BNO055_SYS_STAT_ADDR

#define BNO055_SYS_ERROR_CODE_POS                 (0)
#define BNO055_SYS_ERROR_CODE_MSK                 (0XFF)
#define BNO055_SYS_ERROR_CODE_LEN                 (8)
#define BNO055_SYS_ERROR_CODE_REG                 BNO055_SYS_ERR_ADDR

/* Accel_Unit register*/
#define BNO055_ACCEL_UNIT_POS                     (0)
#define BNO055_ACCEL_UNIT_MSK                     (0X01)
#define BNO055_ACCEL_UNIT_LEN                     (1)
#define BNO055_ACCEL_UNIT_REG                     BNO055_UNIT_SEL_ADDR

/* Gyro_Unit register*/
#define BNO055_GYRO_UNIT_POS                      (1)
#define BNO055_GYRO_UNIT_MSK                      (0X02)
#define BNO055_GYRO_UNIT_LEN                      (1)
#define BNO055_GYRO_UNIT_REG                      BNO055_UNIT_SEL_ADDR

/* Euler_Unit register*/
#define BNO055_EULER_UNIT_POS                     (2)
#define BNO055_EULER_UNIT_MSK                     (0X04)
#define BNO055_EULER_UNIT_LEN                     (1)
#define BNO055_EULER_UNIT_REG                     BNO055_UNIT_SEL_ADDR

/* Tilt_Unit register*/
#define BNO055_TILT_UNIT_POS                      (3)
#define BNO055_TILT_UNIT_MSK                      (0X08)
#define BNO055_TILT_UNIT_LEN                      (1)
#define BNO055_TILT_UNIT_REG                      BNO055_UNIT_SEL_ADDR

/* Temperature_Unit register*/
#define BNO055_TEMP_UNIT_POS                      (4)
#define BNO055_TEMP_UNIT_MSK                      (0X10)
#define BNO055_TEMP_UNIT_LEN                      (1)
#define BNO055_TEMP_UNIT_REG                      BNO055_UNIT_SEL_ADDR

/* ORI android-windows register*/
#define BNO055_DATA_OUTPUT_FORMAT_POS             (7)
#define BNO055_DATA_OUTPUT_FORMAT_MSK             (0X80)
#define BNO055_DATA_OUTPUT_FORMAT_LEN             (1)
#define BNO055_DATA_OUTPUT_FORMAT_REG             BNO055_UNIT_SEL_ADDR

/*Operation Mode data register*/
#define BNO055_OPERATION_MODE_POS                 (0)
#define BNO055_OPERATION_MODE_MSK                 (0X0F)
#define BNO055_OPERATION_MODE_LEN                 (4)
#define BNO055_OPERATION_MODE_REG                 BNO055_OPR_MODE_ADDR

/* Power Mode register*/
#define BNO055_POWER_MODE_POS                     (0)
#define BNO055_POWER_MODE_MSK                     (0X03)
#define BNO055_POWER_MODE_LEN                     (2)
#define BNO055_POWER_MODE_REG                     BNO055_PWR_MODE_ADDR

/*Self Test register*/
#define BNO055_SELFTEST_POS                       (0)
#define BNO055_SELFTEST_MSK                       (0X01)
#define BNO055_SELFTEST_LEN                       (1)
#define BNO055_SELFTEST_REG                       BNO055_SYS_TRIGGER_ADDR

/* RST_SYS register*/
#define BNO055_SYS_RST_POS                        (5)
#define BNO055_SYS_RST_MSK                        (0X20)
#define BNO055_SYS_RST_LEN                        (1)
#define BNO055_SYS_RST_REG                        BNO055_SYS_TRIGGER_ADDR

/* RST_INT register*/
#define BNO055_INTR_RST_POS                       (6)
#define BNO055_INTR_RST_MSK                       (0X40)
#define BNO055_INTR_RST_LEN                       (1)
#define BNO055_INTR_RST_REG                       BNO055_SYS_TRIGGER_ADDR

/* CLK_SRC register*/
#define BNO055_CLK_SRC_POS                        (7)
#define BNO055_CLK_SRC_MSK                        (0X80)
#define BNO055_CLK_SRC_LEN                        (1)
#define BNO055_CLK_SRC_REG                        BNO055_SYS_TRIGGER_ADDR

/* Temp source register*/
#define BNO055_TEMP_SOURCE_POS                    (0)
#define BNO055_TEMP_SOURCE_MSK                    (0X03)
#define BNO055_TEMP_SOURCE_LEN                    (2)
#define BNO055_TEMP_SOURCE_REG                    BNO055_TEMP_SOURCE_ADDR

/* Axis remap value register*/
#define BNO055_REMAP_AXIS_VALUE_POS               (0)
#define BNO055_REMAP_AXIS_VALUE_MSK               (0X3F)
#define BNO055_REMAP_AXIS_VALUE_LEN               (6)
#define BNO055_REMAP_AXIS_VALUE_REG               BNO055_AXIS_MAP_CONFIG_ADDR

/* Axis sign value register*/
#define BNO055_REMAP_Z_SIGN_POS                   (0)
#define BNO055_REMAP_Z_SIGN_MSK                   (0X01)
#define BNO055_REMAP_Z_SIGN_LEN                   (1)
#define BNO055_REMAP_Z_SIGN_REG                   BNO055_AXIS_MAP_SIGN_ADDR

#define BNO055_REMAP_Y_SIGN_POS                   (1)
#define BNO055_REMAP_Y_SIGN_MSK                   (0X02)
#define BNO055_REMAP_Y_SIGN_LEN                   (1)
#define BNO055_REMAP_Y_SIGN_REG                   BNO055_AXIS_MAP_SIGN_ADDR

#define BNO055_REMAP_X_SIGN_POS                   (2)
#define BNO055_REMAP_X_SIGN_MSK                   (0X04)
#define BNO055_REMAP_X_SIGN_LEN                   (1)
#define BNO055_REMAP_X_SIGN_REG                   BNO055_AXIS_MAP_SIGN_ADDR

/* Soft Iron Calibration matrix register*/
#define BNO055_SIC_MATRIX_0_LSB_POS               (0)
#define BNO055_SIC_MATRIX_0_LSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_0_LSB_LEN               (8)
#define BNO055_SIC_MATRIX_0_LSB_REG               BNO055_SIC_MATRIX_0_LSB_ADDR

#define BNO055_SIC_MATRIX_0_MSB_POS               (0)
#define BNO055_SIC_MATRIX_0_MSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_0_MSB_LEN               (8)
#define BNO055_SIC_MATRIX_0_MSB_REG               BNO055_SIC_MATRIX_0_MSB_ADDR

#define BNO055_SIC_MATRIX_1_LSB_POS               (0)
#define BNO055_SIC_MATRIX_1_LSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_1_LSB_LEN               (8)
#define BNO055_SIC_MATRIX_1_LSB_REG               BNO055_SIC_MATRIX_1_LSB_ADDR

#define BNO055_SIC_MATRIX_1_MSB_POS               (0)
#define BNO055_SIC_MATRIX_1_MSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_1_MSB_LEN               (8)
#define BNO055_SIC_MATRIX_1_MSB_REG               BNO055_SIC_MATRIX_1_MSB_ADDR

#define BNO055_SIC_MATRIX_2_LSB_POS               (0)
#define BNO055_SIC_MATRIX_2_LSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_2_LSB_LEN               (8)
#define BNO055_SIC_MATRIX_2_LSB_REG               BNO055_SIC_MATRIX_2_LSB_ADDR

#define BNO055_SIC_MATRIX_2_MSB_POS               (0)
#define BNO055_SIC_MATRIX_2_MSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_2_MSB_LEN               (8)
#define BNO055_SIC_MATRIX_2_MSB_REG               BNO055_SIC_MATRIX_2_MSB_ADDR

#define BNO055_SIC_MATRIX_3_LSB_POS               (0)
#define BNO055_SIC_MATRIX_3_LSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_3_LSB_LEN               (8)
#define BNO055_SIC_MATRIX_3_LSB_REG               BNO055_SIC_MATRIX_3_LSB_ADDR

#define BNO055_SIC_MATRIX_3_MSB_POS               (0)
#define BNO055_SIC_MATRIX_3_MSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_3_MSB_LEN               (8)
#define BNO055_SIC_MATRIX_3_MSB_REG               BNO055_SIC_MATRIX_3_MSB_ADDR

#define BNO055_SIC_MATRIX_4_LSB_POS               (0)
#define BNO055_SIC_MATRIX_4_LSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_4_LSB_LEN               (8)
#define BNO055_SIC_MATRIX_4_LSB_REG               BNO055_SIC_MATRIX_4_LSB_ADDR

#define BNO055_SIC_MATRIX_4_MSB_POS               (0)
#define BNO055_SIC_MATRIX_4_MSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_4_MSB_LEN               (8)
#define BNO055_SIC_MATRIX_4_MSB_REG               BNO055_SIC_MATRIX_4_MSB_ADDR

#define BNO055_SIC_MATRIX_5_LSB_POS               (0)
#define BNO055_SIC_MATRIX_5_LSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_5_LSB_LEN               (8)
#define BNO055_SIC_MATRIX_5_LSB_REG               BNO055_SIC_MATRIX_5_LSB_ADDR

#define BNO055_SIC_MATRIX_5_MSB_POS               (0)
#define BNO055_SIC_MATRIX_5_MSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_5_MSB_LEN               (8)
#define BNO055_SIC_MATRIX_5_MSB_REG               BNO055_SIC_MATRIX_5_MSB_ADDR

#define BNO055_SIC_MATRIX_6_LSB_POS               (0)
#define BNO055_SIC_MATRIX_6_LSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_6_LSB_LEN               (8)
#define BNO055_SIC_MATRIX_6_LSB_REG               BNO055_SIC_MATRIX_6_LSB_ADDR

#define BNO055_SIC_MATRIX_6_MSB_POS               (0)
#define BNO055_SIC_MATRIX_6_MSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_6_MSB_LEN               (8)
#define BNO055_SIC_MATRIX_6_MSB_REG               BNO055_SIC_MATRIX_6_MSB_ADDR

#define BNO055_SIC_MATRIX_7_LSB_POS               (0)
#define BNO055_SIC_MATRIX_7_LSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_7_LSB_LEN               (8)
#define BNO055_SIC_MATRIX_7_LSB_REG               BNO055_SIC_MATRIX_7_LSB_ADDR

#define BNO055_SIC_MATRIX_7_MSB_POS               (0)
#define BNO055_SIC_MATRIX_7_MSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_7_MSB_LEN               (8)
#define BNO055_SIC_MATRIX_7_MSB_REG               BNO055_SIC_MATRIX_7_MSB_ADDR

#define BNO055_SIC_MATRIX_8_LSB_POS               (0)
#define BNO055_SIC_MATRIX_8_LSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_8_LSB_LEN               (8)
#define BNO055_SIC_MATRIX_8_LSB_REG               BNO055_SIC_MATRIX_8_LSB_ADDR

#define BNO055_SIC_MATRIX_8_MSB_POS               (0)
#define BNO055_SIC_MATRIX_8_MSB_MSK               (0XFF)
#define BNO055_SIC_MATRIX_8_MSB_LEN               (8)
#define BNO055_SIC_MATRIX_8_MSB_REG               BNO055_SIC_MATRIX_8_MSB_ADDR

/*Accel Offset registers*/
#define BNO055_ACCEL_OFFSET_X_LSB_POS             (0)
#define BNO055_ACCEL_OFFSET_X_LSB_MSK             (0XFF)
#define BNO055_ACCEL_OFFSET_X_LSB_LEN             (8)
#define BNO055_ACCEL_OFFSET_X_LSB_REG             BNO055_ACCEL_OFFSET_X_LSB_ADDR

#define BNO055_ACCEL_OFFSET_X_MSB_POS             (0)
#define BNO055_ACCEL_OFFSET_X_MSB_MSK             (0XFF)
#define BNO055_ACCEL_OFFSET_X_MSB_LEN             (8)
#define BNO055_ACCEL_OFFSET_X_MSB_REG             BNO055_ACCEL_OFFSET_X_MSB_ADDR

#define BNO055_ACCEL_OFFSET_Y_LSB_POS             (0)
#define BNO055_ACCEL_OFFSET_Y_LSB_MSK             (0XFF)
#define BNO055_ACCEL_OFFSET_Y_LSB_LEN             (8)
#define BNO055_ACCEL_OFFSET_Y_LSB_REG             BNO055_ACCEL_OFFSET_Y_LSB_ADDR

#define BNO055_ACCEL_OFFSET_Y_MSB_POS             (0)
#define BNO055_ACCEL_OFFSET_Y_MSB_MSK             (0XFF)
#define BNO055_ACCEL_OFFSET_Y_MSB_LEN             (8)
#define BNO055_ACCEL_OFFSET_Y_MSB_REG             BNO055_ACCEL_OFFSET_Y_MSB_ADDR

#define BNO055_ACCEL_OFFSET_Z_LSB_POS             (0)
#define BNO055_ACCEL_OFFSET_Z_LSB_MSK             (0XFF)
#define BNO055_ACCEL_OFFSET_Z_LSB_LEN             (8)
#define BNO055_ACCEL_OFFSET_Z_LSB_REG             BNO055_ACCEL_OFFSET_Z_LSB_ADDR

#define BNO055_ACCEL_OFFSET_Z_MSB_POS             (0)
#define BNO055_ACCEL_OFFSET_Z_MSB_MSK             (0XFF)
#define BNO055_ACCEL_OFFSET_Z_MSB_LEN             (8)
#define BNO055_ACCEL_OFFSET_Z_MSB_REG             BNO055_ACCEL_OFFSET_Z_MSB_ADDR

/*Mag Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB_POS               (0)
#define BNO055_MAG_OFFSET_X_LSB_MSK               (0XFF)
#define BNO055_MAG_OFFSET_X_LSB_LEN               (8)
#define BNO055_MAG_OFFSET_X_LSB_REG               BNO055_MAG_OFFSET_X_LSB_ADDR

#define BNO055_MAG_OFFSET_X_MSB_POS               (0)
#define BNO055_MAG_OFFSET_X_MSB_MSK               (0XFF)
#define BNO055_MAG_OFFSET_X_MSB_LEN               (8)
#define BNO055_MAG_OFFSET_X_MSB_REG               BNO055_MAG_OFFSET_X_MSB_ADDR

#define BNO055_MAG_OFFSET_Y_LSB_POS               (0)
#define BNO055_MAG_OFFSET_Y_LSB_MSK               (0XFF)
#define BNO055_MAG_OFFSET_Y_LSB_LEN               (8)
#define BNO055_MAG_OFFSET_Y_LSB_REG               BNO055_MAG_OFFSET_Y_LSB_ADDR

#define BNO055_MAG_OFFSET_Y_MSB_POS               (0)
#define BNO055_MAG_OFFSET_Y_MSB_MSK               (0XFF)
#define BNO055_MAG_OFFSET_Y_MSB_LEN               (8)
#define BNO055_MAG_OFFSET_Y_MSB_REG               BNO055_MAG_OFFSET_Y_MSB_ADDR

#define BNO055_MAG_OFFSET_Z_LSB_POS               (0)
#define BNO055_MAG_OFFSET_Z_LSB_MSK               (0XFF)
#define BNO055_MAG_OFFSET_Z_LSB_LEN               (8)
#define BNO055_MAG_OFFSET_Z_LSB_REG               BNO055_MAG_OFFSET_Z_LSB_ADDR

#define BNO055_MAG_OFFSET_Z_MSB_POS               (0)
#define BNO055_MAG_OFFSET_Z_MSB_MSK               (0XFF)
#define BNO055_MAG_OFFSET_Z_MSB_LEN               (8)
#define BNO055_MAG_OFFSET_Z_MSB_REG               BNO055_MAG_OFFSET_Z_MSB_ADDR

/* Gyro Offset registers*/
#define BNO055_GYRO_OFFSET_X_LSB_POS              (0)
#define BNO055_GYRO_OFFSET_X_LSB_MSK              (0XFF)
#define BNO055_GYRO_OFFSET_X_LSB_LEN              (8)
#define BNO055_GYRO_OFFSET_X_LSB_REG              BNO055_GYRO_OFFSET_X_LSB_ADDR

#define BNO055_GYRO_OFFSET_X_MSB_POS              (0)
#define BNO055_GYRO_OFFSET_X_MSB_MSK              (0XFF)
#define BNO055_GYRO_OFFSET_X_MSB_LEN              (8)
#define BNO055_GYRO_OFFSET_X_MSB_REG              BNO055_GYRO_OFFSET_X_MSB_ADDR

#define BNO055_GYRO_OFFSET_Y_LSB_POS              (0)
#define BNO055_GYRO_OFFSET_Y_LSB_MSK              (0XFF)
#define BNO055_GYRO_OFFSET_Y_LSB_LEN              (8)
#define BNO055_GYRO_OFFSET_Y_LSB_REG              BNO055_GYRO_OFFSET_Y_LSB_ADDR

#define BNO055_GYRO_OFFSET_Y_MSB_POS              (0)
#define BNO055_GYRO_OFFSET_Y_MSB_MSK              (0XFF)
#define BNO055_GYRO_OFFSET_Y_MSB_LEN              (8)
#define BNO055_GYRO_OFFSET_Y_MSB_REG              BNO055_GYRO_OFFSET_Y_MSB_ADDR

#define BNO055_GYRO_OFFSET_Z_LSB_POS              (0)
#define BNO055_GYRO_OFFSET_Z_LSB_MSK              (0XFF)
#define BNO055_GYRO_OFFSET_Z_LSB_LEN              (8)
#define BNO055_GYRO_OFFSET_Z_LSB_REG              BNO055_GYRO_OFFSET_Z_LSB_ADDR

#define BNO055_GYRO_OFFSET_Z_MSB_POS              (0)
#define BNO055_GYRO_OFFSET_Z_MSB_MSK              (0XFF)
#define BNO055_GYRO_OFFSET_Z_MSB_LEN              (8)
#define BNO055_GYRO_OFFSET_Z_MSB_REG              BNO055_GYRO_OFFSET_Z_MSB_ADDR

/* Radius register definition*/
#define BNO055_ACCEL_RADIUS_LSB_POS               (0)
#define BNO055_ACCEL_RADIUS_LSB_MSK               (0XFF)
#define BNO055_ACCEL_RADIUS_LSB_LEN               (8)
#define BNO055_ACCEL_RADIUS_LSB_REG               BNO055_ACCEL_RADIUS_LSB_ADDR

#define BNO055_ACCEL_RADIUS_MSB_POS               (0)
#define BNO055_ACCEL_RADIUS_MSB_MSK               (0XFF)
#define BNO055_ACCEL_RADIUS_MSB_LEN               (8)
#define BNO055_ACCEL_RADIUS_MSB_REG               BNO055_ACCEL_RADIUS_MSB_ADDR

#define BNO055_MAG_RADIUS_LSB_POS                 (0)
#define BNO055_MAG_RADIUS_LSB_MSK                 (0XFF)
#define BNO055_MAG_RADIUS_LSB_LEN                 (8)
#define BNO055_MAG_RADIUS_LSB_REG                 BNO055_MAG_RADIUS_LSB_ADDR

#define BNO055_MAG_RADIUS_MSB_POS                 (0)
#define BNO055_MAG_RADIUS_MSB_MSK                 (0XFF)
#define BNO055_MAG_RADIUS_MSB_LEN                 (8)
#define BNO055_MAG_RADIUS_MSB_REG                 BNO055_MAG_RADIUS_MSB_ADDR

/* PAGE0 DATA REGISTERS DEFINITION END*/
/*************************************************/
/**\name PAGE1 DATA REGISTERS DEFINITION   */
/*************************************************/
/* Configuration registers*/
/* Accel range configuration register*/
#define BNO055_ACCEL_RANGE_POS                 (0)
#define BNO055_ACCEL_RANGE_MSK                 (0X03)
#define BNO055_ACCEL_RANGE_LEN                 (2)
#define BNO055_ACCEL_RANGE_REG                 BNO055_ACCEL_CONFIG_ADDR

/* Accel bandwidth configuration register*/
#define BNO055_ACCEL_BW_POS                    (2)
#define BNO055_ACCEL_BW_MSK                    (0X1C)
#define BNO055_ACCEL_BW_LEN                    (3)
#define BNO055_ACCEL_BW_REG                    BNO055_ACCEL_CONFIG_ADDR

/* Accel power mode configuration register*/
#define BNO055_ACCEL_POWER_MODE_POS            (5)
#define BNO055_ACCEL_POWER_MODE_MSK            (0XE0)
#define BNO055_ACCEL_POWER_MODE_LEN            (3)
#define BNO055_ACCEL_POWER_MODE_REG            BNO055_ACCEL_CONFIG_ADDR

/* Mag data output rate configuration register*/
#define BNO055_MAG_DATA_OUTPUT_RATE_POS        (0)
#define BNO055_MAG_DATA_OUTPUT_RATE_MSK        (0X07)
#define BNO055_MAG_DATA_OUTPUT_RATE_LEN        (3)
#define BNO055_MAG_DATA_OUTPUT_RATE_REG        BNO055_MAG_CONFIG_ADDR

/* Mag operation mode configuration register*/
#define BNO055_MAG_OPERATION_MODE_POS          (3)
#define BNO055_MAG_OPERATION_MODE_MSK          (0X18)
#define BNO055_MAG_OPERATION_MODE_LEN          (2)
#define BNO055_MAG_OPERATION_MODE_REG          BNO055_MAG_CONFIG_ADDR

/* Mag power mode configuration register*/
#define BNO055_MAG_POWER_MODE_POS              (5)
#define BNO055_MAG_POWER_MODE_MSK              (0X60)
#define BNO055_MAG_POWER_MODE_LEN              (2)
#define BNO055_MAG_POWER_MODE_REG              BNO055_MAG_CONFIG_ADDR

/* Gyro range configuration register*/
#define BNO055_GYRO_RANGE_POS                  (0)
#define BNO055_GYRO_RANGE_MSK                  (0X07)
#define BNO055_GYRO_RANGE_LEN                  (3)
#define BNO055_GYRO_RANGE_REG                  BNO055_GYRO_CONFIG_ADDR

/* Gyro bandwidth configuration register*/
#define BNO055_GYRO_BW_POS                     (3)
#define BNO055_GYRO_BW_MSK                     (0X38)
#define BNO055_GYRO_BW_LEN                     (3)
#define BNO055_GYRO_BW_REG                     BNO055_GYRO_CONFIG_ADDR

/* Gyro power mode configuration register*/
#define BNO055_GYRO_POWER_MODE_POS             (0)
#define BNO055_GYRO_POWER_MODE_MSK             (0X07)
#define BNO055_GYRO_POWER_MODE_LEN             (3)
#define BNO055_GYRO_POWER_MODE_REG             BNO055_GYRO_MODE_CONFIG_ADDR

/* Sleep configuration registers*/
/* Accel sleep mode configuration register*/
#define BNO055_ACCEL_SLEEP_MODE_POS            (0)
#define BNO055_ACCEL_SLEEP_MODE_MSK            (0X01)
#define BNO055_ACCEL_SLEEP_MODE_LEN            (1)
#define BNO055_ACCEL_SLEEP_MODE_REG            BNO055_ACCEL_SLEEP_CONFIG_ADDR

/* Accel sleep duration configuration register*/
#define BNO055_ACCEL_SLEEP_DURN_POS            (1)
#define BNO055_ACCEL_SLEEP_DURN_MSK            (0X1E)
#define BNO055_ACCEL_SLEEP_DURN_LEN            (4)
#define BNO055_ACCEL_SLEEP_DURN_REG            BNO055_ACCEL_SLEEP_CONFIG_ADDR

/* Gyro sleep duration configuration register*/
#define BNO055_GYRO_SLEEP_DURN_POS             (0)
#define BNO055_GYRO_SLEEP_DURN_MSK             (0X07)
#define BNO055_GYRO_SLEEP_DURN_LEN             (3)
#define BNO055_GYRO_SLEEP_DURN_REG             BNO055_GYRO_SLEEP_CONFIG_ADDR

/* Gyro auto sleep duration configuration register*/
#define BNO055_GYRO_AUTO_SLEEP_DURN_POS        (3)
#define BNO055_GYRO_AUTO_SLEEP_DURN_MSK        (0X38)
#define BNO055_GYRO_AUTO_SLEEP_DURN_LEN        (3)
#define BNO055_GYRO_AUTO_SLEEP_DURN_REG        BNO055_GYRO_SLEEP_CONFIG_ADDR

/* Mag sleep mode configuration register*/
#define BNO055_MAG_SLEEP_MODE_POS              (0)
#define BNO055_MAG_SLEEP_MODE_MSK              (0X01)
#define BNO055_MAG_SLEEP_MODE_LEN              (1)
#define BNO055_MAG_SLEEP_MODE_REG              BNO055_MAG_SLEEP_CONFIG_ADDR

/* Mag sleep duration configuration register*/
#define BNO055_MAG_SLEEP_DURN_POS              (1)
#define BNO055_MAG_SLEEP_DURN_MSK              (0X1E)
#define BNO055_MAG_SLEEP_DURN_LEN              (4)
#define BNO055_MAG_SLEEP_DURN_REG              BNO055_MAG_SLEEP_CONFIG_ADDR

/* Interrupt registers*/
/* Gyro any motion interrupt msk register*/
#define BNO055_GYRO_ANY_MOTION_INTR_MASK_POS   (2)
#define BNO055_GYRO_ANY_MOTION_INTR_MASK_MSK   (0X04)
#define BNO055_GYRO_ANY_MOTION_INTR_MASK_LEN   (1)
#define BNO055_GYRO_ANY_MOTION_INTR_MASK_REG   BNO055_INT_MASK_ADDR

/* Gyro high rate interrupt msk register*/
#define BNO055_GYRO_HIGHRATE_INTR_MASK_POS     (3)
#define BNO055_GYRO_HIGHRATE_INTR_MASK_MSK     (0X08)
#define BNO055_GYRO_HIGHRATE_INTR_MASK_LEN     (1)
#define BNO055_GYRO_HIGHRATE_INTR_MASK_REG     BNO055_INT_MASK_ADDR

/* Accel high g interrupt msk register*/
#define BNO055_ACCEL_HIGH_G_INTR_MASK_POS      (5)
#define BNO055_ACCEL_HIGH_G_INTR_MASK_MSK      (0X20)
#define BNO055_ACCEL_HIGH_G_INTR_MASK_LEN      (1)
#define BNO055_ACCEL_HIGH_G_INTR_MASK_REG      BNO055_INT_MASK_ADDR

/* Accel any motion interrupt msk register*/
#define BNO055_ACCEL_ANY_MOTION_INTR_MASK_POS  (6)
#define BNO055_ACCEL_ANY_MOTION_INTR_MASK_MSK  (0X40)
#define BNO055_ACCEL_ANY_MOTION_INTR_MASK_LEN  (1)
#define BNO055_ACCEL_ANY_MOTION_INTR_MASK_REG  BNO055_INT_MASK_ADDR

/* Accel any motion interrupt msk register*/
#define BNO055_ACCEL_NO_MOTION_INTR_MASK_POS   (7)
#define BNO055_ACCEL_NO_MOTION_INTR_MASK_MSK   (0X80)
#define BNO055_ACCEL_NO_MOTION_INTR_MASK_LEN   (1)
#define BNO055_ACCEL_NO_MOTION_INTR_MASK_REG   BNO055_INT_MASK_ADDR

/* Gyro any motion interrupt register*/
#define BNO055_GYRO_ANY_MOTION_INTR_POS        (2)
#define BNO055_GYRO_ANY_MOTION_INTR_MSK        (0X04)
#define BNO055_GYRO_ANY_MOTION_INTR_LEN        (1)
#define BNO055_GYRO_ANY_MOTION_INTR_REG        BNO055_INT_ADDR

/* Gyro high rate interrupt register*/
#define BNO055_GYRO_HIGHRATE_INTR_POS          (3)
#define BNO055_GYRO_HIGHRATE_INTR_MSK          (0X08)
#define BNO055_GYRO_HIGHRATE_INTR_LEN          (1)
#define BNO055_GYRO_HIGHRATE_INTR_REG          BNO055_INT_ADDR

/* Accel high g interrupt register*/
#define BNO055_ACCEL_HIGH_G_INTR_POS           (5)
#define BNO055_ACCEL_HIGH_G_INTR_MSK           (0X20)
#define BNO055_ACCEL_HIGH_G_INTR_LEN           (1)
#define BNO055_ACCEL_HIGH_G_INTR_REG           BNO055_INT_ADDR

/* Accel any motion interrupt register*/
#define BNO055_ACCEL_ANY_MOTION_INTR_POS       (6)
#define BNO055_ACCEL_ANY_MOTION_INTR_MSK       (0X40)
#define BNO055_ACCEL_ANY_MOTION_INTR_LEN       (1)
#define BNO055_ACCEL_ANY_MOTION_INTR_REG       BNO055_INT_ADDR

/*Accel any motion interrupt register*/
#define BNO055_ACCEL_NO_MOTION_INTR_POS        (7)
#define BNO055_ACCEL_NO_MOTION_INTR_MSK        (0X80)
#define BNO055_ACCEL_NO_MOTION_INTR_LEN        (1)
#define BNO055_ACCEL_NO_MOTION_INTR_REG        BNO055_INT_ADDR

/*Accel any motion threshold setting*/
#define BNO055_ACCEL_ANY_MOTION_THRES_POS      (0)
#define BNO055_ACCEL_ANY_MOTION_THRES_MSK      (0XFF)
#define BNO055_ACCEL_ANY_MOTION_THRES_LEN      (8)
#define BNO055_ACCEL_ANY_MOTION_THRES_REG      BNO055_ACCEL_ANY_MOTION_THRES_ADDR

/*Accel interrupt setting register*/
#define BNO055_ACCEL_ANY_MOTION_DURN_SET_POS   (0)
#define BNO055_ACCEL_ANY_MOTION_DURN_SET_MSK   (0X03)
#define BNO055_ACCEL_ANY_MOTION_DURN_SET_LEN   (2)
#define BNO055_ACCEL_ANY_MOTION_DURN_SET_REG   BNO055_ACCEL_INTR_SETTINGS_ADDR

/* Accel AM/NM axis selection register*/
#define BNO055_ACCEL_ANY_MOTION_X_AXIS_POS     (2)
#define BNO055_ACCEL_ANY_MOTION_X_AXIS_MSK     (0X04)
#define BNO055_ACCEL_ANY_MOTION_X_AXIS_LEN     (1)
#define BNO055_ACCEL_ANY_MOTION_X_AXIS_REG     BNO055_ACCEL_INTR_SETTINGS_ADDR

#define BNO055_ACCEL_ANY_MOTION_Y_AXIS_POS     (3)
#define BNO055_ACCEL_ANY_MOTION_Y_AXIS_MSK     (0X08)
#define BNO055_ACCEL_ANY_MOTION_Y_AXIS_LEN     (1)
#define BNO055_ACCEL_ANY_MOTION_Y_AXIS_REG     BNO055_ACCEL_INTR_SETTINGS_ADDR

#define BNO055_ACCEL_ANY_MOTION_Z_AXIS_POS     (4)
#define BNO055_ACCEL_ANY_MOTION_Z_AXIS_MSK     (0X10)
#define BNO055_ACCEL_ANY_MOTION_Z_AXIS_LEN     (1)
#define BNO055_ACCEL_ANY_MOTION_Z_AXIS_REG     BNO055_ACCEL_INTR_SETTINGS_ADDR

/* Accel high g axis selection register*/
#define BNO055_ACCEL_HIGH_G_X_AXIS_POS         (5)
#define BNO055_ACCEL_HIGH_G_X_AXIS_MSK         (0X20)
#define BNO055_ACCEL_HIGH_G_X_AXIS_LEN         (1)
#define BNO055_ACCEL_HIGH_G_X_AXIS_REG         BNO055_ACCEL_INTR_SETTINGS_ADDR

#define BNO055_ACCEL_HIGH_G_Y_AXIS_POS         (6)
#define BNO055_ACCEL_HIGH_G_Y_AXIS_MSK         (0X40)
#define BNO055_ACCEL_HIGH_G_Y_AXIS_LEN         (1)
#define BNO055_ACCEL_HIGH_G_Y_AXIS_REG         BNO055_ACCEL_INTR_SETTINGS_ADDR

#define BNO055_ACCEL_HIGH_G_Z_AXIS_POS         (7)
#define BNO055_ACCEL_HIGH_G_Z_AXIS_MSK         (0X80)
#define BNO055_ACCEL_HIGH_G_Z_AXIS_LEN         (1)
#define BNO055_ACCEL_HIGH_G_Z_AXIS_REG         BNO055_ACCEL_INTR_SETTINGS_ADDR

/* Accel High g duration setting register*/
#define BNO055_ACCEL_HIGH_G_DURN_POS           (0)
#define BNO055_ACCEL_HIGH_G_DURN_MSK           (0XFF)
#define BNO055_ACCEL_HIGH_G_DURN_LEN           (8)
#define BNO055_ACCEL_HIGH_G_DURN_REG           BNO055_ACCEL_HIGH_G_DURN_ADDR

/* Accel High g threshold setting register*/
#define BNO055_ACCEL_HIGH_G_THRES_POS          (0)
#define BNO055_ACCEL_HIGH_G_THRES_MSK          (0XFF)
#define BNO055_ACCEL_HIGH_G_THRES_LEN          (8)
#define BNO055_ACCEL_HIGH_G_THRES_REG          BNO055_ACCEL_HIGH_G_THRES_ADDR

/* Accel no/slow motion threshold setting*/
#define BNO055_ACCEL_SLOW_NO_MOTION_THRES_POS  (0)
#define BNO055_ACCEL_SLOW_NO_MOTION_THRES_MSK  (0XFF)
#define BNO055_ACCEL_SLOW_NO_MOTION_THRES_LEN  (8)
#define BNO055_ACCEL_SLOW_NO_MOTION_THRES_REG   \
    BNO055_ACCEL_NO_MOTION_THRES_ADDR

/* Accel no/slow motion enable setting*/
#define BNO055_ACCEL_SLOW_NO_MOTION_ENABLE_POS (0)
#define BNO055_ACCEL_SLOW_NO_MOTION_ENABLE_MSK (0X01)
#define BNO055_ACCEL_SLOW_NO_MOTION_ENABLE_LEN (1)
#define BNO055_ACCEL_SLOW_NO_MOTION_ENABLE_REG BNO055_ACCEL_NO_MOTION_SET_ADDR

/* Accel no/slow motion duration setting*/
#define BNO055_ACCEL_SLOW_NO_MOTION_DURN_POS   (1)
#define BNO055_ACCEL_SLOW_NO_MOTION_DURN_MSK   (0X7E)
#define BNO055_ACCEL_SLOW_NO_MOTION_DURN_LEN   (6)
#define BNO055_ACCEL_SLOW_NO_MOTION_DURN_REG   BNO055_ACCEL_NO_MOTION_SET_ADDR

/*Gyro interrupt setting register*/
/*Gyro any motion axis setting*/
#define BNO055_GYRO_ANY_MOTION_X_AXIS_POS      (0)
#define BNO055_GYRO_ANY_MOTION_X_AXIS_MSK      (0X01)
#define BNO055_GYRO_ANY_MOTION_X_AXIS_LEN      (1)
#define BNO055_GYRO_ANY_MOTION_X_AXIS_REG      BNO055_GYRO_INTR_SETING_ADDR

#define BNO055_GYRO_ANY_MOTION_Y_AXIS_POS      (1)
#define BNO055_GYRO_ANY_MOTION_Y_AXIS_MSK      (0X02)
#define BNO055_GYRO_ANY_MOTION_Y_AXIS_LEN      (1)
#define BNO055_GYRO_ANY_MOTION_Y_AXIS_REG      BNO055_GYRO_INTR_SETING_ADDR

#define BNO055_GYRO_ANY_MOTION_Z_AXIS_POS      (2)
#define BNO055_GYRO_ANY_MOTION_Z_AXIS_MSK      (0X04)
#define BNO055_GYRO_ANY_MOTION_Z_AXIS_LEN      (1)
#define BNO055_GYRO_ANY_MOTION_Z_AXIS_REG      BNO055_GYRO_INTR_SETING_ADDR

/*Gyro high rate axis setting*/
#define BNO055_GYRO_HIGHRATE_X_AXIS_POS        (3)
#define BNO055_GYRO_HIGHRATE_X_AXIS_MSK        (0X08)
#define BNO055_GYRO_HIGHRATE_X_AXIS_LEN        (1)
#define BNO055_GYRO_HIGHRATE_X_AXIS_REG        BNO055_GYRO_INTR_SETING_ADDR

#define BNO055_GYRO_HIGHRATE_Y_AXIS_POS        (4)
#define BNO055_GYRO_HIGHRATE_Y_AXIS_MSK        (0X10)
#define BNO055_GYRO_HIGHRATE_Y_AXIS_LEN        (1)
#define BNO055_GYRO_HIGHRATE_Y_AXIS_REG        BNO055_GYRO_INTR_SETING_ADDR

#define BNO055_GYRO_HIGHRATE_Z_AXIS_POS        (5)
#define BNO055_GYRO_HIGHRATE_Z_AXIS_MSK        (0X20)
#define BNO055_GYRO_HIGHRATE_Z_AXIS_LEN        (1)
#define BNO055_GYRO_HIGHRATE_Z_AXIS_REG        BNO055_GYRO_INTR_SETING_ADDR

/* Gyro filter setting*/
#define BNO055_GYRO_ANY_MOTION_FILTER_POS      (6)
#define BNO055_GYRO_ANY_MOTION_FILTER_MSK      (0X40)
#define BNO055_GYRO_ANY_MOTION_FILTER_LEN      (1)
#define BNO055_GYRO_ANY_MOTION_FILTER_REG      BNO055_GYRO_INTR_SETING_ADDR

#define BNO055_GYRO_HIGHRATE_FILTER_POS        (7)
#define BNO055_GYRO_HIGHRATE_FILTER_MSK        (0X80)
#define BNO055_GYRO_HIGHRATE_FILTER_LEN        (1)
#define BNO055_GYRO_HIGHRATE_FILTER_REG        BNO055_GYRO_INTR_SETING_ADDR

/* Gyro high rate X axis settings*/
#define BNO055_GYRO_HIGHRATE_X_THRES_POS       (0)
#define BNO055_GYRO_HIGHRATE_X_THRES_MSK       (0X1F)
#define BNO055_GYRO_HIGHRATE_X_THRES_LEN       (5)
#define BNO055_GYRO_HIGHRATE_X_THRES_REG       BNO055_GYRO_HIGHRATE_X_SET_ADDR

#define BNO055_GYRO_HIGHRATE_X_HYST_POS        (5)
#define BNO055_GYRO_HIGHRATE_X_HYST_MSK        (0X60)
#define BNO055_GYRO_HIGHRATE_X_HYST_LEN        (2)
#define BNO055_GYRO_HIGHRATE_X_HYST_REG        BNO055_GYRO_HIGHRATE_X_SET_ADDR

#define BNO055_GYRO_HIGHRATE_X_DURN_POS        (0)
#define BNO055_GYRO_HIGHRATE_X_DURN_MSK        (0XFF)
#define BNO055_GYRO_HIGHRATE_X_DURN_LEN        (8)
#define BNO055_GYRO_HIGHRATE_X_DURN_REG        BNO055_GYRO_DURN_X_ADDR

/* Gyro high rate Y axis settings*/
#define BNO055_GYRO_HIGHRATE_Y_THRES_POS       (0)
#define BNO055_GYRO_HIGHRATE_Y_THRES_MSK       (0X1F)
#define BNO055_GYRO_HIGHRATE_Y_THRES_LEN       (5)
#define BNO055_GYRO_HIGHRATE_Y_THRES_REG       BNO055_GYRO_HIGHRATE_Y_SET_ADDR

#define BNO055_GYRO_HIGHRATE_Y_HYST_POS        (5)
#define BNO055_GYRO_HIGHRATE_Y_HYST_MSK        (0X60)
#define BNO055_GYRO_HIGHRATE_Y_HYST_LEN        (2)
#define BNO055_GYRO_HIGHRATE_Y_HYST_REG        BNO055_GYRO_HIGHRATE_Y_SET_ADDR

#define BNO055_GYRO_HIGHRATE_Y_DURN_POS        (0)
#define BNO055_GYRO_HIGHRATE_Y_DURN_MSK        (0XFF)
#define BNO055_GYRO_HIGHRATE_Y_DURN_LEN        (8)
#define BNO055_GYRO_HIGHRATE_Y_DURN_REG        BNO055_GYRO_DURN_Y_ADDR

/* Gyro high rate Z axis settings*/
#define BNO055_GYRO_HIGHRATE_Z_THRES_POS       (0)
#define BNO055_GYRO_HIGHRATE_Z_THRES_MSK       (0X1F)
#define BNO055_GYRO_HIGHRATE_Z_THRES_LEN       (5)
#define BNO055_GYRO_HIGHRATE_Z_THRES_REG       BNO055_GYRO_HIGHRATE_Z_SET_ADDR

#define BNO055_GYRO_HIGHRATE_Z_HYST_POS        (5)
#define BNO055_GYRO_HIGHRATE_Z_HYST_MSK        (0X60)
#define BNO055_GYRO_HIGHRATE_Z_HYST_LEN        (2)
#define BNO055_GYRO_HIGHRATE_Z_HYST_REG        BNO055_GYRO_HIGHRATE_Z_SET_ADDR

#define BNO055_GYRO_HIGHRATE_Z_DURN_POS        (0)
#define BNO055_GYRO_HIGHRATE_Z_DURN_MSK        (0XFF)
#define BNO055_GYRO_HIGHRATE_Z_DURN_LEN        (8)
#define BNO055_GYRO_HIGHRATE_Z_DURN_REG        (BNO055_GYRO_DURN_Z_ADDR)

/*Gyro any motion threshold setting*/
#define BNO055_GYRO_ANY_MOTION_THRES_POS       (0)
#define BNO055_GYRO_ANY_MOTION_THRES_MSK       (0X7F)
#define BNO055_GYRO_ANY_MOTION_THRES_LEN       (7)
#define BNO055_GYRO_ANY_MOTION_THRES_REG    \
    BNO055_GYRO_ANY_MOTION_THRES_ADDR

/* Gyro any motion slope sample setting*/
#define BNO055_GYRO_SLOPE_SAMPLES_POS          (0)
#define BNO055_GYRO_SLOPE_SAMPLES_MSK          (0X03)
#define BNO055_GYRO_SLOPE_SAMPLES_LEN          (2)
#define BNO055_GYRO_SLOPE_SAMPLES_REG          BNO055_GYRO_ANY_MOTION_SET_ADDR

/* Gyro awake duration setting*/
#define BNO055_GYRO_AWAKE_DURN_POS             (2)
#define BNO055_GYRO_AWAKE_DURN_MSK             (0X0C)
#define BNO055_GYRO_AWAKE_DURN_LEN             (2)
#define BNO055_GYRO_AWAKE_DURN_REG             BNO055_GYRO_ANY_MOTION_SET_ADDR

/* PAGE1 DATA REGISTERS DEFINITION END*/
/*************************************************/
/**\name GET AND SET BITSLICE FUNCTIONS    */
/*************************************************/
#define BNO055_GET_BITSLICE(regvar, bitname) \
    ((regvar & bitname##_MSK) >> bitname##_POS)

#define BNO055_SET_BITSLICE(regvar, bitname, val) \
    ((regvar & ~bitname##_MSK) | ((val << bitname##_POS) & bitname##_MSK))

/*************************************************/
/**\name FUNCTION DECLARATION    */
/*************************************************/
/**************************************************/
/**\name INITIALIZATION AND REVISION ID FUNCTIONS */
/**************************************************/

/*!
 *  @brief
 *  This API is used for initialize
 *  bus read, bus write function pointers,device
 *  address,accel revision id, gyro revision id
 *  mag revision id, software revision id, boot loader
 *  revision id and page id
 *
 *  @param  bno055 - structure pointer
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While changing the parameter of the bno055_t
 *  consider the following point:
 *  Changing the reference value of the parameter
 *  will changes the local copy or local reference
 *  make sure your changes will not
 *  affect the reference value of the parameter
 *  (Better case don't change the reference value of the parameter)
 */
BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055);

/*!
 *  @brief
 *  This API gives data to the given register and
 *  the data is written in the corresponding register address
 *
 *  @param addr_u8 : Address of the register
 *  @param data_u8 : Data to be written to the register
 *  @param len_u8  : Length of the Data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_register(u8 addr_u8, u8 *data_u8, u8 len_u8);

/*!
 *  @brief This API reads the data from
 *  the given register address
 *
 *  @param addr_u8 : Address of the register
 *  @param data_u8 : address of the variable,
 *  read value will be kept
 *  @param len_u8  : Length of the data
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_register(u8 addr_u8, u8 *data_u8, u8 len_u8);

/*!
 *  @brief This API reads chip id
 *  from register 0x00 it is a byte of data
 *
 *
 *  @param chip_id_u8 : The chip id value 0xA0
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_chip_id(u8 *chip_id_u8);

/*!
 *  @brief This API reads software revision id
 *  from register 0x04 and 0x05 it is a two byte of data
 *
 *  @param sw_id_u8 : The SW revision id
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_sw_rev_id(u16 *sw_id_u8);

/*!
 *  @brief This API reads page id
 *  from register 0x07 it is a byte of data
 *
 *
 *  @param page_id_u8 : The value of page id
 *
 *  BNO055_PAGE_ZERO -> 0x00
 *  BNO055_PAGE_ONE  -> 0x01
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_page_id(u8 *page_id_u8);

/*!
 *  @brief This API used to write
 *  the page id register 0x07
 *
 *  @param page_id_u8 : The value of page id
 *
 *  BNO055_PAGE_ZERO -> 0x00
 *  BNO055_PAGE_ONE  -> 0x01
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(u8 page_id_u8);

/*!
 *  @brief This API reads accel revision id
 *  from register 0x01 it is a byte of value
 *
 *  @param accel_rev_id_u8 : The accel revision id 0xFB
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_rev_id(u8 *accel_rev_id_u8);

/*!
 *  @brief This API reads mag revision id
 *  from register 0x02 it is a byte of value
 *
 *  @param mag_rev_id_u8 : The mag revision id 0x32
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_rev_id(u8 *mag_rev_id_u8);

/*!
 *  @brief This API reads gyro revision id
 *  from register 0x03 it is a byte of value
 *
 *  @param gyro_rev_id_u8 : The gyro revision id 0xF0
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_rev_id(u8 *gyro_rev_id_u8);

/*!
 *  @brief This API used to read boot loader revision id
 *  from register 0x06 it is a byte of value
 *
 *  @param bl_rev_id_u8 : The boot loader revision id
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_bl_rev_id(u8 *bl_rev_id_u8);

/**************************************************/
/**\name ACCEL DATA READ FUNCTIONS */
/**************************************************/

/*!
 *  @brief This API reads acceleration data X values
 *  from register 0x08 and 0x09 it is a two byte data
 *
 *
 *
 *
 *  @param accel_x_s16 : The X raw data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_x(s16 *accel_x_s16);

/*!
 *  @brief This API reads acceleration data Y values
 *  from register 0x0A and 0x0B it is a two byte data
 *
 *
 *
 *
 *  @param accel_y_s16 : The Y raw data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_y(s16 *accel_y_s16);

/*!
 *  @brief This API reads acceleration data z values
 *  from register 0x0C and 0x0D it is a two byte data
 *
 *
 *
 *
 *  @param accel_z_s16 : The z raw data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_z(s16 *accel_z_s16);

/*!
 *  @brief This API reads acceleration data xyz values
 *  from register 0x08 to 0x0D it is a six byte data
 *
 *
 *  @param accel : The value of accel xyz data
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | The accel x data
 *   y        | The accel y data
 *   z        | The accel z data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_xyz(struct bno055_accel_t *accel);

/**************************************************/
/**\name MAG DATA READ FUNCTIONS */
/**************************************************/

/*!
 *  @brief This API reads mag data x values
 *  from register 0x0E and 0x0F it is a two byte data
 *
 *
 *
 *
 *  @param mag_x_s16 : The x raw data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_x(s16 *mag_x_s16);

/*!
 *  @brief This API reads mag data y values
 *  from register 0x10 and 0x11 it is a two byte data
 *
 *
 *
 *
 *  @param mag_y_s16 : The y raw data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_y(s16 *mag_y_s16);

/*!
 *  @brief This API reads mag data z values
 *  from register 0x12 and 0x13 it is a two byte data
 *
 *
 *
 *
 *  @param mag_z_s16 : The z raw data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_z(s16 *mag_z_s16);

/*!
 *  @brief This API reads mag data xyz values
 *  from register 0x0E to 0x13 it is a six byte data
 *
 *
 *  @param mag : The mag xyz values
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | The mag x data
 *   y        | The mag y data
 *   z        | The mag z data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_xyz(struct bno055_mag_t *mag);

/**************************************************/
/**\name GYRO DATA READ FUNCTIONS */
/**************************************************/

/*!
 *  @brief This API reads gyro data x values
 *  from register 0x14 and 0x15 it is a two byte data
 *
 *
 *
 *
 *  @param gyro_x_s16 : The x raw data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_x(s16 *gyro_x_s16);

/*!
 *  @brief This API reads gyro data y values
 *  from register 0x16 and 0x17 it is a two byte data
 *
 *
 *
 *
 *  @param gyro_y_s16 : The y raw data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_y(s16 *gyro_y_s16);

/*!
 *  @brief This API reads gyro data z values
 *  from register 0x18 and 0x19 it is a two byte data
 *
 *  @param gyro_z_s16 : The z raw data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_z(s16 *gyro_z_s16);

/*!
 *  @brief This API reads gyro data xyz values
 *  from register 0x14 to 0x19 it is a six byte data
 *
 *
 *  @param gyro : The value of gyro xyz data's
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | The gyro x data
 *   y        | The gyro y data
 *   z        | The gyro z data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_xyz(struct bno055_gyro_t *gyro);

/**************************************************/
/**\name EULER DATA READ FUNCTIONS */
/**************************************************/

/*!
 *  @brief This API reads gyro data z values
 *  from register 0x1A and 0x1B it is a two byte data
 *
 *  @param euler_h_s16 : The raw h data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_h(s16 *euler_h_s16);

/*!
 *  @brief This API reads Euler data r values
 *  from register 0x1C and 0x1D it is a two byte data
 *
 *  @param euler_r_s16 : The raw r data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_r(s16 *euler_r_s16);

/*!
 *  @brief This API reads Euler data p values
 *  from register 0x1E and 0x1F it is a two byte data
 *
 *  @param euler_p_s16 : The raw p data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_p(s16 *euler_p_s16);

/*!
 *  @brief This API reads Euler data hrp values
 *  from register 0x1A to 0x1F it is a six byte data
 *
 *
 *  @param euler : The Euler hrp data's
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   h        | The Euler h data
 *   r        | The Euler r data
 *   p        | The Euler p data
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_hrp(struct bno055_euler_t *euler);

/**************************************************/
/**\name QUATERNION DATA READ FUNCTIONS */
/**************************************************/

/*!
 *  @brief This API reads quaternion data w values
 *  from register 0x20 and 0x21 it is a two byte data
 *
 *  @param quaternion_w_s16 : The raw w data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_w(s16 *quaternion_w_s16);

/*!
 *  @brief This API reads quaternion data x values
 *  from register 0x22 and 0x23 it is a two byte data
 *
 *  @param quaternion_x_s16 : The raw x data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_x(s16 *quaternion_x_s16);

/*!
 *  @brief This API reads quaternion data y values
 *  from register 0x24 and 0x25 it is a two byte data
 *
 *  @param quaternion_y_s16 : The raw y data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_y(s16 *quaternion_y_s16);

/*!
 *  @brief This API reads quaternion data z values
 *  from register 0x26 and 0x27 it is a two byte data
 *
 *  @param quaternion_z_s16 : The raw z data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_z(s16 *quaternion_z_s16);

/*!
 *  @brief This API reads Quaternion data wxyz values
 *  from register 0x20 to 0x27 it is a six byte data
 *
 *
 *  @param quaternion : The value of quaternion wxyz data's
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   w        | The quaternion w data
 *   x        | The quaternion x data
 *   y        | The quaternion y data
 *   z        | The quaternion z data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_wxyz(struct bno055_quaternion_t *quaternion);

/**************************************************/
/**\name LINEAR ACCEL DATA READ FUNCTIONS */
/**************************************************/

/*!
 *  @brief This API reads Linear accel data x values
 *  from register 0x29 and 0x2A it is a two byte data
 *
 *  @param linear_accel_x_s16 : The raw x data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_x(s16 *linear_accel_x_s16);

/*!
 *  @brief This API reads Linear accel data x values
 *  from register 0x2B and 0x2C it is a two byte data
 *
 *  @param linear_accel_y_s16 : The raw y data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_y(s16 *linear_accel_y_s16);

/*!
 *  @brief This API reads Linear accel data x values
 *  from register 0x2C and 0x2D it is a two byte data
 *
 *  @param linear_accel_z_s16 : The raw z data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_z(s16 *linear_accel_z_s16);

/*!
 *  @brief This API reads Linear accel data xyz values
 *  from register 0x28 to 0x2D it is a six byte data
 *
 *
 *  @param linear_accel : The value of linear accel xyz data's
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | The linear accel x data
 *   y        | The linear accel y data
 *   z        | The linear accel z data
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_xyz(struct bno055_linear_accel_t *linear_accel);

/**************************************************/
/**\name GRAVITY DATA READ FUNCTIONS */
/**************************************************/

/*!
 *  @brief This API reads gravity data x values
 *  from register 0x2E and 0x2F it is a two byte data
 *
 *  @param gravity_x_s16 : The raw x data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_x(s16 *gravity_x_s16);

/*!
 *  @brief This API reads gravity data y values
 *  from register 0x30 and 0x31 it is a two byte data
 *
 *  @param gravity_y_s16 : The raw y data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_y(s16 *gravity_y_s16);

/*!
 *  @brief This API reads gravity data z values
 *  from register 0x32 and 0x33 it is a two byte data
 *
 *  @param gravity_z_s16 : The raw z data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_z(s16 *gravity_z_s16);

/*!
 *  @brief This API reads gravity data xyz values
 *  from register 0x2E to 0x33 it is a six byte data
 *
 *
 *  @param gravity : The value of gravity xyz data's
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | The gravity x data
 *   y        | The gravity y data
 *   z        | The gravity z data
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_xyz(struct bno055_gravity_t *gravity);

/**************************************************/
/**\name TEMPERATURE DATA READ FUNCTIONS */
/**************************************************/

/*!
 *  @brief This API reads temperature values
 *  from register 0x33 it is a byte data
 *
 *  @param temp_s8 : The raw temperature data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_temp_data(s8 *temp_s8);

#ifdef  BNO055_FLOAT_ENABLE

/********************************************************************/
/**\name FUNCTIONS FOR READING ACCEL DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/

/*!
 *  @brief This API is used to convert the accel x raw data
 *  to meterpersecseq output as float
 *
 *  @param accel_x_f : The accel x meterpersecseq data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_x_msq(float *accel_x_f);

/*!
 *  @brief This API is used to convert the accel x raw data
 *  to meterpersecseq output as float
 *
 *  @param accel_y_f : The accel y meterpersecseq data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_y_msq(float *accel_y_f);

/*!
 *  @brief This API is used to convert the accel z raw data
 *  to meterpersecseq output as float
 *
 *  @param accel_z_f : The accel z meterpersecseq data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_z_msq(float *accel_z_f);

/*!
 *  @brief This API is used to convert the accel y raw data
 *  to millig output as float
 *
 *  @param accel_x_f : The accel y millig data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_x_mg(float *accel_x_f);

/*!
 *  @brief This API is used to convert the accel y raw data
 *  to millig output as float
 *
 *  @param accel_y_f : The accel y millig data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_y_mg(float *accel_y_f);

/*!
 *  @brief This API is used to convert the accel z raw data
 *  to millig output as float
 *
 *  @param accel_z_f : The accel z millig data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_z_mg(float *accel_z_f);

/*!
 *  @brief This API is used to convert the accel xyz raw data
 *  to meterpersecseq output as float
 *
 *  @param accel_xyz : The meterpersecseq data of accel xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | meterpersecseq data of accel
 *   y        | meterpersecseq data of accel
 *   z        | meterpersecseq data of accel
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_xyz_msq(struct bno055_accel_float_t *accel_xyz);

/*!
 *  @brief This API is used to convert the accel xyz raw data
 *  to millig output as float
 *
 *  @param accel_xyz : The millig data of accel xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | millig data of accel
 *   y        | millig data of accel
 *   z        | millig data of accel
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_xyz_mg(struct bno055_accel_float_t *accel_xyz);

/********************************************************************/
/**\name FUNCTIONS FOR READING MAG DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/

/*!
 *  @brief This API is used to convert the mag x raw data
 *  to microTesla output as float
 *
 *  @param mag_x_f : The mag x microTesla data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_x_uT(float *mag_x_f);

/*!
 *  @brief This API is used to convert the mag y raw data
 *  to microTesla output as float
 *
 *  @param mag_y_f : The mag y microTesla data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_y_uT(float *mag_y_f);

/*!
 *  @brief This API is used to convert the mag z raw data
 *  to microTesla output as float
 *
 *  @param mag_z_f : The mag z microTesla data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_z_uT(float *mag_z_f);

/*!
 *  @brief This API is used to convert the mag yz raw data
 *  to microTesla output as float
 *
 *  @param mag_xyz_data : The microTesla data of mag xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *    x       | microTesla data of mag
 *    y       | microTesla data of mag
 *    z       | microTesla data of mag
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_xyz_uT(struct bno055_mag_float_t *mag_xyz_data);

/********************************************************************/
/**\name FUNCTIONS FOR READING GYRO DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/

/*!
 *  @brief This API is used to convert the gyro x raw data
 *  to dps output as float
 *
 *  @param gyro_x_f : The gyro x dps float data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_x_dps(float *gyro_x_f);

/*!
 *  @brief This API is used to convert the gyro x raw data
 *  to rps output as float
 *
 *  @param gyro_x_f : The gyro x dps float data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_x_rps(float *gyro_x_f);

/*!
 *  @brief This API is used to convert the gyro y raw data
 *  to dps output as float
 *
 *  @param gyro_y_f : The gyro y dps float data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_y_dps(float *gyro_y_f);

/*!
 *  @brief This API is used to convert the gyro y raw data
 *  to rps output as float
 *
 *  @param gyro_y_f : The gyro y dps float data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_y_rps(float *gyro_y_f);

/*!
 *  @brief This API is used to convert the gyro z raw data
 *  to dps output as float
 *
 *  @param gyro_z_f : The gyro z dps float data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_z_dps(float *gyro_z_f);

/*!
 *  @brief This API is used to convert the gyro z raw data
 *  to rps output as float
 *
 *  @param gyro_z_f : The gyro z rps float data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_z_rps(float *gyro_z_f);

/*!
 *  @brief This API is used to convert the gyro xyz raw data
 *  to dps output as float
 *
 *  @param gyro_xyz_data : The dps data of gyro xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *    x       | dps data of gyro
 *    y       | dps data of gyro
 *    z       | dps data of gyro
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_xyz_dps(struct bno055_gyro_float_t *gyro_xyz_data);

/*!
 *  @brief This API is used to convert the gyro xyz raw data
 *  to rps output as float
 *
 *  @param gyro_xyz_data : The rps data of gyro xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *    x       | rps data of gyro
 *    y       | rps data of gyro
 *    z       | rps data of gyro
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_xyz_rps(struct bno055_gyro_float_t *gyro_xyz_data);

/********************************************************************/
/**\name FUNCTIONS FOR READING EULER DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/

/*!
 *  @brief This API is used to convert the Euler h raw data
 *  to degree output as float
 *
 *  @param euler_h_f : The float value of Euler h degree
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_h_deg(float *euler_h_f);

/*!
 *  @brief This API is used to convert the Euler h raw data
 *  to radians output as float
 *
 *  @param euler_h_f : The float value of Euler h radians
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_h_rad(float *euler_h_f);

/*!
 *  @brief This API is used to convert the Euler r raw data
 *  to degree output as float
 *
 *  @param euler_r_f : The float value of Euler r degree
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_r_deg(float *euler_r_f);

/*!
 *  @brief This API is used to convert the Euler r raw data
 *  to radians output as float
 *
 *  @param euler_r_f : The float value of Euler r radians
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_r_rad(float *euler_r_f);

/*!
 *  @brief This API is used to convert the Euler p raw data
 *  to degree output as float
 *
 *  @param euler_p_f : The float value of Euler p degree
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_p_deg(float *euler_p_f);

/*!
 *  @brief This API is used to convert the Euler p raw data
 *  to radians output as float
 *
 *  @param euler_p_f : The float value of Euler p radians
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_p_rad(float *euler_p_f);

/*!
 *  @brief This API is used to convert the Euler hrp raw data
 *  to degree output as float
 *
 *  @param euler_hpr : The degree data of Euler hrp
 *
 *  Parameter |    result
 *  --------- | -----------------
 *    h       | degree data of Euler
 *    r       | degree data of Euler
 *    p       | degree data of Euler
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_hpr_deg(struct bno055_euler_float_t *euler_hpr);

/*!
 *  @brief This API is used to convert the Euler xyz raw data
 *  to radians output as float
 *
 *  @param euler_hpr : The radians data of Euler hrp
 *
 *  Parameter |    result
 *  --------- | -----------------
 *    h       | radians data of Euler
 *    r       | radians data of Euler
 *    p       | radians data of Euler
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_hpr_rad(struct bno055_euler_float_t *euler_hpr);

/***************************************************************************/
/**\name FUNCTIONS FOR READING LINEAR ACCEL DATA OUTPUT AS FLOAT PRECISION */
/**************************************************************************/

/*!
 *  @brief This API is used to convert the linear
 *  accel x raw data to meterpersecseq output as float
 *
 *  @param linear_accel_x_f : The float value of linear accel x meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_x_msq(float *linear_accel_x_f);

/*!
 *  @brief This API is used to convert the linear
 *  accel y raw data to meterpersecseq output as float
 *
 *  @param linear_accel_y_f : The float value of linear accel y meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_y_msq(float *linear_accel_y_f);

/*!
 *  @brief This API is used to convert the linear
 *  accel z raw data to meterpersecseq output as float
 *
 *  @param linear_accel_z_f : The float value of linear accel z meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_z_msq(float *linear_accel_z_f);

/*!
 *  @brief This API is used to convert the linear accel xyz raw data
 *  to meterpersecseq output as float
 *
 *  @param linear_accel_xyz : The meterpersecseq data of linear accel xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *    x       | meterpersecseq data of linear accel
 *    y       | meterpersecseq data of linear accel
 *    z       | meterpersecseq data of linear accel
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_xyz_msq(
    struct bno055_linear_accel_float_t *linear_accel_xyz);

/********************************************************************/
/**\name FUNCTIONS FOR READING GRAVITY DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/

/*!
 *  @brief This API is used to convert the gravity
 *  x raw data to meterpersecseq output as float
 *
 *  @param gravity_x_f : The float value of gravity x meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_x_msq(float *gravity_x_f);

/*!
 *  @brief This API is used to convert the gravity
 *  y raw data to meterpersecseq output as float
 *
 *  @param gravity_y_f : The float value of gravity y meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_y_msq(float *gravity_y_f);

/*!
 *  @brief This API is used to convert the gravity
 *  z raw data to meterpersecseq output as float
 *
 *  @param gravity_z_f : The float value of gravity z meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_z_msq(float *gravity_z_f);

/*!
 *  @brief This API is used to convert the gravity xyz raw data
 *  to meterpersecseq output as float
 *
 *  @param gravity_xyz : The meterpersecseq data of gravity xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *    x       | meterpersecseq data of gravity
 *    y       | meterpersecseq data of gravity
 *    z       | meterpersecseq data of gravity
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gravity_xyz_msq(struct bno055_gravity_float_t *gravity_xyz);

/**************************************************************************/
/**\name FUNCTIONS FOR READING TEMPERATURE DATA OUTPUT AS FLOAT PRECISION */
/*************************************************************************/

/*!
 *  @brief This API is used to convert the temperature
 *  data to Fahrenheit output as float
 *
 *  @param temp_f : The float value of temperature Fahrenheit
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_temp_fahrenheit(float *temp_f);

/*!
 *  @brief This API is used to convert the temperature
 *  data to Celsius output as float
 *
 *  @param temp_f : The float value of temperature Celsius
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_temp_celsius(float *temp_f);

#endif
#ifdef  BNO055_DOUBLE_ENABLE

/**************************************************************************/
/**\name FUNCTIONS FOR READING ACCEL DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/

/*!
 *  @brief This API is used to convert the accel x raw data
 *  to meterpersecseq output as double
 *
 *  @param accel_x_d : The accel x meterpersecseq data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_x_msq(double *accel_x_d);

/*!
 *  @brief This API is used to convert the accel y raw data
 *  to meterpersecseq output as double
 *
 *  @param accel_y_d : The accel y meterpersecseq data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_y_msq(double *accel_y_d);

/*!
 *  @brief This API is used to convert the accel z raw data
 *  to meterpersecseq output as double
 *
 *  @param accel_z_d : The accel z meterpersecseq data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_z_msq(double *accel_z_d);

/*!
 *  @brief This API is used to convert the accel x raw data
 *  to millig output as double
 *
 *  @param accel_x_d : The accel x millig data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_x_mg(double *accel_x_d);

/*!
 *  @brief This API is used to convert the accel y raw data
 *  to millig output as double
 *
 *  @param accel_y_d : The accel y millig data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_y_mg(double *accel_y_d);

/*!
 *  @brief This API is used to convert the accel z raw data
 *  to millig output as double
 *
 *  @param accel_z_d : The accel z millig data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_z_mg(double *accel_z_d);

/*!
 *  @brief This API is used to convert the accel xyz raw data
 *  to meterpersecseq output as double
 *
 *  @param accel_xyz : The meterpersecseq data of accel xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | meterpersecseq data of accel
 *   y        | meterpersecseq data of accel
 *   z        | meterpersecseq data of accel
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_xyz_msq(struct bno055_accel_double_t *accel_xyz);

/*!
 *  @brief This API is used to convert the accel xyz raw data
 *  to millig output as double
 *
 *  @param accel_xyz : The millig data of accel xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | millig data of accel
 *   y        | millig data of accel
 *   z        | millig data of accel
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_xyz_mg(struct bno055_accel_double_t *accel_xyz);

/**************************************************************************/
/**\name FUNCTIONS FOR READING MAG DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/

/*!
 *  @brief This API is used to convert the mag x raw data
 *  to microTesla output as double
 *
 *  @param mag_x_d : The mag x microTesla data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_x_uT(double *mag_x_d);

/*!
 *  @brief This API is used to convert the mag x raw data
 *  to microTesla output as double
 *
 *  @param mag_y_d : The mag x microTesla data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_y_uT(double *mag_y_d);

/*!
 *  @brief This API is used to convert the mag z raw data
 *  to microTesla output as double
 *
 *  @param mag_z_d : The mag z microTesla data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_z_uT(double *mag_z_d);

/*!
 *  @brief This API is used to convert the mag yz raw data
 *  to microTesla output as double
 *
 *  @param mag_xyz : The microTesla data of mag xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | microTesla data of mag
 *   y        | microTesla data of mag
 *   z        | microTesla data of mag
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_xyz_uT(struct bno055_mag_double_t *mag_xyz);

/**************************************************************************/
/**\name FUNCTIONS FOR READING GYRO DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/

/*!
 *  @brief This API is used to convert the gyro x raw data
 *  to dps output as double
 *
 *  @param gyro_x_d : The gyro x dps double data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_x_dps(double *gyro_x_d);

/*!
 *  @brief This API is used to convert the gyro y raw data
 *  to dps output as double
 *
 *  @param gyro_y_d : The gyro y dps double data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_y_dps(double *gyro_y_d);

/*!
 *  @brief This API is used to convert the gyro z raw data
 *  to dps output as double
 *
 *  @param gyro_z_d : The gyro z dps double data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_z_dps(double *gyro_z_d);

/*!
 *  @brief This API is used to convert the gyro x raw data
 *  to rps output as double
 *
 *  @param gyro_x_d : The gyro x dps double data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_x_rps(double *gyro_x_d);

/*!
 *  @brief This API is used to convert the gyro y raw data
 *  to rps output as double
 *
 *  @param gyro_y_d : The gyro y dps double data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_y_rps(double *gyro_y_d);

/*!
 *  @brief This API is used to convert the gyro z raw data
 *  to rps output as double
 *
 *  @param gyro_z_d : The gyro z rps double data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_z_rps(double *gyro_z_d);

/*!
 *  @brief This API is used to convert the gyro xyz raw data
 *  to dps output as double
 *
 *  @param gyro_xyz : The dps data of gyro xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | dps data of gyro
 *   y        | dps data of gyro
 *   z        | dps data of gyro
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_xyz_dps(struct bno055_gyro_double_t *gyro_xyz);

/*!
 *  @brief This API is used to convert the gyro xyz raw data
 *  to rps output as double
 *
 *  @param gyro_xyz : The rps data of gyro xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | rps data of gyro
 *   y        | rps data of gyro
 *   z        | rps data of gyro
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_xyz_rps(struct bno055_gyro_double_t *gyro_xyz);

/**************************************************************************/
/**\name FUNCTIONS FOR READING EULER DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/

/*!
 *  @brief This API is used to convert the Euler h raw data
 *  to degree output as double
 *
 *  @param euler_h_d : The double value of Euler h degree
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_h_deg(double *euler_h_d);

/*!
 *  @brief This API is used to convert the Euler p raw data
 *  to degree output as double
 *
 *  @param euler_p_d : The double value of Euler p degree
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_p_deg(double *euler_p_d);

/*!
 *  @brief This API is used to convert the Euler r raw data
 *  to degree output as double
 *
 *  @param euler_r_d : The double value of Euler r degree
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_r_deg(double *euler_r_d);

/*!
 *  @brief This API is used to convert the Euler h raw data
 *  to radians output as double
 *
 *  @param euler_h_d : The double value of Euler h radians
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_h_rad(double *euler_h_d);

/*!
 *  @brief This API is used to convert the Euler p raw data
 *  to radians output as double
 *
 *  @param euler_p_d : The double value of Euler p radians
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_p_rad(double *euler_p_d);

/*!
 *  @brief This API is used to convert the Euler r raw data
 *  to radians output as double
 *
 *  @param euler_r_d : The double value of Euler r radians
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_r_rad(double *euler_r_d);

/*!
 *  @brief This API is used to convert the Euler hpr raw data
 *  to degree output as double
 *
 *  @param euler_hpr : The degree data of Euler hpr
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   h        | degree data of Euler
 *   r        | degree data of Euler
 *   p        | degree data of Euler
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_hpr_deg(struct bno055_euler_double_t *euler_hpr);

/*!
 *  @brief This API is used to convert the Euler hpr raw data
 *  to radians output as double
 *
 *  @param euler_hpr : The radians data of Euler hpr
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   h        | radians data of Euler
 *   r        | radians data of Euler
 *   p        | radians data of Euler
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_hpr_rad(struct bno055_euler_double_t *euler_hpr);

/****************************************************************************/
/**\name FUNCTIONS FOR READING LINEAR ACCEL DATA OUTPUT AS DOUBLE PRECISION */
/****************************************************************************/

/*!
 *  @brief This API is used to convert the linear
 *  accel x raw data to meterpersecseq output as double
 *
 *  @param linear_accel_x_d : The double value of
 *  linear accel x meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_x_msq(double *linear_accel_x_d);

/*!
 *  @brief This API is used to convert the linear
 *  accel y raw data to meterpersecseq output as double
 *
 *  @param linear_accel_y_d : The double value of
 *  linear accel y meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_y_msq(double *linear_accel_y_d);

/*!
 *  @brief This API is used to convert the linear
 *  accel z raw data to meterpersecseq output as double
 *
 *  @param linear_accel_z_d : The double value of
 *  linear accel z meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_z_msq(double *linear_accel_z_d);

/*!
 *  @brief This API is used to convert the linear accel xyz raw data
 *  to meterpersecseq output as double
 *
 *  @param linear_accel_xyz : The meterpersecseq data of linear accel xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | meterpersecseq data of linear accel
 *   y        | meterpersecseq data of linear accel
 *   z        | meterpersecseq data of linear accel
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_xyz_msq(
    struct bno055_linear_accel_double_t *linear_accel_xyz);

/**************************************************************************/
/**\name FUNCTIONS FOR READING GRAVITY DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/

/*!
 *  @brief This API is used to convert the gravity
 *  x raw data to meterpersecseq output as double
 *
 *  @param gravity_x_d : The double value of gravity x meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_x_msq(double *gravity_x_d);

/*!
 *  @brief This API is used to convert the gravity
 *  y raw data to meterpersecseq output as double
 *
 *  @param gravity_y_d : The double value of gravity y meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_y_msq(double *gravity_y_d);

/*!
 *  @brief This API is used to convert the gravity
 *  z raw data to meterpersecseq output as double
 *
 *  @param gravity_z_d : The double value of gravity z meterpersecseq
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_z_msq(double *gravity_z_d);

/*!
 *  @brief This API is used to convert the gravity xyz raw data
 *  to meterpersecseq output as double
 *
 *  @param gravity_xyz : The meterpersecseq data of gravity xyz
 *
 *  Parameter |    result
 *  --------- | -----------------
 *   x        | meterpersecseq data of gravity
 *   y        | meterpersecseq data of gravity
 *   z        | meterpersecseq data of gravity
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gravity_xyz_msq(struct bno055_gravity_double_t *gravity_xyz);

/**************************************************************************/
/**\name FUNCTIONS FOR READING TEMPERATURE DATA OUTPUT AS DOUBLE PRECISION*/
/*************************************************************************/

/*!
 *  @brief This API is used to convert the temperature
 *  data to Fahrenheit output as double
 *
 *  @param temp_d : The double value of temperature Fahrenheit
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_temp_fahrenheit(double *temp_d);

/*!
 *  @brief This API is used to convert the temperature
 *  data to Celsius output as double
 *
 *  @param temp_d : The double value of temperature Celsius
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_temp_celsius(double *temp_d);

#endif

/**************************************************************************/
/**\name FUNCTIONS FOR READING ACCEL,MAG,GYRO AND SYSTEM CALIBRATION STATUS*/
/*************************************************************************/

/*!
 *  @brief This API used to read
 *  mag calibration status from register from 0x35 bit 0 and 1
 *
 *  @param mag_calib_u8 : The value of mag calib status
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_calib_stat(u8 *mag_calib_u8);

/*!
 *  @brief This API used to read
 *  accel calibration status from register from 0x35 bit 2 and 3
 *
 *  @param accel_calib_u8 : The value of accel calib status
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_calib_stat(u8 *accel_calib_u8);

/*!
 *  @brief This API used to read
 *  gyro calibration status from register from 0x35 bit 4 and 5
 *
 *  @param gyro_calib_u8 : The value of gyro calib status
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_calib_stat(u8 *gyro_calib_u8);

/*!
 *  @brief This API used to read
 *  system calibration status from register from 0x35 bit 6 and 7
 *
 *  @param sys_calib_u8 : The value of system calib status
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_calib_stat(u8 *sys_calib_u8);

/******************************************************************/
/**\name FUNCTIONS FOR READING ACCEL,MAG,GYRO AND SYSTEM SELF TEST */
/******************************************************************/

/*!
 *  @brief This API used to read
 *  self test of accel from register from 0x36 bit 0
 *
 *  @param selftest_accel_u8 : The value of self test of accel
 *
 *    selftest_accel_u8 |  result
 *   -------------------- | ---------------------
 *     0x00               | indicates test failed
 *     0x01               | indicated test passed
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_accel(u8 *selftest_accel_u8);

/*!
 *  @brief This API used to read
 *  self test of mag from register from 0x36 bit 1
 *
 *  @param selftest_mag_u8 : The value of self test of mag
 *
 *     selftest_mag_u8  |  result
 *   -------------------- | ---------------------
 *     0x00               | indicates test failed
 *     0x01               | indicated test passed
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_mag(u8 *selftest_mag_u8);

/*!
 *  @brief This API used to read
 *  self test of gyro from register from 0x36 bit 2
 *
 *  @param selftest_gyro_u8 : The value of self test of gyro
 *
 *     selftest_gyro_u8 |  result
 *   -------------------- | ---------------------
 *     0x00               | indicates test failed
 *     0x01               | indicated test passed
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_gyro(u8 *selftest_gyro_u8);

/*!
 *  @brief This API used to read
 *  self test of micro controller from register from 0x36 bit 3
 *
 *  @param selftest_mcu_u8 : The value of self test of micro controller
 *
 *     selftest_mcu_u8  |  result
 *   -------------------- | ---------------------
 *     0x00               | indicates test failed
 *     0x01               | indicated test passed
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_mcu(u8 *selftest_mcu_u8);

/*****************************************************/
/**\name FUNCTIONS FOR READING GYRO INTERRUPT STATUS */
/*****************************************************/

/*!
 *  @brief This API used to read the stat_s8 of
 *  gyro anymotion interrupt from register from 0x37 bit 2
 *
 *  @param gyro_any_motion_u8 : The value of gyro anymotion interrupt
 *
 *     gyro_any_motion_u8  |  result
 *    --------------------   | ---------------------
 *     0x00                  | indicates no interrupt triggered
 *     0x01                  | indicates interrupt triggered
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro anymotion interrupt can be BNO055_BIT_ENABLE
 *  by the following APIs
 *
 *  bno055_set_intr_mask_gyro_any_motion()
 *
 *  bno055_set_intr_gyro_any_motion()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_gyro_any_motion(u8 *gyro_any_motion_u8);

/*!
 *  @brief This API used to read the stat_s8 of
 *  gyro highrate interrupt from register from 0x37 bit 3
 *
 *  @param gyro_highrate_u8 : The value of gyro highrate interrupt
 *
 *     gyro_highrate_u8   |  result
 *    ------------------- | ---------------------
 *     0x00               | indicates no interrupt triggered
 *     0x01               | indicates interrupt triggered
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate interrupt can be configured
 *          by the following APIs
 *
 *  bno055_set_intr_mask_gyro_highrate()
 *
 *  bno055_set_intr_gyro_highrate()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_gyro_highrate(u8 *gyro_highrate_u8);

/*****************************************************/
/**\name FUNCTIONS FOR READING ACCEL INTERRUPT STATUS */
/*****************************************************/

/*!
 *  @brief This API used to read the status of
 *  accel highg interrupt from register from 0x37 bit 5
 *
 *  @param accel_high_g_u8 : The value of accel highg interrupt
 *
 *     accel_high_g_u8    |  result
 *    -------------------   | ---------------------
 *     0x00                 | indicates no interrupt triggered
 *     0x01                 | indicates interrupt triggered
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Accel highg interrupt can be configured
 *          by the following APIs
 *
 *  bno055_set_intr_mask_accel_high_g()
 *
 *  bno055_set_intr_accel_high_g()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_accel_high_g(u8 *accel_high_g_u8);

/*!
 *  @brief This API used to read the stat_s8 of
 *  accel anymotion interrupt from register from 0x37 bit 6
 *
 *  @param accel_any_motion_u8 : The value of accel anymotion interrupt
 *
 *     accel_any_motion_u8 |  result
 *    -------------------   | ---------------------
 *     0x00                 | indicates no interrupt triggered
 *     0x01                 | indicates interrupt triggered
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Accel anymotion interrupt can be configured
 *          by the following APIs
 *
 *  bno055_set_intr_mask_accel_any_motion()
 *
 *  bno055_set_intr_accel_any_motion()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_accel_any_motion(u8 *accel_any_motion_u8);

/*!
 *  @brief This API used to read the stat_s8 of
 *  accel nomotion/slowmotion interrupt from register from 0x37 bit 6
 *
 *  @param accel_no_motion_u8 :
 *  The value of accel nomotion/slowmotion interrupt
 *
 *     accel_no_motion_u8 |  result
 *    -------------------   | ---------------------
 *     0x00                 | indicates no interrupt triggered
 *     0x01                 | indicates interrupt triggered
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Accel nomotion/slowmotion interrupt can be configured
 *          by the following APIs
 *
 *  bno055_set_intr_mask_accel_nomotion()
 *
 *  bno055_set_intr_accel_nomotion()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_accel_no_motion(u8 *accel_no_motion_u8);

/**************************************************************************/
/**\name FUNCTIONS FOR READING SYSTEM CLOCK, STATUS AND BNO055_ERROR CODE */
/*************************************************************************/

/*!
 *  @brief This API is used to read status of main clock
 *  from the register 0x38 bit 0
 *
 *  @param stat_main_clk_u8 : the status of main clock
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_stat_main_clk(u8 *stat_main_clk_u8);

/*!
 *  @brief This API is used to read system status
 *  code from the register 0x39 it is a byte of data
 *
 *  @param sys_stat_u8 : the status of system
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_stat_code(u8 *sys_stat_u8);

/*!
 *  @brief This API is used to read system BNO055_ERROR
 *  code from the register 0x3A it is a byte of data
 *
 *  @param sys_error_u8 : The value of system BNO055_ERROR code
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_error_code(u8 *sys_error_u8);

/********************************************/
/**\name FUNCTIONS FOR ACCEL UNIT SELECTION */
/********************************************/

/*!
 *  @brief This API used to read the accel unit
 *  from register from 0x3B bit 0
 *
 *  @param accel_unit_u8 : The value of accel unit
 *
 *    accel_unit_u8 |   result
 *   -------------    | ---------------
 *        0x00        | BNO055_ACCEL_UNIT_MSQ
 *        0x01        | BNO055_ACCEL_UNIT_MG
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_unit(u8 *accel_unit_u8);

/*!
 *  @brief This API used to write the accel unit
 *  from register from 0x3B bit 0
 *
 *  @param accel_unit_u8 : The value of accel unit
 *
 *    accel_unit_u8 |   result
 *   -------------    | ---------------
 *        0x00        | BNO055_ACCEL_UNIT_MSQ
 *        0x01        | BNO055_ACCEL_UNIT_MG
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_unit(u8 accel_unit_u8);

/********************************************/
/**\name FUNCTIONS FOR GYRO UNIT SELECTION */
/********************************************/

/*!
 *  @brief This API used to read the gyro unit
 *  from register from 0x3B bit 1
 *
 *  @param gyro_unit_u8 : The value of accel unit
 *
 *  gyro_unit_u8  |  result
 *  -------------   | -----------
 *    0x00          | BNO055_GYRO_UNIT_DPS
 *    0x01          | BNO055_GYRO_UNIT_RPS
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_unit(u8 *gyro_unit_u8);

/*!
 *  @brief This API used to write the gyro unit
 *  from register from 0x3B bit 1
 *
 *  @param gyro_unit_u8 : The value of accel unit
 *
 *  gyro_unit_u8  |  result
 *  -------------   | -----------
 *    0x00          | BNO055_GYRO_UNIT_DPS
 *    0x01          | BNO055_GYRO_UNIT_RPS
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_unit(u8 gyro_unit_u8);

/********************************************/
/**\name FUNCTIONS FOR EULER UNIT SELECTION */
/********************************************/

/*!
 *  @brief This API used to read the Euler unit
 *  from register from 0x3B bit 2
 *
 *  @param euler_unit_u8 : The value of accel unit
 *
 *    euler_unit_u8 | result
 *   --------------   | -----------
 *      0x00          | BNO055_EULER_UNIT_DEG
 *      0x01          | BNO055_EULER_UNIT_RAD
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_unit(u8 *euler_unit_u8);

/*!
 *  @brief This API used to write the Euler unit
 *  from register from 0x3B bit 2
 *
 *  @param euler_unit_u8 : The value of Euler unit
 *
 *    euler_unit_u8 | result
 *   --------------   | -----------
 *      0x00          | BNO055_EULER_UNIT_DEG
 *      0x01          | BNO055_EULER_UNIT_RAD
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_unit(u8 euler_unit_u8);

/********************************************/
/**\name FUNCTIONS FOR TILT UNIT SELECTION */
/********************************************/

/*!
 *  @brief This API used to write the tilt unit
 *  from register from 0x3B bit 3
 *
 *  @param tilt_unit_u8 : The value of tilt unit
 *
 *    tilt_unit_u8  | result
 *   ---------------  | ---------
 *     0x00           | degrees
 *     0x01           | radians
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_tilt_unit(u8 *tilt_unit_u8);

/*!
 *  @brief This API used to write the tilt unit
 *  from register from 0x3B bit 3
 *
 *  @param tilt_unit_u8 : The value of tilt unit
 *
 *    tilt_unit_u8  | result
 *   ---------------  | ---------
 *     0x00           | degrees
 *     0x01           | radians
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 *
 *  \return Communication results
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_tilt_unit(u8 tilt_unit_u8);

/**************************************************/
/**\name FUNCTIONS FOR TEMPERATURE UNIT SELECTION */
/**************************************************/

/*!
 *  @brief This API used to read the temperature unit
 *  from register from 0x3B bit 4
 *
 *  @param temp_unit_u8 : The value of temperature unit
 *
 *    temp_unit_u8  |  result
 *   -----------      | --------------
 *      0x00          | BNO055_TEMP_UNIT_CELSIUS
 *      0x01          | BNO055_TEMP_UNIT_FAHRENHEIT
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_unit(u8 *temp_unit_u8);

/*!
 *  @brief This API used to write the temperature unit
 *  from register from 0x3B bit 4
 *
 *  @param temp_unit_u8 : The value of temperature unit
 *
 *    temp_unit_u8  |  result
 *   -----------      | --------------
 *      0x00          | BNO055_TEMP_UNIT_CELSIUS
 *      0x01          | BNO055_TEMP_UNIT_FAHRENHEIT
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_unit(u8 temp_unit_u8);

/**************************************************/
/**\name FUNCTIONS FOR DATA OUTPUT FORMAT SELECT */
/**************************************************/

/*!
 *  @brief This API used to read the current selected orientation mode
 *  from register from 0x3B bit 7
 *
 *  @param data_output_format_u8 : The value of data output format
 *
 *    data_output_format_u8  | result
 *   --------------------      | --------
 *    0x00                     | Windows
 *    0x01                     | Android
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_data_output_format(u8 *data_output_format_u8);

/*!
 *  @brief This API used to read the current selected orientation mode
 *  from register from 0x3B bit 7
 *
 *  @param data_output_format_u8 : The value of data output format
 *
 *    data_output_format_u8  | result
 *   --------------------      | --------
 *    0x00                     | Windows
 *    0x01                     | Android
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_data_output_format(u8 data_output_format_u8);

/**************************************************/
/**\name FUNCTIONS FOR DATA OPERATION MODE  */
/**************************************************/

/*! @brief This API used to read the operation mode
 *  from register from 0x3D bit 0 to 3
 *
 *  @param operation_mode_u8 : The value of operation mode
 *
 * operation_mode_u8 |      result      | comments
 * ----------|----------------------------|----------------------------
 *  0x00     | BNO055_OPERATION_MODE_CONFIG      | Configuration mode
 *  0x01     | BNO055_OPERATION_MODE_ACCONLY     | Reads accel data alone
 *  0x02     | BNO055_OPERATION_MODE_MAGONLY     | Reads mag data alone
 *  0x03     | BNO055_OPERATION_MODE_GYRONLY     | Reads gyro data alone
 *  0x04     | BNO055_OPERATION_MODE_ACCMAG      | Reads accel and mag data
 *  0x05     | BNO055_OPERATION_MODE_ACCGYRO     | Reads accel and gyro data
 *  0x06     | BNO055_OPERATION_MODE_MAGGYRO     | Reads accel and mag data
 *  0x07     | OPERATION_MODE_ANY_MOTION  | Reads accel mag and gyro data
 *  0x08     | BNO055_OPERATION_MODE_IMUPLUS     | Inertial measurement unit
 *   -       |       -                    | Reads accel,gyro and fusion data
 *  0x09     | BNO055_OPERATION_MODE_COMPASS     | Reads accel, mag data
 *   -       |       -                    | and fusion data
 *  0x0A     | BNO055_OPERATION_MODE_M4G         | Reads accel, mag data
 *    -      |       -                    | and fusion data
 *  0x0B     | BNO055_OPERATION_MODE_NDOF_FMC_OFF| Nine degrees of freedom with
 *   -       |       -                    | fast magnetic calibration
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *  0x0C     | BNO055_OPERATION_MODE_NDOF        | Nine degrees of freedom
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note In the config mode, all sensor and fusion data
 *  becomes zero and it is mainly derived
 *  to configure the various settings of the BNO
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_operation_mode(u8 *operation_mode_u8);

/*! @brief This API used to write the operation mode
 *  from register from 0x3D bit 0 to 3
 *
 *  @param operation_mode_u8 : The value of operation mode
 *
 *  operation_mode_u8  |      result            | comments
 * ---------|-----------------------------------|--------------------------
 *  0x00    | BNO055_OPERATION_MODE_CONFIG      | Configuration mode
 *  0x01    | BNO055_OPERATION_MODE_ACCONLY     | Reads accel data alone
 *  0x02    | BNO055_OPERATION_MODE_MAGONLY     | Reads mag data alone
 *  0x03    | BNO055_OPERATION_MODE_GYRONLY     | Reads gyro data alone
 *  0x04    | BNO055_OPERATION_MODE_ACCMAG      | Reads accel and mag data
 *  0x05    | BNO055_OPERATION_MODE_ACCGYRO     | Reads accel and gyro data
 *  0x06    | BNO055_OPERATION_MODE_MAGGYRO     | Reads accel and mag data
 *  0x07    | OPERATION_MODE_ANY_MOTION         | Reads accel mag and
 *          |       -                           | gyro data
 *  0x08    | BNO055_OPERATION_MODE_IMUPLUS     | Inertial measurement unit
 *   -      |                                   | Reads accel,gyro and
 *          |       -                           | fusion data
 *  0x09    | BNO055_OPERATION_MODE_COMPASS     | Reads accel, mag data
 *   -      |       -                           | and fusion data
 *  0x0A    | BNO055_OPERATION_MODE_M4G         | Reads accel, mag data
 *    -     |       -                           | and fusion data
 *  0x0B    | BNO055_OPERATION_MODE_NDOF_FMC_OFF| Nine degrees of freedom with
 *   -      |       -                           | fast magnetic calibration
 *   -      |       -                           | Reads accel,mag, gyro
 *   -      |       -                           | and fusion data
 *  0x0C    | BNO055_OPERATION_MODE_NDOF        | Nine degrees of freedom
 *   -      |       -                           | Reads accel,mag, gyro
 *   -      |       -                           | and fusion data
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note In the config mode, all sensor and fusion data
 *  becomes zero and it is mainly derived
 *  to configure the various settings of the BNO
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(u8 operation_mode_u8);

/**************************************************/
/**\name FUNCTIONS FOR POWER MODE  */
/**************************************************/

/*! @brief This API used to read the power mode
 *  from register from 0x3E bit 0 to 1
 *
 *  @param power_mode_u8 : The value of power mode
 *
 * power_mode_u8|      result           | comments
 * ---------|---------------------------|-------------------------------------
 *  0x00    |BNO055_POWER_MODE_NORMAL   | In the NORMAL mode the register
 *    -     |       -                   | map and the internal peripherals
 *    -     |       -                   | of the MCU are always
 *    -     |       -                   | operative in this mode
 *  0x01    |BNO055_POWER_MODE_LOWPOWER | This is first level of power
 *          |       -                   | saving mode
 *  0x02    |BNO055_POWER_MODE_SUSPEND  | In suspend mode the system is
 *   -      |      -                    | paused and all the sensors and
 *   -      |      -                    | the micro controller are
 *   -      |      -                    | put into sleep mode.
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note For detailed about LOWPOWER mode
 *  refer data sheet 3.4.2
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_power_mode(u8 *power_mode_u8);

/*! @brief This API used to write the power mode
 *  from register from 0x3E bit 0 to 1
 *
 *  @param power_mode_u8 : The value of power mode
 *
 *
 * power_mode_u8|      result          | comments
 * -------|----------------------------|---------------------------------
 *  0x00  | BNO055_POWER_MODE_NORMAL   | In the NORMAL mode the register
 *    -   |       -                    | map and the internal peripherals
 *    -   |       -                    | of the MCU are always
 *    -   |       -                    | operative in this mode
 *  0x01  | BNO055_POWER_MODE_LOWPOWER | This is first level of power
 *        |     -                      | saving mode
 *  0x02  | BNO055_POWER_MODE_SUSPEND  | In suspend mode the system is
 *   -    |      -                     | paused and all the sensors and
 *   -    |      -                     | the micro controller are
 *   -    |      -                     | put into sleep mode.
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note For detailed about LOWPOWER mode
 *  refer data sheet 3.4.2
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_power_mode(u8 power_mode_u8);

BNO055_RETURN_FUNCTION_TYPE bno055_sw_reset();

/**************************************************/
/**\name FUNCTIONS FOR RESET INTERRUPT  */
/**************************************************/

/*!
 *  @brief This API used to read the reset interrupt
 *  from register from 0x3F bit 6
 *  It resets all the interrupt bit and interrupt output
 *
 *  @param intr_rst_u8 : The value of reset interrupt
 *
 *    intr_rst_u8 | result
 *   -------------|----------
 *     0x01       | BNO055_BIT_ENABLE
 *     0x00       | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_rst(u8 *intr_rst_u8);

/*!
 *  @brief This API used to write the reset interrupt
 *  from register from 0x3F bit 6
 *  It resets all the interrupt bit and interrupt output
 *
 *  @param intr_rst_u8 : The value of reset interrupt
 *
 *    intr_rst_u8   | result
 *   -------------- |----------
 *     0x01         | BNO055_BIT_ENABLE
 *     0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_rst(u8 intr_rst_u8);

/**************************************************/
/**\name FUNCTIONS FOR CLOCK SOURCE  */
/**************************************************/

/*!
 *  @brief This API used to read the clk source
 *  from register from 0x3F bit 7
 *
 *  @param clk_src_u8 : The value of clk source
 *
 *   clk_src_u8   | result
 *   -------------|----------
 *     0x01       | BNO055_BIT_ENABLE
 *     0x00       | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_clk_src(u8 *clk_src_u8);

/*!
 *  @brief This API used to write the clk source
 *  from register from 0x3F bit 7
 *
 *  @param clk_src_u8 : The value of clk source
 *
 *   clk_src_u8   | result
 *   -------------- |----------
 *     0x01         | BNO055_BIT_ENABLE
 *     0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_clk_src(u8 clk_src_u8);

/**************************************************/
/**\name FUNCTIONS FOR RESET SYSTEM  */
/**************************************************/

/*!
 *  @brief This API used to read the reset system
 *  from register from 0x3F bit 5
 *
 *  @param sys_rst_u8 : The value of reset system
 *
 *   sys_rst_u8   | result
 *   -------------- |----------
 *     0x01         | BNO055_BIT_ENABLE
 *     0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note It resets the whole system
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_rst(u8 *sys_rst_u8);

/*!
 *  @brief This API used to write the reset system
 *  from register from 0x3F bit 5
 *
 *  @param sys_rst_u8 : The value of reset system
 *
 *   sys_rst_u8   | result
 *   -------------- |----------
 *     0x01         | BNO055_BIT_ENABLE
 *     0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note It resets the whole system
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_sys_rst(u8 sys_rst_u8);

/**************************************************/
/**\name FUNCTIONS FOR SELF TEST  */
/**************************************************/

/*!
 *  @brief This API used to read the self test
 *  from register from 0x3F bit 0
 *
 *  @param selftest_u8 : The value of self test
 *
 *   selftest_u8  | result
 *   -------------- |----------
 *     0x01         | BNO055_BIT_ENABLE
 *     0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note It triggers the self test
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest(u8 *selftest_u8);

/*!
 *  @brief This API used to write the self test
 *  from register from 0x3F bit 0
 *
 *  @param selftest_u8 : The value of self test
 *
 *   selftest_u8  | result
 *   -------------- |----------
 *     0x01         | BNO055_BIT_ENABLE
 *     0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note It triggers the self test
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_selftest(u8 selftest_u8);

/**************************************************/
/**\name FUNCTIONS FOR TEMPERATURE SOURCE  */
/**************************************************/

/*!
 *  @brief This API used to read the temperature source
 *  from register from 0x40 bit 0 and 1
 *
 *  @param temp_source_u8 : The value of selected temperature source
 *
 *     temp_source_u8 | result
 *    ----------------  |---------------
 *      0x00            | BNO055_ACCEL_TEMP_EN
 *      0X01            | BNO055_GYRO_TEMP_EN
 *      0X03            | BNO055_MCU_TEMP_EN
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_source(u8 *temp_source_u8);

/*!
 *  @brief This API used to write the temperature source
 *  from register from 0x40 bit 0 and 1
 *
 *  @param temp_source_u8 : The value of selected temperature source
 *
 *     temp_source_u8 | result
 *    ----------------  |---------------
 *      0x00            | BNO055_ACCEL_TEMP_EN
 *      0X01            | BNO055_GYRO_TEMP_EN
 *      0X03            | BNO055_MCU_TEMP_EN
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_source(u8 temp_source_u8);

/**************************************************/
/**\name APIs FOR AXIS REMAP  */
/**************************************************/

/*!
 *  @brief This API used to read the axis remap value
 *  from register from 0x41 bit 0 and 5
 *
 *  @param remap_axis_u8 : The value of axis remapping
 *
 *    remap_axis_u8 |   result     | comments
 *   ------------|-------------------|------------
 *      0X21     | BNO055_REMAP_X_Y         | Z=Z;X=Y;Y=X
 *      0X18     | BNO055_REMAP_Y_Z         | X=X;Y=Z;Z=Y
 *      0X06     | BNO055_REMAP_Z_X         | Y=Y;X=Z;Z=X
 *      0X12     | BNO055_REMAP_X_Y_Z_TYPE0 | X=Z;Y=X;Z=Y
 *      0X09     | BNO055_REMAP_X_Y_Z_TYPE1 | X=Y;Y=Z;Z=X
 *      0X24     | BNO055_DEFAULT_AXIS      | X=X;Y=Y;Z=Z
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note : For axis sign remap refer the following APIs
 *  x-axis :
 *
 *  bno055_set_x_remap_sign()
 *
 *  y-axis :
 *
 *  bno055_set_y_remap_sign()
 *
 *  z-axis :
 *
 *  bno055_set_z_remap_sign()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_axis_remap_value(u8 *remap_axis_u8);

/*!
 *  @brief This API used to write the axis remap value
 *  from register from 0x41 bit 0 and 5
 *
 *  @param remap_axis_u8 : The value of axis remapping
 *
 * remap_axis_u8 |   result                 | comments
 *   ------------|--------------------------|------------
 *      0X21     | BNO055_REMAP_X_Y         | Z=Z;X=Y;Y=X
 *      0X18     | BNO055_REMAP_Y_Z         | X=X;Y=Z;Z=Y
 *      0X06     | BNO055_REMAP_Z_X         | Y=Y;X=Z;Z=X
 *      0X12     | BNO055_REMAP_X_Y_Z_TYPE0 | X=Z;Y=X;Z=Y
 *      0X09     | BNO055_REMAP_X_Y_Z_TYPE1 | X=Y;Y=Z;Z=X
 *      0X24     | BNO055_DEFAULT_AXIS      | X=X;Y=Y;Z=Z
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note : For axis sign remap refer the following APIs
 *  x-axis :
 *
 *  bno055_set_x_remap_sign()
 *
 *  y-axis :
 *
 *  bno055_set_y_remap_sign()
 *
 *  z-axis :
 *
 *  bno055_set_z_remap_sign()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_axis_remap_value(u8 remap_axis_u8);

/**************************************************/
/**\name APIs FOR AXIS REMAP SIGN  */
/**************************************************/

/*!
 *  @brief This API used to read the x-axis remap
 *  sign from register from 0x42 bit 2
 *
 *  @param remap_x_sign_u8 : The value of x-axis remap sign
 *
 *    remap_x_sign_u8  |    result
 *   ------------------- |--------------------
 *      0X00             | BNO055_REMAP_AXIS_POSITIVE
 *      0X01             | BNO055_REMAP_AXIS_NEGATIVE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_x_sign(u8 *remap_x_sign_u8);

/*!
 *  @brief This API used to write the x-axis remap
 *  sign from register from 0x42 bit 2
 *
 *  @param remap_x_sign_u8 : The value of x-axis remap sign
 *
 *    remap_x_sign_u8  |    result
 *   ------------------- |--------------------
 *      0X00             | BNO055_REMAP_AXIS_POSITIVE
 *      0X01             | BNO055_REMAP_AXIS_NEGATIVE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_x_sign(u8 remap_x_sign_u8);

/*!
 *  @brief This API used to read the y-axis remap
 *  sign from register from 0x42 bit 1
 *
 *  @param remap_y_sign_u8 : The value of y-axis remap sign
 *
 *    remap_y_sign_u8  |   result
 *   ------------------- |--------------------
 *      0X00             | BNO055_REMAP_AXIS_POSITIVE
 *      0X01             | BNO055_REMAP_AXIS_NEGATIVE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_y_sign(u8 *remap_y_sign_u8);

/*!
 *  @brief This API used to write the y-axis remap
 *  sign from register from 0x42 bit 1
 *
 *  @param remap_y_sign_u8 : The value of y-axis remap sign
 *
 *    remap_y_sign_u8  |   result
 *   ------------------- |--------------------
 *      0X00             | BNO055_REMAP_AXIS_POSITIVE
 *      0X01             | BNO055_REMAP_AXIS_NEGATIVE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_y_sign(u8 remap_y_sign_u8);

/*!
 *  @brief This API used to read the z-axis remap
 *  sign from register from 0x42 bit 0
 *
 *  @param remap_z_sign_u8 : The value of z-axis remap sign
 *
 *    remap_z_sign_u8  |   result
 *   ------------------- |--------------------
 *      0X00             | BNO055_REMAP_AXIS_POSITIVE
 *      0X01             | BNO055_REMAP_AXIS_NEGATIVE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_z_sign(u8 *remap_z_sign_u8);

/*!
 *  @brief This API used to write the z-axis remap
 *  sign from register from 0x42 bit 0
 *
 *  @param remap_z_sign_u8 : The value of z-axis remap sign
 *
 *    remap_z_sign_u8  |   result
 *   ------------------|--------------------
 *      0X00           | BNO055_REMAP_AXIS_POSITIVE
 *      0X01           | BNO055_REMAP_AXIS_NEGATIVE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_z_sign(u8 remap_z_sign_u8);

/*****************************************************/
/**\name FUNCTIONS FOR SOFT IRON CALIBRATION MATRIX  */
/*****************************************************/

/*!
 *  @brief This API is used to read soft iron calibration matrix
 *  from the register 0x43 to 0x53 it is a 18 bytes of data
 *
 *  @param sic_matrix : The value of soft iron calibration matrix
 *
 *  sic_matrix         |           result
 * --------------------|----------------------------------
 *       sic_0         | soft iron calibration matrix zero
 *       sic_1         | soft iron calibration matrix one
 *       sic_2         | soft iron calibration matrix two
 *       sic_3         | soft iron calibration matrix three
 *       sic_4         | soft iron calibration matrix four
 *       sic_5         | soft iron calibration matrix five
 *       sic_6         | soft iron calibration matrix six
 *       sic_7         | soft iron calibration matrix seven
 *       sic_8         | soft iron calibration matrix eight
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note : Each soft iron calibration matrix range from -32768 to +32767
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix(struct bno055_sic_matrix_t  *sic_matrix);

/*!
 *  @brief This API is used to write soft iron calibration matrix
 *  from the register 0x43 to 0x53 it is a 18 bytes of data
 *
 *  @param sic_matrix : The value of soft iron calibration matrix
 *
 *  sic_matrix         |           result
 * --------------------|----------------------------------
 *       sic_0         | soft iron calibration matrix zero
 *       sic_1         | soft iron calibration matrix one
 *       sic_2         | soft iron calibration matrix two
 *       sic_3         | soft iron calibration matrix three
 *       sic_4         | soft iron calibration matrix four
 *       sic_5         | soft iron calibration matrix five
 *       sic_6         | soft iron calibration matrix six
 *       sic_7         | soft iron calibration matrix seven
 *       sic_8         | soft iron calibration matrix eight
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note : Each soft iron calibration matrix range from -32768 to +32767
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix(struct bno055_sic_matrix_t  *sic_matrix);

/*****************************************************/
/**\name FUNCTIONS FOR ACCEL OFFSET AND RADIUS */
/*****************************************************/

/*!
 *  @brief This API is used to read accel offset and accel radius
 *  offset form register 0x55 to 0x5A and radius form 0x67 and 0x68
 *
 *  @param accel_offset : The value of accel offset and radius
 *
 *      bno055_accel_offset_t |     result
 *      ------------------- | ----------------
 *               x          |  accel offset x
 *               y          |  accel offset y
 *               z          |  accel offset z
 *               r          |  accel offset r
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note  The range of the accel offset varies based on
 *  the G-range of accel sensor.
 *
 *  accel G range   |  offset range
 * ---------------  |  --------------
 *  BNO055_ACCEL_RANGE_2G  |   +/-2000
 *  BNO055_ACCEL_RANGE_4G  |   +/-4000
 *  BNO055_ACCEL_RANGE_8G  |   +/-8000
 *  BNO055_ACCEL_RANGE_16G |   +/-16000
 *
 *  accel G range can be configured by using the
 *  bno055_set_accel_range() API
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset(struct bno055_accel_offset_t  *accel_offset);

/*!
 *  @brief This API is used to write accel offset and accel radius
 *  offset form register 0x55 to 0x5A and radius form 0x67 and 0x68
 *
 *  @param accel_offset : The value of accel offset and radius
 *
 *      bno055_accel_offset_t |     result
 *      ------------------- | ----------------
 *               x          |  accel offset x
 *               y          |  accel offset y
 *               z          |  accel offset z
 *               r          |  accel offset r
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note  The range of the accel offset varies based on
 *  the G-range of accel sensor.
 *
 *  accel G range   |  offset range
 * ---------------  |  --------------
 *  BNO055_ACCEL_RANGE_2G  |   +/-2000
 *  BNO055_ACCEL_RANGE_4G  |   +/-4000
 *  BNO055_ACCEL_RANGE_8G  |   +/-8000
 *  BNO055_ACCEL_RANGE_16G |   +/-16000
 *
 *  accel G range can be configured by using the
 *  bno055_set_accel_range() API
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset(struct bno055_accel_offset_t  *accel_offset);

/*****************************************************/
/**\name FUNCTIONS FOR MAG OFFSET AND RADIUS*/
/*****************************************************/

/*!
 *  @brief This API is used to read mag offset
 *  offset form register 0x69 to 0x6A
 *
 *  @param mag_offset :  The value of mag offset and radius
 *
 *      bno055_mag_offset_t   |     result
 *      ------------------- | ----------------
 *               x          |  mag offset x
 *               y          |  mag offset y
 *               z          |  mag offset z
 *               r          |  mag radius r
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note  The range of the magnetometer offset is +/-6400 in LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset(struct bno055_mag_offset_t  *mag_offset);

/*!
 *  @brief This API is used to read mag offset
 *  offset form register 0x69 to 0x6A
 *
 *  @param mag_offset :  The value of mag offset and radius
 *
 *      bno055_mag_offset_t   |     result
 *      ------------------- | ----------------
 *               x          |  mag offset x
 *               y          |  mag offset y
 *               z          |  mag offset z
 *               r          |  mag radius r
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note  The range of the magnetometer offset is +/-6400 in LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset(struct bno055_mag_offset_t  *mag_offset);

/*****************************************************/
/**\name FUNCTIONS FOR GYRO OFFSET */
/*****************************************************/

/*!
 *  @brief This API is used to read gyro offset
 *  offset form register 0x61 to 0x66
 *
 *  @param gyro_offset : The value of gyro offset
 *
 *      bno055_gyro_offset_t  |     result
 *      ------------------- | ----------------
 *               x          |  gyro offset x
 *               y          |  gyro offset y
 *               z          |  gyro offset z
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note  The range of the gyro offset varies based on
 *  the range of gyro sensor
 *
 *  gyro G range         | offset range
 * --------------------  | ------------
 *  BNO055_GYRO_RANGE_2000DPS   | +/-32000
 *  BNO055_GYRO_RANGE_1000DPS   | +/-16000
 *  BNO055_GYRO_RANGE_500DPS    | +/-8000
 *  BNO055_GYRO_RANGE_250DPS    | +/-4000
 *  BNO055_GYRO_RANGE_125DPS    | +/-2000
 *
 *  Gyro range can be configured by using the
 *  bno055_set_gyro_range() API
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset(struct bno055_gyro_offset_t  *gyro_offset);

/*!
 *  @brief This API is used to read gyro offset
 *  offset form register 0x61 to 0x66
 *
 *  @param gyro_offset : The value of gyro offset
 *
 *      bno055_gyro_offset_t  |     result
 *      ------------------- | ----------------
 *               x          |  gyro offset x
 *               y          |  gyro offset y
 *               z          |  gyro offset z
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note  The range of the gyro offset varies based on
 *  the range of gyro sensor
 *
 *  gyro G range         | offset range
 * --------------------  | ------------
 *  BNO055_GYRO_RANGE_2000DPS   | +/-32000
 *  BNO055_GYRO_RANGE_1000DPS   | +/-16000
 *  BNO055_GYRO_RANGE_500DPS    | +/-8000
 *  BNO055_GYRO_RANGE_250DPS    | +/-4000
 *  BNO055_GYRO_RANGE_125DPS    | +/-2000
 *
 *  Gyro range can be configured by using the
 *  bno055_set_gyro_range() API
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset(struct bno055_gyro_offset_t *gyro_offset);

/********************************************************/
/************** PAGE1 Functions *********************/
/********************************************************/
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL CONFIGURATION */
/*****************************************************/

/*!
 *  @brief This API used to read the accel range
 *  from page one register from 0x08 bit 0 and 1
 *
 *  @param accel_range_u8 : The value of accel range
 *        accel_range_u8 |   result
 *       ----------------- | --------------
 *              0x00       | BNO055_ACCEL_RANGE_2G
 *              0x01       | BNO055_ACCEL_RANGE_4G
 *              0x02       | BNO055_ACCEL_RANGE_8G
 *              0x03       | BNO055_ACCEL_RANGE_16G
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_range(u8 *accel_range_u8);

/*!
 *  @brief This API used to write the accel range
 *  from page one register from 0x08 bit 0 and 1
 *
 *  @param accel_range_u8 : The value of accel range
 *
 *        accel_range_u8 |   result
 *       ----------------- | --------------
 *              0x00       | BNO055_ACCEL_RANGE_2G
 *              0x01       | BNO055_ACCEL_RANGE_4G
 *              0x02       | BNO055_ACCEL_RANGE_8G
 *              0x03       | BNO055_ACCEL_RANGE_16G
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_range(u8 accel_range_u8);

/*!
 *  @brief This API used to read the accel bandwidth
 *  from page one register from 0x08 bit 2 to 4
 *
 *  @param accel_bw_u8 : The value of accel bandwidth
 *
 *           accel_bw_u8 |     result
 *       ----------------- | ---------------
 *              0x00       | BNO055_ACCEL_BW_7_81HZ
 *              0x01       | BNO055_ACCEL_BW_15_63HZ
 *              0x02       | BNO055_ACCEL_BW_31_25HZ
 *              0x03       | BNO055_ACCEL_BW_62_5HZ
 *              0x04       | BNO055_ACCEL_BW_125HZ
 *              0x05       | BNO055_ACCEL_BW_250HZ
 *              0x06       | BNO055_ACCEL_BW_500HZ
 *              0x07       | BNO055_ACCEL_BW_1000HZ
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_bw(u8 *accel_bw_u8);

/*!
 *  @brief This API used to write the accel bandwidth
 *  from page one register from 0x08 bit 2 to 4
 *
 *  @param accel_bw_u8 : The value of accel bandwidth
 *
 *           accel_bw_u8 |     result
 *       ----------------- | ---------------
 *              0x00       | BNO055_ACCEL_BW_7_81HZ
 *              0x01       | BNO055_ACCEL_BW_15_63HZ
 *              0x02       | BNO055_ACCEL_BW_31_25HZ
 *              0x03       | BNO055_ACCEL_BW_62_5HZ
 *              0x04       | BNO055_ACCEL_BW_125HZ
 *              0x05       | BNO055_ACCEL_BW_250HZ
 *              0x06       | BNO055_ACCEL_BW_500HZ
 *              0x07       | BNO055_ACCEL_BW_1000HZ
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_bw(u8 accel_bw_u8);

/*!
 *  @brief This API used to read the accel power mode
 *  from page one register from 0x08 bit 5 to 7
 *
 *  @param accel_power_mode_u8 : The value of accel power mode
 * accel_power_mode_u8 |   result
 *   -----------------   | -------------
 *              0x00     | BNO055_ACCEL_NORMAL
 *              0x01     | BNO055_ACCEL_SUSPEND
 *              0x02     | BNO055_ACCEL_LOWPOWER_1
 *              0x03     | BNO055_ACCEL_STANDBY
 *              0x04     | BNO055_ACCEL_LOWPOWER_2
 *              0x05     | BNO055_ACCEL_DEEPSUSPEND
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_power_mode(u8 *accel_power_mode_u8);

/*!
 *  @brief This API used to write the accel power mode
 *  from page one register from 0x08 bit 5 to 7
 *
 *  @param accel_power_mode_u8 : The value of accel power mode
 * accel_power_mode_u8 |   result
 *   -----------------   | -------------
 *              0x00     | BNO055_ACCEL_NORMAL
 *              0x01     | BNO055_ACCEL_SUSPEND
 *              0x02     | BNO055_ACCEL_LOWPOWER_1
 *              0x03     | BNO055_ACCEL_STANDBY
 *              0x04     | BNO055_ACCEL_LOWPOWER_2
 *              0x05     | BNO055_ACCEL_DEEPSUSPEND
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_power_mode(u8 accel_power_mode_u8);

/*****************************************************/
/**\name FUNCTIONS FOR MAG CONFIGURATION */
/*****************************************************/

/*!
 *  @brief This API used to read the mag output data rate
 *  from page one register from 0x09 bit 0 to 2
 *
 *  @param mag_data_output_rate_u8 : The value of mag output data rate
 *
 *  mag_data_output_rate_u8 |   result
 *  ----------------------    |----------------------
 *     0x00                   | MAG_DATA_OUTPUT_RATE_2HZ
 *     0x01                   | MAG_DATA_OUTPUT_RATE_6HZ
 *     0x02                   | MAG_DATA_OUTPUT_RATE_8HZ
 *     0x03                   | MAG_DATA_OUTPUT_RATE_10HZ
 *     0x04                   | MAG_DATA_OUTPUT_RATE_15HZ
 *     0x05                   | MAG_DATA_OUTPUT_RATE_20HZ
 *     0x06                   | MAG_DATA_OUTPUT_RATE_25HZ
 *     0x07                   | MAG_DATA_OUTPUT_RATE_30HZ
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_data_output_rate(u8 *mag_data_output_rate_u8);

/*!
 *  @brief This API used to write the mag output data rate
 *  from page one register from 0x09 bit 0 to 2
 *
 *  @param mag_data_output_rate_u8 : The value of mag output data rate
 *
 *  mag_data_output_rate_u8 |   result
 *  ----------------------  |----------------------
 *     0x00                 | MAG_DATA_OUTPUT_RATE_2HZ
 *     0x01                 | MAG_DATA_OUTPUT_RATE_6HZ
 *     0x02                 | MAG_DATA_OUTPUT_RATE_8HZ
 *     0x03                 | MAG_DATA_OUTPUT_RATE_10HZ
 *     0x04                 | MAG_DATA_OUTPUT_RATE_15HZ
 *     0x05                 | MAG_DATA_OUTPUT_RATE_20HZ
 *     0x06                 | MAG_DATA_OUTPUT_RATE_25HZ
 *     0x07                 | MAG_DATA_OUTPUT_RATE_30HZ
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_data_output_rate(u8 mag_data_output_rate_u8);

/*!
 *  @brief This API used to read the mag operation mode
 *  from page one register from 0x09 bit 3 to 4
 *
 *  @param mag_operation_mode_u8 : The value of mag operation mode
 *
 *  mag_operation_mode_u8  |      result
 * ------------------------|--------------------------
 *     0x00                | MAG_OPR_MODE_LOWPOWER
 *     0x01                | MAG_OPR_MODE_REGULAR
 *     0x02                | MAG_OPR_MODE_ENHANCED_REGULAR
 *     0x03                | MAG_OPR_MODE_HIGH_ACCURACY
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_operation_mode(u8 *mag_operation_mode_u8);

/*!
 *  @brief This API used to write the mag operation mode
 *  from page one register from 0x09 bit 3 to 4
 *
 *  @param mag_operation_mode_u8 : The value of mag operation mode
 *
 *  mag_operation_mode_u8  |      result
 * ------------------------|--------------------------
 *     0x00                | MAG_OPR_MODE_LOWPOWER
 *     0x01                | MAG_OPR_MODE_REGULAR
 *     0x02                | MAG_OPR_MODE_ENHANCED_REGULAR
 *     0x03                | MAG_OPR_MODE_HIGH_ACCURACY
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_operation_mode(u8 mag_operation_mode_u8);

/*!
 *  @brief This API used to read the mag power mode
 *  from page one register from 0x09 bit 4 to 6
 *
 *  @param mag_power_mode_u8 : The value of mag power mode
 *
 * mag_power_mode_u8   |  result
 * --------------------|-----------------
 *     0x00            | BNO055_MAG_POWER_MODE_NORMAL
 *     0x01            | BNO055_MAG_POWER_MODE_SLEEP
 *     0x02            | BNO055_MAG_POWER_MODE_SUSPEND
 *     0x03            | BNO055_MAG_POWER_MODE_FORCE_MODE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_power_mode(u8 *mag_power_mode_u8);

/*!
 *  @brief This API used to write the mag power mode
 *  from page one register from 0x09 bit 4 to 6
 *
 *  @param mag_power_mode_u8 : The value of mag power mode
 *
 * mag_power_mode_u8 |   result
 * ------------------|-----------------
 *     0x00          | BNO055_MAG_POWER_MODE_NORMAL
 *     0x01          | BNO055_MAG_POWER_MODE_SLEEP
 *     0x02          | BNO055_MAG_POWER_MODE_SUSPEND
 *     0x03          | BNO055_MAG_POWER_MODE_FORCE_MODE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_power_mode(u8 mag_power_mode_u8);

/*****************************************************/
/**\name FUNCTIONS FOR GYRO CONFIGURATION */
/*****************************************************/

/*!
 *  @brief This API used to read the gyro range
 *  from page one register from 0x0A bit 0 to 3
 *
 *  @param gyro_range_u8 : The value of gyro range
 *
 *     gyro_range_u8 |   result
 * --------------------|-----------------
 *     0x00            | BNO055_GYRO_RANGE_2000DPS
 *     0x01            | BNO055_GYRO_RANGE_1000DPS
 *     0x02            | BNO055_GYRO_RANGE_500DPS
 *     0x03            | BNO055_GYRO_RANGE_250DPS
 *     0x04            | BNO055_GYRO_RANGE_125DPS
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_range(u8 *gyro_range_u8);

/*!
 *  @brief This API used to write the gyro range
 *  from page one register from 0x0A bit 0 to 3
 *
 *  @param gyro_range_u8 : The value of gyro range
 *
 *     gyro_range_u8 |   result
 * --------------------|-----------------
 *     0x00            | BNO055_GYRO_RANGE_2000DPS
 *     0x01            | BNO055_GYRO_RANGE_1000DPS
 *     0x02            | BNO055_GYRO_RANGE_500DPS
 *     0x03            | BNO055_GYRO_RANGE_250DPS
 *     0x04            | BNO055_GYRO_RANGE_125DPS
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_range(u8 gyro_range_u8);

/*!
 *  @brief This API used to read the gyro bandwidth
 *  from page one register from 0x0A bit 3 to 5
 *
 *  @param gyro_bw_u8 : The value of gyro bandwidth
 *
 *     gyro_bw_u8    |   result
 * --------------------|-----------------
 *     0x00            | BNO055_GYRO_BW_523HZ
 *     0x01            | BNO055_GYRO_BW_230HZ
 *     0x02            | BNO055_GYRO_BW_116HZ
 *     0x03            | BNO055_GYRO_BW_47HZ
 *     0x04            | BNO055_GYRO_BW_23HZ
 *     0x05            | BNO055_GYRO_BW_12HZ
 *     0x06            | BNO055_GYRO_BW_64HZ
 *     0x07            | BNO055_GYRO_BW_32HZ
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_bw(u8 *gyro_bw_u8);

/*!
 *  @brief This API used to write the gyro bandwidth
 *  from page one register from 0x0A bit 3 to 5
 *
 *  @param gyro_bw_u8 : The value of gyro bandwidth
 *
 *     gyro_bw_u8    |   result
 * --------------------|-----------------
 *     0x00            | BNO055_GYRO_BW_523HZ
 *     0x01            | BNO055_GYRO_BW_230HZ
 *     0x02            | BNO055_GYRO_BW_116HZ
 *     0x03            | BNO055_GYRO_BW_47HZ
 *     0x04            | BNO055_GYRO_BW_23HZ
 *     0x05            | BNO055_GYRO_BW_12HZ
 *     0x06            | BNO055_GYRO_BW_64HZ
 *     0x07            | BNO055_GYRO_BW_32HZ
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_bw(u8 gyro_bw_u8);

/*!
 *  @brief This API used to read the gyro power mode
 *  from page one register from 0x0B bit 0 to 2
 *
 *  @param gyro_power_mode_u8 : The value of gyro power mode
 *
 *  gyro_power_mode_u8 |          result
 * ----------------------|----------------------------
 *     0x00              | GYRO_OPR_MODE_NORMAL
 *     0x01              | GYRO_OPR_MODE_FASTPOWERUP
 *     0x02              | GYRO_OPR_MODE_DEEPSUSPEND
 *     0x03              | GYRO_OPR_MODE_SUSPEND
 *     0x04              | GYRO_OPR_MODE_ADVANCE_POWERSAVE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_power_mode(u8 *gyro_power_mode_u8);

/*!
 *  @brief This API used to write the gyro power mode
 *  from page one register from 0x0B bit 0 to 2
 *
 *  @param gyro_power_mode_u8 : The value of gyro power mode
 *
 *  gyro_power_mode_u8 |          result
 * ----------------------|----------------------------
 *     0x00              | GYRO_OPR_MODE_NORMAL
 *     0x01              | GYRO_OPR_MODE_FASTPOWERUP
 *     0x02              | GYRO_OPR_MODE_DEEPSUSPEND
 *     0x03              | GYRO_OPR_MODE_SUSPEND
 *     0x04              | GYRO_OPR_MODE_ADVANCE_POWERSAVE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_power_mode(u8 gyro_power_mode_u8);

/*****************************************************/
/**\name FUNCTIONS FOR ACCEL SLEEP SETTINGS  */
/*****************************************************/

/*!
 *  @brief This API used to read the accel sleep mode
 *  from page one register from 0x0C bit 0
 *
 *  @param sleep_tmr_u8 : The value of accel sleep mode
 *
 *  sleep_tmr_u8   |   result
 * ----------------- |------------------------------------
 *     0x00          | enable EventDrivenSampling(EDT)
 *     0x01          | enable Equidistant sampling mode(EST)
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleep_tmr_mode(u8 *sleep_tmr_u8);

/*!
 *  @brief This API used to write the accel sleep mode
 *  from page one register from 0x0C bit 0
 *
 *  @param sleep_tmr_u8 : The value of accel sleep mode
 *
 *  sleep_tmr_u8   |   result
 * ----------------- |------------------------------------
 *     0x00          | enable EventDrivenSampling(EDT)
 *     0x01          | enable Equidistant sampling mode(EST)
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleep_tmr_mode(u8 sleep_tmr_u8);

/*!
 *  @brief This API used to read the accel sleep duration
 *  from page one register from 0x0C bit 1 to 4
 *
 *  @param sleep_durn_u8 : The value of accel sleep duration
 *
 * sleep_durn_u8  |      result
 * ---------------- |-----------------------------
 *     0x05         | BNO055_ACCEL_SLEEP_DURN_0_5MS
 *     0x06         | BNO055_ACCEL_SLEEP_DURN_1MS
 *     0x07         | BNO055_ACCEL_SLEEP_DURN_2MS
 *     0x08         | BNO055_ACCEL_SLEEP_DURN_4MS
 *     0x09         | BNO055_ACCEL_SLEEP_DURN_6MS
 *     0x0A         | BNO055_ACCEL_SLEEP_DURN_10MS
 *     0x0B         | BNO055_ACCEL_SLEEP_DURN_25MS
 *     0x0C         | BNO055_ACCEL_SLEEP_DURN_50MS
 *     0x0D         | BNO055_ACCEL_SLEEP_DURN_100MS
 *     0x0E         | BNO055_ACCEL_SLEEP_DURN_500MS
 *     0x0F         | BNO055_ACCEL_SLEEP_DURN_1S
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleep_durn(u8 *sleep_durn_u8);

/*!
 *  @brief This API used to write the accel sleep duration
 *  from page one register from 0x0C bit 1 to 4
 *
 *  @param sleep_durn_u8 : The value of accel sleep duration
 *
 * sleep_durn_u8  |      result
 * ---------------- |-----------------------------
 *     0x05         | BNO055_ACCEL_SLEEP_DURN_0_5MS
 *     0x06         | BNO055_ACCEL_SLEEP_DURN_1MS
 *     0x07         | BNO055_ACCEL_SLEEP_DURN_2MS
 *     0x08         | BNO055_ACCEL_SLEEP_DURN_4MS
 *     0x09         | BNO055_ACCEL_SLEEP_DURN_6MS
 *     0x0A         | BNO055_ACCEL_SLEEP_DURN_10MS
 *     0x0B         | BNO055_ACCEL_SLEEP_DURN_25MS
 *     0x0C         | BNO055_ACCEL_SLEEP_DURN_50MS
 *     0x0D         | BNO055_ACCEL_SLEEP_DURN_100MS
 *     0x0E         | BNO055_ACCEL_SLEEP_DURN_500MS
 *     0x0F         | BNO055_ACCEL_SLEEP_DURN_1S
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleep_durn(u8 sleep_durn_u8);

/*****************************************************/
/**\name FUNCTIONS FOR GYRO SLEEP SETTINGS  */
/*****************************************************/

/*!
 *  @brief This API used to write the gyro sleep duration
 *  from page one register from 0x0D bit 0 to 2
 *
 *  @param sleep_durn_u8 : The value of gyro sleep duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_sleep_durn(u8 *sleep_durn_u8);

/*!
 *  @brief This API used to write the gyro sleep duration
 *  from page one register from 0x0D bit 0 to 2
 *
 *  @param sleep_durn_u8 : The value of gyro sleep duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_sleep_durn(u8 sleep_durn);

/*!
 *  @brief This API used to read the gyro auto sleep duration
 *  from page one register from 0x0D bit 3 to 5
 *
 *  @param auto_sleep_durn_u8 : The value of gyro auto sleep duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_auto_sleep_durn(u8 *auto_sleep_durn_u8);

/*!
 *  @brief This API used to write the gyro auto sleep duration
 *  from page one register from 0x0D bit 3 to 5
 *
 *  @param auto_sleep_durn_u8 : The value of gyro auto sleep duration
 *  @param bw : The value of gyro bandwidth
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_gyro_set_auto_sleep_durn(u8 auto_sleep_durn_u8, u8 bw);

/*****************************************************/
/**\name FUNCTIONS FOR MAG SLEEP SETTINGS  */
/*****************************************************/

/*!
 *  @brief This API used to read the mag sleep mode
 *  from page one register from 0x0E bit 0
 *
 *  @param sleep_mode_u8 : The value of mag sleep mode
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_mode(u8 *sleep_mode_u8);

/*!
 *  @brief This API used to write the mag sleep mode
 *  from page one register from 0x0E bit 0
 *
 *  @param sleep_mode_u8 : The value of mag sleep mode
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_mode(u8 sleep_mode_u8);

/*!
 *  @brief This API used to read the mag sleep duration
 *  from page one register from 0x0E bit 1 to 4
 *
 *  @param sleep_durn_u8 : The value of mag sleep duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_durn(u8 *sleep_durn_u8);

/*!
 *  @brief This API used to write the mag sleep duration
 *  from page one register from 0x0E bit 1 to 4
 *
 *  @param sleep_durn_u8 : The value of mag sleep duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_durn(u8 sleep_durn_u8);

/*****************************************************/
/**\name FUNCTIONS FOR GYRO INTERRUPT MASK  */
/*****************************************************/

/*!
 *  @brief This API used to read the gyro anymotion interrupt mask
 *  from page one register from 0x0F bit 2
 *
 *  @param gyro_any_motion_u8 : The value of gyro anymotion interrupt mask
 *      gyro_any_motion_u8 |   result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the gyro anymotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *  bno055_set_gyro_any_motion_axis_enable()
 *
 *  Filter setting:
 *  bno055_set_gyro_any_motion_filter()
 *
 *  Threshold :
 *
 *  bno055_set_gyro_any_motion_thres()
 *
 *  Slope samples :
 *
 *  bno055_set_gyro_any_motion_slope_samples()
 *
 *  Awake duration :
 *
 *  bno055_set_gyro_any_motion_awake_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_gyro_any_motion(u8 *gyro_any_motion_u8);

/*!
 *  @brief This API used to write the gyro anymotion interrupt mask
 *  from page one register from 0x0F bit 2
 *
 *  @param gyro_any_motion_u8 : The value of gyro anymotion interrupt mask
 *      gyro_any_motion_u8 |   result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the gyro anymotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *  bno055_set_gyro_any_motion_axis_enable()
 *
 *  Filter setting:
 *  bno055_set_gyro_any_motion_filter()
 *
 *  Threshold :
 *
 *  bno055_set_gyro_any_motion_thres()
 *
 *  Slope samples :
 *
 *  bno055_set_gyro_any_motion_slope_samples()
 *
 *  Awake duration :
 *
 *  bno055_set_gyro_any_motion_awake_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_gyro_any_motion(u8 gyro_any_motion_u8);

/*!
 *  @brief This API used to read the gyro highrate interrupt mask
 *  from page one register from 0x0F bit 3
 *
 *  @param gyro_highrate_u8 : The value of gyro highrate interrupt mask
 *        gyro_highrate_u8 |  result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the gyro highrate interrupt
 *  configure the below settings by using
 *  the following APIs
 *
 *  Axis :
 *
 *  bno055_set_gyro_highrate_axis_enable()
 *
 *  Filter :
 *
 *  bno055_set_gyro_highrate_filter()
 *
 *  Threshold :
 *
 *  bno055_get_gyro_highrate_x_thres()
 *
 *  bno055_get_gyro_highrate_y_thres()
 *
 *  bno055_get_gyro_highrate_z_thres()
 *
 *  Hysteresis :
 *
 *  bno055_set_gyro_highrate_x_hyst()
 *
 *  bno055_set_gyro_highrate_y_hyst()
 *
 *  bno055_set_gyro_highrate_z_hyst()
 *
 *  Duration :
 *
 *  bno055_set_gyro_highrate_x_durn()
 *
 *  bno055_set_gyro_highrate_y_durn()
 *
 *  bno055_set_gyro_highrate_z_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_gyro_highrate(u8 *gyro_highrate_u8);

/*!
 *  @brief This API used to write the gyro highrate interrupt mask
 *  from page one register from 0x0F bit 3
 *
 *  @param gyro_highrate_u8 : The value of gyro highrate interrupt mask
 *        gyro_highrate_u8 |  result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the gyro highrate interrupt
 *  configure the below settings by using
 *  the following APIs
 *
 *  Axis :
 *
 *  bno055_set_gyro_highrate_axis_enable()
 *
 *  Filter :
 *
 *  bno055_set_gyro_highrate_filter()
 *
 *  Threshold :
 *
 *  bno055_get_gyro_highrate_x_thres()
 *
 *  bno055_get_gyro_highrate_y_thres()
 *
 *  bno055_get_gyro_highrate_z_thres()
 *
 *  Hysteresis :
 *
 *  bno055_set_gyro_highrate_x_hyst()
 *
 *  bno055_set_gyro_highrate_y_hyst()
 *
 *  bno055_set_gyro_highrate_z_hyst()
 *
 *  Duration :
 *
 *  bno055_set_gyro_highrate_x_durn()
 *
 *  bno055_set_gyro_highrate_y_durn()
 *
 *  bno055_set_gyro_highrate_z_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_gyro_highrate(u8 gyro_highrate_u8);

/*****************************************************/
/**\name APIs FOR ACCEL INTERRUPT MASK  */
/*****************************************************/

/*!
 *  @brief This API used to read the accel highg interrupt mask
 *  from page one register from 0x0F bit 5
 *
 *  @param accel_high_g_u8 : The value of accel highg interrupt mask
 *         accel_high_g_u8 |   result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel highg interrupt
 *  configure the below settings by using
 *  the following APIs
 *
 *  Axis :
 *
 *  bno055_set_accel_high_g_axis_enable()
 *
 *  Threshold :
 *
 *  bno055_set_accel_high_g_thres()
 *
 *  Duration :
 *
 *  bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_accel_high_g(u8 *accel_high_g_u8);

/*!
 *  @brief This API used to write the accel highg interrupt mask
 *  from page one register from 0x0F bit 5
 *
 *  @param accel_high_g_u8 : The value of accel highg interrupt mask
 *         accel_high_g_u8 |   result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel highg interrupt
 *  configure the below settings by using
 *  the following APIs
 *
 *  Axis :
 *
 *  bno055_set_accel_high_g_axis_enable()
 *
 *  Threshold :
 *
 *  bno055_set_accel_high_g_thres()
 *
 *  Duration :
 *
 *  bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_accel_high_g(u8 accel_high_g_u8);

/*!
 *  @brief This API used to read the accel anymotion interrupt mask
 *  from page one register from 0x0F bit 6
 *
 *  @param accel_any_motion_u8 : The value of accel anymotion interrupt mask
 *     accel_any_motion_u8 | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel highg interrupt
 *  configure the below settings by using
 *  the following APIs
 *
 *  Axis :
 *
 *  bno055_set_accel_high_g_axis_enable()
 *
 *  Threshold :
 *
 *  bno055_set_accel_high_g_thres()
 *
 *  Duration :
 *
 *  bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_accel_any_motion(u8 *accel_any_motion_u8);

/*!
 *  @brief This API used to write the accel anymotion interrupt mask
 *  from page one register from 0x0F bit 6
 *
 *  @param accel_any_motion_u8 : The value of accel anymotion interrupt mask
 *     accel_any_motion_u8 | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel anymotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *
 *  bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *  Duration:
 *
 *  bno055_set_accel_any_motion_durn()
 *
 * Threshold:
 *
 *  bno055_set_accel_any_motion_thres()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_accel_any_motion(u8 accel_any_motion_u8);

/*!
 *  @brief This API used to read the accel nomotion interrupt mask
 *  from page one register from 0x0F bit 7
 *
 *  @param accel_nomotion_u8 : The value of accel nomotion interrupt mask
 *     accel_nomotion_u8   | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *
 *  @note While enabling the accel anymotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *
 *  bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *  Duration:
 *
 *  bno055_set_accel_any_motion_durn()
 *
 * Threshold:
 *
 *  bno055_set_accel_any_motion_thres())
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_accel_no_motion(u8 *accel_nomotion_u8);

/*!
 *  @brief This API used to write the accel nomotion interrupt mask
 *  from page one register from 0x0F bit 7
 *
 *  @param accel_nomotion_u8 : The value of accel nomotion interrupt mask
 *     accel_nomotion_u8   | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel nomotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *
 *  bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *  Threshold :
 *
 *  bno055_set_accel_slow_no_motion_thres()
 *
 *  Duration :
 *
 *  bno055_set_accel_slow_no_motion_durn()
 *
 *  Slow/no motion enable:
 *
 *  bno055_set_accel_slow_no_motion_enable()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_accel_no_motion(u8 accel_nomotion_u8);

/*****************************************************/
/**\name FUNCTIONS FOR GYRO INTERRUPT */
/*****************************************************/

/*!
 *  @brief This API used to read the gyro anymotion interrupt
 *  from page one register from 0x10 bit 2
 *
 *  @param gyro_any_motion_u8 : The value of gyro anymotion interrupt
 *      gyro_any_motion_u8 | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the gyro anymotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *  bno055_set_gyro_any_motion_axis_enable()
 *
 *  Filter setting:
 *  bno055_set_gyro_any_motion_filter()
 *
 *  Threshold :
 *
 *  bno055_set_gyro_any_motion_thres()
 *
 *  Slope samples :
 *
 *  bno055_set_gyro_any_motion_slope_samples()
 *
 *  Awake duration :
 *
 *  bno055_set_gyro_any_motion_awake_durn()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_gyro_any_motion(u8 *gyro_any_motion_u8);

/*!
 *  @brief This API used to write the gyro anymotion interrupt
 *  from page one register from 0x10 bit 2
 *
 *  @param gyro_any_motion_u8 : The value of gyro anymotion interrupt
 *    gyro_any_motion_u8   | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the gyro anymotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *  bno055_set_gyro_any_motion_axis_enable()
 *
 *  Filter setting:
 *  bno055_set_gyro_any_motion_filter()
 *
 *  Threshold :
 *
 *  bno055_set_gyro_any_motion_thres()
 *
 *  Slope samples :
 *
 *  bno055_set_gyro_any_motion_slope_samples()
 *
 *  Awake duration :
 *
 *  bno055_set_gyro_any_motion_awake_durn()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_gyro_any_motion(u8 gyro_any_motion_u8);

/*!
 *  @brief This API used to read the gyro highrate interrupt
 *  from page one register from 0x10 bit 3
 *
 *  @param gyro_highrate_u8 : The value of gyro highrate interrupt
 *      gyro_highrate_u8   | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the gyro highrate interrupt
 *  configure the below settings by using
 *  the following APIs
 *
 *  Axis :
 *
 *  bno055_set_gyro_highrate_axis_enable()
 *
 *  Filter :
 *
 *  bno055_set_gyro_highrate_filter()
 *
 *  Threshold :
 *
 *  bno055_get_gyro_highrate_x_thres()
 *
 *  bno055_get_gyro_highrate_y_thres()
 *
 *  bno055_get_gyro_highrate_z_thres()
 *
 *  Hysteresis :
 *
 *  bno055_set_gyro_highrate_x_hyst()
 *
 *  bno055_set_gyro_highrate_y_hyst()
 *
 *  bno055_set_gyro_highrate_z_hyst()
 *
 *  Duration :
 *
 *  bno055_set_gyro_highrate_x_durn()
 *
 *  bno055_set_gyro_highrate_y_durn()
 *
 *  bno055_set_gyro_highrate_z_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_gyro_highrate(u8 *gyro_highrate_u8);

/*!
 *  @brief This API used to write the gyro highrate interrupt
 *  from page one register from 0x10 bit 3
 *
 *  @param gyro_highrate_u8 : The value of gyro highrate interrupt
 *      gyro_highrate_u8   | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the gyro highrate interrupt
 *  configure the below settings by using
 *  the following APIs
 *
 *  Axis :
 *
 *  bno055_set_gyro_highrate_axis_enable()
 *
 *  Filter :
 *
 *  bno055_set_gyro_highrate_filter()
 *
 *  Threshold :
 *
 *  bno055_get_gyro_highrate_x_thres()
 *
 *  bno055_get_gyro_highrate_y_thres()
 *
 *  bno055_get_gyro_highrate_z_thres()
 *
 *  Hysteresis :
 *
 *  bno055_set_gyro_highrate_x_hyst()
 *
 *  bno055_set_gyro_highrate_y_hyst()
 *
 *  bno055_set_gyro_highrate_z_hyst()
 *
 *  Duration :
 *
 *  bno055_set_gyro_highrate_x_durn()
 *
 *  bno055_set_gyro_highrate_y_durn()
 *
 *  bno055_set_gyro_highrate_z_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_gyro_highrate(u8 gyro_highrate_u8);

/*****************************************************/
/**\name FUNCTIONS FOR ACCEL INTERRUPT  */
/*****************************************************/

/*!
 *  @brief This API used to read the accel highg interrupt
 *  from page one register from 0x10 bit 5
 *
 *  @param accel_high_g_u8 : The value of accel highg interrupt
 *      accel_high_g_u8    | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel highg interrupt
 *  configure the below settings by using
 *  the following APIs
 *
 *  Axis :
 *
 *  bno055_set_accel_high_g_axis_enable()
 *
 *  Threshold :
 *
 *  bno055_set_accel_high_g_thres()
 *
 *  Duration :
 *
 *  bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_accel_high_g(u8 *accel_high_g_u8);

/*!
 *  @brief This API used to write the accel highg interrupt
 *  from page one register from 0x10 bit 5
 *
 *  @param accel_high_g_u8 : The value of accel highg interrupt
 *      accel_high_g_u8    | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel highg interrupt
 *  configure the below settings by using
 *  the following APIs
 *
 *  Axis :
 *
 *  bno055_set_accel_high_g_axis_enable()
 *
 *  Threshold :
 *
 *  bno055_set_accel_high_g_thres()
 *
 *  Duration :
 *
 *  bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_accel_high_g(u8 accel_high_g_u8);

/*!
 *  @brief This API used to read the accel anymotion interrupt
 *  from page one register from 0x10 bit 6
 *
 *  @param accel_any_motion_u8 : The value of accel anymotion interrupt
 *  accel_any_motion_u8    | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel anymotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *
 *  bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *  Duration:
 *
 *  bno055_set_accel_any_motion_durn()
 *
 * Threshold:
 *
 *  bno055_set_accel_any_motion_thres()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_accel_any_motion(u8 *accel_any_motion_u8);

/*!
 *  @brief This API used to write the accel anymotion interrupt
 *  from page one register from 0x10 bit 6
 *
 *  @param accel_any_motion_u8 : The value of accel anymotion interrupt
 *  accel_any_motion_u8    | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel anymotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *
 *  bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *  Duration:
 *
 *  bno055_set_accel_any_motion_durn()
 *
 *  Threshold:
 *
 *  bno055_set_accel_any_motion_thres()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_accel_any_motion(u8 accel_any_motion_u8);

/*!
 *  @brief This API used to read the accel nomotion interrupt
 *  from page one register from 0x10 bit 6
 *
 *  @param accel_nomotion_u8 : The value of accel nomotion interrupt
 *    accel_nomotion_u8    | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel nomotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *
 *  bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *  Threshold :
 *
 *  bno055_set_accel_slow_no_motion_thres()
 *
 *  Duration :
 *
 *  bno055_set_accel_slow_no_motion_durn()
 *
 *  Slow/no motion enable:
 *
 *  bno055_set_accel_slow_no_motion_enable()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_accel_no_motion(u8 *accel_nomotion_u8);

/*!
 *  @brief This API used to write the accel nomotion interrupt
 *  from page one register from 0x10 bit 6
 *
 *  @param accel_nomotion_u8 : The value of accel nomotion interrupt
 *    accel_nomotion_u8    | result
 *     --------------------  |------------
 *              0x01         | BNO055_BIT_ENABLE
 *              0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note While enabling the accel nomotion interrupt
 *  configure the following settings
 *
 *  Axis:
 *
 *  bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *  Threshold :
 *
 *  bno055_set_accel_slow_no_motion_thres()
 *
 *  Duration :
 *
 *  bno055_set_accel_slow_no_motion_durn()
 *
 *  Slow/no motion enable:
 *
 *  bno055_set_accel_slow_no_motion_enable()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_accel_no_motion(u8 accel_nomotion_u8);

/*****************************************************/
/**\name FUNCTIONS FOR ACCEL ANY_MOTION THRESHOLD  */
/*****************************************************/

/*!
 *  @brief This API used to read the accel any motion threshold
 *  from page one register from 0x11 bit 0 to 7
 *
 *  @param accel_any_motion_thres_u8 : The value of any motion threshold
 *  accel_any_motion_thres_u8 | result
 *  ------------------------    | -------------
 *              0x01            | BNO055_BIT_ENABLE
 *              0x00            | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Accel anymotion threshold dependent on the
 *  range values
 *
 *  accel_range_u8 |    threshold | LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_any_motion_thres(u8 *accel_any_motion_thres_u8);

/*!
 *  @brief This API used to write the accel any motion threshold
 *  from page one register from 0x11 bit 0 to 7
 *
 *  @param accel_any_motion_thres_u8 : The value of any motion threshold
 *  accel_any_motion_thres_u8 | result
 *  ------------------------    | -------------
 *              0x01            | BNO055_BIT_ENABLE
 *              0x00            | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Accel anymotion threshold dependent on the
 *  range values
 *
 *  accel_range_u8   |  threshold    |  LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_any_motion_thres(u8 accel_any_motion_thres_u8);

/*****************************************************/
/**\name FUNCTIONS FOR ACCEL ANY_MOTION DURATION  */
/*****************************************************/

/*!
 *  @brief This API used to read the accel anymotion duration
 *  from page one register from 0x12 bit 0 to 1
 *
 *  @param accel_any_motion_durn_u8 : The value of accel anymotion duration
 * accel_any_motion_durn_u8  | result
 *  -------------------------  | -------------
 *              0x01           | BNO055_BIT_ENABLE
 *              0x00           | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_any_motion_durn(u8 *accel_any_motion_durn_u8);

/*!
 *  @brief This API used to write the accel anymotion duration
 *  from page one register from 0x12 bit 0 to 1
 *
 *  @param accel_any_motion_durn_u8 : The value of accel anymotion duration
 *
 * accel_any_motion_durn_u8  | result
 *  -------------------------  | -------------
 *              0x01           | BNO055_BIT_ENABLE
 *              0x00           | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_any_motion_durn(u8 accel_any_motion_durn_u8);

/*****************************************************/
/**\name FUNCTIONS FOR ACCEL ANY_MOTION AXIS ENABLE  */
/*****************************************************/

/*!
 *  @brief This API used to read the accel anymotion enable
 *  from page one register from 0x12 bit 2 to 4
 *
 *  @param data_u8 : The value of accel anymotion enable
 *     data_u8 | result
 *  ------------ | -------------
 *      0x01     | BNO055_BIT_ENABLE
 *      0x00     | BNO055_BIT_DISABLE
 *  @param channel_u8 : The value of accel anymotion axis selection
 *           channel_u8                        | value
 *     --------------------------                | ----------
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS  |   0
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS  |   1
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS  |   2
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_any_motion_no_motion_axis_enable(u8 channel_u8, u8 *data_u8);

/*!
 *  @brief This API used to write the accel anymotion enable
 *  from page one register from 0x12 bit 2 to 4
 *
 *  @param data_u8 : The value of accel anymotion enable
 *     data_u8 | result
 *  ------------ | -------------
 *      0x01     | BNO055_BIT_ENABLE
 *      0x00     | BNO055_BIT_DISABLE
 *  @param channel_u8 : The value of accel anymotion axis selection
 *           channel_u8                        | value
 *     --------------------------                | ----------
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS  |   0
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS  |   1
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS  |   2
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_any_motion_no_motion_axis_enable(u8 channel_u8, u8 data_u8);

/*****************************************************/
/**\name FUNCTIONS FOR ACCEL HIGHG AXIS ENABLE  */
/*****************************************************/

/*!
 *  @brief This API used to read the accel highg enable
 *  from page one register from 0x12 bit 5 to 7
 *
 *  @param data_u8 : The value of accel highg enable
 *      data_u8| result
 *  ------------ | -------------
 *      0x01     | BNO055_BIT_ENABLE
 *      0x00     | BNO055_BIT_DISABLE
 *  @param channel_u8 : The value of accel highg axis selection
 *               channel_u8     | value
 *     -------------------------- | ----------
 *     BNO055_ACCEL_HIGH_G_X_AXIS |   0
 *     BNO055_ACCEL_HIGH_G_Y_AXIS |   1
 *     BNO055_ACCEL_HIGH_G_Z_AXIS |   2
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_axis_enable(u8 channel_u8, u8 *data_u8);

/*!
 *  @brief This API used to write the accel highg enable
 *  from page one register from 0x12 bit 5 to 7
 *
 *  @param data_u8 : The value of accel highg enable
 *      data_u8| result
 *  ------------ | -------------
 *      0x01     | BNO055_BIT_ENABLE
 *      0x00     | BNO055_BIT_DISABLE
 *  @param channel_u8 : The value of accel highg axis selection
 *               channel_u8     | value
 *     -------------------------- | ----------
 *     BNO055_ACCEL_HIGH_G_X_AXIS |   0
 *     BNO055_ACCEL_HIGH_G_Y_AXIS |   1
 *     BNO055_ACCEL_HIGH_G_Z_AXIS |   2
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_axis_enable(u8 channel_u8, u8 data_u8);

/*****************************************************/
/**\name FUNCTIONS FOR ACCEL HIGHG DURATION */
/*****************************************************/

/*!
 *  @brief This API used to read the accel highg duration
 *  from page one register from 0x13 bit 0 to 7
 *
 *  @param accel_high_g_durn_u8 : The value of accel highg duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note The high-g interrupt trigger delay according
 *  to [highg duration + 1] * 2 ms
 *
 *  in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_durn(u8 *accel_high_g_durn_u8);

/*!
 *  @brief This API used to write the accel highg duration
 *  from page one register from 0x13 bit 0 to 7
 *
 *  @param accel_high_g_durn_u8 : The value of accel highg duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note The high-g interrupt trigger delay according
 *  to [highg duration + 1] * 2 ms
 *
 *  in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_durn(u8 accel_high_g_durn_u8);

/*****************************************************/
/**\name FUNCTIONS FOR ACCEL HIGHG THRESHOLD */
/*****************************************************/

/*!
 *  @brief This API used to read the accel highg threshold
 *  from page one register from 0x14 bit 0 to 7
 *
 *  @param accel_high_g_thres_u8 : The value of accel highg threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Accel highg interrupt threshold dependent
 *  for accel g range
 *
 *  accel_range_u8   |  threshold    |  LSB
 * ------------- | ------------- | ---------
 *     2g        |    7.81mg     |   1LSB
 *     4g        |    15.63mg    |   1LSB
 *     8g        |    31.25mg    |   1LSB
 *     16g       |    62.5mg     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_thres(u8 *accel_high_g_thres_u8);

/*!
 *  @brief This API used to write the accel highg threshold
 *  from page one register from 0x14 bit 0 to 7
 *
 *  @param accel_high_g_thres_u8 : The value of accel highg threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Accel highg interrupt threshold dependent
 *  for accel g range
 *
 *  accel_range_u8   |  threshold    |  LSB
 * ------------- | ------------- | ---------
 *     2g        |    7.81mg     |   1LSB
 *     4g        |    15.63mg    |   1LSB
 *     8g        |    31.25mg    |   1LSB
 *     16g       |    62.5mg     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_thres(u8 accel_high_g_thres_u8);

/**************************************************************/
/**\name FUNCTIONS FOR ACCEL SLOWNOMOTION THRESHOLD */
/**************************************************************/

/*!
 *  @brief This API used to read the accel slownomotion threshold
 *  from page one register from 0x15 bit 0 to 7
 *
 *  @param accel_slow_no_motion_thres_u8 :
 *  The value of accel slownomotion threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Accel slow no motion interrupt threshold dependent
 *  for accel g range
 *
 *  accel_range_u8   |  threshold    |  LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_thres(u8 *accel_slow_no_motion_thres_u8);

/*!
 *  @brief This API used to write the accel slownomotion threshold
 *  from page one register from 0x15 bit 0 to 7
 *
 *  @param accel_slow_no_motion_thres_u8 :
 *  The value of accel slownomotion threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Accel slow no motion interrupt threshold dependent
 *  for accel g range
 *
 *  accel_range_u8   |  threshold    |  LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_thres(u8 accel_slow_no_motion_thres_u8);

/**************************************************************/
/**\name FUNCTIONS FOR ACCEL SLOWNOMOTION ENABLE */
/**************************************************************/

/*!
 *  @brief This API used to read accel slownomotion enable
 *  from page one register from 0x16 bit 0
 *
 *  @param accel_slow_no_motion_en_u8 : The value of accel slownomotion enable
 *    accel_slow_no_motion_en_u8   | result
 *     ------------------------      | --------
 *              0x01                 | Slow motion
 *              0x00                 | No motion
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_enable(u8 *accel_slow_no_motion_en_u8);

/*!
 *  @brief This API used to write accel slownomotion enable
 *  from page one register from 0x16 bit 0
 *
 *  @param accel_slow_no_motion_en_u8 : The value of accel slownomotion enable
 *    accel_slow_no_motion_en_u8   | result
 *     ------------------------      | --------
 *              0x01                 | Slow motion
 *              0x00                 | No motion
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_enable(u8 accel_slow_no_motion_en_u8);

/**************************************************************/
/**\name FUNCTIONS FOR ACCEL SLOWNOMOTION DURATION */
/**************************************************************/

/*!
 *  @brief This API used to read accel slownomotion duration
 *  from page one register from 0x16 bit 1 to 6
 *
 *  @param accel_slow_no_motion_durn_u8 :
 *  The value of accel slownomotion duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_durn(u8 *accel_slow_no_motion_durn_u8);

/*!
 *  @brief This API used to write accel slownomotion duration
 *  from page one register from 0x16 bit 1 to 6
 *
 *  @param accel_slow_no_motion_durn_u8 :
 *  The value of accel slownomotion duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_durn(u8 accel_slow_no_motion_durn_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANY_MOTION AXIS ENABLE */
/**************************************************************/

/*!
 *  @brief This API used to read the gyro anymotion enable
 *  from page one register from 0x17 bit 0 to 2
 *
 *  @param data_u8 : The value of gyro anymotion enable
 *      data_u8     | result
 *  ----------------- |-------------
 *      0x01          | BNO055_BIT_ENABLE
 *      0x00          | BNO055_BIT_DISABLE
 *  @param channel_u8 : The value of gyro anymotion axis selection
 *               channel_u8         | value
 *     ---------------------------    | ----------
 *     BNO055_GYRO_ANY_MOTIONX_AXIS   |   0
 *     BNO055_GYRO_ANY_MOTIONY_AXIS   |   1
 *     BNO055_GYRO_ANY_MOTIONZ_AXIS   |   2
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_axis_enable(u8 channel_u8, u8 *data_u8);

/*!
 *  @brief This API used to write the gyro anymotion enable
 *  from page one register from 0x17 bit 0 to 2
 *
 *  @param data_u8 : The value of gyro anymotion enable
 *      data_u8     | result
 *  ----------------- |-------------
 *      0x01          | BNO055_BIT_ENABLE
 *      0x00          | BNO055_BIT_DISABLE
 *  @param channel_u8 : The value of gyro anymotion axis selection
 *               channel_u8         | value
 *     ---------------------------    | ----------
 *     BNO055_GYRO_ANY_MOTIONX_AXIS   |   0
 *     BNO055_GYRO_ANY_MOTIONY_AXIS   |   1
 *     BNO055_GYRO_ANY_MOTIONZ_AXIS   |   2
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_axis_enable(u8 channel_u8, u8 data_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE ENABLE */
/**************************************************************/

/*!
 *  @brief This API used to read the gyro highrate enable
 *  from page one register from 0x17 bit 3 to 5
 *
 *  @param data_u8 : The value of gyro highrate enable
 *      data_u8     | result
 *  ----------------  |-------------
 *      0x01          | BNO055_BIT_ENABLE
 *      0x00          | BNO055_BIT_DISABLE
 *  @param channel_u8 : The value of gyro highrate axis selection
 *               channel_u8         | value
 *     ------------------------       | ----------
 *     BNO055_GYRO_HIGHRATE_X_AXIS    |   0
 *     BNO055_GYRO_HIGHRATE_Y_AXIS    |   1
 *     BNO055_GYRO_HIGHRATE_Z_AXIS    |   2
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_axis_enable(u8 channel_u8, u8 *data_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE AXIS ENABLE */
/**************************************************************/

/*!
 *  @brief This API used to write the gyro highrate enable
 *  from page one register from 0x17 bit 3 to 5
 *
 *  @param data_u8 : The value of gyro highrate enable
 *      data_u8     | result
 *  ----------------  |-------------
 *      0x01          | BNO055_BIT_ENABLE
 *      0x00          | BNO055_BIT_DISABLE
 *  @param channel_u8 : The value of gyro highrate axis selection
 *               channel_u8         | value
 *     ------------------------       | ----------
 *     BNO055_GYRO_HIGHRATE_X_AXIS    |   0
 *     BNO055_GYRO_HIGHRATE_Y_AXIS    |   1
 *     BNO055_GYRO_HIGHRATE_Z_AXIS    |   2
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_axis_enable(u8 channel_u8, u8 data_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANY_MOTION FILTER */
/**************************************************************/

/*!
 *  @brief This API used to read gyro anymotion filter
 *  from page one register from 0x17 bit 6
 *
 *  @param gyro_any_motion_filter_u8 : The value of gyro anymotion filter
 *   gyro_any_motion_filter_u8  | result
 *  ---------------------------   |------------
 *      0x00                      | BNO055_GYRO_FILTERED_CONFIG
 *      0x01                      | BNO055_GYRO_UNFILTERED_CONFIG
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_filter(u8 *gyro_any_motion_filter_u8);

/*!
 *  @brief This API used to write gyro anymotion filter
 *  from page one register from 0x17 bit 6
 *
 *  @param gyro_any_motion_filter_u8 : The value of gyro anymotion filter
 *   gyro_any_motion_filter_u8  | result
 *  ---------------------------   |------------
 *      0x00                      | BNO055_GYRO_FILTERED_CONFIG
 *      0x01                      | BNO055_GYRO_UNFILTERED_CONFIG
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_filter(u8 gyro_any_motion_filter_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE FILTER */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate filter
 *  from page one register from 0x17 bit 7
 *
 *  @param gyro_highrate_filter_u8 : The value of gyro highrate filter
 *   gyro_highrate_filter_u8  | result
 *  --------------------------- |------------
 *         0x00                 | BNO055_GYRO_FILTERED_CONFIG
 *         0x01                 | BNO055_GYRO_UNFILTERED_CONFIG
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_filter(u8 *gyro_highrate_filter_u8);

/*!
 *  @brief This API used to write gyro highrate filter
 *  from page one register from 0x17 bit 7
 *
 *  @param gyro_highrate_filter_u8 : The value of gyro highrate filter
 *   gyro_highrate_filter_u8  | result
 *  --------------------------- |------------
 *         0x00                 | BNO055_GYRO_FILTERED_CONFIG
 *         0x01                 | BNO055_GYRO_UNFILTERED_CONFIG
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_filter(u8 gyro_highrate_filter_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE X THRESHOLD */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate x threshold
 *  from page one register from 0x18 bit 0 to 4
 *
 *  @param gyro_highrate_x_thres_u8 : The value of gyro x highrate threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate threshold dependent on the
 *  selection of gyro range
 *
 *  gyro_range_u8     | threshold       |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_thres(u8 *gyro_highrate_x_thres_u8);

/*!
 *  @brief This API used to write gyro highrate x threshold
 *  from page one register from 0x18 bit 0 to 4
 *
 *  @param gyro_highrate_x_thres_u8 : The value of gyro x highrate threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate threshold dependent on the
 *  selection of gyro range
 *
 *  gyro_range_u8     | threshold       |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_thres(u8 gyro_highrate_x_thres_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE X HYSTERESIS */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate x hysteresis
 *  from page one register from 0x18 bit 5 to 6
 *
 *  @param gyro_highrate_x_hyst_u8 : The value of gyro highrate x hysteresis
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro high rate hysteresis calculated by
 *
 *  using this (255 + 256 * gyro_highrate_x_hyst_u8) *4 LSB
 *
 *  The high rate value scales with the range setting
 *
 *  gyro_range_u8     | hysteresis      |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_hyst(u8 *gyro_highrate_x_hyst_u8);

/*!
 *  @brief This API used to write gyro highrate x hysteresis
 *  from page one register from 0x18 bit 5 to 6
 *
 *  @param gyro_highrate_x_hyst_u8 : The value of gyro highrate x hysteresis
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro high rate hysteresis calculated by
 *
 *  using this (255 + 256 * gyro_highrate_x_hyst_u8) *4 LSB
 *
 *  The high rate value scales with the range setting
 *
 *  gyro_range_u8     | hysteresis      |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_hyst(u8 gyro_highrate_x_hyst_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE X DURATION */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate x duration
 *  from page one register from 0x19 bit 0 to 7
 *
 *  @param gyro_highrate_x_durn_u8 : The value of gyro highrate x duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate duration calculate by using the formula
 *
 *  (1 + gyro_highrate_x_durn_u8)*2.5ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_durn(u8 *gyro_highrate_x_durn_u8);

/*!
 *  @brief This API used to write gyro highrate x duration
 *  from page one register from 0x19 bit 0 to 7
 *
 *  @param gyro_highrate_x_durn_u8 : The value of gyro highrate x duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate duration calculate by using the formula
 *
 *  (1 + gyro_highrate_x_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_durn(u8 gyro_highrate_x_durn_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Y THRESHOLD */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate y threshold
 *  from page one register from 0x1A bit 0 to 4
 *
 *  @param gyro_highrate_y_thres_u8 : The value of gyro highrate y threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate threshold dependent on the
 *  selection of gyro range
 *
 *  gyro_range_u8     | threshold       |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_thres(u8 *gyro_highrate_y_thres_u8);

/*!
 *  @brief This API used to write gyro highrate y threshold
 *  from page one register from 0x1A bit 0 to 4
 *
 *  @param gyro_highrate_y_thres_u8 : The value of gyro highrate y threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate threshold dependent on the
 *  selection of gyro range
 *
 *  gyro_range_u8     | threshold       |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_thres(u8 gyro_highrate_y_thres_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Y HYSTERESIS */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate y hysteresis
 *  from page one register from 0x1A bit 5 to 6
 *
 *  @param gyro_highrate_y_hyst_u8 : The value of gyro highrate y hysteresis
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro high rate hysteresis calculated by
 *
 *  using this (255 + 256 * gyro_highrate_y_hyst_u8) *4 LSB
 *
 *  The high rate value scales with the range setting
 *
 *  gyro_range_u8     | hysteresis      |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_hyst(u8 *gyro_highrate_y_hyst_u8);

/*!
 *  @brief This API used to write gyro highrate y hysteresis
 *  from page one register from 0x1A bit 5 to 6
 *
 *  @param gyro_highrate_y_hyst_u8 : The value of gyro highrate y hysteresis
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro high rate hysteresis calculated by
 *
 *  using this (255 + 256 * gyro_highrate_y_hyst_u8) *4 LSB
 *
 *  The high rate value scales with the range setting
 *
 *  gyro_range_u8     | hysteresis      |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_hyst(u8 gyro_highrate_y_hyst_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Y DURATION */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate y duration
 *  from page one register from 0x1B bit 0 to 7
 *
 *  @param gyro_highrate_y_durn_u8 : The value of gyro highrate y duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate duration calculate by using the formula
 *
 *  (1 + gyro_highrate_y_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_durn(u8 *gyro_highrate_y_durn_u8);

/*!
 *  @brief This API used to write gyro highrate y duration
 *  from page one register from 0x1B bit 0 to 7
 *
 *  @param gyro_highrate_y_durn_u8 : The value of gyro highrate y duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate duration calculate by using the formula
 *
 *  (1 + gyro_highrate_y_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_durn(u8 gyro_highrate_y_durn_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Z THRESHOLD */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate z threshold
 *  from page one register from 0x1C bit 0 to 4
 *
 *  @param gyro_highrate_z_thres_u8 : The value of gyro highrate z threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate threshold dependent on the
 *  selection of gyro range
 *
 *  gyro_range_u8     | threshold       |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_thres(u8 *gyro_highrate_z_thres_u8);

/*!
 *  @brief This API used to write gyro highrate z threshold
 *  from page one register from 0x1C bit 0 to 4
 *
 *  @param gyro_highrate_z_thres_u8 : The value of gyro highrate z threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate threshold dependent on the
 *  selection of gyro range
 *
 *  gyro_range_u8     | threshold       |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_thres(u8 gyro_highrate_z_thres_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Z HYSTERESIS */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate z hysteresis
 *  from page one register from 0x1C bit 5 to 6
 *
 *  @param gyro_highrate_z_hyst_u8 : The value of gyro highrate z hysteresis
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro high rate hysteresis calculated by
 *
 *  using this (255 + 256 * gyro_highrate_z_hyst_u8) *4 LSB
 *
 *  The high rate value scales with the range setting
 *
 *  gyro_range_u8     |  hysteresis     |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_hyst(u8 *gyro_highrate_z_hyst_u8);

/*!
 *  @brief This API used to write gyro highrate z hysteresis
 *  from page one register from 0x1C bit 5 to 6
 *
 *  @param gyro_highrate_z_hyst_u8 : The value of gyro highrate z hysteresis
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro high rate hysteresis calculated by
 *
 *  using this (255 + 256 * gyro_highrate_z_hyst_u8) *4 LSB
 *
 *  The high rate value scales with the range setting
 *
 *  gyro_range_u8     |  hysteresis     |     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_hyst(u8 gyro_highrate_z_hyst_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Z DURATION */
/**************************************************************/

/*!
 *  @brief This API used to read gyro highrate z duration
 *  from page one register from 0x1D bit 0 to 7
 *
 *  @param gyro_highrate_z_durn_u8 : The value of gyro highrate z duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate duration calculate by using the formula
 *
 *  (1 + gyro_highrate_z_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_durn(u8 *gyro_highrate_z_durn_u8);

/*!
 *  @brief This API used to write gyro highrate z duration
 *  from page one register from 0x1D bit 0 to 7
 *
 *  @param gyro_highrate_z_durn_u8 : The value of gyro highrate z duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro highrate duration calculate by using the formula
 *
 *  (1 + gyro_highrate_z_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_durn(u8 gyro_highrate_z_durn_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANY_MOTION THRESHOLD */
/**************************************************************/

/*!
 *  @brief This API used to read gyro anymotion threshold
 *  from page one register from 0x1E bit 0 to 6
 *
 *  @param gyro_any_motion_thres_u8 : The value of gyro anymotion threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro anymotion interrupt threshold dependent
 *  on the selection of gyro range
 *
 *  gyro_range_u8     | threshold     |    LSB
 * -----------------  | ------------- | ---------
 *     2000           |    1dps       |   1LSB
 *     1000           |    0.5dps     |   1LSB
 *     500            |    0.25dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_thres(u8 *gyro_any_motion_thres_u8);

/*!
 *  @brief This API used to write gyro anymotion threshold
 *  from page one register from 0x1E bit 0 to 6
 *
 *  @param gyro_any_motion_thres_u8 : The value of gyro anymotion threshold
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *  @note Gyro anymotion interrupt threshold dependent
 *  on the selection of gyro range
 *
 *  gyro_range_u8     | threshold     |    LSB
 * -----------------  | ------------- | ---------
 *     2000           |    1dps       |   1LSB
 *     1000           |    0.5dps     |   1LSB
 *     500            |    0.25dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_thres(u8 gyro_any_motion_thres_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANY_MOTION SLOPE SAMPLES */
/**************************************************************/

/*!
 *  @brief This API used to read gyro anymotion slope samples
 *  from page one register from 0x1F bit 0 to 1
 *
 *  @param gyro_any_motion_slope_samples_u8 :
 *  The value of gyro anymotion slope samples
 *  gyro_any_motion_slope_samples_u8   |   result
 *  ----------------------------------   | -----------
 *            0                          |    8 samples
 *            1                          |    16 samples
 *            2                          |    32 samples
 *            3                          |    64 samples
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_slope_samples(u8 *gyro_any_motion_slope_samples_u8);

/*!
 *  @brief This API used to write gyro anymotion slope samples
 *  from page one register from 0x1F bit 0 to 1
 *
 *  @param gyro_any_motion_slope_samples_u8 :
 *  The value of gyro anymotion slope samples
 *  gyro_any_motion_slope_samples_u8   |   result
 *  ---------------------------------- | -----------
 *            0                        |    8 samples
 *            1                        |    16 samples
 *            2                        |    32 samples
 *            3                        |    64 samples
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_slope_samples(u8 gyro_any_motion_slope_samples_u8);

/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANY_MOTION AWAKE DURATION */
/**************************************************************/

/*!
 *  @brief This API used to read gyro anymotion awake duration
 *  from page one register from 0x1F bit 2 to 3
 *
 *  @param gyro_awake_durn_u8 : The value of gyro anymotion awake duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_awake_durn(u8 *gyro_awake_durn_u8);

/*!
 *  @brief This API used to write gyro anymotion awake duration
 *  from page one register from 0x1F bit 2 to 3
 *
 *  @param gyro_awake_durn_u8 : The value of gyro anymotion awake duration
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_awake_durn(u8 gyro_awake_durn_u8);

#endif
