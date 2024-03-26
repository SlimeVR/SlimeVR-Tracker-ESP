/**
  ******************************************************************************
  * @file    lsm6dsv_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lsm6dsv_reg.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LSM6DSV_REGS_H
#define LSM6DSV_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup LSM6DSV
  * @{
  *
  */

/** @defgroup  Endianness definitions
  * @{
  *
  */

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t millisec);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Component optional fields **/
  stmdev_mdelay_ptr   mdelay;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
  *              You can create a sensor configuration by your own or using
  *              Unico / Unicleo tools available on STMicroelectronics
  *              web site.
  *
  * @{
  *
  */

typedef struct
{
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup LSM6DSV_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> D5 if SA0=1 -> D7 **/
#define LSM6DSV_I2C_ADD_L                    0xD5U
#define LSM6DSV_I2C_ADD_H                    0xD7U

/** Device Identification (Who am I) **/
#define LSM6DSV_ID                           0x70U

/**
  * @}
  *
  */

/** @defgroup bitfields page main
  * @{
  *
  */

#define LSM6DSV_FUNC_CFG_ACCESS              0x1U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ois_ctrl_from_ui     : 1;
  uint8_t spi2_reset           : 1;
  uint8_t sw_por               : 1;
  uint8_t fsm_wr_ctrl_en       : 1;
  uint8_t not_used0            : 2;
  uint8_t shub_reg_access      : 1;
  uint8_t emb_func_reg_access  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t emb_func_reg_access  : 1;
  uint8_t shub_reg_access      : 1;
  uint8_t not_used0            : 2;
  uint8_t fsm_wr_ctrl_en       : 1;
  uint8_t sw_por               : 1;
  uint8_t spi2_reset           : 1;
  uint8_t ois_ctrl_from_ui     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_func_cfg_access_t;

#define LSM6DSV_PIN_CTRL                     0x2U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 5;
  uint8_t ibhr_por_en          : 1;
  uint8_t sdo_pu_en            : 1;
  uint8_t ois_pu_dis           : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ois_pu_dis           : 1;
  uint8_t sdo_pu_en            : 1;
  uint8_t ibhr_por_en          : 1;
  uint8_t not_used0            : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_pin_ctrl_t;

#define LSM6DSV_IF_CFG                       0x3U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t i2c_i3c_disable      : 1;
  uint8_t not_used0            : 1;
  uint8_t sim                  : 1;
  uint8_t pp_od                : 1;
  uint8_t h_lactive            : 1;
  uint8_t asf_ctrl             : 1;
  uint8_t shub_pu_en           : 1;
  uint8_t sda_pu_en            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sda_pu_en            : 1;
  uint8_t shub_pu_en           : 1;
  uint8_t asf_ctrl             : 1;
  uint8_t h_lactive            : 1;
  uint8_t pp_od                : 1;
  uint8_t sim                  : 1;
  uint8_t not_used0            : 1;
  uint8_t i2c_i3c_disable      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_if_cfg_t;

#define LSM6DSV_ODR_TRIG_CFG                 0x6U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t odr_trig_nodr        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_trig_nodr        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_odr_trig_cfg_t;

#define LSM6DSV_FIFO_CTRL1                   0x7U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wtm                  : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wtm                  : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_ctrl1_t;

#define LSM6DSV_FIFO_CTRL2                   0x8U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl_dualc_batch_from_fsm       : 1;
  uint8_t uncompr_rate                  : 2;
  uint8_t not_used0                     : 1;
  uint8_t odr_chg_en                    : 1;
  uint8_t not_used1                     : 1;
  uint8_t fifo_compr_rt_en              : 1;
  uint8_t stop_on_wtm                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t stop_on_wtm                   : 1;
  uint8_t fifo_compr_rt_en              : 1;
  uint8_t not_used1                     : 1;
  uint8_t odr_chg_en                    : 1;
  uint8_t not_used0                     : 1;
  uint8_t uncompr_rate                  : 2;
  uint8_t xl_dualc_batch_from_fsm       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_ctrl2_t;

#define LSM6DSV_FIFO_CTRL3                  0x9U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bdr_xl               : 4;
  uint8_t bdr_gy               : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bdr_gy               : 4;
  uint8_t bdr_xl               : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_ctrl3_t;

#define LSM6DSV_FIFO_CTRL4                  0x0AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_mode            : 3;
  uint8_t g_eis_fifo_en        : 1;
  uint8_t odr_t_batch          : 2;
  uint8_t dec_ts_batch         : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dec_ts_batch         : 2;
  uint8_t odr_t_batch          : 2;
  uint8_t g_eis_fifo_en        : 1;
  uint8_t fifo_mode            : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_ctrl4_t;

#define LSM6DSV_COUNTER_BDR_REG1            0x0BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t cnt_bdr_th           : 2;
  uint8_t not_used0            : 3;
  uint8_t trig_counter_bdr     : 2;
  uint8_t not_used1            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 1;
  uint8_t trig_counter_bdr     : 2;
  uint8_t not_used0            : 3;
  uint8_t cnt_bdr_th           : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_counter_bdr_reg1_t;

#define LSM6DSV_COUNTER_BDR_REG2            0x0CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t cnt_bdr_th           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t cnt_bdr_th           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_counter_bdr_reg2_t;

#define LSM6DSV_INT1_CTRL                   0x0DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy_xl         : 1;
  uint8_t int1_drdy_g          : 1;
  uint8_t not_used0            : 1;
  uint8_t int1_fifo_th         : 1;
  uint8_t int1_fifo_ovr        : 1;
  uint8_t int1_fifo_full       : 1;
  uint8_t int1_cnt_bdr         : 1;
  uint8_t not_used1            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 1;
  uint8_t int1_cnt_bdr         : 1;
  uint8_t int1_fifo_full       : 1;
  uint8_t int1_fifo_ovr        : 1;
  uint8_t int1_fifo_th         : 1;
  uint8_t not_used0            : 1;
  uint8_t int1_drdy_g          : 1;
  uint8_t int1_drdy_xl         : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_int1_ctrl_t;

#define LSM6DSV_INT2_CTRL                   0x0EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy_xl         : 1;
  uint8_t int2_drdy_g          : 1;
  uint8_t int2_drdy_g_eis      : 1;
  uint8_t int2_fifo_th         : 1;
  uint8_t int2_fifo_ovr        : 1;
  uint8_t int2_fifo_full       : 1;
  uint8_t int2_cnt_bdr         : 1;
  uint8_t int2_emb_func_endop  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_emb_func_endop  : 1;
  uint8_t int2_cnt_bdr         : 1;
  uint8_t int2_fifo_full       : 1;
  uint8_t int2_fifo_ovr        : 1;
  uint8_t int2_fifo_th         : 1;
  uint8_t int2_drdy_g_eis      : 1;
  uint8_t int2_drdy_g          : 1;
  uint8_t int2_drdy_xl         : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_int2_ctrl_t;

#define LSM6DSV_WHO_AM_I                    0x0FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t id                   : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t id                   : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_who_am_i_t;

#define LSM6DSV_CTRL1                       0x10U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t odr_xl               : 4;
  uint8_t op_mode_xl           : 3;
  uint8_t not_used0            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t op_mode_xl           : 3;
  uint8_t odr_xl               : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl1_t;

#define LSM6DSV_CTRL2                       0x11U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t odr_g                : 4;
  uint8_t op_mode_g            : 3;
  uint8_t not_used0            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t op_mode_g            : 3;
  uint8_t odr_g                : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl2_t;

#define LSM6DSV_CTRL3                       0x12U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sw_reset             : 1;
  uint8_t not_used0            : 1;
  uint8_t if_inc               : 1;
  uint8_t not_used1            : 3;
  uint8_t bdu                  : 1;
  uint8_t boot                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                 : 1;
  uint8_t bdu                  : 1;
  uint8_t not_used1            : 3;
  uint8_t if_inc               : 1;
  uint8_t not_used0            : 1;
  uint8_t sw_reset             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl3_t;

#define LSM6DSV_CTRL4                       0x13U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_in_lh           : 1;
  uint8_t drdy_pulsed          : 1;
  uint8_t int2_drdy_temp       : 1;
  uint8_t drdy_mask            : 1;
  uint8_t int2_on_int1         : 1;
  uint8_t not_used0            : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t int2_on_int1         : 1;
  uint8_t drdy_mask            : 1;
  uint8_t int2_drdy_temp       : 1;
  uint8_t drdy_pulsed          : 1;
  uint8_t int2_in_lh           : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl4_t;

#define LSM6DSV_CTRL5                       0x14U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int_en_i3c           : 1;
  uint8_t bus_act_sel          : 2;
  uint8_t not_used0            : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 5;
  uint8_t bus_act_sel          : 2;
  uint8_t int_en_i3c           : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl5_t;

#define LSM6DSV_CTRL6                       0x15U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_g                 : 4;
  uint8_t lpf1_g_bw            : 3;
  uint8_t not_used0            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t lpf1_g_bw            : 3;
  uint8_t fs_g                 : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl6_t;

#define LSM6DSV_CTRL7                       0x16U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lpf1_g_en            : 1;
  uint8_t not_used0            : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 7;
  uint8_t lpf1_g_en            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl7_t;

#define LSM6DSV_CTRL8                       0x17U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_xl                : 2;
  uint8_t not_used0            : 1;
  uint8_t xl_dualc_en          : 1;
  uint8_t not_used1            : 1;
  uint8_t hp_lpf2_xl_bw        : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hp_lpf2_xl_bw        : 3;
  uint8_t not_used1            : 1;
  uint8_t xl_dualc_en          : 1;
  uint8_t not_used0            : 1;
  uint8_t fs_xl                : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl8_t;

#define LSM6DSV_CTRL9                       0x18U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t usr_off_on_out       : 1;
  uint8_t usr_off_w            : 1;
  uint8_t not_used0            : 1;
  uint8_t lpf2_xl_en           : 1;
  uint8_t hp_slope_xl_en       : 1;
  uint8_t xl_fastsettl_mode    : 1;
  uint8_t hp_ref_mode_xl       : 1;
  uint8_t not_used1            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 1;
  uint8_t hp_ref_mode_xl       : 1;
  uint8_t xl_fastsettl_mode    : 1;
  uint8_t hp_slope_xl_en       : 1;
  uint8_t lpf2_xl_en           : 1;
  uint8_t not_used0            : 1;
  uint8_t usr_off_w            : 1;
  uint8_t usr_off_on_out       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl9_t;

#define LSM6DSV_CTRL10                      0x19U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_xl                : 2;
  uint8_t st_g                 : 2;
  uint8_t not_used0            : 2;
  uint8_t emb_func_debug       : 1;
  uint8_t not_used1            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 1;
  uint8_t emb_func_debug       : 1;
  uint8_t not_used0            : 2;
  uint8_t st_g                 : 2;
  uint8_t st_xl                : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl10_t;

#define LSM6DSV_CTRL_STATUS                 0x1AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 2;
  uint8_t fsm_wr_ctrl_status   : 1;
  uint8_t not_used1            : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 5;
  uint8_t fsm_wr_ctrl_status   : 1;
  uint8_t not_used0            : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl_status_t;

#define LSM6DSV_FIFO_STATUS1                0x1BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff_fifo            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t diff_fifo            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_status1_t;

#define LSM6DSV_FIFO_STATUS2                0x1CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff_fifo            : 1;
  uint8_t not_used0            : 2;
  uint8_t fifo_ovr_latched     : 1;
  uint8_t counter_bdr_ia       : 1;
  uint8_t fifo_full_ia         : 1;
  uint8_t fifo_ovr_ia          : 1;
  uint8_t fifo_wtm_ia          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_wtm_ia          : 1;
  uint8_t fifo_ovr_ia          : 1;
  uint8_t fifo_full_ia         : 1;
  uint8_t counter_bdr_ia       : 1;
  uint8_t fifo_ovr_latched     : 1;
  uint8_t not_used0            : 2;
  uint8_t diff_fifo            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_status2_t;

#define LSM6DSV_ALL_INT_SRC                 0x1DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ia                : 1;
  uint8_t wu_ia                : 1;
  uint8_t tap_ia               : 1;
  uint8_t not_used0            : 1;
  uint8_t d6d_ia               : 1;
  uint8_t sleep_change_ia      : 1;
  uint8_t shub_ia              : 1;
  uint8_t emb_func_ia          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t emb_func_ia          : 1;
  uint8_t shub_ia              : 1;
  uint8_t sleep_change_ia      : 1;
  uint8_t d6d_ia               : 1;
  uint8_t not_used0            : 1;
  uint8_t tap_ia               : 1;
  uint8_t wu_ia                : 1;
  uint8_t ff_ia                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_all_int_src_t;

#define LSM6DSV_STATUS_REG                  0x1EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                 : 1;
  uint8_t gda                  : 1;
  uint8_t tda                  : 1;
  uint8_t not_used1            : 1;
  uint8_t gda_eis              : 1;
  uint8_t ois_drdy             : 1;
  uint8_t not_used0            : 1;
  uint8_t timestamp_endcount   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp_endcount   : 1;
  uint8_t not_used0            : 1;
  uint8_t ois_drdy             : 1;
  uint8_t gda_eis              : 1;
  uint8_t not_used1            : 1;
  uint8_t tda                  : 1;
  uint8_t gda                  : 1;
  uint8_t xlda                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_status_reg_t;

#define LSM6DSV_OUT_TEMP_L                  0x20U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t temp                 : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t temp                 : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_out_temp_l_t;

#define LSM6DSV_OUT_TEMP_H                  0x21U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t temp                 : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t temp                 : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_out_temp_h_t;

#define LSM6DSV_OUTX_L_G                    0x22U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outx_g               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outx_g               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outx_l_g_t;

#define LSM6DSV_OUTX_H_G                    0x23U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outx_g               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outx_g               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outx_h_g_t;

#define LSM6DSV_OUTY_L_G                    0x24U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outy_g               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outy_g               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outy_l_g_t;

#define LSM6DSV_OUTY_H_G                    0x25U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outy_g               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outy_g               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outy_h_g_t;

#define LSM6DSV_OUTZ_L_G                    0x26U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outz_g               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outz_g               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outz_l_g_t;

#define LSM6DSV_OUTZ_H_G                    0x27U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outz_g               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outz_g               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outz_h_g_t;

#define LSM6DSV_OUTX_L_A                    0x28U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outx_a               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outx_a               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outx_l_a_t;

#define LSM6DSV_OUTX_H_A                    0x29U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outx_a               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outx_a               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outx_h_a_t;

#define LSM6DSV_OUTY_L_A                    0x2AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outy_a               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outy_a               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outy_l_a_t;

#define LSM6DSV_OUTY_H_A                    0x2BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outy_a               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outy_a               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outy_h_a_t;

#define LSM6DSV_OUTZ_L_A                    0x2CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outz_a               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outz_a               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outz_l_a_t;

#define LSM6DSV_OUTZ_H_A                    0x2DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outz_a               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outz_a               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_outz_h_a_t;

#define LSM6DSV_UI_OUTX_L_G_OIS_EIS         0x2EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outx_g_ois_eis    : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outx_g_ois_eis    : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outx_l_g_ois_eis_t;

#define LSM6DSV_UI_OUTX_H_G_OIS_EIS         0x2FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outx_g_ois_eis    : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outx_g_ois_eis    : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outx_h_g_ois_eis_t;

#define LSM6DSV_UI_OUTY_L_G_OIS_EIS         0x30U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outy_g_ois_eis    : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outy_g_ois_eis    : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outy_l_g_ois_eis_t;

#define LSM6DSV_UI_OUTY_H_G_OIS_EIS         0x31U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outy_g_ois_eis    : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outy_g_ois_eis    : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outy_h_g_ois_eis_t;

#define LSM6DSV_UI_OUTZ_L_G_OIS_EIS         0x32U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outz_g_ois_eis    : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outz_g_ois_eis    : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outz_l_g_ois_eis_t;

#define LSM6DSV_UI_OUTZ_H_G_OIS_EIS         0x33U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outz_g_ois_eis    : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outz_g_ois_eis    : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outz_h_g_ois_eis_t;

#define LSM6DSV_UI_OUTX_L_A_OIS_DUALC       0x34U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outx_a_ois_dualc  : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outx_a_ois_dualc  : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outx_l_a_ois_dualc_t;

#define LSM6DSV_UI_OUTX_H_A_OIS_DUALC       0x35U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outx_a_ois_dualc  : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outx_a_ois_dualc  : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outx_h_a_ois_dualc_t;

#define LSM6DSV_UI_OUTY_L_A_OIS_DUALC       0x36U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outy_a_ois_dualc  : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outy_a_ois_dualc  : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outy_l_a_ois_dualc_t;

#define LSM6DSV_UI_OUTY_H_A_OIS_DUALC       0x37U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outy_a_ois_dualc  : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outy_a_ois_dualc  : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outy_h_a_ois_dualc_t;

#define LSM6DSV_UI_OUTZ_L_A_OIS_DUALC       0x38U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outz_a_ois_dualc  : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outz_a_ois_dualc  : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outz_l_a_ois_dualc_t;

#define LSM6DSV_UI_OUTZ_H_A_OIS_DUALC       0x39U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_outz_a_ois_dualc  : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_outz_a_ois_dualc  : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_outz_h_a_ois_dualc_t;

#define LSM6DSV_TIMESTAMP0                  0x40U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t timestamp            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_timestamp0_t;

#define LSM6DSV_TIMESTAMP1                  0x41U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t timestamp            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_timestamp1_t;

#define LSM6DSV_TIMESTAMP2                  0x42U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t timestamp            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_timestamp2_t;

#define LSM6DSV_TIMESTAMP3                  0x43U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t timestamp            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_timestamp3_t;

#define LSM6DSV_UI_STATUS_REG_OIS           0x44U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda_ois             : 1;
  uint8_t gda_ois              : 1;
  uint8_t gyro_settling        : 1;
  uint8_t not_used0            : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 5;
  uint8_t gyro_settling        : 1;
  uint8_t gda_ois              : 1;
  uint8_t xlda_ois             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_status_reg_ois_t;

#define LSM6DSV_WAKE_UP_SRC                 0x45U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_wu                 : 1;
  uint8_t y_wu                 : 1;
  uint8_t x_wu                 : 1;
  uint8_t wu_ia                : 1;
  uint8_t sleep_state          : 1;
  uint8_t ff_ia                : 1;
  uint8_t sleep_change_ia      : 1;
  uint8_t not_used0            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t sleep_change_ia      : 1;
  uint8_t ff_ia                : 1;
  uint8_t sleep_state          : 1;
  uint8_t wu_ia                : 1;
  uint8_t x_wu                 : 1;
  uint8_t y_wu                 : 1;
  uint8_t z_wu                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_wake_up_src_t;

#define LSM6DSV_TAP_SRC                     0x46U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_tap                : 1;
  uint8_t y_tap                : 1;
  uint8_t x_tap                : 1;
  uint8_t tap_sign             : 1;
  uint8_t double_tap           : 1;
  uint8_t single_tap           : 1;
  uint8_t tap_ia               : 1;
  uint8_t not_used0            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t tap_ia               : 1;
  uint8_t single_tap           : 1;
  uint8_t double_tap           : 1;
  uint8_t tap_sign             : 1;
  uint8_t x_tap                : 1;
  uint8_t y_tap                : 1;
  uint8_t z_tap                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_tap_src_t;

#define LSM6DSV_D6D_SRC                     0x47U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                   : 1;
  uint8_t xh                   : 1;
  uint8_t yl                   : 1;
  uint8_t yh                   : 1;
  uint8_t zl                   : 1;
  uint8_t zh                   : 1;
  uint8_t d6d_ia               : 1;
  uint8_t not_used0            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t d6d_ia               : 1;
  uint8_t zh                   : 1;
  uint8_t zl                   : 1;
  uint8_t yh                   : 1;
  uint8_t yl                   : 1;
  uint8_t xh                   : 1;
  uint8_t xl                   : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_d6d_src_t;

#define LSM6DSV_STATUS_MASTER_MAINPAGE      0x48U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sens_hub_endop       : 1;
  uint8_t not_used0            : 2;
  uint8_t slave0_nack          : 1;
  uint8_t slave1_nack          : 1;
  uint8_t slave2_nack          : 1;
  uint8_t slave3_nack          : 1;
  uint8_t wr_once_done         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wr_once_done         : 1;
  uint8_t slave3_nack          : 1;
  uint8_t slave2_nack          : 1;
  uint8_t slave1_nack          : 1;
  uint8_t slave0_nack          : 1;
  uint8_t not_used0            : 2;
  uint8_t sens_hub_endop       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_status_master_mainpage_t;

#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE    0x49U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t is_step_det          : 1;
  uint8_t is_tilt              : 1;
  uint8_t is_sigmot            : 1;
  uint8_t not_used1            : 1;
  uint8_t is_fsm_lc            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm_lc            : 1;
  uint8_t not_used1            : 1;
  uint8_t is_sigmot            : 1;
  uint8_t is_tilt              : 1;
  uint8_t is_step_det          : 1;
  uint8_t not_used0            : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_status_mainpage_t;

#define LSM6DSV_FSM_STATUS_MAINPAGE         0x4AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm1              : 1;
  uint8_t is_fsm2              : 1;
  uint8_t is_fsm3              : 1;
  uint8_t is_fsm4              : 1;
  uint8_t is_fsm5              : 1;
  uint8_t is_fsm6              : 1;
  uint8_t is_fsm7              : 1;
  uint8_t is_fsm8              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm8              : 1;
  uint8_t is_fsm7              : 1;
  uint8_t is_fsm6              : 1;
  uint8_t is_fsm5              : 1;
  uint8_t is_fsm4              : 1;
  uint8_t is_fsm3              : 1;
  uint8_t is_fsm2              : 1;
  uint8_t is_fsm1              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_status_mainpage_t;

#define LSM6DSV_INTERNAL_FREQ               0x4FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t freq_fine            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t freq_fine            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_internal_freq_t;

#define LSM6DSV_FUNCTIONS_ENABLE            0x50U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t inact_en             : 2;
  uint8_t not_used0            : 1;
  uint8_t dis_rst_lir_all_int  : 1;
  uint8_t not_used1            : 2;
  uint8_t timestamp_en         : 1;
  uint8_t interrupts_enable    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t interrupts_enable    : 1;
  uint8_t timestamp_en         : 1;
  uint8_t not_used1            : 2;
  uint8_t dis_rst_lir_all_int  : 1;
  uint8_t not_used0            : 1;
  uint8_t inact_en             : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_functions_enable_t;

#define LSM6DSV_DEN                         0x51U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t den_xl_g             : 1;
  uint8_t den_z                : 1;
  uint8_t den_y                : 1;
  uint8_t den_x                : 1;
  uint8_t den_xl_en            : 1;
  uint8_t lvl2_en              : 1;
  uint8_t lvl1_en              : 1;
  uint8_t not_used0            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t lvl1_en              : 1;
  uint8_t lvl2_en              : 1;
  uint8_t den_xl_en            : 1;
  uint8_t den_x                : 1;
  uint8_t den_y                : 1;
  uint8_t den_z                : 1;
  uint8_t den_xl_g             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_den_t;

#define LSM6DSV_INACTIVITY_DUR              0x54U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t inact_dur            : 2;
  uint8_t xl_inact_odr         : 2;
  uint8_t wu_inact_ths_w       : 3;
  uint8_t sleep_status_on_int  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sleep_status_on_int  : 1;
  uint8_t wu_inact_ths_w       : 3;
  uint8_t xl_inact_odr         : 2;
  uint8_t inact_dur            : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_inactivity_dur_t;

#define LSM6DSV_INACTIVITY_THS              0x55U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t inact_ths            : 6;
  uint8_t not_used0            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 2;
  uint8_t inact_ths            : 6;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_inactivity_ths_t;

#define LSM6DSV_TAP_CFG0                    0x56U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lir                  : 1;
  uint8_t tap_z_en             : 1;
  uint8_t tap_y_en             : 1;
  uint8_t tap_x_en             : 1;
  uint8_t slope_fds            : 1;
  uint8_t hw_func_mask_xl_settl  : 1;
  uint8_t low_pass_on_6d       : 1;
  uint8_t not_used1            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 1;
  uint8_t low_pass_on_6d       : 1;
  uint8_t hw_func_mask_xl_settl  : 1;
  uint8_t slope_fds            : 1;
  uint8_t tap_x_en             : 1;
  uint8_t tap_y_en             : 1;
  uint8_t tap_z_en             : 1;
  uint8_t lir                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_tap_cfg0_t;

#define LSM6DSV_TAP_CFG1                    0x57U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_x            : 5;
  uint8_t tap_priority         : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tap_priority         : 3;
  uint8_t tap_ths_x            : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_tap_cfg1_t;

#define LSM6DSV_TAP_CFG2                    0x58U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_y            : 5;
  uint8_t not_used0            : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t tap_ths_y            : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_tap_cfg2_t;

#define LSM6DSV_TAP_THS_6D                  0x59U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_z            : 5;
  uint8_t sixd_ths             : 2;
  uint8_t d4d_en               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t d4d_en               : 1;
  uint8_t sixd_ths             : 2;
  uint8_t tap_ths_z            : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_tap_ths_6d_t;

#define LSM6DSV_TAP_DUR                     0x5AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t shock                : 2;
  uint8_t quiet                : 2;
  uint8_t dur                  : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dur                  : 4;
  uint8_t quiet                : 2;
  uint8_t shock                : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_tap_dur_t;

#define LSM6DSV_WAKE_UP_THS                 0x5BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wk_ths               : 6;
  uint8_t usr_off_on_wu        : 1;
  uint8_t single_double_tap    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t single_double_tap    : 1;
  uint8_t usr_off_on_wu        : 1;
  uint8_t wk_ths               : 6;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_wake_up_ths_t;

#define LSM6DSV_WAKE_UP_DUR                 0x5CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur            : 4;
  uint8_t not_used0            : 1;
  uint8_t wake_dur             : 2;
  uint8_t ff_dur               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur               : 1;
  uint8_t wake_dur             : 2;
  uint8_t not_used0            : 1;
  uint8_t sleep_dur            : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_wake_up_dur_t;

#define LSM6DSV_FREE_FALL                   0x5DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ths               : 3;
  uint8_t ff_dur               : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur               : 5;
  uint8_t ff_ths               : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_free_fall_t;

#define LSM6DSV_MD1_CFG                    0x5EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_shub            : 1;
  uint8_t int1_emb_func        : 1;
  uint8_t int1_6d              : 1;
  uint8_t int1_double_tap      : 1;
  uint8_t int1_ff              : 1;
  uint8_t int1_wu              : 1;
  uint8_t int1_single_tap      : 1;
  uint8_t int1_sleep_change    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_sleep_change    : 1;
  uint8_t int1_single_tap      : 1;
  uint8_t int1_wu              : 1;
  uint8_t int1_ff              : 1;
  uint8_t int1_double_tap      : 1;
  uint8_t int1_6d              : 1;
  uint8_t int1_emb_func        : 1;
  uint8_t int1_shub            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_md1_cfg_t;

#define LSM6DSV_MD2_CFG                    0x5FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_timestamp       : 1;
  uint8_t int2_emb_func        : 1;
  uint8_t int2_6d              : 1;
  uint8_t int2_double_tap      : 1;
  uint8_t int2_ff              : 1;
  uint8_t int2_wu              : 1;
  uint8_t int2_single_tap      : 1;
  uint8_t int2_sleep_change    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_sleep_change    : 1;
  uint8_t int2_single_tap      : 1;
  uint8_t int2_wu              : 1;
  uint8_t int2_ff              : 1;
  uint8_t int2_double_tap      : 1;
  uint8_t int2_6d              : 1;
  uint8_t int2_emb_func        : 1;
  uint8_t int2_timestamp       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_md2_cfg_t;

#define LSM6DSV_HAODR_CFG                  0x62U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t haodr_sel            : 2;
  uint8_t not_used0            : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 6;
  uint8_t haodr_sel            : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_haodr_cfg_t;

#define LSM6DSV_EMB_FUNC_CFG               0x63U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t emb_func_disable     : 1;
  uint8_t emb_func_irq_mask_xl_settl : 1;
  uint8_t emb_func_irq_mask_g_settl  : 1;
  uint8_t not_used1            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 2;
  uint8_t emb_func_irq_mask_g_settl  : 1;
  uint8_t emb_func_irq_mask_xl_settl : 1;
  uint8_t emb_func_disable     : 1;
  uint8_t not_used0            : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_cfg_t;

#define LSM6DSV_UI_HANDSHAKE_CTRL          0x64U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_shared_req        : 1;
  uint8_t ui_shared_ack        : 1;
  uint8_t not_used0            : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 6;
  uint8_t ui_shared_ack        : 1;
  uint8_t ui_shared_req        : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_handshake_ctrl_t;

#define LSM6DSV_UI_SPI2_SHARED_0           0x65U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_spi2_shared_0_t;

#define LSM6DSV_UI_SPI2_SHARED_1           0x66U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_spi2_shared_1_t;

#define LSM6DSV_UI_SPI2_SHARED_2           0x67U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_spi2_shared_2_t;

#define LSM6DSV_UI_SPI2_SHARED_3           0x68U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_spi2_shared_3_t;

#define LSM6DSV_UI_SPI2_SHARED_4           0x69U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_spi2_shared_4_t;

#define LSM6DSV_UI_SPI2_SHARED_5           0x6AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ui_spi2_shared       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_spi2_shared_5_t;

#define LSM6DSV_CTRL_EIS                   0x6BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_g_eis             : 3;
  uint8_t g_eis_on_g_ois_out_reg : 1;
  uint8_t lpf_g_eis_bw         : 1;
  uint8_t not_used0            : 1;
  uint8_t odr_g_eis            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_g_eis            : 2;
  uint8_t not_used0            : 1;
  uint8_t lpf_g_eis_bw         : 1;
  uint8_t g_eis_on_g_ois_out_reg : 1;
  uint8_t fs_g_eis             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ctrl_eis_t;

#define LSM6DSV_UI_INT_OIS                 0x6FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 4;
  uint8_t st_ois_clampdis      : 1;
  uint8_t not_used1            : 1;
  uint8_t drdy_mask_ois        : 1;
  uint8_t int2_drdy_ois        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_drdy_ois        : 1;
  uint8_t drdy_mask_ois        : 1;
  uint8_t not_used1            : 1;
  uint8_t st_ois_clampdis      : 1;
  uint8_t not_used0            : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_int_ois_t;

#define LSM6DSV_UI_CTRL1_OIS               0x70U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_read_en         : 1;
  uint8_t ois_g_en             : 1;
  uint8_t ois_xl_en            : 1;
  uint8_t not_used0            : 2;
  uint8_t sim_ois              : 1;
  uint8_t not_used1            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 2;
  uint8_t sim_ois              : 1;
  uint8_t not_used0            : 2;
  uint8_t ois_xl_en            : 1;
  uint8_t ois_g_en             : 1;
  uint8_t spi2_read_en         : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_ctrl1_ois_t;

#define LSM6DSV_UI_CTRL2_OIS               0x71U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_g_ois             : 3;
  uint8_t lpf1_g_ois_bw        : 2;
  uint8_t not_used0            : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t lpf1_g_ois_bw        : 2;
  uint8_t fs_g_ois             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_ctrl2_ois_t;

#define LSM6DSV_UI_CTRL3_OIS               0x72U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_xl_ois            : 2;
  uint8_t not_used0            : 1;
  uint8_t lpf_xl_ois_bw        : 3;
  uint8_t not_used1            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 2;
  uint8_t lpf_xl_ois_bw        : 3;
  uint8_t not_used0            : 1;
  uint8_t fs_xl_ois            : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ui_ctrl3_ois_t;

#define LSM6DSV_X_OFS_USR                  0x73U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t x_ofs_usr            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t x_ofs_usr            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_x_ofs_usr_t;

#define LSM6DSV_Y_OFS_USR                  0x74U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t y_ofs_usr            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t y_ofs_usr            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_y_ofs_usr_t;

#define LSM6DSV_Z_OFS_USR                  0x75U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_ofs_usr            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t z_ofs_usr            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_z_ofs_usr_t;

#define LSM6DSV_FIFO_DATA_OUT_TAG          0x78U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t tag_cnt              : 2;
  uint8_t tag_sensor           : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tag_sensor           : 5;
  uint8_t tag_cnt              : 2;
  uint8_t not_used0            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_data_out_tag_t;

#define LSM6DSV_FIFO_DATA_OUT_X_L          0x79U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_data_out        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_data_out        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_data_out_x_l_t;

#define LSM6DSV_FIFO_DATA_OUT_X_H          0x7AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_data_out        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_data_out        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_data_out_x_h_t;

#define LSM6DSV_FIFO_DATA_OUT_Y_L          0x7BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_data_out        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_data_out        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_data_out_y_l_t;

#define LSM6DSV_FIFO_DATA_OUT_Y_H          0x7CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_data_out        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_data_out        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_data_out_y_h_t;

#define LSM6DSV_FIFO_DATA_OUT_Z_L          0x7DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_data_out        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_data_out        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_data_out_z_l_t;

#define LSM6DSV_FIFO_DATA_OUT_Z_H          0x7EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_data_out        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_data_out        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fifo_data_out_z_h_t;

/**
  * @}
  *
  */

/** @defgroup bitfields page spi2
  * @{
  *
  */

#define LSM6DSV_SPI2_WHO_AM_I              0x0FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t id                   : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t id                   : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_who_am_i_t;

#define LSM6DSV_SPI2_STATUS_REG_OIS        0x1EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                 : 1;
  uint8_t gda                  : 1;
  uint8_t gyro_settling        : 1;
  uint8_t not_used0            : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 5;
  uint8_t gyro_settling        : 1;
  uint8_t gda                  : 1;
  uint8_t xlda                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_status_reg_ois_t;

#define LSM6DSV_SPI2_OUT_TEMP_L            0x20U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t temp                 : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t temp                 : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_out_temp_l_t;

#define LSM6DSV_SPI2_OUT_TEMP_H            0x21U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t temp                 : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t temp                 : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_out_temp_h_t;

#define LSM6DSV_SPI2_OUTX_L_G_OIS          0x22U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outx_g_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outx_g_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outx_l_g_ois_t;

#define LSM6DSV_SPI2_OUTX_H_G_OIS          0x23U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outx_g_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outx_g_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outx_h_g_ois_t;

#define LSM6DSV_SPI2_OUTY_L_G_OIS          0x24U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outy_g_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outy_g_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outy_l_g_ois_t;

#define LSM6DSV_SPI2_OUTY_H_G_OIS          0x25U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outy_g_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outy_g_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outy_h_g_ois_t;

#define LSM6DSV_SPI2_OUTZ_L_G_OIS          0x26U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outz_g_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outz_g_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outz_l_g_ois_t;

#define LSM6DSV_SPI2_OUTZ_H_G_OIS          0x27U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outz_g_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outz_g_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outz_h_g_ois_t;

#define LSM6DSV_SPI2_OUTX_L_A_OIS          0x28U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outx_a_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outx_a_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outx_l_a_ois_t;

#define LSM6DSV_SPI2_OUTX_H_A_OIS          0x29U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outx_a_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outx_a_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outx_h_a_ois_t;

#define LSM6DSV_SPI2_OUTY_L_A_OIS          0x2AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outy_a_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outy_a_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outy_l_a_ois_t;

#define LSM6DSV_SPI2_OUTY_H_A_OIS          0x2BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outy_a_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outy_a_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outy_h_a_ois_t;

#define LSM6DSV_SPI2_OUTZ_L_A_OIS          0x2CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outz_a_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outz_a_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outz_l_a_ois_t;

#define LSM6DSV_SPI2_OUTZ_H_A_OIS          0x2DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_outz_a_ois      : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t spi2_outz_a_ois      : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_outz_h_a_ois_t;

#define LSM6DSV_SPI2_HANDSHAKE_CTRL        0x6EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_shared_ack      : 1;
  uint8_t spi2_shared_req      : 1;
  uint8_t not_used0            : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 6;
  uint8_t spi2_shared_req      : 1;
  uint8_t spi2_shared_ack      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_handshake_ctrl_t;

#define LSM6DSV_SPI2_INT_OIS               0x6FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_xl_ois            : 2;
  uint8_t st_g_ois             : 2;
  uint8_t st_ois_clampdis      : 1;
  uint8_t not_used0            : 1;
  uint8_t drdy_mask_ois        : 1;
  uint8_t int2_drdy_ois        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_drdy_ois        : 1;
  uint8_t drdy_mask_ois        : 1;
  uint8_t not_used0            : 1;
  uint8_t st_ois_clampdis      : 1;
  uint8_t st_g_ois             : 2;
  uint8_t st_xl_ois            : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_int_ois_t;

#define LSM6DSV_SPI2_CTRL1_OIS             0x70U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t spi2_read_en         : 1;
  uint8_t ois_g_en             : 1;
  uint8_t ois_xl_en            : 1;
  uint8_t not_used0            : 2;
  uint8_t sim_ois              : 1;
  uint8_t not_used1            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 2;
  uint8_t sim_ois              : 1;
  uint8_t not_used0            : 2;
  uint8_t ois_xl_en            : 1;
  uint8_t ois_g_en             : 1;
  uint8_t spi2_read_en         : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_ctrl1_ois_t;

#define LSM6DSV_SPI2_CTRL2_OIS             0x71U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_g_ois             : 3;
  uint8_t lpf1_g_ois_bw        : 2;
  uint8_t not_used0            : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t lpf1_g_ois_bw        : 2;
  uint8_t fs_g_ois             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_ctrl2_ois_t;

#define LSM6DSV_SPI2_CTRL3_OIS             0x72U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_xl_ois            : 2;
  uint8_t not_used0            : 1;
  uint8_t lpf_xl_ois_bw        : 3;
  uint8_t not_used1            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 2;
  uint8_t lpf_xl_ois_bw        : 3;
  uint8_t not_used0            : 1;
  uint8_t fs_xl_ois            : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_spi2_ctrl3_ois_t;

/**
  * @}
  *
  */

/** @defgroup bitfields page embedded
  * @{
  *
  */

#define LSM6DSV_PAGE_SEL                   0x2U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 4;
  uint8_t page_sel             : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t page_sel             : 4;
  uint8_t not_used0            : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_page_sel_t;

#define LSM6DSV_EMB_FUNC_EN_A              0x4U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t sflp_game_en         : 1;
  uint8_t not_used2            : 1;
  uint8_t pedo_en              : 1;
  uint8_t tilt_en              : 1;
  uint8_t sign_motion_en       : 1;
  uint8_t not_used1            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 2;
  uint8_t sign_motion_en       : 1;
  uint8_t tilt_en              : 1;
  uint8_t pedo_en              : 1;
  uint8_t not_used2            : 1;
  uint8_t sflp_game_en         : 1;
  uint8_t not_used0            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_en_a_t;

#define LSM6DSV_EMB_FUNC_EN_B              0x5U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_en               : 1;
  uint8_t not_used0            : 2;
  uint8_t fifo_compr_en        : 1;
  uint8_t not_used1            : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 4;
  uint8_t fifo_compr_en        : 1;
  uint8_t not_used0            : 2;
  uint8_t fsm_en               : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_en_b_t;

#define LSM6DSV_EMB_FUNC_EXEC_STATUS       0x7U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t emb_func_endop       : 1;
  uint8_t emb_func_exec_ovr    : 1;
  uint8_t not_used0            : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 6;
  uint8_t emb_func_exec_ovr    : 1;
  uint8_t emb_func_endop       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_exec_status_t;

#define LSM6DSV_PAGE_ADDRESS               0x8U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t page_addr            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t page_addr            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_page_address_t;

#define LSM6DSV_PAGE_VALUE                 0x9U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t page_value           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t page_value           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_page_value_t;

#define LSM6DSV_EMB_FUNC_INT1              0x0AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t int1_step_detector   : 1;
  uint8_t int1_tilt            : 1;
  uint8_t int1_sig_mot         : 1;
  uint8_t not_used1            : 1;
  uint8_t int1_fsm_lc          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_fsm_lc          : 1;
  uint8_t not_used1            : 1;
  uint8_t int1_sig_mot         : 1;
  uint8_t int1_tilt            : 1;
  uint8_t int1_step_detector   : 1;
  uint8_t not_used0            : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_int1_t;

#define LSM6DSV_FSM_INT1                   0x0BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_fsm1            : 1;
  uint8_t int1_fsm2            : 1;
  uint8_t int1_fsm3            : 1;
  uint8_t int1_fsm4            : 1;
  uint8_t int1_fsm5            : 1;
  uint8_t int1_fsm6            : 1;
  uint8_t int1_fsm7            : 1;
  uint8_t int1_fsm8            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_fsm8            : 1;
  uint8_t int1_fsm7            : 1;
  uint8_t int1_fsm6            : 1;
  uint8_t int1_fsm5            : 1;
  uint8_t int1_fsm4            : 1;
  uint8_t int1_fsm3            : 1;
  uint8_t int1_fsm2            : 1;
  uint8_t int1_fsm1            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_int1_t;

#define LSM6DSV_EMB_FUNC_INT2              0x0EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t int2_step_detector   : 1;
  uint8_t int2_tilt            : 1;
  uint8_t int2_sig_mot         : 1;
  uint8_t not_used1            : 1;
  uint8_t int2_fsm_lc          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_fsm_lc          : 1;
  uint8_t not_used1            : 1;
  uint8_t int2_sig_mot         : 1;
  uint8_t int2_tilt            : 1;
  uint8_t int2_step_detector   : 1;
  uint8_t not_used0            : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_int2_t;

#define LSM6DSV_FSM_INT2                   0x0FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_fsm1            : 1;
  uint8_t int2_fsm2            : 1;
  uint8_t int2_fsm3            : 1;
  uint8_t int2_fsm4            : 1;
  uint8_t int2_fsm5            : 1;
  uint8_t int2_fsm6            : 1;
  uint8_t int2_fsm7            : 1;
  uint8_t int2_fsm8            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_fsm8            : 1;
  uint8_t int2_fsm7            : 1;
  uint8_t int2_fsm6            : 1;
  uint8_t int2_fsm5            : 1;
  uint8_t int2_fsm4            : 1;
  uint8_t int2_fsm3            : 1;
  uint8_t int2_fsm2            : 1;
  uint8_t int2_fsm1            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_int2_t;

#define LSM6DSV_EMB_FUNC_STATUS            0x12U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t is_step_det          : 1;
  uint8_t is_tilt              : 1;
  uint8_t is_sigmot            : 1;
  uint8_t not_used1            : 1;
  uint8_t is_fsm_lc            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm_lc            : 1;
  uint8_t not_used1            : 1;
  uint8_t is_sigmot            : 1;
  uint8_t is_tilt              : 1;
  uint8_t is_step_det          : 1;
  uint8_t not_used0            : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_status_t;

#define LSM6DSV_FSM_STATUS                 0x13U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm1              : 1;
  uint8_t is_fsm2              : 1;
  uint8_t is_fsm3              : 1;
  uint8_t is_fsm4              : 1;
  uint8_t is_fsm5              : 1;
  uint8_t is_fsm6              : 1;
  uint8_t is_fsm7              : 1;
  uint8_t is_fsm8              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm8              : 1;
  uint8_t is_fsm7              : 1;
  uint8_t is_fsm6              : 1;
  uint8_t is_fsm5              : 1;
  uint8_t is_fsm4              : 1;
  uint8_t is_fsm3              : 1;
  uint8_t is_fsm2              : 1;
  uint8_t is_fsm1              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_status_t;

#define LSM6DSV_PAGE_RW                    0x17U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 5;
  uint8_t page_read            : 1;
  uint8_t page_write           : 1;
  uint8_t emb_func_lir         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t emb_func_lir         : 1;
  uint8_t page_write           : 1;
  uint8_t page_read            : 1;
  uint8_t not_used0            : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_page_rw_t;

#define LSM6DSV_EMB_FUNC_FIFO_EN_A         0x44U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t sflp_game_fifo_en    : 1;
  uint8_t not_used1            : 2;
  uint8_t sflp_gravity_fifo_en : 1;
  uint8_t sflp_gbias_fifo_en   : 1;
  uint8_t step_counter_fifo_en : 1;
  uint8_t not_used2            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used2            : 1;
  uint8_t step_counter_fifo_en : 1;
  uint8_t sflp_gbias_fifo_en   : 1;
  uint8_t sflp_gravity_fifo_en : 1;
  uint8_t not_used1            : 2;
  uint8_t sflp_game_fifo_en    : 1;
  uint8_t not_used0            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_fifo_en_a_t;

#define LSM6DSV_FSM_ENABLE                 0x46U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm1_en              : 1;
  uint8_t fsm2_en              : 1;
  uint8_t fsm3_en              : 1;
  uint8_t fsm4_en              : 1;
  uint8_t fsm5_en              : 1;
  uint8_t fsm6_en              : 1;
  uint8_t fsm7_en              : 1;
  uint8_t fsm8_en              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm8_en              : 1;
  uint8_t fsm7_en              : 1;
  uint8_t fsm6_en              : 1;
  uint8_t fsm5_en              : 1;
  uint8_t fsm4_en              : 1;
  uint8_t fsm3_en              : 1;
  uint8_t fsm2_en              : 1;
  uint8_t fsm1_en              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_enable_t;

#define LSM6DSV_FSM_LONG_COUNTER_L         0x48U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_lc               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_lc               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_long_counter_l_t;

#define LSM6DSV_FSM_LONG_COUNTER_H         0x49U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_lc               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_lc               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_long_counter_h_t;

#define LSM6DSV_INT_ACK_MASK               0x4BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t iack_mask            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t iack_mask            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_int_ack_mask_t;

#define LSM6DSV_FSM_OUTS1                  0x4CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm1_n_v             : 1;
  uint8_t fsm1_p_v             : 1;
  uint8_t fsm1_n_z             : 1;
  uint8_t fsm1_p_z             : 1;
  uint8_t fsm1_n_y             : 1;
  uint8_t fsm1_p_y             : 1;
  uint8_t fsm1_n_x             : 1;
  uint8_t fsm1_p_x             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm1_p_x             : 1;
  uint8_t fsm1_n_x             : 1;
  uint8_t fsm1_p_y             : 1;
  uint8_t fsm1_n_y             : 1;
  uint8_t fsm1_p_z             : 1;
  uint8_t fsm1_n_z             : 1;
  uint8_t fsm1_p_v             : 1;
  uint8_t fsm1_n_v             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_outs1_t;

#define LSM6DSV_FSM_OUTS2                  0x4DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm2_n_v             : 1;
  uint8_t fsm2_p_v             : 1;
  uint8_t fsm2_n_z             : 1;
  uint8_t fsm2_p_z             : 1;
  uint8_t fsm2_n_y             : 1;
  uint8_t fsm2_p_y             : 1;
  uint8_t fsm2_n_x             : 1;
  uint8_t fsm2_p_x             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm2_p_x             : 1;
  uint8_t fsm2_n_x             : 1;
  uint8_t fsm2_p_y             : 1;
  uint8_t fsm2_n_y             : 1;
  uint8_t fsm2_p_z             : 1;
  uint8_t fsm2_n_z             : 1;
  uint8_t fsm2_p_v             : 1;
  uint8_t fsm2_n_v             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_outs2_t;

#define LSM6DSV_FSM_OUTS3                  0x4EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm3_n_v             : 1;
  uint8_t fsm3_p_v             : 1;
  uint8_t fsm3_n_z             : 1;
  uint8_t fsm3_p_z             : 1;
  uint8_t fsm3_n_y             : 1;
  uint8_t fsm3_p_y             : 1;
  uint8_t fsm3_n_x             : 1;
  uint8_t fsm3_p_x             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm3_p_x             : 1;
  uint8_t fsm3_n_x             : 1;
  uint8_t fsm3_p_y             : 1;
  uint8_t fsm3_n_y             : 1;
  uint8_t fsm3_p_z             : 1;
  uint8_t fsm3_n_z             : 1;
  uint8_t fsm3_p_v             : 1;
  uint8_t fsm3_n_v             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_outs3_t;

#define LSM6DSV_FSM_OUTS4                  0x4FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm4_n_v             : 1;
  uint8_t fsm4_p_v             : 1;
  uint8_t fsm4_n_z             : 1;
  uint8_t fsm4_p_z             : 1;
  uint8_t fsm4_n_y             : 1;
  uint8_t fsm4_p_y             : 1;
  uint8_t fsm4_n_x             : 1;
  uint8_t fsm4_p_x             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm4_p_x             : 1;
  uint8_t fsm4_n_x             : 1;
  uint8_t fsm4_p_y             : 1;
  uint8_t fsm4_n_y             : 1;
  uint8_t fsm4_p_z             : 1;
  uint8_t fsm4_n_z             : 1;
  uint8_t fsm4_p_v             : 1;
  uint8_t fsm4_n_v             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_outs4_t;

#define LSM6DSV_FSM_OUTS5                  0x50U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm5_n_v             : 1;
  uint8_t fsm5_p_v             : 1;
  uint8_t fsm5_n_z             : 1;
  uint8_t fsm5_p_z             : 1;
  uint8_t fsm5_n_y             : 1;
  uint8_t fsm5_p_y             : 1;
  uint8_t fsm5_n_x             : 1;
  uint8_t fsm5_p_x             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm5_p_x             : 1;
  uint8_t fsm5_n_x             : 1;
  uint8_t fsm5_p_y             : 1;
  uint8_t fsm5_n_y             : 1;
  uint8_t fsm5_p_z             : 1;
  uint8_t fsm5_n_z             : 1;
  uint8_t fsm5_p_v             : 1;
  uint8_t fsm5_n_v             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_outs5_t;

#define LSM6DSV_FSM_OUTS6                  0x51U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm6_n_v             : 1;
  uint8_t fsm6_p_v             : 1;
  uint8_t fsm6_n_z             : 1;
  uint8_t fsm6_p_z             : 1;
  uint8_t fsm6_n_y             : 1;
  uint8_t fsm6_p_y             : 1;
  uint8_t fsm6_n_x             : 1;
  uint8_t fsm6_p_x             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm6_p_x             : 1;
  uint8_t fsm6_n_x             : 1;
  uint8_t fsm6_p_y             : 1;
  uint8_t fsm6_n_y             : 1;
  uint8_t fsm6_p_z             : 1;
  uint8_t fsm6_n_z             : 1;
  uint8_t fsm6_p_v             : 1;
  uint8_t fsm6_n_v             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_outs6_t;

#define LSM6DSV_FSM_OUTS7                  0x52U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm7_n_v             : 1;
  uint8_t fsm7_p_v             : 1;
  uint8_t fsm7_n_z             : 1;
  uint8_t fsm7_p_z             : 1;
  uint8_t fsm7_n_y             : 1;
  uint8_t fsm7_p_y             : 1;
  uint8_t fsm7_n_x             : 1;
  uint8_t fsm7_p_x             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm7_p_x             : 1;
  uint8_t fsm7_n_x             : 1;
  uint8_t fsm7_p_y             : 1;
  uint8_t fsm7_n_y             : 1;
  uint8_t fsm7_p_z             : 1;
  uint8_t fsm7_n_z             : 1;
  uint8_t fsm7_p_v             : 1;
  uint8_t fsm7_n_v             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_outs7_t;

#define LSM6DSV_FSM_OUTS8                  0x53U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm8_n_v             : 1;
  uint8_t fsm8_p_v             : 1;
  uint8_t fsm8_n_z             : 1;
  uint8_t fsm8_p_z             : 1;
  uint8_t fsm8_n_y             : 1;
  uint8_t fsm8_p_y             : 1;
  uint8_t fsm8_n_x             : 1;
  uint8_t fsm8_p_x             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm8_p_x             : 1;
  uint8_t fsm8_n_x             : 1;
  uint8_t fsm8_p_y             : 1;
  uint8_t fsm8_n_y             : 1;
  uint8_t fsm8_p_z             : 1;
  uint8_t fsm8_n_z             : 1;
  uint8_t fsm8_p_v             : 1;
  uint8_t fsm8_n_v             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_outs8_t;

#define LSM6DSV_SFLP_ODR                   0x5EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t sflp_game_odr        : 3;
  uint8_t not_used1            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 2;
  uint8_t sflp_game_odr        : 3;
  uint8_t not_used0            : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sflp_odr_t;

#define LSM6DSV_FSM_ODR                    0x5FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t fsm_odr              : 3;
  uint8_t not_used1            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 2;
  uint8_t fsm_odr              : 3;
  uint8_t not_used0            : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_odr_t;

#define LSM6DSV_STEP_COUNTER_L             0x62U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t step                 : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t step                 : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_step_counter_l_t;

#define LSM6DSV_STEP_COUNTER_H             0x63U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t step                 : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t step                 : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_step_counter_h_t;

#define LSM6DSV_EMB_FUNC_SRC               0x64U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 2;
  uint8_t stepcounter_bit_set  : 1;
  uint8_t step_overflow        : 1;
  uint8_t step_count_delta_ia  : 1;
  uint8_t step_detected        : 1;
  uint8_t not_used1            : 1;
  uint8_t pedo_rst_step        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t pedo_rst_step        : 1;
  uint8_t not_used1            : 1;
  uint8_t step_detected        : 1;
  uint8_t step_count_delta_ia  : 1;
  uint8_t step_overflow        : 1;
  uint8_t stepcounter_bit_set  : 1;
  uint8_t not_used0            : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_src_t;

#define LSM6DSV_EMB_FUNC_INIT_A            0x66U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 1;
  uint8_t sflp_game_init       : 1;
  uint8_t not_used2            : 1;
  uint8_t step_det_init        : 1;
  uint8_t tilt_init            : 1;
  uint8_t sig_mot_init         : 1;
  uint8_t not_used1            : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 2;
  uint8_t sig_mot_init         : 1;
  uint8_t tilt_init            : 1;
  uint8_t step_det_init        : 1;
  uint8_t not_used2            : 1;
  uint8_t sflp_game_init       : 1;
  uint8_t not_used0            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_init_a_t;

#define LSM6DSV_EMB_FUNC_INIT_B            0x67U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_init             : 1;
  uint8_t not_used0            : 2;
  uint8_t fifo_compr_init      : 1;
  uint8_t not_used1            : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 4;
  uint8_t fifo_compr_init      : 1;
  uint8_t not_used0            : 2;
  uint8_t fsm_init             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_emb_func_init_b_t;

/**
  * @}
  *
  */

/** @defgroup bitfields page pg0_emb_adv
  * @{
  *
  */
#define LSM6DSV_EMB_ADV_PG_0              0x000U

#define LSM6DSV_SFLP_GAME_GBIASX_L        0x6EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t gbiasx               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t gbiasx               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sflp_game_gbiasx_l_t;

#define LSM6DSV_SFLP_GAME_GBIASX_H        0x6FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t gbiasx               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t gbiasx               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sflp_game_gbiasx_h_t;

#define LSM6DSV_SFLP_GAME_GBIASY_L        0x70U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t gbiasy               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t gbiasy               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sflp_game_gbiasy_l_t;

#define LSM6DSV_SFLP_GAME_GBIASY_H        0x71U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t gbiasy               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t gbiasy               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sflp_game_gbiasy_h_t;

#define LSM6DSV_SFLP_GAME_GBIASZ_L        0x72U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t gbiasz               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t gbiasz               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sflp_game_gbiasz_l_t;

#define LSM6DSV_SFLP_GAME_GBIASZ_H        0x73U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t gbiasz               : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t gbiasz               : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sflp_game_gbiasz_h_t;

#define LSM6DSV_FSM_EXT_SENSITIVITY_L     0xBAU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_s            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_s            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_sensitivity_l_t;

#define LSM6DSV_FSM_EXT_SENSITIVITY_H     0xBBU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_s            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_s            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_sensitivity_h_t;

#define LSM6DSV_FSM_EXT_OFFX_L            0xC0U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_offx         : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_offx         : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_offx_l_t;

#define LSM6DSV_FSM_EXT_OFFX_H            0xC1U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_offx         : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_offx         : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_offx_h_t;

#define LSM6DSV_FSM_EXT_OFFY_L            0xC2U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_offy         : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_offy         : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_offy_l_t;

#define LSM6DSV_FSM_EXT_OFFY_H            0xC3U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_offy         : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_offy         : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_offy_h_t;

#define LSM6DSV_FSM_EXT_OFFZ_L            0xC4U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_offz         : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_offz         : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_offz_l_t;

#define LSM6DSV_FSM_EXT_OFFZ_H            0xC5U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_offz         : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_offz         : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_offz_h_t;

#define LSM6DSV_FSM_EXT_MATRIX_XX_L       0xC6U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_xx       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_xx       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_xx_l_t;

#define LSM6DSV_FSM_EXT_MATRIX_XX_H       0xC7U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_xx       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_xx       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_xx_h_t;

#define LSM6DSV_FSM_EXT_MATRIX_XY_L       0xC8U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_xy       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_xy       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_xy_l_t;

#define LSM6DSV_FSM_EXT_MATRIX_XY_H       0xC9U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_xy       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_xy       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_xy_h_t;

#define LSM6DSV_FSM_EXT_MATRIX_XZ_L       0xCAU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_xz       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_xz       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_xz_l_t;

#define LSM6DSV_FSM_EXT_MATRIX_XZ_H       0xCBU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_xz       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_xz       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_xz_h_t;

#define LSM6DSV_FSM_EXT_MATRIX_YY_L       0xCCU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_yy       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_yy       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_yy_l_t;

#define LSM6DSV_FSM_EXT_MATRIX_YY_H       0xCDU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_yy       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_yy       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_yy_h_t;

#define LSM6DSV_FSM_EXT_MATRIX_YZ_L       0xCEU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_yz       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_yz       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_yz_l_t;

#define LSM6DSV_FSM_EXT_MATRIX_YZ_H       0xCFU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_yz       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_yz       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_yz_h_t;

#define LSM6DSV_FSM_EXT_MATRIX_ZZ_L       0xD0U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_zz       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_zz       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_zz_l_t;

#define LSM6DSV_FSM_EXT_MATRIX_ZZ_H       0xD1U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_ext_mat_zz       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_ext_mat_zz       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_ext_matrix_zz_h_t;

#define LSM6DSV_EXT_CFG_A                 0xD4U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ext_z_axis           : 3;
  uint8_t not_used0            : 1;
  uint8_t ext_y_axis           : 3;
  uint8_t not_used1            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 1;
  uint8_t ext_y_axis           : 3;
  uint8_t not_used0            : 1;
  uint8_t ext_z_axis           : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ext_cfg_a_t;

#define LSM6DSV_EXT_CFG_B                 0xD5U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ext_x_axis           : 3;
  uint8_t not_used0            : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 5;
  uint8_t ext_x_axis           : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ext_cfg_b_t;

/**
  * @}
  *
  */

/** @defgroup bitfields page pg1_emb_adv
  * @{
  *
  */
#define LSM6DSV_EMB_ADV_PG_1             0x100U

#define LSM6DSV_FSM_LC_TIMEOUT_L         0x7AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_lc_timeout       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_lc_timeout       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_lc_timeout_l_t;

#define LSM6DSV_FSM_LC_TIMEOUT_H         0x7BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_lc_timeout       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_lc_timeout       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_lc_timeout_h_t;

#define LSM6DSV_FSM_PROGRAMS             0x7CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_n_prog           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_n_prog           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_programs_t;

#define LSM6DSV_FSM_START_ADD_L          0x7EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_start            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_start            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_start_add_l_t;

#define LSM6DSV_FSM_START_ADD_H          0x7FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_start            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm_start            : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_fsm_start_add_h_t;

#define LSM6DSV_PEDO_CMD_REG             0x83U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 3;
  uint8_t carry_count_en       : 1;
  uint8_t not_used1            : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 4;
  uint8_t carry_count_en       : 1;
  uint8_t not_used0            : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_pedo_cmd_reg_t;

#define LSM6DSV_PEDO_DEB_STEPS_CONF      0x84U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t deb_step             : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t deb_step             : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_pedo_deb_steps_conf_t;

#define LSM6DSV_PEDO_SC_DELTAT_L         0xD0U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pd_sc                : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t pd_sc                : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_pedo_sc_deltat_l_t;

#define LSM6DSV_PEDO_SC_DELTAT_H         0xD1U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pd_sc                : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t pd_sc                : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_pedo_sc_deltat_h_t;

/**
  * @}
  *
  */

/** @defgroup bitfields page pg2_emb_adv
  * @{
  *
  */
#define LSM6DSV_EMB_ADV_PG_2             0x200U

#define LSM6DSV_EXT_FORMAT               0x00
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0            : 2;
  uint8_t ext_format_sel       : 1;
  uint8_t not_used1            : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1            : 5;
  uint8_t ext_format_sel       : 1;
  uint8_t not_used0            : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ext_format_t;

#define LSM6DSV_EXT_3BYTE_SENSITIVITY_L  0x02U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ext_3byte_s          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ext_3byte_s          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ext_3byte_sensitivity_l_t;

#define LSM6DSV_EXT_3BYTE_SENSITIVITY_H  0x03U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ext_3byte_s          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ext_3byte_s          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ext_3byte_sensitivity_h_t;

#define LSM6DSV_EXT_3BYTE_OFFSET_XL      0x06U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ext_3byte_off        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ext_3byte_off        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ext_3byte_offset_xl_t;

#define LSM6DSV_EXT_3BYTE_OFFSET_L       0x07U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ext_3byte_off        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ext_3byte_off        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ext_3byte_offset_l_t;

#define LSM6DSV_EXT_3BYTE_OFFSET_H       0x08U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ext_3byte_off        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ext_3byte_off        : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_ext_3byte_offset_h_t;

/**
  * @}
  *
  */

/** @defgroup bitfields page sensor_hub
  * @{
  *
  */

#define LSM6DSV_SENSOR_HUB_1             0x2U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub1           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub1           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_1_t;

#define LSM6DSV_SENSOR_HUB_2             0x3U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub2           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub2           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_2_t;

#define LSM6DSV_SENSOR_HUB_3             0x4U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub3           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub3           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_3_t;

#define LSM6DSV_SENSOR_HUB_4             0x5U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub4           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub4           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_4_t;

#define LSM6DSV_SENSOR_HUB_5             0x6U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub5           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub5           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_5_t;

#define LSM6DSV_SENSOR_HUB_6             0x7U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub6           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub6           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_6_t;

#define LSM6DSV_SENSOR_HUB_7             0x8U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub7           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub7           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_7_t;

#define LSM6DSV_SENSOR_HUB_8             0x9U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub8           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub8           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_8_t;

#define LSM6DSV_SENSOR_HUB_9             0x0AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub9           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub9           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_9_t;

#define LSM6DSV_SENSOR_HUB_10            0x0BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub10          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub10          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_10_t;

#define LSM6DSV_SENSOR_HUB_11            0x0CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub11          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub11          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_11_t;

#define LSM6DSV_SENSOR_HUB_12            0x0DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub12          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub12          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_12_t;

#define LSM6DSV_SENSOR_HUB_13            0x0EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub13          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub13          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_13_t;

#define LSM6DSV_SENSOR_HUB_14            0x0FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub14          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub14          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_14_t;

#define LSM6DSV_SENSOR_HUB_15            0x10U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub15          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub15          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_15_t;

#define LSM6DSV_SENSOR_HUB_16            0x11U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub16          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub16          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_16_t;

#define LSM6DSV_SENSOR_HUB_17            0x12U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub17          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub17          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_17_t;

#define LSM6DSV_SENSOR_HUB_18            0x13U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub18          : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sensorhub18          : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_sensor_hub_18_t;

#define LSM6DSV_MASTER_CONFIG            0x14U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t aux_sens_on          : 2;
  uint8_t master_on            : 1;
  uint8_t not_used0            : 1;
  uint8_t pass_through_mode    : 1;
  uint8_t start_config         : 1;
  uint8_t write_once           : 1;
  uint8_t rst_master_regs      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t rst_master_regs      : 1;
  uint8_t write_once           : 1;
  uint8_t start_config         : 1;
  uint8_t pass_through_mode    : 1;
  uint8_t not_used0            : 1;
  uint8_t master_on            : 1;
  uint8_t aux_sens_on          : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_master_config_t;

#define LSM6DSV_SLV0_ADD                 0x15U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t rw_0                 : 1;
  uint8_t slave0_add           : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave0_add           : 7;
  uint8_t rw_0                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv0_add_t;

#define LSM6DSV_SLV0_SUBADD              0x16U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave0_reg           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave0_reg           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv0_subadd_t;

#define LSM6DSV_SLV0_CONFIG              0x17U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave0_numop         : 3;
  uint8_t batch_ext_sens_0_en  : 1;
  uint8_t not_used0            : 1;
  uint8_t shub_odr             : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t shub_odr             : 3;
  uint8_t not_used0            : 1;
  uint8_t batch_ext_sens_0_en  : 1;
  uint8_t slave0_numop         : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv0_config_t;

#define LSM6DSV_SLV1_ADD                 0x18U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_1                  : 1;
  uint8_t slave1_add           : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave1_add           : 7;
  uint8_t r_1                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv1_add_t;

#define LSM6DSV_SLV1_SUBADD              0x19U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave1_reg           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave1_reg           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv1_subadd_t;

#define LSM6DSV_SLV1_CONFIG              0x1AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave1_numop         : 3;
  uint8_t batch_ext_sens_1_en  : 1;
  uint8_t not_used0            : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 4;
  uint8_t batch_ext_sens_1_en  : 1;
  uint8_t slave1_numop         : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv1_config_t;

#define LSM6DSV_SLV2_ADD                 0x1BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_2                  : 1;
  uint8_t slave2_add           : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave2_add           : 7;
  uint8_t r_2                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv2_add_t;

#define LSM6DSV_SLV2_SUBADD              0x1CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave2_reg           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave2_reg           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv2_subadd_t;

#define LSM6DSV_SLV2_CONFIG              0x1DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave2_numop         : 3;
  uint8_t batch_ext_sens_2_en  : 1;
  uint8_t not_used0            : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 4;
  uint8_t batch_ext_sens_2_en  : 1;
  uint8_t slave2_numop         : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv2_config_t;

#define LSM6DSV_SLV3_ADD                 0x1EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_3                  : 1;
  uint8_t slave3_add           : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave3_add           : 7;
  uint8_t r_3                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv3_add_t;

#define LSM6DSV_SLV3_SUBADD              0x1FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave3_reg           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave3_reg           : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv3_subadd_t;

#define LSM6DSV_SLV3_CONFIG              0x20U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave3_numop         : 3;
  uint8_t batch_ext_sens_3_en  : 1;
  uint8_t not_used0            : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0            : 4;
  uint8_t batch_ext_sens_3_en  : 1;
  uint8_t slave3_numop         : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_slv3_config_t;

#define LSM6DSV_DATAWRITE_SLV0           0x21U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave0_dataw         : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave0_dataw         : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_datawrite_slv0_t;

#define LSM6DSV_STATUS_MASTER            0x22U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sens_hub_endop       : 1;
  uint8_t not_used0            : 2;
  uint8_t slave0_nack          : 1;
  uint8_t slave1_nack          : 1;
  uint8_t slave2_nack          : 1;
  uint8_t slave3_nack          : 1;
  uint8_t wr_once_done         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wr_once_done         : 1;
  uint8_t slave3_nack          : 1;
  uint8_t slave2_nack          : 1;
  uint8_t slave1_nack          : 1;
  uint8_t slave0_nack          : 1;
  uint8_t not_used0            : 2;
  uint8_t sens_hub_endop       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv_status_master_t;

/**
  * @}
  *
  */

/**
  * @defgroup LSM6DSV_Register_Union
  * @brief    This union group all the registers having a bit-field
  *           description.
  *           This union is useful but it's not needed by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union
{
  lsm6dsv_func_cfg_access_t          func_cfg_access;
  lsm6dsv_pin_ctrl_t                 pin_ctrl;
  lsm6dsv_if_cfg_t                   if_cfg;
  lsm6dsv_odr_trig_cfg_t             odr_trig_cfg;
  lsm6dsv_fifo_ctrl1_t               fifo_ctrl1;
  lsm6dsv_fifo_ctrl2_t               fifo_ctrl2;
  lsm6dsv_fifo_ctrl3_t               fifo_ctrl3;
  lsm6dsv_fifo_ctrl4_t               fifo_ctrl4;
  lsm6dsv_counter_bdr_reg1_t         counter_bdr_reg1;
  lsm6dsv_counter_bdr_reg2_t         counter_bdr_reg2;
  lsm6dsv_int1_ctrl_t                int1_ctrl;
  lsm6dsv_int2_ctrl_t                int2_ctrl;
  lsm6dsv_who_am_i_t                 who_am_i;
  lsm6dsv_ctrl1_t                    ctrl1;
  lsm6dsv_ctrl2_t                    ctrl2;
  lsm6dsv_ctrl3_t                    ctrl3;
  lsm6dsv_ctrl4_t                    ctrl4;
  lsm6dsv_ctrl5_t                    ctrl5;
  lsm6dsv_ctrl6_t                    ctrl6;
  lsm6dsv_ctrl7_t                    ctrl7;
  lsm6dsv_ctrl8_t                    ctrl8;
  lsm6dsv_ctrl9_t                    ctrl9;
  lsm6dsv_ctrl10_t                   ctrl10;
  lsm6dsv_ctrl_status_t              ctrl_status;
  lsm6dsv_fifo_status1_t             fifo_status1;
  lsm6dsv_fifo_status2_t             fifo_status2;
  lsm6dsv_all_int_src_t              all_int_src;
  lsm6dsv_status_reg_t               status_reg;
  lsm6dsv_out_temp_l_t               out_temp_l;
  lsm6dsv_out_temp_h_t               out_temp_h;
  lsm6dsv_outx_l_g_t                 outx_l_g;
  lsm6dsv_outx_h_g_t                 outx_h_g;
  lsm6dsv_outy_l_g_t                 outy_l_g;
  lsm6dsv_outy_h_g_t                 outy_h_g;
  lsm6dsv_outz_l_g_t                 outz_l_g;
  lsm6dsv_outz_h_g_t                 outz_h_g;
  lsm6dsv_outx_l_a_t                 outx_l_a;
  lsm6dsv_outx_h_a_t                 outx_h_a;
  lsm6dsv_outy_l_a_t                 outy_l_a;
  lsm6dsv_outy_h_a_t                 outy_h_a;
  lsm6dsv_outz_l_a_t                 outz_l_a;
  lsm6dsv_outz_h_a_t                 outz_h_a;
  lsm6dsv_ui_outx_l_g_ois_eis_t      ui_outx_l_g_ois_eis;
  lsm6dsv_ui_outx_h_g_ois_eis_t      ui_outx_h_g_ois_eis;
  lsm6dsv_ui_outy_l_g_ois_eis_t      ui_outy_l_g_ois_eis;
  lsm6dsv_ui_outy_h_g_ois_eis_t      ui_outy_h_g_ois_eis;
  lsm6dsv_ui_outz_l_g_ois_eis_t      ui_outz_l_g_ois_eis;
  lsm6dsv_ui_outz_h_g_ois_eis_t      ui_outz_h_g_ois_eis;
  lsm6dsv_ui_outx_l_a_ois_dualc_t    ui_outx_l_a_ois_dualc;
  lsm6dsv_ui_outx_h_a_ois_dualc_t    ui_outx_h_a_ois_dualc;
  lsm6dsv_ui_outy_l_a_ois_dualc_t    ui_outy_l_a_ois_dualc;
  lsm6dsv_ui_outy_h_a_ois_dualc_t    ui_outy_h_a_ois_dualc;
  lsm6dsv_ui_outz_l_a_ois_dualc_t    ui_outz_l_a_ois_dualc;
  lsm6dsv_ui_outz_h_a_ois_dualc_t    ui_outz_h_a_ois_dualc;
  lsm6dsv_timestamp0_t               timestamp0;
  lsm6dsv_timestamp1_t               timestamp1;
  lsm6dsv_timestamp2_t               timestamp2;
  lsm6dsv_timestamp3_t               timestamp3;
  lsm6dsv_ui_status_reg_ois_t        ui_status_reg_ois;
  lsm6dsv_wake_up_src_t              wake_up_src;
  lsm6dsv_tap_src_t                  tap_src;
  lsm6dsv_d6d_src_t                  d6d_src;
  lsm6dsv_status_master_mainpage_t   status_master_mainpage;
  lsm6dsv_emb_func_status_mainpage_t emb_func_status_mainpage;
  lsm6dsv_fsm_status_mainpage_t      fsm_status_mainpage;
  lsm6dsv_internal_freq_t            internal_freq;
  lsm6dsv_functions_enable_t         functions_enable;
  lsm6dsv_den_t                      den;
  lsm6dsv_inactivity_dur_t           inactivity_dur;
  lsm6dsv_inactivity_ths_t           inactivity_ths;
  lsm6dsv_tap_cfg0_t                 tap_cfg0;
  lsm6dsv_tap_cfg1_t                 tap_cfg1;
  lsm6dsv_tap_cfg2_t                 tap_cfg2;
  lsm6dsv_tap_ths_6d_t               tap_ths_6d;
  lsm6dsv_tap_dur_t                  tap_dur;
  lsm6dsv_wake_up_ths_t              wake_up_ths;
  lsm6dsv_wake_up_dur_t              wake_up_dur;
  lsm6dsv_free_fall_t                free_fall;
  lsm6dsv_md1_cfg_t                  md1_cfg;
  lsm6dsv_md2_cfg_t                  md2_cfg;
  lsm6dsv_emb_func_cfg_t             emb_func_cfg;
  lsm6dsv_ui_handshake_ctrl_t        ui_handshake_ctrl;
  lsm6dsv_ui_spi2_shared_0_t         ui_spi2_shared_0;
  lsm6dsv_ui_spi2_shared_1_t         ui_spi2_shared_1;
  lsm6dsv_ui_spi2_shared_2_t         ui_spi2_shared_2;
  lsm6dsv_ui_spi2_shared_3_t         ui_spi2_shared_3;
  lsm6dsv_ui_spi2_shared_4_t         ui_spi2_shared_4;
  lsm6dsv_ui_spi2_shared_5_t         ui_spi2_shared_5;
  lsm6dsv_ctrl_eis_t                 ctrl_eis;
  lsm6dsv_ui_int_ois_t               ui_int_ois;
  lsm6dsv_ui_ctrl1_ois_t             ui_ctrl1_ois;
  lsm6dsv_ui_ctrl2_ois_t             ui_ctrl2_ois;
  lsm6dsv_ui_ctrl3_ois_t             ui_ctrl3_ois;
  lsm6dsv_x_ofs_usr_t                x_ofs_usr;
  lsm6dsv_y_ofs_usr_t                y_ofs_usr;
  lsm6dsv_z_ofs_usr_t                z_ofs_usr;
  lsm6dsv_fifo_data_out_tag_t        fifo_data_out_tag;
  lsm6dsv_fifo_data_out_x_l_t        fifo_data_out_x_l;
  lsm6dsv_fifo_data_out_x_h_t        fifo_data_out_x_h;
  lsm6dsv_fifo_data_out_y_l_t        fifo_data_out_y_l;
  lsm6dsv_fifo_data_out_y_h_t        fifo_data_out_y_h;
  lsm6dsv_fifo_data_out_z_l_t        fifo_data_out_z_l;
  lsm6dsv_fifo_data_out_z_h_t        fifo_data_out_z_h;
  lsm6dsv_spi2_who_am_i_t            spi2_who_am_i;
  lsm6dsv_spi2_status_reg_ois_t      spi2_status_reg_ois;
  lsm6dsv_spi2_out_temp_l_t          spi2_out_temp_l;
  lsm6dsv_spi2_out_temp_h_t          spi2_out_temp_h;
  lsm6dsv_spi2_outx_l_g_ois_t        spi2_outx_l_g_ois;
  lsm6dsv_spi2_outx_h_g_ois_t        spi2_outx_h_g_ois;
  lsm6dsv_spi2_outy_l_g_ois_t        spi2_outy_l_g_ois;
  lsm6dsv_spi2_outy_h_g_ois_t        spi2_outy_h_g_ois;
  lsm6dsv_spi2_outz_l_g_ois_t        spi2_outz_l_g_ois;
  lsm6dsv_spi2_outz_h_g_ois_t        spi2_outz_h_g_ois;
  lsm6dsv_spi2_outx_l_a_ois_t        spi2_outx_l_a_ois;
  lsm6dsv_spi2_outx_h_a_ois_t        spi2_outx_h_a_ois;
  lsm6dsv_spi2_outy_l_a_ois_t        spi2_outy_l_a_ois;
  lsm6dsv_spi2_outy_h_a_ois_t        spi2_outy_h_a_ois;
  lsm6dsv_spi2_outz_l_a_ois_t        spi2_outz_l_a_ois;
  lsm6dsv_spi2_outz_h_a_ois_t        spi2_outz_h_a_ois;
  lsm6dsv_spi2_handshake_ctrl_t      spi2_handshake_ctrl;
  lsm6dsv_spi2_int_ois_t             spi2_int_ois;
  lsm6dsv_spi2_ctrl1_ois_t           spi2_ctrl1_ois;
  lsm6dsv_spi2_ctrl2_ois_t           spi2_ctrl2_ois;
  lsm6dsv_spi2_ctrl3_ois_t           spi2_ctrl3_ois;
  lsm6dsv_page_sel_t                 page_sel;
  lsm6dsv_emb_func_en_a_t            emb_func_en_a;
  lsm6dsv_emb_func_en_b_t            emb_func_en_b;
  lsm6dsv_emb_func_exec_status_t     emb_func_exec_status;
  lsm6dsv_page_address_t             page_address;
  lsm6dsv_page_value_t               page_value;
  lsm6dsv_emb_func_int1_t            emb_func_int1;
  lsm6dsv_fsm_int1_t                 fsm_int1;
  lsm6dsv_emb_func_int2_t            emb_func_int2;
  lsm6dsv_fsm_int2_t                 fsm_int2;
  lsm6dsv_emb_func_status_t          emb_func_status;
  lsm6dsv_fsm_status_t               fsm_status;
  lsm6dsv_page_rw_t                  page_rw;
  lsm6dsv_emb_func_fifo_en_a_t       emb_func_fifo_en_a;
  lsm6dsv_fsm_enable_t               fsm_enable;
  lsm6dsv_fsm_long_counter_l_t       fsm_long_counter_l;
  lsm6dsv_fsm_long_counter_h_t       fsm_long_counter_h;
  lsm6dsv_int_ack_mask_t             int_ack_mask;
  lsm6dsv_fsm_outs1_t                fsm_outs1;
  lsm6dsv_fsm_outs2_t                fsm_outs2;
  lsm6dsv_fsm_outs3_t                fsm_outs3;
  lsm6dsv_fsm_outs4_t                fsm_outs4;
  lsm6dsv_fsm_outs5_t                fsm_outs5;
  lsm6dsv_fsm_outs6_t                fsm_outs6;
  lsm6dsv_fsm_outs7_t                fsm_outs7;
  lsm6dsv_fsm_outs8_t                fsm_outs8;
  lsm6dsv_fsm_odr_t                  fsm_odr;
  lsm6dsv_step_counter_l_t           step_counter_l;
  lsm6dsv_step_counter_h_t           step_counter_h;
  lsm6dsv_emb_func_src_t             emb_func_src;
  lsm6dsv_emb_func_init_a_t          emb_func_init_a;
  lsm6dsv_emb_func_init_b_t          emb_func_init_b;
  lsm6dsv_fsm_ext_sensitivity_l_t    fsm_ext_sensitivity_l;
  lsm6dsv_fsm_ext_sensitivity_h_t    fsm_ext_sensitivity_h;
  lsm6dsv_fsm_ext_offx_l_t           fsm_ext_offx_l;
  lsm6dsv_fsm_ext_offx_h_t           fsm_ext_offx_h;
  lsm6dsv_fsm_ext_offy_l_t           fsm_ext_offy_l;
  lsm6dsv_fsm_ext_offy_h_t           fsm_ext_offy_h;
  lsm6dsv_fsm_ext_offz_l_t           fsm_ext_offz_l;
  lsm6dsv_fsm_ext_offz_h_t           fsm_ext_offz_h;
  lsm6dsv_fsm_ext_matrix_xx_l_t      fsm_ext_matrix_xx_l;
  lsm6dsv_fsm_ext_matrix_xx_h_t      fsm_ext_matrix_xx_h;
  lsm6dsv_fsm_ext_matrix_xy_l_t      fsm_ext_matrix_xy_l;
  lsm6dsv_fsm_ext_matrix_xy_h_t      fsm_ext_matrix_xy_h;
  lsm6dsv_fsm_ext_matrix_xz_l_t      fsm_ext_matrix_xz_l;
  lsm6dsv_fsm_ext_matrix_xz_h_t      fsm_ext_matrix_xz_h;
  lsm6dsv_fsm_ext_matrix_yy_l_t      fsm_ext_matrix_yy_l;
  lsm6dsv_fsm_ext_matrix_yy_h_t      fsm_ext_matrix_yy_h;
  lsm6dsv_fsm_ext_matrix_yz_l_t      fsm_ext_matrix_yz_l;
  lsm6dsv_fsm_ext_matrix_yz_h_t      fsm_ext_matrix_yz_h;
  lsm6dsv_fsm_ext_matrix_zz_l_t      fsm_ext_matrix_zz_l;
  lsm6dsv_fsm_ext_matrix_zz_h_t      fsm_ext_matrix_zz_h;
  lsm6dsv_ext_cfg_a_t                ext_cfg_a;
  lsm6dsv_ext_cfg_b_t                ext_cfg_b;
  lsm6dsv_fsm_lc_timeout_l_t         fsm_lc_timeout_l;
  lsm6dsv_fsm_lc_timeout_h_t         fsm_lc_timeout_h;
  lsm6dsv_fsm_programs_t             fsm_programs;
  lsm6dsv_fsm_start_add_l_t          fsm_start_add_l;
  lsm6dsv_fsm_start_add_h_t          fsm_start_add_h;
  lsm6dsv_pedo_cmd_reg_t             pedo_cmd_reg;
  lsm6dsv_pedo_deb_steps_conf_t      pedo_deb_steps_conf;
  lsm6dsv_pedo_sc_deltat_l_t         pedo_sc_deltat_l;
  lsm6dsv_pedo_sc_deltat_h_t         pedo_sc_deltat_h;
  lsm6dsv_sensor_hub_1_t             sensor_hub_1;
  lsm6dsv_sensor_hub_2_t             sensor_hub_2;
  lsm6dsv_sensor_hub_3_t             sensor_hub_3;
  lsm6dsv_sensor_hub_4_t             sensor_hub_4;
  lsm6dsv_sensor_hub_5_t             sensor_hub_5;
  lsm6dsv_sensor_hub_6_t             sensor_hub_6;
  lsm6dsv_sensor_hub_7_t             sensor_hub_7;
  lsm6dsv_sensor_hub_8_t             sensor_hub_8;
  lsm6dsv_sensor_hub_9_t             sensor_hub_9;
  lsm6dsv_sensor_hub_10_t            sensor_hub_10;
  lsm6dsv_sensor_hub_11_t            sensor_hub_11;
  lsm6dsv_sensor_hub_12_t            sensor_hub_12;
  lsm6dsv_sensor_hub_13_t            sensor_hub_13;
  lsm6dsv_sensor_hub_14_t            sensor_hub_14;
  lsm6dsv_sensor_hub_15_t            sensor_hub_15;
  lsm6dsv_sensor_hub_16_t            sensor_hub_16;
  lsm6dsv_sensor_hub_17_t            sensor_hub_17;
  lsm6dsv_sensor_hub_18_t            sensor_hub_18;
  lsm6dsv_master_config_t            master_config;
  lsm6dsv_slv0_add_t                 slv0_add;
  lsm6dsv_slv0_subadd_t              slv0_subadd;
  lsm6dsv_slv0_config_t              slv0_config;
  lsm6dsv_slv1_add_t                 slv1_add;
  lsm6dsv_slv1_subadd_t              slv1_subadd;
  lsm6dsv_slv1_config_t              slv1_config;
  lsm6dsv_slv2_add_t                 slv2_add;
  lsm6dsv_slv2_subadd_t              slv2_subadd;
  lsm6dsv_slv2_config_t              slv2_config;
  lsm6dsv_slv3_add_t                 slv3_add;
  lsm6dsv_slv3_subadd_t              slv3_subadd;
  lsm6dsv_slv3_config_t              slv3_config;
  lsm6dsv_datawrite_slv0_t           datawrite_slv0;
  lsm6dsv_status_master_t            status_master;
  bitwise_t                             bitwise;
  uint8_t                               byte;
} lsm6dsv_reg_t;

/**
  * @}
  *
  */

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

/*
 * These are the basic platform dependent I/O routines to read
 * and write device registers connected on a standard bus.
 * The driver keeps offering a default implementation based on function
 * pointers to read/write routines for backward compatibility.
 * The __weak directive allows the final application to overwrite
 * them with a custom implementation.
 */

int32_t lsm6dsv_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                         uint8_t *data,
                         uint16_t len);
int32_t lsm6dsv_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len);

float_t lsm6dsv_from_sflp_to_mg(int16_t lsb);
float_t lsm6dsv_from_fs2_to_mg(int16_t lsb);
float_t lsm6dsv_from_fs4_to_mg(int16_t lsb);
float_t lsm6dsv_from_fs8_to_mg(int16_t lsb);
float_t lsm6dsv_from_fs16_to_mg(int16_t lsb);

float_t lsm6dsv_from_fs125_to_mdps(int16_t lsb);
float_t lsm6dsv_from_fs500_to_mdps(int16_t lsb);
float_t lsm6dsv_from_fs250_to_mdps(int16_t lsb);
float_t lsm6dsv_from_fs1000_to_mdps(int16_t lsb);
float_t lsm6dsv_from_fs2000_to_mdps(int16_t lsb);
float_t lsm6dsv_from_fs4000_to_mdps(int16_t lsb);

float_t lsm6dsv_from_lsb_to_celsius(int16_t lsb);

float_t lsm6dsv_from_lsb_to_nsec(uint32_t lsb);

int32_t lsm6dsv_xl_offset_on_out_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_xl_offset_on_out_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  float_t z_mg;
  float_t y_mg;
  float_t x_mg;
} lsm6dsv_xl_offset_mg_t;
int32_t lsm6dsv_xl_offset_mg_set(stmdev_ctx_t *ctx,
                                 lsm6dsv_xl_offset_mg_t val);
int32_t lsm6dsv_xl_offset_mg_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_xl_offset_mg_t *val);

typedef enum
{
  LSM6DSV_READY             = 0x0,
  LSM6DSV_GLOBAL_RST        = 0x1,
  LSM6DSV_RESTORE_CAL_PARAM = 0x2,
  LSM6DSV_RESTORE_CTRL_REGS = 0x4,
} lsm6dsv_reset_t;
int32_t lsm6dsv_reset_set(stmdev_ctx_t *ctx, lsm6dsv_reset_t val);
int32_t lsm6dsv_reset_get(stmdev_ctx_t *ctx, lsm6dsv_reset_t *val);

typedef enum
{
  LSM6DSV_MAIN_MEM_BANK       = 0x0,
  LSM6DSV_EMBED_FUNC_MEM_BANK = 0x1,
  LSM6DSV_SENSOR_HUB_MEM_BANK = 0x2,
} lsm6dsv_mem_bank_t;
int32_t lsm6dsv_mem_bank_set(stmdev_ctx_t *ctx, lsm6dsv_mem_bank_t val);
int32_t lsm6dsv_mem_bank_get(stmdev_ctx_t *ctx, lsm6dsv_mem_bank_t *val);

int32_t lsm6dsv_device_id_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_ODR_OFF              = 0x0,
  LSM6DSV_ODR_AT_1Hz875        = 0x1,
  LSM6DSV_ODR_AT_7Hz5          = 0x2,
  LSM6DSV_ODR_AT_15Hz          = 0x3,
  LSM6DSV_ODR_AT_30Hz          = 0x4,
  LSM6DSV_ODR_AT_60Hz          = 0x5,
  LSM6DSV_ODR_AT_120Hz         = 0x6,
  LSM6DSV_ODR_AT_240Hz         = 0x7,
  LSM6DSV_ODR_AT_480Hz         = 0x8,
  LSM6DSV_ODR_AT_960Hz         = 0x9,
  LSM6DSV_ODR_AT_1920Hz        = 0xA,
  LSM6DSV_ODR_AT_3840Hz        = 0xB,
  LSM6DSV_ODR_AT_7680Hz        = 0xC,
  LSM6DSV_ODR_HA01_AT_15Hz625  = 0x13,
  LSM6DSV_ODR_HA01_AT_31Hz25   = 0x14,
  LSM6DSV_ODR_HA01_AT_62Hz5    = 0x15,
  LSM6DSV_ODR_HA01_AT_125Hz    = 0x16,
  LSM6DSV_ODR_HA01_AT_250Hz    = 0x17,
  LSM6DSV_ODR_HA01_AT_500Hz    = 0x18,
  LSM6DSV_ODR_HA01_AT_1000Hz   = 0x19,
  LSM6DSV_ODR_HA01_AT_2000Hz   = 0x1A,
  LSM6DSV_ODR_HA01_AT_4000Hz   = 0x1B,
  LSM6DSV_ODR_HA01_AT_8000Hz   = 0x1C,
  LSM6DSV_ODR_HA02_AT_12Hz5    = 0x23,
  LSM6DSV_ODR_HA02_AT_25Hz     = 0x24,
  LSM6DSV_ODR_HA02_AT_50Hz     = 0x25,
  LSM6DSV_ODR_HA02_AT_100Hz    = 0x26,
  LSM6DSV_ODR_HA02_AT_200Hz    = 0x27,
  LSM6DSV_ODR_HA02_AT_400Hz    = 0x28,
  LSM6DSV_ODR_HA02_AT_800Hz    = 0x29,
  LSM6DSV_ODR_HA02_AT_1600Hz   = 0x2A,
  LSM6DSV_ODR_HA02_AT_3200Hz   = 0x2B,
  LSM6DSV_ODR_HA02_AT_6400Hz   = 0x2C,
} lsm6dsv_data_rate_t;
int32_t lsm6dsv_xl_data_rate_set(stmdev_ctx_t *ctx,
                                 lsm6dsv_data_rate_t val);
int32_t lsm6dsv_xl_data_rate_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_data_rate_t *val);
int32_t lsm6dsv_gy_data_rate_set(stmdev_ctx_t *ctx,
                                 lsm6dsv_data_rate_t val);
int32_t lsm6dsv_gy_data_rate_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_data_rate_t *val);


typedef enum
{
  LSM6DSV_XL_HIGH_PERFORMANCE_MD   = 0x0,
  LSM6DSV_XL_HIGH_ACCURACY_ODR_MD = 0x1,
  LSM6DSV_XL_ODR_TRIGGERED_MD      = 0x3,
  LSM6DSV_XL_LOW_POWER_2_AVG_MD    = 0x4,
  LSM6DSV_XL_LOW_POWER_4_AVG_MD    = 0x5,
  LSM6DSV_XL_LOW_POWER_8_AVG_MD    = 0x6,
  LSM6DSV_XL_NORMAL_MD             = 0x7,
} lsm6dsv_xl_mode_t;
int32_t lsm6dsv_xl_mode_set(stmdev_ctx_t *ctx, lsm6dsv_xl_mode_t val);
int32_t lsm6dsv_xl_mode_get(stmdev_ctx_t *ctx, lsm6dsv_xl_mode_t *val);

typedef enum
{
  LSM6DSV_GY_HIGH_PERFORMANCE_MD   = 0x0,
  LSM6DSV_GY_HIGH_ACCURACY_ODR_MD = 0x1,
  LSM6DSV_GY_SLEEP_MD              = 0x4,
  LSM6DSV_GY_LOW_POWER_MD          = 0x5,
} lsm6dsv_gy_mode_t;
int32_t lsm6dsv_gy_mode_set(stmdev_ctx_t *ctx, lsm6dsv_gy_mode_t val);
int32_t lsm6dsv_gy_mode_get(stmdev_ctx_t *ctx, lsm6dsv_gy_mode_t *val);

int32_t lsm6dsv_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_odr_trig_cfg_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_odr_trig_cfg_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_DRDY_LATCHED = 0x0,
  LSM6DSV_DRDY_PULSED  = 0x1,
} lsm6dsv_data_ready_mode_t;
int32_t lsm6dsv_data_ready_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsv_data_ready_mode_t val);
int32_t lsm6dsv_data_ready_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsv_data_ready_mode_t *val);

typedef struct
{
  uint8_t enable               : 1; /* interrupt enable */
  uint8_t lir                  : 1; /* interrupt pulsed or latched */
} lsm6dsv_interrupt_mode_t;
int32_t lsm6dsv_interrupt_enable_set(stmdev_ctx_t *ctx,
                                     lsm6dsv_interrupt_mode_t val);
int32_t lsm6dsv_interrupt_enable_get(stmdev_ctx_t *ctx,
                                     lsm6dsv_interrupt_mode_t *val);

typedef enum
{
  LSM6DSV_125dps  = 0x0,
  LSM6DSV_250dps  = 0x1,
  LSM6DSV_500dps  = 0x2,
  LSM6DSV_1000dps = 0x3,
  LSM6DSV_2000dps = 0x4,
  LSM6DSV_4000dps = 0xc,
} lsm6dsv_gy_full_scale_t;
int32_t lsm6dsv_gy_full_scale_set(stmdev_ctx_t *ctx,
                                  lsm6dsv_gy_full_scale_t val);
int32_t lsm6dsv_gy_full_scale_get(stmdev_ctx_t *ctx,
                                  lsm6dsv_gy_full_scale_t *val);

typedef enum
{
  LSM6DSV_2g  = 0x0,
  LSM6DSV_4g  = 0x1,
  LSM6DSV_8g  = 0x2,
  LSM6DSV_16g = 0x3,
} lsm6dsv_xl_full_scale_t;
int32_t lsm6dsv_xl_full_scale_set(stmdev_ctx_t *ctx,
                                  lsm6dsv_xl_full_scale_t val);
int32_t lsm6dsv_xl_full_scale_get(stmdev_ctx_t *ctx,
                                  lsm6dsv_xl_full_scale_t *val);

int32_t lsm6dsv_xl_dual_channel_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_xl_dual_channel_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_XL_ST_DISABLE  = 0x0,
  LSM6DSV_XL_ST_POSITIVE = 0x1,
  LSM6DSV_XL_ST_NEGATIVE = 0x2,
} lsm6dsv_xl_self_test_t;
int32_t lsm6dsv_xl_self_test_set(stmdev_ctx_t *ctx,
                                 lsm6dsv_xl_self_test_t val);
int32_t lsm6dsv_xl_self_test_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_xl_self_test_t *val);

typedef enum
{
  LSM6DSV_OIS_XL_ST_DISABLE  = 0x0,
  LSM6DSV_OIS_XL_ST_POSITIVE = 0x1,
  LSM6DSV_OIS_XL_ST_NEGATIVE = 0x2,
} lsm6dsv_ois_xl_self_test_t;
int32_t lsm6dsv_ois_xl_self_test_set(stmdev_ctx_t *ctx,
                                     lsm6dsv_ois_xl_self_test_t val);
int32_t lsm6dsv_ois_xl_self_test_get(stmdev_ctx_t *ctx,
                                     lsm6dsv_ois_xl_self_test_t *val);

typedef enum
{
  LSM6DSV_GY_ST_DISABLE  = 0x0,
  LSM6DSV_GY_ST_POSITIVE = 0x1,
  LSM6DSV_GY_ST_NEGATIVE = 0x2,

} lsm6dsv_gy_self_test_t;
int32_t lsm6dsv_gy_self_test_set(stmdev_ctx_t *ctx,
                                 lsm6dsv_gy_self_test_t val);
int32_t lsm6dsv_gy_self_test_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_gy_self_test_t *val);

typedef enum
{
  LSM6DSV_OIS_GY_ST_DISABLE   = 0x0,
  LSM6DSV_OIS_GY_ST_POSITIVE  = 0x1,
  LSM6DSV_OIS_GY_ST_NEGATIVE  = 0x2,
  LSM6DSV_OIS_GY_ST_CLAMP_POS = 0x5,
  LSM6DSV_OIS_GY_ST_CLAMP_NEG = 0x6,

} lsm6dsv_ois_gy_self_test_t;
int32_t lsm6dsv_ois_gy_self_test_set(stmdev_ctx_t *ctx,
                                     lsm6dsv_ois_gy_self_test_t val);
int32_t lsm6dsv_ois_gy_self_test_get(stmdev_ctx_t *ctx,
                                     lsm6dsv_ois_gy_self_test_t *val);

typedef struct
{
  uint8_t drdy_xl              : 1;
  uint8_t drdy_gy              : 1;
  uint8_t drdy_temp            : 1;
  uint8_t drdy_eis             : 1;
  uint8_t drdy_ois             : 1;
  uint8_t gy_settling          : 1;
  uint8_t timestamp            : 1;
  uint8_t free_fall            : 1;
  uint8_t wake_up              : 1;
  uint8_t wake_up_z            : 1;
  uint8_t wake_up_y            : 1;
  uint8_t wake_up_x            : 1;
  uint8_t single_tap           : 1;
  uint8_t double_tap           : 1;
  uint8_t tap_z                : 1;
  uint8_t tap_y                : 1;
  uint8_t tap_x                : 1;
  uint8_t tap_sign             : 1;
  uint8_t six_d                : 1;
  uint8_t six_d_xl             : 1;
  uint8_t six_d_xh             : 1;
  uint8_t six_d_yl             : 1;
  uint8_t six_d_yh             : 1;
  uint8_t six_d_zl             : 1;
  uint8_t six_d_zh             : 1;
  uint8_t sleep_change         : 1;
  uint8_t sleep_state          : 1;
  uint8_t step_detector        : 1;
  uint8_t step_count_inc       : 1;
  uint8_t step_count_overflow  : 1;
  uint8_t step_on_delta_time   : 1;
  uint8_t emb_func_stand_by    : 1;
  uint8_t emb_func_time_exceed : 1;
  uint8_t tilt                 : 1;
  uint8_t sig_mot              : 1;
  uint8_t fsm_lc               : 1;
  uint8_t fsm1                 : 1;
  uint8_t fsm2                 : 1;
  uint8_t fsm3                 : 1;
  uint8_t fsm4                 : 1;
  uint8_t fsm5                 : 1;
  uint8_t fsm6                 : 1;
  uint8_t fsm7                 : 1;
  uint8_t fsm8                 : 1;
  uint8_t sh_endop             : 1;
  uint8_t sh_slave0_nack       : 1;
  uint8_t sh_slave1_nack       : 1;
  uint8_t sh_slave2_nack       : 1;
  uint8_t sh_slave3_nack       : 1;
  uint8_t sh_wr_once           : 1;
  uint8_t fifo_bdr             : 1;
  uint8_t fifo_full            : 1;
  uint8_t fifo_ovr             : 1;
  uint8_t fifo_th              : 1;
} lsm6dsv_all_sources_t;
int32_t lsm6dsv_all_sources_get(stmdev_ctx_t *ctx,
                                lsm6dsv_all_sources_t *val);

typedef struct
{
  uint8_t drdy_xl              : 1;
  uint8_t drdy_g               : 1;
  uint8_t drdy_g_eis           : 1;
  uint8_t fifo_th              : 1;
  uint8_t fifo_ovr             : 1;
  uint8_t fifo_full            : 1;
  uint8_t cnt_bdr              : 1;
  uint8_t emb_func_endop       : 1;
  uint8_t timestamp            : 1;
  uint8_t shub                 : 1;
  uint8_t emb_func             : 1;
  uint8_t sixd                 : 1;
  uint8_t single_tap           : 1;
  uint8_t double_tap           : 1;
  uint8_t wakeup               : 1;
  uint8_t freefall             : 1;
  uint8_t sleep_change         : 1;
} lsm6dsv_pin_int_route_t;
int32_t lsm6dsv_pin_int1_route_set(stmdev_ctx_t *ctx,
                                   lsm6dsv_pin_int_route_t *val);
int32_t lsm6dsv_pin_int1_route_get(stmdev_ctx_t *ctx,
                                   lsm6dsv_pin_int_route_t *val);
int32_t lsm6dsv_pin_int2_route_set(stmdev_ctx_t *ctx,
                                   lsm6dsv_pin_int_route_t *val);
int32_t lsm6dsv_pin_int2_route_get(stmdev_ctx_t *ctx,
                                   lsm6dsv_pin_int_route_t *val);

typedef struct
{
  uint8_t drdy_xl              : 1;
  uint8_t drdy_gy              : 1;
  uint8_t drdy_temp            : 1;
} lsm6dsv_data_ready_t;
int32_t lsm6dsv_flag_data_ready_get(stmdev_ctx_t *ctx,
                                    lsm6dsv_data_ready_t *val);

int32_t lsm6dsv_int_ack_mask_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_int_ack_mask_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm6dsv_angular_rate_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm6dsv_ois_angular_rate_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm6dsv_ois_eis_angular_rate_raw_get(stmdev_ctx_t *ctx,
                                             int16_t *val);

int32_t lsm6dsv_acceleration_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm6dsv_dual_acceleration_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm6dsv_ois_dual_acceleration_raw_get(stmdev_ctx_t *ctx,
                                              int16_t *val);

int32_t lsm6dsv_odr_cal_reg_get(stmdev_ctx_t *ctx, int8_t *val);

int32_t lsm6dsv_ln_pg_write(stmdev_ctx_t *ctx, uint16_t address,
                            uint8_t *buf, uint8_t len);
int32_t lsm6dsv_ln_pg_read(stmdev_ctx_t *ctx, uint16_t address, uint8_t *buf,
                           uint8_t len);

int32_t lsm6dsv_emb_function_dbg_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_emb_function_dbg_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_DEN_ACT_LOW  = 0x0,
  LSM6DSV_DEN_ACT_HIGH = 0x1,
} lsm6dsv_den_polarity_t;
int32_t lsm6dsv_den_polarity_set(stmdev_ctx_t *ctx,
                                 lsm6dsv_den_polarity_t val);
int32_t lsm6dsv_den_polarity_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_den_polarity_t *val);

typedef struct
{
  uint8_t stamp_in_gy_data     : 1;
  uint8_t stamp_in_xl_data     : 1;
  uint8_t den_x                : 1;
  uint8_t den_y                : 1;
  uint8_t den_z                : 1;
  enum
  {
    LSM6DSV_DEN_NOT_DEFINED = 0x00,
    LSM6DSV_LEVEL_TRIGGER   = 0x02,
    LSM6DSV_LEVEL_LATCHED   = 0x03,
  } mode;
} lsm6dsv_den_conf_t;
int32_t lsm6dsv_den_conf_set(stmdev_ctx_t *ctx, lsm6dsv_den_conf_t val);
int32_t lsm6dsv_den_conf_get(stmdev_ctx_t *ctx, lsm6dsv_den_conf_t *val);

typedef enum
{
  LSM6DSV_EIS_125dps  = 0x0,
  LSM6DSV_EIS_250dps  = 0x1,
  LSM6DSV_EIS_500dps  = 0x2,
  LSM6DSV_EIS_1000dps = 0x3,
  LSM6DSV_EIS_2000dps = 0x4,
} lsm6dsv_eis_gy_full_scale_t;
int32_t lsm6dsv_eis_gy_full_scale_set(stmdev_ctx_t *ctx,
                                      lsm6dsv_eis_gy_full_scale_t val);
int32_t lsm6dsv_eis_gy_full_scale_get(stmdev_ctx_t *ctx,
                                      lsm6dsv_eis_gy_full_scale_t *val);

int32_t lsm6dsv_eis_gy_on_spi2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_eis_gy_on_spi2_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_EIS_ODR_OFF = 0x0,
  LSM6DSV_EIS_1920Hz  = 0x1,
  LSM6DSV_EIS_960Hz   = 0x2,
} lsm6dsv_gy_eis_data_rate_t;
int32_t lsm6dsv_gy_eis_data_rate_set(stmdev_ctx_t *ctx,
                                     lsm6dsv_gy_eis_data_rate_t val);
int32_t lsm6dsv_gy_eis_data_rate_get(stmdev_ctx_t *ctx,
                                     lsm6dsv_gy_eis_data_rate_t *val);

int32_t lsm6dsv_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_fifo_xl_dual_fsm_batch_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_fifo_xl_dual_fsm_batch_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_CMP_DISABLE = 0x0,
  LSM6DSV_CMP_8_TO_1  = 0x1,
  LSM6DSV_CMP_16_TO_1 = 0x2,
  LSM6DSV_CMP_32_TO_1 = 0x3,
} lsm6dsv_fifo_compress_algo_t;
int32_t lsm6dsv_fifo_compress_algo_set(stmdev_ctx_t *ctx,
                                       lsm6dsv_fifo_compress_algo_t val);
int32_t lsm6dsv_fifo_compress_algo_get(stmdev_ctx_t *ctx,
                                       lsm6dsv_fifo_compress_algo_t *val);

int32_t lsm6dsv_fifo_virtual_sens_odr_chg_set(stmdev_ctx_t *ctx,
                                              uint8_t val);
int32_t lsm6dsv_fifo_virtual_sens_odr_chg_get(stmdev_ctx_t *ctx,
                                              uint8_t *val);

int32_t lsm6dsv_fifo_compress_algo_real_time_set(stmdev_ctx_t *ctx,
                                                 uint8_t val);
int32_t lsm6dsv_fifo_compress_algo_real_time_get(stmdev_ctx_t *ctx,
                                                 uint8_t *val);

int32_t lsm6dsv_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_XL_NOT_BATCHED       = 0x0,
  LSM6DSV_XL_BATCHED_AT_1Hz875 = 0x1,
  LSM6DSV_XL_BATCHED_AT_7Hz5   = 0x2,
  LSM6DSV_XL_BATCHED_AT_15Hz   = 0x3,
  LSM6DSV_XL_BATCHED_AT_30Hz   = 0x4,
  LSM6DSV_XL_BATCHED_AT_60Hz   = 0x5,
  LSM6DSV_XL_BATCHED_AT_120Hz  = 0x6,
  LSM6DSV_XL_BATCHED_AT_240Hz  = 0x7,
  LSM6DSV_XL_BATCHED_AT_480Hz  = 0x8,
  LSM6DSV_XL_BATCHED_AT_960Hz  = 0x9,
  LSM6DSV_XL_BATCHED_AT_1920Hz = 0xa,
  LSM6DSV_XL_BATCHED_AT_3840Hz = 0xb,
  LSM6DSV_XL_BATCHED_AT_7680Hz = 0xc,
} lsm6dsv_fifo_xl_batch_t;
int32_t lsm6dsv_fifo_xl_batch_set(stmdev_ctx_t *ctx,
                                  lsm6dsv_fifo_xl_batch_t val);
int32_t lsm6dsv_fifo_xl_batch_get(stmdev_ctx_t *ctx,
                                  lsm6dsv_fifo_xl_batch_t *val);

typedef enum
{
  LSM6DSV_GY_NOT_BATCHED       = 0x0,
  LSM6DSV_GY_BATCHED_AT_1Hz875 = 0x1,
  LSM6DSV_GY_BATCHED_AT_7Hz5   = 0x2,
  LSM6DSV_GY_BATCHED_AT_15Hz   = 0x3,
  LSM6DSV_GY_BATCHED_AT_30Hz   = 0x4,
  LSM6DSV_GY_BATCHED_AT_60Hz   = 0x5,
  LSM6DSV_GY_BATCHED_AT_120Hz  = 0x6,
  LSM6DSV_GY_BATCHED_AT_240Hz  = 0x7,
  LSM6DSV_GY_BATCHED_AT_480Hz  = 0x8,
  LSM6DSV_GY_BATCHED_AT_960Hz  = 0x9,
  LSM6DSV_GY_BATCHED_AT_1920Hz = 0xa,
  LSM6DSV_GY_BATCHED_AT_3840Hz = 0xb,
  LSM6DSV_GY_BATCHED_AT_7680Hz = 0xc,
} lsm6dsv_fifo_gy_batch_t;
int32_t lsm6dsv_fifo_gy_batch_set(stmdev_ctx_t *ctx,
                                  lsm6dsv_fifo_gy_batch_t val);
int32_t lsm6dsv_fifo_gy_batch_get(stmdev_ctx_t *ctx,
                                  lsm6dsv_fifo_gy_batch_t *val);

typedef enum
{
  LSM6DSV_BYPASS_MODE             = 0x0,
  LSM6DSV_FIFO_MODE               = 0x1,
  LSM6DSV_STREAM_WTM_TO_FULL_MODE = 0x2,
  LSM6DSV_STREAM_TO_FIFO_MODE     = 0x3,
  LSM6DSV_BYPASS_TO_STREAM_MODE   = 0x4,
  LSM6DSV_STREAM_MODE             = 0x6,
  LSM6DSV_BYPASS_TO_FIFO_MODE     = 0x7,
} lsm6dsv_fifo_mode_t;
int32_t lsm6dsv_fifo_mode_set(stmdev_ctx_t *ctx, lsm6dsv_fifo_mode_t val);
int32_t lsm6dsv_fifo_mode_get(stmdev_ctx_t *ctx,
                              lsm6dsv_fifo_mode_t *val);

int32_t lsm6dsv_fifo_gy_eis_batch_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_fifo_gy_eis_batch_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_TEMP_NOT_BATCHED       = 0x0,
  LSM6DSV_TEMP_BATCHED_AT_1Hz875 = 0x1,
  LSM6DSV_TEMP_BATCHED_AT_15Hz   = 0x2,
  LSM6DSV_TEMP_BATCHED_AT_60Hz   = 0x3,
} lsm6dsv_fifo_temp_batch_t;
int32_t lsm6dsv_fifo_temp_batch_set(stmdev_ctx_t *ctx,
                                    lsm6dsv_fifo_temp_batch_t val);
int32_t lsm6dsv_fifo_temp_batch_get(stmdev_ctx_t *ctx,
                                    lsm6dsv_fifo_temp_batch_t *val);

typedef enum
{
  LSM6DSV_TMSTMP_NOT_BATCHED = 0x0,
  LSM6DSV_TMSTMP_DEC_1       = 0x1,
  LSM6DSV_TMSTMP_DEC_8       = 0x2,
  LSM6DSV_TMSTMP_DEC_32      = 0x3,
} lsm6dsv_fifo_timestamp_batch_t;
int32_t lsm6dsv_fifo_timestamp_batch_set(stmdev_ctx_t *ctx,
                                         lsm6dsv_fifo_timestamp_batch_t val);
int32_t lsm6dsv_fifo_timestamp_batch_get(stmdev_ctx_t *ctx,
                                         lsm6dsv_fifo_timestamp_batch_t *val);

int32_t lsm6dsv_fifo_batch_counter_threshold_set(stmdev_ctx_t *ctx,
                                                 uint16_t val);
int32_t lsm6dsv_fifo_batch_counter_threshold_get(stmdev_ctx_t *ctx,
                                                 uint16_t *val);

typedef enum
{
  LSM6DSV_XL_BATCH_EVENT     = 0x0,
  LSM6DSV_GY_BATCH_EVENT     = 0x1,
  LSM6DSV_GY_EIS_BATCH_EVENT = 0x2,
} lsm6dsv_fifo_batch_cnt_event_t;
int32_t lsm6dsv_fifo_batch_cnt_event_set(stmdev_ctx_t *ctx,
                                         lsm6dsv_fifo_batch_cnt_event_t val);
int32_t lsm6dsv_fifo_batch_cnt_event_get(stmdev_ctx_t *ctx,
                                         lsm6dsv_fifo_batch_cnt_event_t *val);

typedef struct
{
  uint16_t fifo_level          : 9;
  uint8_t fifo_bdr             : 1;
  uint8_t fifo_full            : 1;
  uint8_t fifo_ovr             : 1;
  uint8_t fifo_th              : 1;
} lsm6dsv_fifo_status_t;

int32_t lsm6dsv_fifo_status_get(stmdev_ctx_t *ctx,
                                lsm6dsv_fifo_status_t *val);

typedef struct
{
  enum
  {
    LSM6DSV_FIFO_EMPTY                    = 0x0,
    LSM6DSV_GY_NC_TAG                     = 0x1,
    LSM6DSV_XL_NC_TAG                     = 0x2,
    LSM6DSV_TEMPERATURE_TAG               = 0x3,
    LSM6DSV_TIMESTAMP_TAG                 = 0x4,
    LSM6DSV_CFG_CHANGE_TAG                = 0x5,
    LSM6DSV_XL_NC_T_2_TAG                 = 0x6,
    LSM6DSV_XL_NC_T_1_TAG                 = 0x7,
    LSM6DSV_XL_2XC_TAG                    = 0x8,
    LSM6DSV_XL_3XC_TAG                    = 0x9,
    LSM6DSV_GY_NC_T_2_TAG                 = 0xA,
    LSM6DSV_GY_NC_T_1_TAG                 = 0xB,
    LSM6DSV_GY_2XC_TAG                    = 0xC,
    LSM6DSV_GY_3XC_TAG                    = 0xD,
    LSM6DSV_SENSORHUB_SLAVE0_TAG          = 0xE,
    LSM6DSV_SENSORHUB_SLAVE1_TAG          = 0xF,
    LSM6DSV_SENSORHUB_SLAVE2_TAG          = 0x10,
    LSM6DSV_SENSORHUB_SLAVE3_TAG          = 0x11,
    LSM6DSV_STEP_COUNTER_TAG              = 0x12,
    LSM6DSV_SFLP_GAME_ROTATION_VECTOR_TAG = 0x13,
    LSM6DSV_SFLP_GYROSCOPE_BIAS_TAG       = 0x16,
    LSM6DSV_SFLP_GRAVITY_VECTOR_TAG       = 0x17,
    LSM6DSV_SENSORHUB_NACK_TAG            = 0x19,
    LSM6DSV_XL_DUAL_CORE                  = 0x1D,
    LSM6DSV_GY_ENHANCED_EIS               = 0x1E,
  } tag;
  uint8_t cnt;
  uint8_t data[6];
} lsm6dsv_fifo_out_raw_t;
int32_t lsm6dsv_fifo_out_raw_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_fifo_out_raw_t *val);

int32_t lsm6dsv_fifo_stpcnt_batch_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_fifo_stpcnt_batch_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_fifo_sh_batch_slave_set(stmdev_ctx_t *ctx, uint8_t idx, uint8_t val);
int32_t lsm6dsv_fifo_sh_batch_slave_get(stmdev_ctx_t *ctx, uint8_t idx, uint8_t *val);

typedef struct
{
  uint8_t game_rotation        : 1;
  uint8_t gravity              : 1;
  uint8_t gbias                : 1;
} lsm6dsv_fifo_sflp_raw_t;
int32_t lsm6dsv_fifo_sflp_batch_set(stmdev_ctx_t *ctx,
                                       lsm6dsv_fifo_sflp_raw_t val);
int32_t lsm6dsv_fifo_sflp_batch_get(stmdev_ctx_t *ctx,
                                       lsm6dsv_fifo_sflp_raw_t *val);

typedef enum
{
  LSM6DSV_AUTO          = 0x0,
  LSM6DSV_ALWAYS_ACTIVE = 0x1,
} lsm6dsv_filt_anti_spike_t;
int32_t lsm6dsv_filt_anti_spike_set(stmdev_ctx_t *ctx,
                                    lsm6dsv_filt_anti_spike_t val);
int32_t lsm6dsv_filt_anti_spike_get(stmdev_ctx_t *ctx,
                                    lsm6dsv_filt_anti_spike_t *val);

typedef struct
{
  uint8_t drdy                 : 1;
  uint8_t ois_drdy             : 1;
  uint8_t irq_xl               : 1;
  uint8_t irq_g                : 1;
} lsm6dsv_filt_settling_mask_t;
int32_t lsm6dsv_filt_settling_mask_set(stmdev_ctx_t *ctx,
                                       lsm6dsv_filt_settling_mask_t val);
int32_t lsm6dsv_filt_settling_mask_get(stmdev_ctx_t *ctx,
                                       lsm6dsv_filt_settling_mask_t *val);

typedef struct
{
  uint8_t ois_drdy             : 1;
} lsm6dsv_filt_ois_settling_mask_t;
int32_t lsm6dsv_filt_ois_settling_mask_set(stmdev_ctx_t *ctx,
                                           lsm6dsv_filt_ois_settling_mask_t val);
int32_t lsm6dsv_filt_ois_settling_mask_get(stmdev_ctx_t *ctx,
                                           lsm6dsv_filt_ois_settling_mask_t *val);

typedef enum
{
  LSM6DSV_GY_ULTRA_LIGHT   = 0x0,
  LSM6DSV_GY_VERY_LIGHT    = 0x1,
  LSM6DSV_GY_LIGHT         = 0x2,
  LSM6DSV_GY_MEDIUM        = 0x3,
  LSM6DSV_GY_STRONG        = 0x4,
  LSM6DSV_GY_VERY_STRONG   = 0x5,
  LSM6DSV_GY_AGGRESSIVE    = 0x6,
  LSM6DSV_GY_XTREME        = 0x7,
} lsm6dsv_filt_gy_lp1_bandwidth_t;
int32_t lsm6dsv_filt_gy_lp1_bandwidth_set(stmdev_ctx_t *ctx,
                                          lsm6dsv_filt_gy_lp1_bandwidth_t val);
int32_t lsm6dsv_filt_gy_lp1_bandwidth_get(stmdev_ctx_t *ctx,
                                          lsm6dsv_filt_gy_lp1_bandwidth_t *val);

int32_t lsm6dsv_filt_gy_lp1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_filt_gy_lp1_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_XL_ULTRA_LIGHT = 0x0,
  LSM6DSV_XL_VERY_LIGHT  = 0x1,
  LSM6DSV_XL_LIGHT       = 0x2,
  LSM6DSV_XL_MEDIUM      = 0x3,
  LSM6DSV_XL_STRONG      = 0x4,
  LSM6DSV_XL_VERY_STRONG = 0x5,
  LSM6DSV_XL_AGGRESSIVE  = 0x6,
  LSM6DSV_XL_XTREME      = 0x7,
} lsm6dsv_filt_xl_lp2_bandwidth_t;
int32_t lsm6dsv_filt_xl_lp2_bandwidth_set(stmdev_ctx_t *ctx,
                                          lsm6dsv_filt_xl_lp2_bandwidth_t val);
int32_t lsm6dsv_filt_xl_lp2_bandwidth_get(stmdev_ctx_t *ctx,
                                          lsm6dsv_filt_xl_lp2_bandwidth_t *val);

int32_t lsm6dsv_filt_xl_lp2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_filt_xl_lp2_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_filt_xl_hp_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_filt_xl_hp_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_filt_xl_fast_settling_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_filt_xl_fast_settling_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_HP_MD_NORMAL    = 0x0,
  LSM6DSV_HP_MD_REFERENCE = 0x1,
} lsm6dsv_filt_xl_hp_mode_t;
int32_t lsm6dsv_filt_xl_hp_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsv_filt_xl_hp_mode_t val);
int32_t lsm6dsv_filt_xl_hp_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsv_filt_xl_hp_mode_t *val);

typedef enum
{
  LSM6DSV_WK_FEED_SLOPE          = 0x0,
  LSM6DSV_WK_FEED_HIGH_PASS      = 0x1,
  LSM6DSV_WK_FEED_LP_WITH_OFFSET = 0x2,
} lsm6dsv_filt_wkup_act_feed_t;
int32_t lsm6dsv_filt_wkup_act_feed_set(stmdev_ctx_t *ctx,
                                       lsm6dsv_filt_wkup_act_feed_t val);
int32_t lsm6dsv_filt_wkup_act_feed_get(stmdev_ctx_t *ctx,
                                       lsm6dsv_filt_wkup_act_feed_t *val);

int32_t lsm6dsv_mask_trigger_xl_settl_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_mask_trigger_xl_settl_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_SIXD_FEED_ODR_DIV_2 = 0x0,
  LSM6DSV_SIXD_FEED_LOW_PASS  = 0x1,
} lsm6dsv_filt_sixd_feed_t;
int32_t lsm6dsv_filt_sixd_feed_set(stmdev_ctx_t *ctx,
                                   lsm6dsv_filt_sixd_feed_t val);
int32_t lsm6dsv_filt_sixd_feed_get(stmdev_ctx_t *ctx,
                                   lsm6dsv_filt_sixd_feed_t *val);

typedef enum
{
  LSM6DSV_EIS_LP_NORMAL = 0x0,
  LSM6DSV_EIS_LP_LIGHT  = 0x1,
} lsm6dsv_filt_gy_eis_lp_bandwidth_t;
int32_t lsm6dsv_filt_gy_eis_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                             lsm6dsv_filt_gy_eis_lp_bandwidth_t val);
int32_t lsm6dsv_filt_gy_eis_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                             lsm6dsv_filt_gy_eis_lp_bandwidth_t *val);

typedef enum
{
  LSM6DSV_OIS_GY_LP_NORMAL     = 0x0,
  LSM6DSV_OIS_GY_LP_STRONG     = 0x1,
  LSM6DSV_OIS_GY_LP_AGGRESSIVE = 0x2,
  LSM6DSV_OIS_GY_LP_LIGHT      = 0x3,
} lsm6dsv_filt_gy_ois_lp_bandwidth_t;
int32_t lsm6dsv_filt_gy_ois_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                             lsm6dsv_filt_gy_ois_lp_bandwidth_t val);
int32_t lsm6dsv_filt_gy_ois_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                             lsm6dsv_filt_gy_ois_lp_bandwidth_t *val);

typedef enum
{
  LSM6DSV_OIS_XL_LP_ULTRA_LIGHT = 0x0,
  LSM6DSV_OIS_XL_LP_VERY_LIGHT  = 0x1,
  LSM6DSV_OIS_XL_LP_LIGHT       = 0x2,
  LSM6DSV_OIS_XL_LP_NORMAL      = 0x3,
  LSM6DSV_OIS_XL_LP_STRONG      = 0x4,
  LSM6DSV_OIS_XL_LP_VERY_STRONG = 0x5,
  LSM6DSV_OIS_XL_LP_AGGRESSIVE  = 0x6,
  LSM6DSV_OIS_XL_LP_XTREME      = 0x7,
} lsm6dsv_filt_xl_ois_lp_bandwidth_t;
int32_t lsm6dsv_filt_xl_ois_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                             lsm6dsv_filt_xl_ois_lp_bandwidth_t val);
int32_t lsm6dsv_filt_xl_ois_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                             lsm6dsv_filt_xl_ois_lp_bandwidth_t *val);

typedef enum
{
  LSM6DSV_PROTECT_CTRL_REGS = 0x0,
  LSM6DSV_WRITE_CTRL_REG    = 0x1,
} lsm6dsv_fsm_permission_t;
int32_t lsm6dsv_fsm_permission_set(stmdev_ctx_t *ctx,
                                   lsm6dsv_fsm_permission_t val);
int32_t lsm6dsv_fsm_permission_get(stmdev_ctx_t *ctx,
                                   lsm6dsv_fsm_permission_t *val);
int32_t lsm6dsv_fsm_permission_status(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  uint8_t fsm1_en              : 1;
  uint8_t fsm2_en              : 1;
  uint8_t fsm3_en              : 1;
  uint8_t fsm4_en              : 1;
  uint8_t fsm5_en              : 1;
  uint8_t fsm6_en              : 1;
  uint8_t fsm7_en              : 1;
  uint8_t fsm8_en              : 1;
} lsm6dsv_fsm_mode_t;
int32_t lsm6dsv_fsm_mode_set(stmdev_ctx_t *ctx, lsm6dsv_fsm_mode_t val);
int32_t lsm6dsv_fsm_mode_get(stmdev_ctx_t *ctx, lsm6dsv_fsm_mode_t *val);

int32_t lsm6dsv_fsm_long_cnt_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm6dsv_fsm_long_cnt_get(stmdev_ctx_t *ctx, uint16_t *val);


typedef struct
{
  uint8_t fsm_outs1;
  uint8_t fsm_outs2;
  uint8_t fsm_outs3;
  uint8_t fsm_outs4;
  uint8_t fsm_outs5;
  uint8_t fsm_outs6;
  uint8_t fsm_outs7;
  uint8_t fsm_outs8;
} lsm6dsv_fsm_out_t;
int32_t lsm6dsv_fsm_out_get(stmdev_ctx_t *ctx, lsm6dsv_fsm_out_t *val);

typedef enum
{
  LSM6DSV_FSM_15Hz  = 0x0,
  LSM6DSV_FSM_30Hz  = 0x1,
  LSM6DSV_FSM_60Hz  = 0x2,
  LSM6DSV_FSM_120Hz = 0x3,
  LSM6DSV_FSM_240Hz = 0x4,
  LSM6DSV_FSM_480Hz = 0x5,
  LSM6DSV_FSM_960Hz = 0x6,
} lsm6dsv_fsm_data_rate_t;
int32_t lsm6dsv_fsm_data_rate_set(stmdev_ctx_t *ctx,
                                  lsm6dsv_fsm_data_rate_t val);
int32_t lsm6dsv_fsm_data_rate_get(stmdev_ctx_t *ctx,
                                  lsm6dsv_fsm_data_rate_t *val);

int32_t lsm6dsv_fsm_ext_sens_sensitivity_set(stmdev_ctx_t *ctx,
                                             uint16_t val);
int32_t lsm6dsv_fsm_ext_sens_sensitivity_get(stmdev_ctx_t *ctx,
                                             uint16_t *val);

typedef struct
{
  uint16_t z;
  uint16_t y;
  uint16_t x;
} lsm6dsv_xl_fsm_ext_sens_offset_t;
int32_t lsm6dsv_fsm_ext_sens_offset_set(stmdev_ctx_t *ctx,
                                        lsm6dsv_xl_fsm_ext_sens_offset_t val);
int32_t lsm6dsv_fsm_ext_sens_offset_get(stmdev_ctx_t *ctx,
                                        lsm6dsv_xl_fsm_ext_sens_offset_t *val);

typedef struct
{
  uint16_t xx;
  uint16_t xy;
  uint16_t xz;
  uint16_t yy;
  uint16_t yz;
  uint16_t zz;
} lsm6dsv_xl_fsm_ext_sens_matrix_t;
int32_t lsm6dsv_fsm_ext_sens_matrix_set(stmdev_ctx_t *ctx,
                                        lsm6dsv_xl_fsm_ext_sens_matrix_t val);
int32_t lsm6dsv_fsm_ext_sens_matrix_get(stmdev_ctx_t *ctx,
                                        lsm6dsv_xl_fsm_ext_sens_matrix_t *val);

typedef enum
{
  LSM6DSV_Z_EQ_Y     = 0x0,
  LSM6DSV_Z_EQ_MIN_Y = 0x1,
  LSM6DSV_Z_EQ_X     = 0x2,
  LSM6DSV_Z_EQ_MIN_X = 0x3,
  LSM6DSV_Z_EQ_MIN_Z = 0x4,
  LSM6DSV_Z_EQ_Z     = 0x5,
} lsm6dsv_fsm_ext_sens_z_orient_t;
int32_t lsm6dsv_fsm_ext_sens_z_orient_set(stmdev_ctx_t *ctx,
                                          lsm6dsv_fsm_ext_sens_z_orient_t val);
int32_t lsm6dsv_fsm_ext_sens_z_orient_get(stmdev_ctx_t *ctx,
                                          lsm6dsv_fsm_ext_sens_z_orient_t *val);

typedef enum
{
  LSM6DSV_Y_EQ_Y     = 0x0,
  LSM6DSV_Y_EQ_MIN_Y = 0x1,
  LSM6DSV_Y_EQ_X     = 0x2,
  LSM6DSV_Y_EQ_MIN_X = 0x3,
  LSM6DSV_Y_EQ_MIN_Z = 0x4,
  LSM6DSV_Y_EQ_Z     = 0x5,
} lsm6dsv_fsm_ext_sens_y_orient_t;
int32_t lsm6dsv_fsm_ext_sens_y_orient_set(stmdev_ctx_t *ctx,
                                          lsm6dsv_fsm_ext_sens_y_orient_t val);
int32_t lsm6dsv_fsm_ext_sens_y_orient_get(stmdev_ctx_t *ctx,
                                          lsm6dsv_fsm_ext_sens_y_orient_t *val);

typedef enum
{
  LSM6DSV_X_EQ_Y     = 0x0,
  LSM6DSV_X_EQ_MIN_Y = 0x1,
  LSM6DSV_X_EQ_X     = 0x2,
  LSM6DSV_X_EQ_MIN_X = 0x3,
  LSM6DSV_X_EQ_MIN_Z = 0x4,
  LSM6DSV_X_EQ_Z     = 0x5,
} lsm6dsv_fsm_ext_sens_x_orient_t;
int32_t lsm6dsv_fsm_ext_sens_x_orient_set(stmdev_ctx_t *ctx,
                                          lsm6dsv_fsm_ext_sens_x_orient_t val);
int32_t lsm6dsv_fsm_ext_sens_x_orient_get(stmdev_ctx_t *ctx,
                                          lsm6dsv_fsm_ext_sens_x_orient_t *val);

int32_t lsm6dsv_fsm_long_cnt_timeout_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm6dsv_fsm_long_cnt_timeout_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsv_fsm_number_of_programs_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_fsm_number_of_programs_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_fsm_start_address_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm6dsv_fsm_start_address_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsv_ff_time_windows_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_ff_time_windows_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_156_mg = 0x0,
  LSM6DSV_219_mg = 0x1,
  LSM6DSV_250_mg = 0x2,
  LSM6DSV_312_mg = 0x3,
  LSM6DSV_344_mg = 0x4,
  LSM6DSV_406_mg = 0x5,
  LSM6DSV_469_mg = 0x6,
  LSM6DSV_500_mg = 0x7,
} lsm6dsv_ff_thresholds_t;
int32_t lsm6dsv_ff_thresholds_set(stmdev_ctx_t *ctx,
                                  lsm6dsv_ff_thresholds_t val);
int32_t lsm6dsv_ff_thresholds_get(stmdev_ctx_t *ctx,
                                  lsm6dsv_ff_thresholds_t *val);

typedef enum
{
  LSM6DSV_OIS_CTRL_FROM_OIS = 0x0,
  LSM6DSV_OIS_CTRL_FROM_UI  = 0x1,
} lsm6dsv_ois_ctrl_mode_t;
int32_t lsm6dsv_ois_ctrl_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsv_ois_ctrl_mode_t val);
int32_t lsm6dsv_ois_ctrl_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsv_ois_ctrl_mode_t *val);

int32_t lsm6dsv_ois_reset_set(stmdev_ctx_t *ctx, int8_t val);
int32_t lsm6dsv_ois_reset_get(stmdev_ctx_t *ctx, int8_t *val);

int32_t lsm6dsv_ois_interface_pull_up_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_ois_interface_pull_up_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  uint8_t ack                  : 1;
  uint8_t req                  : 1;
} lsm6dsv_ois_handshake_t;
int32_t lsm6dsv_ois_handshake_from_ui_set(stmdev_ctx_t *ctx,
                                          lsm6dsv_ois_handshake_t val);
int32_t lsm6dsv_ois_handshake_from_ui_get(stmdev_ctx_t *ctx,
                                          lsm6dsv_ois_handshake_t *val);
int32_t lsm6dsv_ois_handshake_from_ois_set(stmdev_ctx_t *ctx,
                                           lsm6dsv_ois_handshake_t val);
int32_t lsm6dsv_ois_handshake_from_ois_get(stmdev_ctx_t *ctx,
                                           lsm6dsv_ois_handshake_t *val);

int32_t lsm6dsv_ois_shared_set(stmdev_ctx_t *ctx, uint8_t val[6]);
int32_t lsm6dsv_ois_shared_get(stmdev_ctx_t *ctx, uint8_t val[6]);

int32_t lsm6dsv_ois_on_spi2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_ois_on_spi2_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  uint8_t gy                   : 1;
  uint8_t xl                   : 1;
} lsm6dsv_ois_chain_t;
int32_t lsm6dsv_ois_chain_set(stmdev_ctx_t *ctx, lsm6dsv_ois_chain_t val);
int32_t lsm6dsv_ois_chain_get(stmdev_ctx_t *ctx,
                              lsm6dsv_ois_chain_t *val);

typedef enum
{
  LSM6DSV_OIS_125dps  = 0x0,
  LSM6DSV_OIS_250dps  = 0x1,
  LSM6DSV_OIS_500dps  = 0x2,
  LSM6DSV_OIS_1000dps = 0x3,
  LSM6DSV_OIS_2000dps = 0x4,
} lsm6dsv_ois_gy_full_scale_t;
int32_t lsm6dsv_ois_gy_full_scale_set(stmdev_ctx_t *ctx,
                                      lsm6dsv_ois_gy_full_scale_t val);
int32_t lsm6dsv_ois_gy_full_scale_get(stmdev_ctx_t *ctx,
                                      lsm6dsv_ois_gy_full_scale_t *val);

typedef enum
{
  LSM6DSV_OIS_2g  = 0x0,
  LSM6DSV_OIS_4g  = 0x1,
  LSM6DSV_OIS_8g  = 0x2,
  LSM6DSV_OIS_16g = 0x3,
} lsm6dsv_ois_xl_full_scale_t;
int32_t lsm6dsv_ois_xl_full_scale_set(stmdev_ctx_t *ctx,
                                      lsm6dsv_ois_xl_full_scale_t val);
int32_t lsm6dsv_ois_xl_full_scale_get(stmdev_ctx_t *ctx,
                                      lsm6dsv_ois_xl_full_scale_t *val);

typedef enum
{
  LSM6DSV_DEG_80 = 0x0,
  LSM6DSV_DEG_70 = 0x1,
  LSM6DSV_DEG_60 = 0x2,
  LSM6DSV_DEG_50 = 0x3,
} lsm6dsv_6d_threshold_t;
int32_t lsm6dsv_6d_threshold_set(stmdev_ctx_t *ctx,
                                 lsm6dsv_6d_threshold_t val);
int32_t lsm6dsv_6d_threshold_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_6d_threshold_t *val);

int32_t lsm6dsv_4d_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_4d_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_SW_RST_DYN_ADDRESS_RST = 0x0,
  LSM6DSV_I3C_GLOBAL_RST         = 0x1,
} lsm6dsv_i3c_reset_mode_t;
int32_t lsm6dsv_i3c_reset_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsv_i3c_reset_mode_t val);
int32_t lsm6dsv_i3c_reset_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsv_i3c_reset_mode_t *val);

typedef enum
{
  LSM6DSV_IBI_2us  = 0x0,
  LSM6DSV_IBI_50us = 0x1,
  LSM6DSV_IBI_1ms  = 0x2,
  LSM6DSV_IBI_25ms = 0x3,
} lsm6dsv_i3c_ibi_time_t;
int32_t lsm6dsv_i3c_ibi_time_set(stmdev_ctx_t *ctx,
                                 lsm6dsv_i3c_ibi_time_t val);
int32_t lsm6dsv_i3c_ibi_time_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_i3c_ibi_time_t *val);

int32_t lsm6dsv_sh_master_interface_pull_up_set(stmdev_ctx_t *ctx,
                                                uint8_t val);
int32_t lsm6dsv_sh_master_interface_pull_up_get(stmdev_ctx_t *ctx,
                                                uint8_t *val);

int32_t lsm6dsv_sh_read_data_raw_get(stmdev_ctx_t *ctx, uint8_t *val,
                                     uint8_t len);

typedef enum
{
  LSM6DSV_SLV_0       = 0x0,
  LSM6DSV_SLV_0_1     = 0x1,
  LSM6DSV_SLV_0_1_2   = 0x2,
  LSM6DSV_SLV_0_1_2_3 = 0x3,
} lsm6dsv_sh_slave_connected_t;
int32_t lsm6dsv_sh_slave_connected_set(stmdev_ctx_t *ctx,
                                       lsm6dsv_sh_slave_connected_t val);
int32_t lsm6dsv_sh_slave_connected_get(stmdev_ctx_t *ctx,
                                       lsm6dsv_sh_slave_connected_t *val);

int32_t lsm6dsv_sh_master_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_sh_master_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_sh_pass_through_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_sh_pass_through_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_SH_TRG_XL_GY_DRDY = 0x0,
  LSM6DSV_SH_TRIG_INT2      = 0x1,
} lsm6dsv_sh_syncro_mode_t;
int32_t lsm6dsv_sh_syncro_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsv_sh_syncro_mode_t val);
int32_t lsm6dsv_sh_syncro_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsv_sh_syncro_mode_t *val);

typedef enum
{
  LSM6DSV_EACH_SH_CYCLE    = 0x0,
  LSM6DSV_ONLY_FIRST_CYCLE = 0x1,
} lsm6dsv_sh_write_mode_t;
int32_t lsm6dsv_sh_write_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsv_sh_write_mode_t val);
int32_t lsm6dsv_sh_write_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsv_sh_write_mode_t *val);

int32_t lsm6dsv_sh_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_sh_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  uint8_t   slv0_add;
  uint8_t   slv0_subadd;
  uint8_t   slv0_data;
} lsm6dsv_sh_cfg_write_t;
int32_t lsm6dsv_sh_cfg_write(stmdev_ctx_t *ctx,
                             lsm6dsv_sh_cfg_write_t *val);
typedef enum
{
  LSM6DSV_SH_15Hz  = 0x1,
  LSM6DSV_SH_30Hz  = 0x2,
  LSM6DSV_SH_60Hz  = 0x3,
  LSM6DSV_SH_120Hz = 0x4,
  LSM6DSV_SH_240Hz = 0x5,
  LSM6DSV_SH_480Hz = 0x6,
} lsm6dsv_sh_data_rate_t;
int32_t lsm6dsv_sh_data_rate_set(stmdev_ctx_t *ctx,
                                 lsm6dsv_sh_data_rate_t val);
int32_t lsm6dsv_sh_data_rate_get(stmdev_ctx_t *ctx,
                                 lsm6dsv_sh_data_rate_t *val);

typedef struct
{
  uint8_t   slv_add;
  uint8_t   slv_subadd;
  uint8_t   slv_len;
} lsm6dsv_sh_cfg_read_t;
int32_t lsm6dsv_sh_slv_cfg_read(stmdev_ctx_t *ctx, uint8_t idx,
                                   lsm6dsv_sh_cfg_read_t *val);

int32_t lsm6dsv_sh_status_get(stmdev_ctx_t *ctx,
                              lsm6dsv_status_master_t *val);

int32_t lsm6dsv_ui_sdo_pull_up_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_ui_sdo_pull_up_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_I2C_I3C_ENABLE  = 0x0,
  LSM6DSV_I2C_I3C_DISABLE = 0x1,
} lsm6dsv_ui_i2c_i3c_mode_t;
int32_t lsm6dsv_ui_i2c_i3c_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsv_ui_i2c_i3c_mode_t val);
int32_t lsm6dsv_ui_i2c_i3c_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsv_ui_i2c_i3c_mode_t *val);

typedef enum
{
  LSM6DSV_SPI_4_WIRE = 0x0,
  LSM6DSV_SPI_3_WIRE = 0x1,
} lsm6dsv_spi_mode_t;
int32_t lsm6dsv_spi_mode_set(stmdev_ctx_t *ctx, lsm6dsv_spi_mode_t val);
int32_t lsm6dsv_spi_mode_get(stmdev_ctx_t *ctx, lsm6dsv_spi_mode_t *val);

int32_t lsm6dsv_ui_sda_pull_up_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_ui_sda_pull_up_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_SPI2_4_WIRE = 0x0,
  LSM6DSV_SPI2_3_WIRE = 0x1,
} lsm6dsv_spi2_mode_t;
int32_t lsm6dsv_spi2_mode_set(stmdev_ctx_t *ctx, lsm6dsv_spi2_mode_t val);
int32_t lsm6dsv_spi2_mode_get(stmdev_ctx_t *ctx,
                              lsm6dsv_spi2_mode_t *val);

int32_t lsm6dsv_sigmot_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_sigmot_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  uint8_t step_counter_enable  : 1;
} lsm6dsv_stpcnt_mode_t;
int32_t lsm6dsv_stpcnt_mode_set(stmdev_ctx_t *ctx,
                                lsm6dsv_stpcnt_mode_t val);
int32_t lsm6dsv_stpcnt_mode_get(stmdev_ctx_t *ctx,
                                lsm6dsv_stpcnt_mode_t *val);

int32_t lsm6dsv_stpcnt_steps_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsv_stpcnt_rst_step_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_stpcnt_rst_step_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_stpcnt_debounce_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_stpcnt_debounce_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_stpcnt_period_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm6dsv_stpcnt_period_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsv_sflp_game_rotation_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_sflp_game_rotation_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  float_t gbias_x; /* dps */
  float_t gbias_y; /* dps */
  float_t gbias_z; /* dps */
} lsm6dsv_sflp_gbias_t;
int32_t lsm6dsv_sflp_game_gbias_set(stmdev_ctx_t *ctx,
                                       lsm6dsv_sflp_gbias_t *val);

typedef enum
{
  LSM6DSV_SFLP_15Hz  = 0x0,
  LSM6DSV_SFLP_30Hz  = 0x1,
  LSM6DSV_SFLP_60Hz  = 0x2,
  LSM6DSV_SFLP_120Hz = 0x3,
  LSM6DSV_SFLP_240Hz = 0x4,
  LSM6DSV_SFLP_480Hz = 0x5,
} lsm6dsv_sflp_data_rate_t;
int32_t lsm6dsv_sflp_data_rate_set(stmdev_ctx_t *ctx,
                                      lsm6dsv_sflp_data_rate_t val);
int32_t lsm6dsv_sflp_data_rate_get(stmdev_ctx_t *ctx,
                                      lsm6dsv_sflp_data_rate_t *val);

typedef struct
{
  uint8_t tap_x_en             : 1;
  uint8_t tap_y_en             : 1;
  uint8_t tap_z_en             : 1;
} lsm6dsv_tap_detection_t;
int32_t lsm6dsv_tap_detection_set(stmdev_ctx_t *ctx,
                                  lsm6dsv_tap_detection_t val);
int32_t lsm6dsv_tap_detection_get(stmdev_ctx_t *ctx,
                                  lsm6dsv_tap_detection_t *val);

typedef struct
{
  uint8_t x                    : 5;
  uint8_t y                    : 5;
  uint8_t z                    : 5;
} lsm6dsv_tap_thresholds_t;
int32_t lsm6dsv_tap_thresholds_set(stmdev_ctx_t *ctx,
                                   lsm6dsv_tap_thresholds_t val);
int32_t lsm6dsv_tap_thresholds_get(stmdev_ctx_t *ctx,
                                   lsm6dsv_tap_thresholds_t *val);

typedef enum
{
  LSM6DSV_XYZ  = 0x0,
  LSM6DSV_YXZ  = 0x1,
  LSM6DSV_XZY  = 0x2,
  LSM6DSV_ZYX  = 0x3,
  LSM6DSV_YZX  = 0x5,
  LSM6DSV_ZXY  = 0x6,
} lsm6dsv_tap_axis_priority_t;
int32_t lsm6dsv_tap_axis_priority_set(stmdev_ctx_t *ctx,
                                      lsm6dsv_tap_axis_priority_t val);
int32_t lsm6dsv_tap_axis_priority_get(stmdev_ctx_t *ctx,
                                      lsm6dsv_tap_axis_priority_t *val);

typedef struct
{
  uint8_t shock                : 2;
  uint8_t quiet                : 2;
  uint8_t tap_gap              : 4;
} lsm6dsv_tap_time_windows_t;
int32_t lsm6dsv_tap_time_windows_set(stmdev_ctx_t *ctx,
                                     lsm6dsv_tap_time_windows_t val);
int32_t lsm6dsv_tap_time_windows_get(stmdev_ctx_t *ctx,
                                     lsm6dsv_tap_time_windows_t *val);

typedef enum
{
  LSM6DSV_ONLY_SINGLE        = 0x0,
  LSM6DSV_BOTH_SINGLE_DOUBLE = 0x1,
} lsm6dsv_tap_mode_t;
int32_t lsm6dsv_tap_mode_set(stmdev_ctx_t *ctx, lsm6dsv_tap_mode_t val);
int32_t lsm6dsv_tap_mode_get(stmdev_ctx_t *ctx, lsm6dsv_tap_mode_t *val);

int32_t lsm6dsv_tilt_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_tilt_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsv_timestamp_raw_get(stmdev_ctx_t *ctx, uint32_t *val);

int32_t lsm6dsv_timestamp_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsv_timestamp_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSV_XL_AND_GY_NOT_AFFECTED       = 0x0,
  LSM6DSV_XL_LOW_POWER_GY_NOT_AFFECTED = 0x1,
  LSM6DSV_XL_LOW_POWER_GY_SLEEP        = 0x2,
  LSM6DSV_XL_LOW_POWER_GY_POWER_DOWN   = 0x3,
} lsm6dsv_act_mode_t;
int32_t lsm6dsv_act_mode_set(stmdev_ctx_t *ctx, lsm6dsv_act_mode_t val);
int32_t lsm6dsv_act_mode_get(stmdev_ctx_t *ctx, lsm6dsv_act_mode_t *val);

typedef enum
{
  LSM6DSV_SLEEP_TO_ACT_AT_1ST_SAMPLE = 0x0,
  LSM6DSV_SLEEP_TO_ACT_AT_2ND_SAMPLE = 0x1,
  LSM6DSV_SLEEP_TO_ACT_AT_3RD_SAMPLE = 0x2,
  LSM6DSV_SLEEP_TO_ACT_AT_4th_SAMPLE = 0x3,
} lsm6dsv_act_from_sleep_to_act_dur_t;
int32_t lsm6dsv_act_from_sleep_to_act_dur_set(stmdev_ctx_t *ctx,
                                              lsm6dsv_act_from_sleep_to_act_dur_t val);
int32_t lsm6dsv_act_from_sleep_to_act_dur_get(stmdev_ctx_t *ctx,
                                              lsm6dsv_act_from_sleep_to_act_dur_t *val);

typedef enum
{
  LSM6DSV_1Hz875 = 0x0,
  LSM6DSV_15Hz   = 0x1,
  LSM6DSV_30Hz   = 0x2,
  LSM6DSV_60Hz   = 0x3,
} lsm6dsv_act_sleep_xl_odr_t;
int32_t lsm6dsv_act_sleep_xl_odr_set(stmdev_ctx_t *ctx,
                                     lsm6dsv_act_sleep_xl_odr_t val);
int32_t lsm6dsv_act_sleep_xl_odr_get(stmdev_ctx_t *ctx,
                                     lsm6dsv_act_sleep_xl_odr_t *val);

typedef struct
{
  lsm6dsv_inactivity_dur_t inactivity_cfg;
  uint8_t inactivity_ths;
  uint8_t threshold;
  uint8_t duration;
} lsm6dsv_act_thresholds_t;
int32_t lsm6dsv_act_thresholds_set(stmdev_ctx_t *ctx,
                                   lsm6dsv_act_thresholds_t *val);
int32_t lsm6dsv_act_thresholds_get(stmdev_ctx_t *ctx,
                                   lsm6dsv_act_thresholds_t *val);

typedef struct
{
  uint8_t shock                : 2;
  uint8_t quiet                : 4;
} lsm6dsv_act_wkup_time_windows_t;
int32_t lsm6dsv_act_wkup_time_windows_set(stmdev_ctx_t *ctx,
                                          lsm6dsv_act_wkup_time_windows_t val);
int32_t lsm6dsv_act_wkup_time_windows_get(stmdev_ctx_t *ctx,
                                          lsm6dsv_act_wkup_time_windows_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*LSM6DSV_DRIVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
