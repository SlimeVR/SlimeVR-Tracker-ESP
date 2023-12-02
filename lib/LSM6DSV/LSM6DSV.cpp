/**
 ******************************************************************************
 * @file    LSM6DSV.cpp
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    July 2022
 * @brief   Implementation of a LSM6DSV inertial measurement sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "LSM6DSV.h"


/* Class Implementation ------------------------------------------------------*/
/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LSM6DSV::LSM6DSV(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  reg_ctx.write_reg = LSM6DSV_io_write;
  reg_ctx.read_reg = LSM6DSV_io_read;
  reg_ctx.mdelay = LSM6DSV_sleep;
  reg_ctx.handle = (void *)this;
  dev_spi = NULL;
  acc_is_enabled = 0L;
  gyro_is_enabled = 0L;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LSM6DSV::LSM6DSV(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LSM6DSV_io_write;
  reg_ctx.read_reg = LSM6DSV_io_read;
  reg_ctx.mdelay = LSM6DSV_sleep;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  acc_is_enabled = 0L;
  gyro_is_enabled = 0L;
}

/**
 * @brief  Initialize the LSM6DSV sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::begin()
{
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }

  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (lsm6dsv_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable BDU */
  if (Enable_Block_Data_Update() != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* FIFO mode selection */
  if (lsm6dsv_fifo_mode_set(&reg_ctx, LSM6DSV_BYPASS_MODE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Select default output data rate. */
  acc_odr = LSM6DSV_ODR_AT_120Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsv_xl_data_rate_set(&reg_ctx, LSM6DSV_ODR_OFF) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsv_xl_full_scale_set(&reg_ctx, LSM6DSV_2g) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Select default output data rate. */
  gyro_odr = LSM6DSV_ODR_AT_120Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsv_gy_data_rate_set(&reg_ctx, LSM6DSV_ODR_OFF) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsv_gy_full_scale_set(&reg_ctx, LSM6DSV_2000dps) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  initialized = 1U;

  return LSM6DSV_OK;
}

/**
 * @brief  Deinitialize the LSM6DSV sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::end()
{
  /* Disable the component */
  if (Disable_X() != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (Disable_G() != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset output data rate. */
  acc_odr = LSM6DSV_ODR_OFF;
  gyro_odr = LSM6DSV_ODR_OFF;

  initialized = 0U;

  return LSM6DSV_OK;
}

/**
 * @brief  Read component ID
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::ReadID(uint8_t *Id)
{
  if (lsm6dsv_device_id_get(&reg_ctx, Id) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Enable the LSM6DSV accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_X()
{
  /* Check if the component is already enabled */
  if (acc_is_enabled == 1U) {
    return LSM6DSV_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsv_xl_data_rate_set(&reg_ctx, acc_odr) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  acc_is_enabled = 1U;

  return LSM6DSV_OK;
}

/**
 * @brief  Disable the LSM6DSV accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_X()
{
  /* Check if the component is already disabled */
  if (acc_is_enabled == 0U) {
    return LSM6DSV_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsv_xl_data_rate_get(&reg_ctx, &acc_odr) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsv_xl_data_rate_set(&reg_ctx, LSM6DSV_ODR_OFF) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  acc_is_enabled = 0U;

  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV accelerometer sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_X_Sensitivity(float *Sensitivity)
{
  lsm6dsv_xl_full_scale_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsv_xl_full_scale_get(&reg_ctx, &full_scale) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *Sensitivity = Convert_X_Sensitivity(full_scale);
  if (*Sensitivity == 0.0f) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV accelerometer sensor from type lsm6dsv_xl_full_scale_t
 * @param  full_scale enum with type lsm6dsv_xl_full_scale_t to be converted
 * @retval sensitivity as float
 */
float LSM6DSV::Convert_X_Sensitivity(lsm6dsv_xl_full_scale_t full_scale) {
  float Sensitivity = 0.0f;
  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSV_2g:
      Sensitivity = LSM6DSV_ACC_SENSITIVITY_FS_2G;
      break;

    case LSM6DSV_4g:
      Sensitivity = LSM6DSV_ACC_SENSITIVITY_FS_4G;
      break;

    case LSM6DSV_8g:
      Sensitivity = LSM6DSV_ACC_SENSITIVITY_FS_8G;
      break;

    case LSM6DSV_16g:
      Sensitivity = LSM6DSV_ACC_SENSITIVITY_FS_16G;
      break;
  }
  return Sensitivity;
}

/**
 * @brief  Get the LSM6DSV accelerometer sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_X_ODR(float *Odr)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_data_rate_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsv_xl_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  switch (odr_low_level) {
    case LSM6DSV_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSV_ODR_AT_1Hz875:
      *Odr = 1.875f;
      break;

    case LSM6DSV_ODR_AT_7Hz5:
      *Odr = 7.5f;
      break;

    case LSM6DSV_ODR_AT_15Hz:
      *Odr = 15.0f;
      break;

    case LSM6DSV_ODR_AT_30Hz:
      *Odr = 30.0f;
      break;

    case LSM6DSV_ODR_AT_60Hz:
      *Odr = 60.0f;
      break;

    case LSM6DSV_ODR_AT_120Hz:
      *Odr = 120.0f;
      break;

    case LSM6DSV_ODR_AT_240Hz:
      *Odr = 240.0f;
      break;

    case LSM6DSV_ODR_AT_480Hz:
      *Odr = 480.0f;
      break;

    case LSM6DSV_ODR_AT_960Hz:
      *Odr = 960.0f;
      break;

    case LSM6DSV_ODR_AT_1920Hz:
      *Odr = 1920.0f;
      break;

    case LSM6DSV_ODR_AT_3840Hz:
      *Odr = 3840.0f;
      break;

    case LSM6DSV_ODR_AT_7680Hz:
      *Odr = 7680.0f;
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSV accelerometer sensor axes
 * @param  Acceleration pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_X_Axes(int32_t *Acceleration)
{
  lsm6dsv_axis3bit16_t data_raw;
  float sensitivity = Convert_X_Sensitivity(acc_fs);

  /* Read raw data values. */
  if (lsm6dsv_acceleration_raw_get(&reg_ctx, data_raw.i16bit) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Get LSM6DSV actual sensitivity. */
  if (sensitivity == 0.0f) {
    return LSM6DSV_ERROR;
  }

  /* Calculate the data. */
  Acceleration[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  Acceleration[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  Acceleration[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSV_OK;
}

/**
 * @brief  Set the LSM6DSV accelerometer sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_X_ODR(float Odr, LSM6DSV_ACC_Operating_Mode_t Mode)
{
  switch (Mode) {
    case LSM6DSV_ACC_HIGH_PERFORMANCE_MODE: {
        if (lsm6dsv_xl_mode_set(&reg_ctx, LSM6DSV_XL_HIGH_PERFORMANCE_MD) != LSM6DSV_OK) {
          return LSM6DSV_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 7.68kHz */
        Odr = (Odr <    7.5f) ?    7.5f
              : (Odr > 7680.0f) ? 7680.0f
              :                       Odr;
        break;
      }

    case LSM6DSV_ACC_HIGH_ACCURACY_MODE:
      // TODO: Not implemented.
      // NOTE: According to datasheet, section `6.5 High-accuracy ODR mode`:
      // "... the other sensor also has to be configured in high-accuracy ODR (HAODR) mode."
      return LSM6DSV_ERROR;

    case LSM6DSV_ACC_NORMAL_MODE: {
        if (lsm6dsv_xl_mode_set(&reg_ctx, LSM6DSV_XL_NORMAL_MD) != LSM6DSV_OK) {
          return LSM6DSV_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 1.92kHz */
        Odr = (Odr <    7.5f) ?    7.5f
              : (Odr > 1920.0f) ? 1920.0f
              :                       Odr;
        break;
      }

    case LSM6DSV_ACC_LOW_POWER_MODE1: {
        if (lsm6dsv_xl_mode_set(&reg_ctx, LSM6DSV_XL_LOW_POWER_2_AVG_MD) != LSM6DSV_OK) {
          return LSM6DSV_ERROR;
        }

        /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
        Odr = (Odr ==   1.875f) ?    Odr
              : (Odr <   15.000f) ?  15.0f
              : (Odr >  240.000f) ? 240.0f
              :                        Odr;
        break;
      }

    case LSM6DSV_ACC_LOW_POWER_MODE2: {
        if (lsm6dsv_xl_mode_set(&reg_ctx, LSM6DSV_XL_LOW_POWER_4_AVG_MD) != LSM6DSV_OK) {
          return LSM6DSV_ERROR;
        }

        /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
        Odr = (Odr ==   1.875f) ?    Odr
              : (Odr <   15.000f) ?  15.0f
              : (Odr >  240.000f) ? 240.0f
              :                        Odr;
        break;
      }

    case LSM6DSV_ACC_LOW_POWER_MODE3: {
        if (lsm6dsv_xl_mode_set(&reg_ctx, LSM6DSV_XL_LOW_POWER_8_AVG_MD) != LSM6DSV_OK) {
          return LSM6DSV_ERROR;
        }

        /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
        Odr = (Odr ==   1.875f) ?    Odr
              : (Odr <   15.000f) ?  15.0f
              : (Odr >  240.000f) ? 240.0f
              :                        Odr;
        break;
      }

    default:
      return LSM6DSV_ERROR;
  }

  if (acc_is_enabled == 1U) {
    return Set_X_ODR_When_Enabled(Odr);
  } else {
    return Set_X_ODR_When_Disabled(Odr);
  }
}

/**
 * @brief  Set the LSM6DSV accelerometer sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_X_ODR_When_Enabled(float Odr)
{
  lsm6dsv_data_rate_t new_odr;

  new_odr = (Odr <=    1.875f) ? LSM6DSV_ODR_AT_1Hz875
            : (Odr <=    7.5f) ? LSM6DSV_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? LSM6DSV_ODR_AT_15Hz
            : (Odr <=   30.0f) ? LSM6DSV_ODR_AT_30Hz
            : (Odr <=   60.0f) ? LSM6DSV_ODR_AT_60Hz
            : (Odr <=  120.0f) ? LSM6DSV_ODR_AT_120Hz
            : (Odr <=  240.0f) ? LSM6DSV_ODR_AT_240Hz
            : (Odr <=  480.0f) ? LSM6DSV_ODR_AT_480Hz
            : (Odr <=  960.0f) ? LSM6DSV_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? LSM6DSV_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? LSM6DSV_ODR_AT_3840Hz
            :                    LSM6DSV_ODR_AT_7680Hz;

  /* Output data rate selection. */
  if (lsm6dsv_xl_data_rate_set(&reg_ctx, new_odr) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set the LSM6DSV accelerometer sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_X_ODR_When_Disabled(float Odr)
{
  acc_odr = (Odr <=    1.875f) ? LSM6DSV_ODR_AT_1Hz875
            : (Odr <=    7.5f) ? LSM6DSV_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? LSM6DSV_ODR_AT_15Hz
            : (Odr <=   30.0f) ? LSM6DSV_ODR_AT_30Hz
            : (Odr <=   60.0f) ? LSM6DSV_ODR_AT_60Hz
            : (Odr <=  120.0f) ? LSM6DSV_ODR_AT_120Hz
            : (Odr <=  240.0f) ? LSM6DSV_ODR_AT_240Hz
            : (Odr <=  480.0f) ? LSM6DSV_ODR_AT_480Hz
            : (Odr <=  960.0f) ? LSM6DSV_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? LSM6DSV_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? LSM6DSV_ODR_AT_3840Hz
            :                    LSM6DSV_ODR_AT_7680Hz;

  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV accelerometer sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_X_FS(int32_t *FullScale)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_xl_full_scale_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsv_xl_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  switch (fs_low_level) {
    case LSM6DSV_2g:
      *FullScale =  2;
      break;

    case LSM6DSV_4g:
      *FullScale =  4;
      break;

    case LSM6DSV_8g:
      *FullScale =  8;
      break;

    case LSM6DSV_16g:
      *FullScale = 16;
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSV accelerometer sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_X_FS(int32_t FullScale)
{
  lsm6dsv_xl_full_scale_t new_fs;

  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  new_fs = (FullScale <= 2) ? LSM6DSV_2g
           : (FullScale <= 4) ? LSM6DSV_4g
           : (FullScale <= 8) ? LSM6DSV_8g
           :                    LSM6DSV_16g;

  if (new_fs == acc_fs) {
    return LSM6DSV_OK;
  }
  acc_fs = new_fs;

  if (lsm6dsv_xl_full_scale_set(&reg_ctx, acc_fs) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV accelerometer sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_X_AxesRaw(int16_t *Value)
{
  lsm6dsv_axis3bit16_t data_raw;

  /* Read raw data values. */
  if (lsm6dsv_acceleration_raw_get(&reg_ctx, data_raw.i16bit) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV ACC data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_X_DRDY_Status(uint8_t *Status)
{
  lsm6dsv_all_sources_t val;

  if (lsm6dsv_all_sources_get(&reg_ctx, &val) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *Status = val.drdy_xl;
  return LSM6DSV_OK;
}

/**
 * @brief  Get the status of all hardware events
 * @param  Status the status of all hardware events
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_X_Event_Status(LSM6DSV_Event_Status_t *Status)
{
  lsm6dsv_emb_func_status_t emb_func_status;
  lsm6dsv_wake_up_src_t wake_up_src;
  lsm6dsv_tap_src_t tap_src;
  lsm6dsv_d6d_src_t d6d_src;
  lsm6dsv_emb_func_src_t func_src;
  lsm6dsv_md1_cfg_t md1_cfg;
  lsm6dsv_md2_cfg_t md2_cfg;
  lsm6dsv_emb_func_int1_t int1_ctrl;
  lsm6dsv_emb_func_int2_t int2_ctrl;


  (void)memset((void *)Status, 0x0, sizeof(LSM6DSV_Event_Status_t));

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_WAKE_UP_SRC, (uint8_t *)&wake_up_src, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_SRC, (uint8_t *)&tap_src, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_D6D_SRC, (uint8_t *)&d6d_src, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_EMBED_FUNC_MEM_BANK) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_SRC, (uint8_t *)&func_src, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT1, (uint8_t *)&int1_ctrl, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT2, (uint8_t *)&int2_ctrl, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_STATUS, (uint8_t *)&emb_func_status, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_MAIN_MEM_BANK) != 0) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&md1_cfg, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&md2_cfg, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }


  if ((md1_cfg.int1_ff == 1U) || (md2_cfg.int2_ff == 1U)) {
    if (wake_up_src.ff_ia == 1U) {
      Status->FreeFallStatus = 1;
    }
  }

  if ((md1_cfg.int1_wu == 1U) || (md2_cfg.int2_wu == 1U)) {
    if (wake_up_src.wu_ia == 1U) {
      Status->WakeUpStatus = 1;
    }
  }

  if ((md1_cfg.int1_single_tap == 1U) || (md2_cfg.int2_single_tap == 1U)) {
    if (tap_src.single_tap == 1U) {
      Status->TapStatus = 1;
    }
  }

  if ((md1_cfg.int1_double_tap == 1U) || (md2_cfg.int2_double_tap == 1U)) {
    if (tap_src.double_tap == 1U) {
      Status->DoubleTapStatus = 1;
    }
  }

  if ((md1_cfg.int1_6d == 1U) || (md2_cfg.int2_6d == 1U)) {
    if (d6d_src.d6d_ia == 1U) {
      Status->D6DOrientationStatus = 1;
    }
  }

  if (int1_ctrl.int1_step_detector == 1U || int2_ctrl.int2_step_detector == 1U) {
    if (func_src.step_detected == 1U) {
      Status->StepStatus = 1;
    }
  }

  if ((int1_ctrl.int1_tilt == 1U) || (int2_ctrl.int2_tilt == 1U)) {
    if (emb_func_status.is_tilt == 1U) {
      Status->TiltStatus = 1;
    }
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set the LSM6DSV accelerometer power mode
 * @param  PowerMode Value of the powerMode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_X_Power_Mode(uint8_t PowerMode)
{
  if (lsm6dsv_xl_mode_set(&reg_ctx, (lsm6dsv_xl_mode_t)PowerMode) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set the LSM6DSV accelerometer filter mode
 * @param  LowHighPassFlag 0/1 for setting low-pass/high-pass filter mode
 * @param  FilterMode Value of the filter Mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_X_Filter_Mode(uint8_t LowHighPassFlag, uint8_t FilterMode)
{
  if (LowHighPassFlag == 0) {
    /*Set accelerometer low_pass filter-mode*/

    /*Set to 1 LPF2 bit (CTRL8_XL)*/
    if (lsm6dsv_filt_xl_lp2_set(&reg_ctx, 1) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
    if (lsm6dsv_filt_xl_lp2_bandwidth_set(&reg_ctx, (lsm6dsv_filt_xl_lp2_bandwidth_t)FilterMode) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
  } else {
    if (lsm6dsv_filt_xl_lp2_set(&reg_ctx, 0) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
    /*Set accelerometer high_pass filter-mode*/
    if (lsm6dsv_filt_xl_lp2_bandwidth_set(&reg_ctx, (lsm6dsv_filt_xl_lp2_bandwidth_t)FilterMode) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Enable acceleration offset
 * @param  enable the acceleration offset
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_X_User_Offset()
{
  lsm6dsv_ctrl9_t ctrl9;
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_CTRL9, (uint8_t *)&ctrl9, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  ctrl9.usr_off_on_out = PROPERTY_ENABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_CTRL9, (uint8_t *)&ctrl9, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Disable acceleration offset
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_X_User_Offset()
{
  lsm6dsv_ctrl9_t ctrl9;
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_CTRL9, (uint8_t *)&ctrl9, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  ctrl9.usr_off_on_out = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_CTRL9, (uint8_t *)&ctrl9, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Set the offset values for the acceleration
 * @param  x the acceleration offset in the x axis to be set
 * @param  y the acceleration offset in the y axis to be set
 * @param  z the acceleration offset in the z axis to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_X_User_Offset(float x, float y, float z)
{
  lsm6dsv_ctrl9_t ctrl9;
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_CTRL9, (uint8_t *)&ctrl9, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  int8_t xyz[3];

  if ( // about +- 2 G's for high and +- 0.124 G's for low
      (x <= LSM6DSV_ACC_USR_OFF_W_LOW_MAX && x >= -LSM6DSV_ACC_USR_OFF_W_LOW_MAX) &&
      (y <= LSM6DSV_ACC_USR_OFF_W_LOW_MAX && y >= -LSM6DSV_ACC_USR_OFF_W_LOW_MAX) &&
      (z <= LSM6DSV_ACC_USR_OFF_W_LOW_MAX && z >= -LSM6DSV_ACC_USR_OFF_W_LOW_MAX)) { // Then we are under the low requirements
    xyz[0] = (int8_t)(x / LSM6DSV_ACC_USR_OFF_W_LOW_LSB);
    xyz[1] = (int8_t)(y / LSM6DSV_ACC_USR_OFF_W_LOW_LSB);
    xyz[2] = (int8_t)(z / LSM6DSV_ACC_USR_OFF_W_LOW_LSB);
    ctrl9.usr_off_w = false; //(0: 2^-10 g/LSB; 1: 2^-6 g/LSB)
  }

  else if ( // about +- 2 G's for high and +- 0.124 G's for low
      (x <= LSM6DSV_ACC_USR_OFF_W_HIGH_MAX && x >= -LSM6DSV_ACC_USR_OFF_W_HIGH_MAX) &&
      (y <= LSM6DSV_ACC_USR_OFF_W_HIGH_MAX && y >= -LSM6DSV_ACC_USR_OFF_W_HIGH_MAX) &&
      (z <= LSM6DSV_ACC_USR_OFF_W_HIGH_MAX && z >= -LSM6DSV_ACC_USR_OFF_W_HIGH_MAX)) { // Then we are under the high requirements
    xyz[0] = (int8_t)(x / LSM6DSV_ACC_USR_OFF_W_HIGH_LSB);
    xyz[1] = (int8_t)(y / LSM6DSV_ACC_USR_OFF_W_HIGH_LSB);
    xyz[2] = (int8_t)(z / LSM6DSV_ACC_USR_OFF_W_HIGH_LSB);
    ctrl9.usr_off_w = true; //(0: 2^-10 g/LSB; 1: 2^-6 g/LSB)
  }
  else {
    return LSM6DSV_ERROR; // Value too big
  }

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_CTRL9, (uint8_t *)&ctrl9, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  // convert from float in G's to what it wants. Signed byte
  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_X_OFS_USR, (uint8_t *)&xyz, 3) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Enable 6D orientation detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_6D_Orientation(LSM6DSV_SensorIntPin_t IntPin)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;
  lsm6dsv_functions_enable_t functions_enable;

  /* Output Data Rate selection */
  if (Set_X_ODR(480.0f) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* 6D orientation enabled. */
  if (Set_6D_Orientation_Threshold(2) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  /* Enable 6D orientation event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV_INT1_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val1.int1_6d = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    case LSM6DSV_INT2_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val2.int2_6d = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable 6D orientation detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_6D_Orientation()
{
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;

  /* Reset threshold */
  if (Set_6D_Orientation_Threshold(0) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Disable 6D orientation event on both INT1 and INT2 pins */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val1.int1_6d = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val2.int2_6d = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set 6D orientation threshold
 * @param  Threshold 6D Orientation detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_6D_Orientation_Threshold(uint8_t Threshold)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_6d_threshold_t newThreshold = LSM6DSV_DEG_80;

  switch (Threshold) {
    case 0:
      newThreshold = LSM6DSV_DEG_80;
      break;
    case 1:
      newThreshold = LSM6DSV_DEG_70;
      break;
    case 2:
      newThreshold = LSM6DSV_DEG_60;
      break;
    case 3:
      newThreshold = LSM6DSV_DEG_50;
      break;
    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  if (ret == LSM6DSV_ERROR) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_6d_threshold_set(&reg_ctx, newThreshold) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;

}

/**
 * @brief  Get the status of XLow orientation
 * @param  XLow the status of XLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_6D_Orientation_XL(uint8_t *XLow)
{
  lsm6dsv_d6d_src_t data;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *XLow = data.xl;

  return LSM6DSV_OK;
}

/**
 * @brief  Get the status of XHigh orientation
 * @param  XHigh the status of XHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_6D_Orientation_XH(uint8_t *XHigh)
{
  lsm6dsv_d6d_src_t data;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *XHigh = data.xh;

  return LSM6DSV_OK;
}

/**
 * @brief  Get the status of YLow orientation
 * @param  YLow the status of YLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_6D_Orientation_YL(uint8_t *YLow)
{
  lsm6dsv_d6d_src_t data;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *YLow = data.yl;

  return LSM6DSV_OK;
}

/**
 * @brief  Get the status of YHigh orientation
 * @param  YHigh the status of YHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_6D_Orientation_YH(uint8_t *YHigh)
{
  lsm6dsv_d6d_src_t data;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *YHigh = data.yh;

  return LSM6DSV_OK;
}

/**
 * @brief  Get the status of ZLow orientation
 * @param  ZLow the status of ZLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_6D_Orientation_ZL(uint8_t *ZLow)
{
  lsm6dsv_d6d_src_t data;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *ZLow = data.zl;

  return LSM6DSV_OK;
}

/**
 * @brief  Get the status of ZHigh orientation
 * @param  ZHigh the status of ZHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_6D_Orientation_ZH(uint8_t *ZHigh)
{
  lsm6dsv_d6d_src_t data;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *ZHigh = data.zh;

  return LSM6DSV_OK;
}


/**
 * @brief  Enable free fall detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Free_Fall_Detection(LSM6DSV_SensorIntPin_t IntPin)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;
  lsm6dsv_functions_enable_t functions_enable;

  /* Output Data Rate selection */
  if (Set_X_ODR(480) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /*  Set free fall duration.*/
  if (Set_Free_Fall_Duration(3) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set free fall threshold. */
  if (Set_Free_Fall_Threshold(3) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV_INT1_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val1.int1_ff = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    case LSM6DSV_INT2_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val2.int2_ff = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable free fall detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Free_Fall_Detection()
{
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;

  /* Disable free fall event on both INT1 and INT2 pins */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val1.int1_ff = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val2.int2_ff = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset free fall threshold. */
  if (Set_Free_Fall_Threshold(0) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset free fall duration */
  if (Set_Free_Fall_Duration(0) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set free fall threshold
 * @param  Threshold free fall detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_Free_Fall_Threshold(uint8_t Threshold)
{
  lsm6dsv_ff_thresholds_t val;
  switch (Threshold) {
    case LSM6DSV_156_mg:
      val = LSM6DSV_156_mg;
      break;

    case LSM6DSV_219_mg:
      val = LSM6DSV_219_mg;
      break;

    case LSM6DSV_250_mg:
      val = LSM6DSV_250_mg;
      break;

    case LSM6DSV_312_mg:
      val = LSM6DSV_312_mg;
      break;

    case LSM6DSV_344_mg:
      val = LSM6DSV_344_mg;
      break;

    case LSM6DSV_406_mg:
      val = LSM6DSV_406_mg;
      break;

    case LSM6DSV_469_mg:
      val = LSM6DSV_469_mg;
      break;

    case LSM6DSV_500_mg:
      val = LSM6DSV_500_mg;
      break;

    default:
      val = LSM6DSV_156_mg;
      break;
  }

  /* Set free fall threshold. */
  if (lsm6dsv_ff_thresholds_set(&reg_ctx, val) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}


/**
 * @brief  Set free fall duration
 * @param  Duration free fall detection duration
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_Free_Fall_Duration(uint8_t Duration)
{
  if (lsm6dsv_ff_time_windows_set(&reg_ctx, Duration) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Enable wake up detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Wake_Up_Detection(LSM6DSV_SensorIntPin_t IntPin)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;
  lsm6dsv_functions_enable_t functions_enable;

  /* Output Data Rate selection */
  if (Set_X_ODR(480) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set wake-up threshold */
  if (Set_Wake_Up_Threshold(63) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set wake-up durantion */
  if (Set_Wake_Up_Duration(0) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable wake up event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV_INT1_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val1.int1_wu = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    case LSM6DSV_INT2_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val2.int2_wu = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}


/**
 * @brief  Disable wake up detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Wake_Up_Detection()
{
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;

  /* Disable wake up event on both INT1 and INT2 pins */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val1.int1_wu = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val2.int2_wu = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset wake-up threshold */
  if (Set_Wake_Up_Threshold(0) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset wake-up durantion */
  if (Set_Wake_Up_Duration(0) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set wake up threshold
 * @param  Threshold wake up detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_Wake_Up_Threshold(uint32_t Threshold)
{
  lsm6dsv_act_thresholds_t wake_up_ths;

  if (lsm6dsv_act_thresholds_get(&reg_ctx, &wake_up_ths) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  wake_up_ths.threshold = Threshold;

  if (lsm6dsv_act_thresholds_set(&reg_ctx, &wake_up_ths) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set wake up duration
 * @param  Duration wake up detection duration
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_Wake_Up_Duration(uint8_t Duration)
{
  lsm6dsv_act_wkup_time_windows_t dur_t;

  if (lsm6dsv_act_wkup_time_windows_get(&reg_ctx, &dur_t) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  dur_t.shock = Duration;

  if (lsm6dsv_act_wkup_time_windows_set(&reg_ctx, dur_t) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Enable single tap detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Single_Tap_Detection(LSM6DSV_SensorIntPin_t IntPin)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;
  lsm6dsv_functions_enable_t functions_enable;

  lsm6dsv_tap_dur_t tap_dur;
  lsm6dsv_tap_cfg0_t tap_cfg0;
  lsm6dsv_tap_ths_6d_t tap_ths_6d;

  /* Output Data Rate selection */
  if (Set_X_ODR(480) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(8) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable tap detection on Z-axis. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_cfg0.tap_z_en = 0x01U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set Z-axis threshold. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_ths_6d.tap_ths_z = 0x2U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set quiet and shock time windows. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_dur.quiet = (uint8_t)0x02U;
  tap_dur.shock = (uint8_t)0x01U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set tap mode. */
  if (lsm6dsv_tap_mode_set(&reg_ctx, LSM6DSV_ONLY_SINGLE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable single tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV_INT1_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val1.int1_single_tap = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    case LSM6DSV_INT2_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val2.int2_single_tap = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable single tap detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Single_Tap_Detection()
{
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;
  lsm6dsv_tap_dur_t tap_dur;
  lsm6dsv_tap_cfg0_t tap_cfg0;
  lsm6dsv_tap_ths_6d_t tap_ths_6d;


  /* Disable single tap event on both INT1 and INT2 pins */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val1.int1_single_tap = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val2.int2_single_tap = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Disable tap detection on Z-axis. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_cfg0.tap_z_en = 0x0U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset Z-axis threshold. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_ths_6d.tap_ths_z = 0x0U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset quiet and shock time windows. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_dur.quiet = (uint8_t)0x0U;
  tap_dur.shock = (uint8_t)0x0U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Enable double tap detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Double_Tap_Detection(LSM6DSV_SensorIntPin_t IntPin)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;
  lsm6dsv_functions_enable_t functions_enable;

  lsm6dsv_tap_dur_t tap_dur;
  lsm6dsv_tap_cfg0_t tap_cfg0;
  lsm6dsv_tap_ths_6d_t tap_ths_6d;


  /* Enable tap detection on Z-axis. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_cfg0.tap_z_en = 0x01U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set Z-axis threshold. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_ths_6d.tap_ths_z = 0x03U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set quiet shock and duration. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_dur.dur = (uint8_t)0x03U;
  tap_dur.quiet = (uint8_t)0x02U;
  tap_dur.shock = (uint8_t)0x02U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set tap mode. */
  if (lsm6dsv_tap_mode_set(&reg_ctx, LSM6DSV_BOTH_SINGLE_DOUBLE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Output Data Rate selection */
  if (Set_X_ODR(480) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(8) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable double tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV_INT1_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val1.int1_double_tap = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    case LSM6DSV_INT2_PIN:
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val2.int2_double_tap = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable double tap detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Double_Tap_Detection()
{
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;

  lsm6dsv_tap_dur_t tap_dur;
  lsm6dsv_tap_cfg0_t tap_cfg0;
  lsm6dsv_tap_ths_6d_t tap_ths_6d;

  /* Disable double tap event on both INT1 and INT2 pins */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val1.int1_ff = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val2.int2_ff = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Disable tap detection on Z-axis. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_cfg0.tap_z_en = 0x0U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset Z-axis threshold. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_ths_6d.tap_ths_z = 0x0U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset quiet shock and duratio. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_dur.dur = (uint8_t)0x0U;
  tap_dur.quiet = (uint8_t)0x0U;
  tap_dur.shock = (uint8_t)0x0U;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Set tap mode. */
  if (lsm6dsv_tap_mode_set(&reg_ctx, LSM6DSV_ONLY_SINGLE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }


  return LSM6DSV_OK;
}

/**
 * @brief  Set tap threshold
 * @param  Threshold tap threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_Tap_Threshold(uint8_t Threshold)
{
  lsm6dsv_tap_ths_6d_t tap_ths_6d;

  /* Set Z-axis threshold */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_ths_6d.tap_ths_z = Threshold;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set tap shock time
 * @param  Time tap shock time
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_Tap_Shock_Time(uint8_t Time)
{
  lsm6dsv_tap_dur_t tap_dur;

  /* Set shock time */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_dur.shock = Time;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set tap quiet time
 * @param  Time tap quiet time
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_Tap_Quiet_Time(uint8_t Time)
{
  lsm6dsv_tap_dur_t tap_dur;

  /* Set quiet time */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_dur.quiet = Time;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Set tap duration time
 * @param  Time tap duration time
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_Tap_Duration_Time(uint8_t Time)
{
  lsm6dsv_tap_dur_t tap_dur;

  /* Set duration time */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  tap_dur.dur = Time;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Enable pedometer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Pedometer(LSM6DSV_SensorIntPin_t IntPin)
{
  lsm6dsv_stpcnt_mode_t mode;
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;
  lsm6dsv_emb_func_int1_t emb_func_int1;
  lsm6dsv_emb_func_int2_t emb_func_int2;

  /* Output Data Rate selection */
  if (Set_X_ODR(30) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_stpcnt_mode_get(&reg_ctx, &mode) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable pedometer algorithm. */
  mode.step_counter_enable = PROPERTY_ENABLE;

  /* Turn on embedded features */
  if (lsm6dsv_stpcnt_mode_set(&reg_ctx, mode) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV_INT1_PIN:
      /* Enable access to embedded functions registers. */
      if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_EMBED_FUNC_MEM_BANK) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Step detector interrupt driven to INT1 pin */
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      emb_func_int1.int1_step_detector = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Disable access to embedded functions registers */
      if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_MAIN_MEM_BANK) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val1.int1_emb_func = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    case LSM6DSV_INT2_PIN:
      /* Enable access to embedded functions registers. */
      if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_EMBED_FUNC_MEM_BANK) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Step detector interrupt driven to INT1 pin */
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      emb_func_int2.int2_step_detector = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Disable access to embedded functions registers */
      if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_MAIN_MEM_BANK) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val2.int2_emb_func = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    default:
      return LSM6DSV_ERROR;
      break;
  }

  return LSM6DSV_OK;
}


/**
 * @brief  Disable pedometer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Pedometer()
{
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;

  lsm6dsv_emb_func_int1_t emb_func_int1;
  lsm6dsv_emb_func_int2_t emb_func_int2;

  lsm6dsv_stpcnt_mode_t mode;


  if (lsm6dsv_stpcnt_mode_get(&reg_ctx, &mode) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable pedometer algorithm. */
  mode.step_counter_enable = PROPERTY_DISABLE;

  /* Turn off embedded features */
  if (lsm6dsv_stpcnt_mode_set(&reg_ctx, mode) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Disable emb func event on either INT1 or INT2 pin */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val1.int1_emb_func = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val2.int2_emb_func = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable access to embedded functions registers. */
  if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_EMBED_FUNC_MEM_BANK) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset interrupt driven to INT1 pin. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  emb_func_int1.int1_step_detector = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset interrupt driven to INT2 pin. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  emb_func_int2.int2_step_detector = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Disable access to embedded functions registers. */
  if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_MAIN_MEM_BANK) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }


  return LSM6DSV_OK;
}

/**
 * @brief  Get step count
 * @param  StepCount step counter
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_Step_Count(uint16_t *StepCount)
{
  if (lsm6dsv_stpcnt_steps_get(&reg_ctx, StepCount) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Enable step counter reset
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Step_Counter_Reset()
{
  if (lsm6dsv_stpcnt_rst_step_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Enable tilt detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Tilt_Detection(LSM6DSV_SensorIntPin_t IntPin)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;
  lsm6dsv_emb_func_en_a_t emb_func_en_a;
  lsm6dsv_emb_func_int1_t emb_func_int1;
  lsm6dsv_emb_func_int2_t emb_func_int2;

  /* Output Data Rate selection */
  if (Set_X_ODR(30) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  switch (IntPin) {
    case LSM6DSV_INT1_PIN:
      /* Enable access to embedded functions registers */
      if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_EMBED_FUNC_MEM_BANK) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Enable tilt detection */
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      emb_func_en_a.tilt_en = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Tilt interrupt driven to INT1 pin */
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      emb_func_int1.int1_tilt = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Disable access to embedded functions registers */
      if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_MAIN_MEM_BANK) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Enable routing the embedded functions interrupt */
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val1.int1_emb_func = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;


    case LSM6DSV_INT2_PIN:
      /* Enable access to embedded functions registers */
      if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_EMBED_FUNC_MEM_BANK) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Enable tilt detection */
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      emb_func_en_a.tilt_en = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Tilt interrupt driven to INT2 pin */
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      emb_func_int2.int2_tilt = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Disable access to embedded functions registers */
      if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_MAIN_MEM_BANK) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      /* Enable routing the embedded functions interrupt */
      if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }

      val2.int2_emb_func = PROPERTY_ENABLE;

      if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
        return LSM6DSV_ERROR;
      }
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable tilt detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Tilt_Detection()
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_md1_cfg_t val1;
  lsm6dsv_md2_cfg_t val2;

  lsm6dsv_emb_func_en_a_t emb_func_en_a;
  lsm6dsv_emb_func_int1_t emb_func_int1;
  lsm6dsv_emb_func_int2_t emb_func_int2;

  /* Disable emb func event on either INT1 or INT2 pin */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val1.int1_emb_func = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  val2.int2_emb_func = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Enable access to embedded functions registers. */
  if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_EMBED_FUNC_MEM_BANK) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Disable tilt detection. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  emb_func_en_a.tilt_en = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset interrupt driven to INT1 pin. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  emb_func_int1.int1_tilt = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Reset interrupt driven to INT2 pin. */
  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  emb_func_int2.int2_tilt = PROPERTY_DISABLE;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Disable access to embedded functions registers. */
  if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_MAIN_MEM_BANK) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return ret;
}

/**
 * @brief  Resets the fifo to an empty state
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Reset()
{
  if (lsm6dsv_fifo_mode_set(&reg_ctx, LSM6DSV_BYPASS_MODE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_fifo_mode_set(&reg_ctx, fifo_mode) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
  * @brief  Get the LSM6DSV FIFO number of samples

  * @param  NumSamples number of samples
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_Num_Samples(uint16_t *NumSamples)
{
  lsm6dsv_fifo_status_t status;
  if (lsm6dsv_fifo_status_get(&reg_ctx, &status) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  *NumSamples = status.fifo_level;

  return LSM6DSV_OK;
}

/**
  * @brief  Get the LSM6DSV FIFO full status

  * @param  Status FIFO full status
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_Full_Status(uint8_t *Status)
{
  lsm6dsv_fifo_status2_t val;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FIFO_STATUS2, (uint8_t *)&val, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *Status = val.fifo_full_ia;

  return LSM6DSV_OK;
}

/**
  * @brief  Set the LSM6DSV FIFO full interrupt on INT1 pin

  * @param  Status FIFO full interrupt on INT1 pin status
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Set_INT1_FIFO_Full(uint8_t Status)
{
  lsm6dsv_int1_ctrl_t reg;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_INT1_CTRL, (uint8_t *)&reg, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  reg.int1_fifo_full = Status;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_INT1_CTRL, (uint8_t *)&reg, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
  * @brief  Set the LSM6DSV FIFO full interrupt on INT2 pin

  * @param  Status FIFO full interrupt on INT1 pin status
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Set_INT2_FIFO_Full(uint8_t Status)
{
  lsm6dsv_int2_ctrl_t reg;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_INT2_CTRL, (uint8_t *)&reg, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  reg.int2_fifo_full = Status;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_INT2_CTRL, (uint8_t *)&reg, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
  * @brief  Set the LSM6DSV FIFO watermark level

  * @param  Watermark FIFO watermark level
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Set_Watermark_Level(uint8_t Watermark)
{
  return (LSM6DSVStatusTypeDef) lsm6dsv_fifo_watermark_set(&reg_ctx, Watermark);
}

/**
  * @brief  Set the LSM6DSV FIFO stop on watermark

  * @param  Status FIFO stop on watermark status
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Set_Stop_On_Fth(uint8_t Status)
{
  return (LSM6DSVStatusTypeDef) lsm6dsv_fifo_stop_on_wtm_set(&reg_ctx, Status);
}

/**
  * @brief  Set the LSM6DSV FIFO mode

  * @param  Mode FIFO mode
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Set_Mode(uint8_t Mode)
{
  lsm6dsv_fifo_mode_t newMode = LSM6DSV_BYPASS_MODE;

  switch (Mode) {
    case 0:
      newMode = LSM6DSV_BYPASS_MODE;
      break;
    case 1:
      newMode = LSM6DSV_FIFO_MODE;
      break;
    case 3:
      newMode = LSM6DSV_STREAM_TO_FIFO_MODE;
      break;
    case 4:
      newMode = LSM6DSV_BYPASS_TO_STREAM_MODE;
      break;
    case 6:
      newMode = LSM6DSV_STREAM_MODE;
      break;
    case 7:
      newMode = LSM6DSV_BYPASS_TO_FIFO_MODE;
      break;
    default:
    return LSM6DSV_ERROR;
  }
  fifo_mode = newMode;
  if (lsm6dsv_fifo_mode_set(&reg_ctx, fifo_mode) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
  * @brief  Get the LSM6DSV FIFO tag

  * @param  Tag FIFO tag
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_Tag(uint8_t *Tag)
{
  lsm6dsv_fifo_data_out_tag_t tag_local;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FIFO_DATA_OUT_TAG, (uint8_t *)&tag_local, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *Tag = (uint8_t)tag_local.tag_sensor;

  return LSM6DSV_OK;
}

/**
  * @brief  Get the LSM6DSV FIFO raw data

  * @param  Data FIFO raw data array [6]
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_Data(uint8_t *Data)
{
  return (LSM6DSVStatusTypeDef) lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FIFO_DATA_OUT_X_L, Data, 6);
}

/**
  * @brief  Get the LSM6DSV FIFO accelero single sample (16-bit data per 3 axes) and calculate acceleration [mg]
  * @param  Acceleration FIFO accelero axes [mg]
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_X_Axes(int32_t *Acceleration)
{
  lsm6dsv_axis3bit16_t data_raw;
  float sensitivity = Convert_X_Sensitivity(acc_fs);
  float acceleration_float[3];

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (sensitivity == 0.0f) {
    return LSM6DSV_ERROR;
  }
  acceleration_float[0] = (float)data_raw.i16bit[0] * sensitivity;
  acceleration_float[1] = (float)data_raw.i16bit[1] * sensitivity;
  acceleration_float[2] = (float)data_raw.i16bit[2] * sensitivity;

  Acceleration[0] = (int32_t)acceleration_float[0];
  Acceleration[1]  = (int32_t)acceleration_float[1];
  Acceleration[2]  = (int32_t)acceleration_float[2];

  return LSM6DSV_OK;

}

/**
  * @brief  Set the LSM6DSV FIFO accelero BDR value

  * @param  Bdr FIFO accelero BDR value
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Set_X_BDR(float Bdr)
{
  lsm6dsv_fifo_xl_batch_t new_bdr;

  new_bdr = (Bdr <=    0.0f) ? LSM6DSV_XL_NOT_BATCHED
            : (Bdr <=    1.8f) ? LSM6DSV_XL_BATCHED_AT_1Hz875
            : (Bdr <=    7.5f) ? LSM6DSV_XL_BATCHED_AT_7Hz5
            : (Bdr <=   15.0f) ? LSM6DSV_XL_BATCHED_AT_15Hz
            : (Bdr <=   30.0f) ? LSM6DSV_XL_BATCHED_AT_30Hz
            : (Bdr <=   60.0f) ? LSM6DSV_XL_BATCHED_AT_60Hz
            : (Bdr <=  120.0f) ? LSM6DSV_XL_BATCHED_AT_120Hz
            : (Bdr <=  240.0f) ? LSM6DSV_XL_BATCHED_AT_240Hz
            : (Bdr <=  480.0f) ? LSM6DSV_XL_BATCHED_AT_480Hz
            : (Bdr <=  960.0f) ? LSM6DSV_XL_BATCHED_AT_960Hz
            : (Bdr <=  1920.0f) ? LSM6DSV_XL_BATCHED_AT_1920Hz
            : (Bdr <= 3840.0f) ? LSM6DSV_XL_BATCHED_AT_3840Hz
            :                    LSM6DSV_XL_BATCHED_AT_7680Hz;

  return (LSM6DSVStatusTypeDef) lsm6dsv_fifo_xl_batch_set(&reg_ctx, new_bdr);
}

/**
  * @brief  Get the LSM6DSV FIFO gyro single sample (16-bit data per 3 axes) and calculate angular velocity [mDPS]

  * @param  AngularVelocity FIFO gyro axes [mDPS]
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_G_Axes(int32_t *AngularVelocity)
{
  lsm6dsv_axis3bit16_t data_raw;
  float sensitivity = Convert_G_Sensitivity(gyro_fs);
  float angular_velocity_float[3];

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (sensitivity == 0.0f) {
    return LSM6DSV_ERROR;
  }

  angular_velocity_float[0] = (float)data_raw.i16bit[0] * sensitivity;
  angular_velocity_float[1] = (float)data_raw.i16bit[1] * sensitivity;
  angular_velocity_float[2] = (float)data_raw.i16bit[2] * sensitivity;

  AngularVelocity[0] = (int32_t)angular_velocity_float[0];
  AngularVelocity[1] = (int32_t)angular_velocity_float[1];
  AngularVelocity[2] = (int32_t)angular_velocity_float[2];

  return LSM6DSV_OK;
}

/**
  * @brief  Set the LSM6DSV FIFO gyro BDR value
  * @param  Bdr FIFO gyro BDR value
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Set_G_BDR(float Bdr)
{
  lsm6dsv_fifo_gy_batch_t new_bdr;

  new_bdr = (Bdr <=    0.0f) ? LSM6DSV_GY_NOT_BATCHED
            : (Bdr <=    1.8f) ? LSM6DSV_GY_BATCHED_AT_1Hz875
            : (Bdr <=    7.5f) ? LSM6DSV_GY_BATCHED_AT_7Hz5
            : (Bdr <=   15.0f) ? LSM6DSV_GY_BATCHED_AT_15Hz
            : (Bdr <=   30.0f) ? LSM6DSV_GY_BATCHED_AT_30Hz
            : (Bdr <=   60.0f) ? LSM6DSV_GY_BATCHED_AT_60Hz
            : (Bdr <=  120.0f) ? LSM6DSV_GY_BATCHED_AT_120Hz
            : (Bdr <=  240.0f) ? LSM6DSV_GY_BATCHED_AT_240Hz
            : (Bdr <=  480.0f) ? LSM6DSV_GY_BATCHED_AT_480Hz
            : (Bdr <=  960.0f) ? LSM6DSV_GY_BATCHED_AT_960Hz
            : (Bdr <=  1920.0f) ? LSM6DSV_GY_BATCHED_AT_1920Hz
            : (Bdr <= 3840.0f) ? LSM6DSV_GY_BATCHED_AT_3840Hz
            :                    LSM6DSV_GY_BATCHED_AT_7680Hz;

  return (LSM6DSVStatusTypeDef) lsm6dsv_fifo_gy_batch_set(&reg_ctx, new_bdr);
}

/**
 * @brief  Get the LSM6DSV FIFO status
 * @param  Status pointer for FIFO status
 * @retval 0 in case of success, an error code otherwise
*/
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_Status(lsm6dsv_fifo_status_t *Status)
{
  return (LSM6DSVStatusTypeDef)lsm6dsv_fifo_status_get(&reg_ctx, Status);
}

/**
  * @brief  Get the Rotation Vector values
  * @param  rvec pointer where the Rotation Vector values are written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_Rotation_Vector(float *rvec)
{
  lsm6dsv_axis3bit16_t data_raw;

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  sflp2q(rvec, (uint16_t *)&data_raw.i16bit[0]);

  return LSM6DSV_OK;
}

/**
  * @brief  Get the Gravity Vector values
  * @param  gvec pointer where the Gravity Vector values are written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_Gravity_Vector(float *gvec)
{
  lsm6dsv_axis3bit16_t data_raw;

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  gvec[0] = lsm6dsv_from_sflp_to_mg(data_raw.i16bit[0]);
  gvec[1] = lsm6dsv_from_sflp_to_mg(data_raw.i16bit[1]);
  gvec[2] = lsm6dsv_from_sflp_to_mg(data_raw.i16bit[2]);

  return LSM6DSV_OK;
}

/**
  * @brief  Get the Gravity Bias values
  * @param  gbias pointer where the Gravity Bias values are written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_Gyroscope_Bias(float *gbias)
{
  lsm6dsv_axis3bit16_t data_raw;

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  gbias[0] = lsm6dsv_from_fs125_to_mdps(data_raw.i16bit[0]);
  gbias[1] = lsm6dsv_from_fs125_to_mdps(data_raw.i16bit[1]);
  gbias[2] = lsm6dsv_from_fs125_to_mdps(data_raw.i16bit[2]);

  return LSM6DSV_OK;
}

/**
 * @brief Enable the LSM6DSV FIFO Timestamp
 * @retval 0 in case of success, an error code otherwise
*/
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Enable_Timestamp()
{
  return (LSM6DSVStatusTypeDef)lsm6dsv_timestamp_set(&reg_ctx, PROPERTY_ENABLE);
}

/**
 * @brief Disable the LSM6DSV FIFO Timestamp
 * @retval 0 in case of success, an error code otherwise
*/
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Disable_Timestamp()
{
  return (LSM6DSVStatusTypeDef)lsm6dsv_timestamp_set(&reg_ctx, PROPERTY_DISABLE);
}

/**
 * @brief  Set the decimation of timestamp for how often FIFO will be updated with timestamp value 
 * @param  decimation FIFO Timestamp Decimation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Set_Timestamp_Decimation(uint8_t decimation)
{
  return (LSM6DSVStatusTypeDef)lsm6dsv_fifo_timestamp_batch_set(&reg_ctx, (lsm6dsv_fifo_timestamp_batch_t)decimation);
}

/**
 * @brief  Get the LSM6DSV FIFO Timestamp
 * @param  timestamp FIFO Timestamp
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::FIFO_Get_Timestamp(uint32_t *timestamp)
{
  uint32_t raw_data[2]; //first is timestamp second is half full of meta data
  if (FIFO_Get_Data((uint8_t *)raw_data) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  *timestamp = raw_data[0];
  return LSM6DSV_OK;
}

/**
 * @brief  Enable the LSM6DSV gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_G()
{
  /* Check if the component is already enabled */
  if (gyro_is_enabled == 1U) {
    return LSM6DSV_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsv_gy_data_rate_set(&reg_ctx, gyro_odr) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  gyro_is_enabled = 1U;

  return LSM6DSV_OK;
}

/**
 * @brief  Disable the LSM6DSV gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_G()
{
  /* Check if the component is already disabled */
  if (gyro_is_enabled == 0U) {
    return LSM6DSV_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsv_gy_data_rate_get(&reg_ctx, &gyro_odr) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsv_gy_data_rate_set(&reg_ctx, LSM6DSV_ODR_OFF) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  gyro_is_enabled = 0U;

  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV gyroscope sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_G_Sensitivity(float *Sensitivity)
{
  lsm6dsv_gy_full_scale_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsv_gy_full_scale_get(&reg_ctx, &full_scale) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *Sensitivity = Convert_G_Sensitivity(full_scale);
  if (*Sensitivity == 0.0f) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

float LSM6DSV::Convert_G_Sensitivity(lsm6dsv_gy_full_scale_t full_scale)
{
  float Sensitivity = 0.0f;
  /* Store the sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSV_125dps:
      Sensitivity = LSM6DSV_GYRO_SENSITIVITY_FS_125DPS;
      break;

    case LSM6DSV_250dps:
      Sensitivity = LSM6DSV_GYRO_SENSITIVITY_FS_250DPS;
      break;

    case LSM6DSV_500dps:
      Sensitivity = LSM6DSV_GYRO_SENSITIVITY_FS_500DPS;
      break;

    case LSM6DSV_1000dps:
      Sensitivity = LSM6DSV_GYRO_SENSITIVITY_FS_1000DPS;
      break;

    case LSM6DSV_2000dps:
      Sensitivity = LSM6DSV_GYRO_SENSITIVITY_FS_2000DPS;
      break;

    case LSM6DSV_4000dps:
      Sensitivity = LSM6DSV_GYRO_SENSITIVITY_FS_4000DPS;
      break;
  }
  return Sensitivity;
}

/**
 * @brief  Get the LSM6DSV gyroscope sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_G_ODR(float *Odr)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_data_rate_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsv_gy_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  switch (odr_low_level) {
    case LSM6DSV_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSV_ODR_AT_7Hz5:
      *Odr = 7.5f;
      break;

    case LSM6DSV_ODR_AT_15Hz:
      *Odr = 15.0f;
      break;

    case LSM6DSV_ODR_AT_30Hz:
      *Odr = 30.0f;
      break;

    case LSM6DSV_ODR_AT_60Hz:
      *Odr = 60.0f;
      break;

    case LSM6DSV_ODR_AT_120Hz:
      *Odr = 120.0f;
      break;

    case LSM6DSV_ODR_AT_240Hz:
      *Odr = 240.0f;
      break;

    case LSM6DSV_ODR_AT_480Hz:
      *Odr = 480.0f;
      break;

    case LSM6DSV_ODR_AT_960Hz:
      *Odr = 960.0f;
      break;

    case LSM6DSV_ODR_AT_1920Hz:
      *Odr = 1920.0f;
      break;

    case LSM6DSV_ODR_AT_3840Hz:
      *Odr = 3840.0f;
      break;

    case LSM6DSV_ODR_AT_7680Hz:
      *Odr = 7680.0f;
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSV gyroscope sensor output data rate with operating mode
 * @param  Odr the output data rate value to be set
 * @param  Mode the gyroscope operating mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_G_ODR(float Odr, LSM6DSV_GYRO_Operating_Mode_t Mode)
{
  switch (Mode) {
    case LSM6DSV_GYRO_HIGH_PERFORMANCE_MODE: {
        if (lsm6dsv_gy_mode_set(&reg_ctx, LSM6DSV_GY_HIGH_PERFORMANCE_MD) != LSM6DSV_OK) {
          return LSM6DSV_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 7.68kHz */
        Odr = (Odr <    7.5f) ?    7.5f
              : (Odr > 7680.0f) ? 7680.0f
              :                       Odr;
        break;
      }

    case LSM6DSV_GYRO_HIGH_ACCURACY_MODE:
      // TODO: Not implemented.
      // NOTE: According to datasheet, section `6.5 High-accuracy ODR mode`:
      // "... the other sensor also has to be configured in high-accuracy ODR (HAODR) mode."
      return LSM6DSV_ERROR;

    case LSM6DSV_GYRO_SLEEP_MODE:
      // TODO: Not implemented.
      // NOTE: Unknown ODR validity for this mode
      return LSM6DSV_ERROR;

    case LSM6DSV_GYRO_LOW_POWER_MODE: {
        if (lsm6dsv_gy_mode_set(&reg_ctx, LSM6DSV_GY_LOW_POWER_MD) != LSM6DSV_OK) {
          return LSM6DSV_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 240kHz */
        Odr = (Odr <   7.5f) ?   7.5f
              : (Odr > 240.0f) ? 240.0f
              :                     Odr;
        break;
      }

    default:
      return LSM6DSV_ERROR;
  }

  if (gyro_is_enabled == 1U) {
    return Set_G_ODR_When_Enabled(Odr);
  } else {
    return Set_G_ODR_When_Disabled(Odr);
  }
}

/**
 * @brief  Set the LSM6DSV gyroscope sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_G_ODR_When_Enabled(float Odr)
{
  lsm6dsv_data_rate_t new_odr;

  new_odr = (Odr <=    7.5f) ? LSM6DSV_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? LSM6DSV_ODR_AT_15Hz
            : (Odr <=   30.0f) ? LSM6DSV_ODR_AT_30Hz
            : (Odr <=   60.0f) ? LSM6DSV_ODR_AT_60Hz
            : (Odr <=  120.0f) ? LSM6DSV_ODR_AT_120Hz
            : (Odr <=  240.0f) ? LSM6DSV_ODR_AT_240Hz
            : (Odr <=  480.0f) ? LSM6DSV_ODR_AT_480Hz
            : (Odr <=  960.0f) ? LSM6DSV_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? LSM6DSV_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? LSM6DSV_ODR_AT_3840Hz
            :                    LSM6DSV_ODR_AT_7680Hz;

  return (LSM6DSVStatusTypeDef) lsm6dsv_gy_data_rate_set(&reg_ctx, new_odr);
}

/**
 * @brief  Set the LSM6DSV gyroscope sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_G_ODR_When_Disabled(float Odr)
{
  gyro_odr = (Odr <=    7.5f) ? LSM6DSV_ODR_AT_7Hz5
             : (Odr <=   15.0f) ? LSM6DSV_ODR_AT_15Hz
             : (Odr <=   30.0f) ? LSM6DSV_ODR_AT_30Hz
             : (Odr <=   60.0f) ? LSM6DSV_ODR_AT_60Hz
             : (Odr <=  120.0f) ? LSM6DSV_ODR_AT_120Hz
             : (Odr <=  240.0f) ? LSM6DSV_ODR_AT_240Hz
             : (Odr <=  480.0f) ? LSM6DSV_ODR_AT_480Hz
             : (Odr <=  960.0f) ? LSM6DSV_ODR_AT_960Hz
             : (Odr <= 1920.0f) ? LSM6DSV_ODR_AT_1920Hz
             : (Odr <= 3840.0f) ? LSM6DSV_ODR_AT_3840Hz
             :                    LSM6DSV_ODR_AT_7680Hz;

  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV gyroscope sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_G_FS(int32_t  *FullScale)
{
  LSM6DSVStatusTypeDef ret = LSM6DSV_OK;
  lsm6dsv_gy_full_scale_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsv_gy_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  switch (fs_low_level) {
    case LSM6DSV_125dps:
      *FullScale =  125;
      break;

    case LSM6DSV_250dps:
      *FullScale =  250;
      break;

    case LSM6DSV_500dps:
      *FullScale =  500;
      break;

    case LSM6DSV_1000dps:
      *FullScale = 1000;
      break;

    case LSM6DSV_2000dps:
      *FullScale = 2000;
      break;

    case LSM6DSV_4000dps:
      *FullScale = 4000;
      break;

    default:
      ret = LSM6DSV_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSV gyroscope sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_G_FS(int32_t FullScale)
{
  lsm6dsv_gy_full_scale_t new_fs;

  new_fs = (FullScale <= 125)  ? LSM6DSV_125dps
           : (FullScale <= 250)  ? LSM6DSV_250dps
           : (FullScale <= 500)  ? LSM6DSV_500dps
           : (FullScale <= 1000) ? LSM6DSV_1000dps
           : (FullScale <= 2000) ? LSM6DSV_2000dps
           :                       LSM6DSV_4000dps;

  if (new_fs == gyro_fs) {
    return LSM6DSV_OK;
  }
  gyro_fs = new_fs;

  return (LSM6DSVStatusTypeDef) lsm6dsv_gy_full_scale_set(&reg_ctx, gyro_fs);
}

/**
 * @brief  Get the LSM6DSV gyroscope sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_G_AxesRaw(int16_t *Value)
{
  lsm6dsv_axis3bit16_t data_raw;

  /* Read raw data values. */
  if (lsm6dsv_angular_rate_raw_get(&reg_ctx, data_raw.i16bit) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV gyroscope sensor axes
 * @param  AngularRate pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_G_Axes(int32_t *AngularRate)
{
  lsm6dsv_axis3bit16_t data_raw;
  float sensitivity = Convert_G_Sensitivity(gyro_fs);

  /* Read raw data values. */
  if (lsm6dsv_angular_rate_raw_get(&reg_ctx, data_raw.i16bit) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /* Get LSM6DSV actual sensitivity. */
  if (sensitivity == 0.0f) {
    return LSM6DSV_ERROR;
  }

  /* Calculate the data. */
  AngularRate[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  AngularRate[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  AngularRate[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV GYRO data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_G_DRDY_Status(uint8_t *Status)
{
  lsm6dsv_all_sources_t val;

  if (lsm6dsv_all_sources_get(&reg_ctx, &val) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  *Status = val.drdy_gy;
  return LSM6DSV_OK;
}

/**
 * @brief  Set the LSM6DSV gyroscope power mode
 * @param  PowerMode Value of the powerMode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_G_Power_Mode(uint8_t PowerMode)
{
  if (lsm6dsv_gy_mode_set(&reg_ctx, (lsm6dsv_gy_mode_t)PowerMode) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Set the LSM6DSV gyroscope filter mode
 * @param  LowHighPassFlag 0/1 for setting low-pass/high-pass filter mode
 * @param  FilterMode Value of the filter Mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_G_Filter_Mode(uint8_t LowHighPassFlag, uint8_t FilterMode)
{
  if (LowHighPassFlag == 0) {
    /*Set gyroscope low_pass 1 filter-mode*/
    /* Enable low-pass filter */
    if (lsm6dsv_filt_gy_lp1_set(&reg_ctx, 1) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
    if (lsm6dsv_filt_gy_lp1_bandwidth_set(&reg_ctx, (lsm6dsv_filt_gy_lp1_bandwidth_t)FilterMode) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
  } else {
    /*Set gyroscope high_pass filter-mode*/
    /* Enable high-pass filter */
    if (lsm6dsv_filt_gy_lp1_set(&reg_ctx, 0) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
    if (lsm6dsv_filt_gy_lp1_bandwidth_set(&reg_ctx, (lsm6dsv_filt_gy_lp1_bandwidth_t)FilterMode) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV temperature sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_Temp_ODR(float *Odr)
{
  lsm6dsv_fifo_ctrl4_t ctrl4;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FIFO_CTRL4, (uint8_t *)&ctrl4, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  switch (ctrl4.odr_t_batch) {
  case LSM6DSV_TEMP_NOT_BATCHED:
    *Odr = 0;
    break;
  case LSM6DSV_TEMP_BATCHED_AT_1Hz875:
    *Odr = 1.875f;
    break;
  case LSM6DSV_TEMP_BATCHED_AT_15Hz:
    *Odr = 15;
    break;
  case LSM6DSV_TEMP_BATCHED_AT_60Hz:
    *Odr = 60;
    break;
  default:
    break;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set the LSM6DSV temperature sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_Temp_ODR(float Odr)
{
  lsm6dsv_fifo_ctrl4_t ctrl4;

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_FIFO_CTRL4, (uint8_t *)&ctrl4, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (Odr == 0.0F) {
    ctrl4.odr_t_batch = LSM6DSV_TEMP_NOT_BATCHED;
  }
  else if (Odr <= 1.875F) {
    ctrl4.odr_t_batch = LSM6DSV_TEMP_BATCHED_AT_1Hz875;
  }
  else if (Odr <= 15.0F) {
    ctrl4.odr_t_batch = LSM6DSV_TEMP_BATCHED_AT_15Hz;
  }
  else {
    ctrl4.odr_t_batch = LSM6DSV_TEMP_BATCHED_AT_60Hz;
  }

  return (LSM6DSVStatusTypeDef)lsm6dsv_write_reg(
      &reg_ctx,
      LSM6DSV_FIFO_CTRL4,
      (uint8_t *)&ctrl4,
      1);
}

/**
 * @brief  Returns the raw temperature value from the imu
 * @param  value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_Temp_Raw(int16_t *value)
{
  return (LSM6DSVStatusTypeDef)lsm6dsv_temperature_raw_get(&reg_ctx, value);
}

/**
 * @brief  Runs the ST specified accelerometer and gyroscope self test
 * @param XTestType LSM6DSV_XL_ST_DISABLE  = 0x0, LSM6DSV_XL_ST_POSITIVE = 0x1, LSM6DSV_XL_ST_NEGATIVE = 0x2
 * @param GTestType LSM6DSV_GY_ST_DISABLE  = 0x0, LSM6DSV_GY_ST_POSITIVE = 0x1, LSM6DSV_GY_ST_NEGATIVE = 0x2
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Test_IMU(uint8_t XTestType, uint8_t GTestType)
{
  uint8_t whoamI;

  if (ReadID(&whoamI) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (whoamI != LSM6DSV_ID) {
    return LSM6DSV_ERROR;
  }

  if (Test_X_IMU(XTestType) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (Test_G_IMU(GTestType) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Runs the ST specified accelerometer self test
 * @param TestType LSM6DSV_XL_ST_DISABLE  = 0x0, LSM6DSV_XL_ST_POSITIVE = 0x1, LSM6DSV_XL_ST_NEGATIVE = 0x2
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Test_X_IMU(uint8_t TestType)
{
  int16_t data_raw[3];
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];

  if (Device_Reset(LSM6DSV_RESET_CTRL_REGS) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /*
   * Accelerometer Self Test
   */
  if (lsm6dsv_xl_data_rate_set(&reg_ctx, LSM6DSV_ODR_AT_60Hz) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_xl_full_scale_set(&reg_ctx, LSM6DSV_4g) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  delay(100); // Wait for Accelerometer to stabalize;
  memset(val_st_off, 0x00, 3 * sizeof(float));
  memset(val_st_on, 0x00, 3 * sizeof(float));

  /*Ignore First Data*/
  if (Get_X_AxesRaw_When_Aval(data_raw) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  for (uint8_t i = 0; i < 5; i++) {
    if (Get_X_AxesRaw_When_Aval(data_raw) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }

    /*Average the data in each axis*/
    for (uint8_t j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsv_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (uint8_t i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  if (lsm6dsv_xl_self_test_set(&reg_ctx, (lsm6dsv_xl_self_test_t)TestType) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  delay(100);

  /*Ignore First Data*/
  if (Get_X_AxesRaw_When_Aval(data_raw) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  for (uint8_t i = 0; i < 5; i++) {
    if (Get_X_AxesRaw_When_Aval(data_raw) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }

    /*Average the data in each axis*/
    for (uint8_t j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsv_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (uint8_t i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (uint8_t i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  for (uint8_t i = 0; i < 3; i++) {
    if ((LSM6DSV_MIN_ST_LIMIT_mg > test_val[i]) || (test_val[i] > LSM6DSV_MAX_ST_LIMIT_mg))
      return LSM6DSV_ERROR;
  }

  if (lsm6dsv_xl_self_test_set(&reg_ctx, LSM6DSV_XL_ST_DISABLE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_xl_data_rate_set(&reg_ctx, LSM6DSV_ODR_OFF) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV accelerometer sensor raw axes when avaiable (Blocking)
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_X_AxesRaw_When_Aval(int16_t *Value)
{
  lsm6dsv_data_ready_t drdy;
  do {
    if (lsm6dsv_flag_data_ready_get(&reg_ctx, &drdy) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
  } while (!drdy.drdy_xl);

  if (Get_X_AxesRaw(Value) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Runs the ST specified self test on the acceleration axis of the IMU
 * @param TestType LSM6DSV_GY_ST_DISABLE  = 0x0, LSM6DSV_GY_ST_POSITIVE = 0x1, LSM6DSV_GY_ST_NEGATIVE = 0x2
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Test_G_IMU(uint8_t TestType = LSM6DSV_GY_ST_POSITIVE)
{
  int16_t data_raw[3];
  float test_val[3];

  if (Device_Reset(LSM6DSV_RESET_CTRL_REGS) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  /*
   * Gyroscope Self Test
   */

  if (lsm6dsv_gy_data_rate_set(&reg_ctx, LSM6DSV_ODR_AT_240Hz) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_gy_full_scale_set(&reg_ctx, LSM6DSV_2000dps) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  delay(100);

  /*Ignore First Data*/
  if (Get_G_AxesRaw_When_Aval(data_raw) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  float val_st_off[3] = {0};
  float val_st_on[3] = {0};

  for (uint8_t i = 0; i < 5; i++) {
    if (Get_G_AxesRaw_When_Aval(data_raw) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }

    /*Average the data in each axis*/
    for (uint8_t j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsv_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (uint8_t i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  if (lsm6dsv_gy_self_test_set(&reg_ctx, (lsm6dsv_gy_self_test_t)TestType) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  delay(100);

  /*Ignore First Data*/
  if (Get_G_AxesRaw_When_Aval(data_raw) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  for (uint8_t i = 0; i < 5; i++) {
    if (Get_G_AxesRaw_When_Aval(data_raw) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }

    /*Average the data in each axis*/
    for (uint8_t j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsv_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (uint8_t i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (uint8_t i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (uint8_t i = 0; i < 3; i++) {
    if ((LSM6DSV_MIN_ST_LIMIT_mdps > test_val[i]) || (test_val[i] > LSM6DSV_MAX_ST_LIMIT_mdps)) {
      return LSM6DSV_ERROR;
    }
  }

  if (lsm6dsv_gy_self_test_set(&reg_ctx, LSM6DSV_GY_ST_DISABLE) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_xl_data_rate_set(&reg_ctx, LSM6DSV_ODR_OFF) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV gyroscope sensor raw axes when data available (Blocking)
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Get_G_AxesRaw_When_Aval(int16_t *Value)
{
  lsm6dsv_data_ready_t drdy;
  do {
    if (lsm6dsv_flag_data_ready_get(&reg_ctx, &drdy) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
  } while (!drdy.drdy_gy);

  if (Get_G_AxesRaw(Value) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Enable Rotation Vector SFLP feature
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Rotation_Vector()
{
  lsm6dsv_fifo_sflp_raw_t fifo_sflp;

  if (lsm6dsv_fifo_sflp_batch_get(&reg_ctx, &fifo_sflp)) {
    return LSM6DSV_ERROR;
  }
  /* Enable Rotation Vector SFLP feature */
  fifo_sflp.game_rotation = 1;

  if (lsm6dsv_fifo_sflp_batch_set(&reg_ctx, fifo_sflp)) {
    return LSM6DSV_ERROR;
  }


  /* Enable SFLP low power */
  if (lsm6dsv_sflp_game_rotation_set(&reg_ctx, PROPERTY_ENABLE)) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Disable Rotation Vector SFLP feature
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Rotation_Vector()
{
  lsm6dsv_fifo_sflp_raw_t fifo_sflp;

  if (lsm6dsv_fifo_sflp_batch_get(&reg_ctx, &fifo_sflp)) {
    return LSM6DSV_ERROR;
  }
  /* Disable Rotation Vector SFLP feature */
  fifo_sflp.game_rotation = 0;

  if (lsm6dsv_fifo_sflp_batch_set(&reg_ctx, fifo_sflp)) {
    return LSM6DSV_ERROR;
  }

  /* Check if all SFLP features are disabled */
  if (!fifo_sflp.game_rotation && !fifo_sflp.gravity && !fifo_sflp.gbias) {
    /* Disable SFLP */
    if (lsm6dsv_sflp_game_rotation_set(&reg_ctx, PROPERTY_DISABLE)) {
      return LSM6DSV_ERROR;
    }
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Enable Gravity Vector SFLP feature
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Gravity_Vector()
{
  lsm6dsv_fifo_sflp_raw_t fifo_sflp;


  if (lsm6dsv_fifo_sflp_batch_get(&reg_ctx, &fifo_sflp)) {
    return LSM6DSV_ERROR;
  }
  /* Enable Gravity Vector SFLP feature */
  fifo_sflp.gravity = 1;

  if (lsm6dsv_fifo_sflp_batch_set(&reg_ctx, fifo_sflp)) {
    return LSM6DSV_ERROR;
  }


  /* Enable SFLP low power */
  if (lsm6dsv_sflp_game_rotation_set(&reg_ctx, PROPERTY_ENABLE)) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Disable Gravity Vector SFLP feature
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Gravity_Vector()
{
  lsm6dsv_fifo_sflp_raw_t fifo_sflp;

  /* Set full scale */
  if (lsm6dsv_xl_full_scale_set(&reg_ctx, LSM6DSV_4g)) {
    return LSM6DSV_ERROR;
  }
  if (lsm6dsv_gy_full_scale_set(&reg_ctx, LSM6DSV_2000dps)) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_fifo_sflp_batch_get(&reg_ctx, &fifo_sflp)) {
    return LSM6DSV_ERROR;
  }
  /* Disable Gravity Vector SFLP feature */
  fifo_sflp.gravity = 0;

  if (lsm6dsv_fifo_sflp_batch_set(&reg_ctx, fifo_sflp)) {
    return LSM6DSV_ERROR;
  }

  /* Check if all SFLP features are disabled */
  if (!fifo_sflp.game_rotation && !fifo_sflp.gravity && !fifo_sflp.gbias) {
    /* Disable SFLP */
    if (lsm6dsv_sflp_game_rotation_set(&reg_ctx, PROPERTY_DISABLE)) {
      return LSM6DSV_ERROR;
    }
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Enable Gyroscope Bias SFLP feature
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Gyroscope_Bias()
{
  lsm6dsv_fifo_sflp_raw_t fifo_sflp;

  if (lsm6dsv_fifo_sflp_batch_get(&reg_ctx, &fifo_sflp)) {
    return LSM6DSV_ERROR;
  }
  /* Enable Gyroscope Bias SFLP feature */
  fifo_sflp.gbias = 1;

  if (lsm6dsv_fifo_sflp_batch_set(&reg_ctx, fifo_sflp)) {
    return LSM6DSV_ERROR;
  }

  /* Enable SFLP low power */
  if (lsm6dsv_sflp_game_rotation_set(&reg_ctx, PROPERTY_ENABLE)) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Disable Gyroscope Bias SFLP feature
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Gyroscope_Bias()
{
  lsm6dsv_fifo_sflp_raw_t fifo_sflp;


  if (lsm6dsv_fifo_sflp_batch_get(&reg_ctx, &fifo_sflp)) {
    return LSM6DSV_ERROR;
  }
  /* Disable Gyroscope Bias SFLP feature */
  fifo_sflp.gbias = 0;

  if (lsm6dsv_fifo_sflp_batch_set(&reg_ctx, fifo_sflp)) {
    return LSM6DSV_ERROR;
  }

  /* Check if all SFLP features are disabled */
  if (!fifo_sflp.game_rotation && !fifo_sflp.gravity && !fifo_sflp.gbias) {
    /* Disable SFLP */
    if (lsm6dsv_sflp_game_rotation_set(&reg_ctx, PROPERTY_DISABLE)) {
      return LSM6DSV_ERROR;
    }
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Bulk set SFLP feature
 * @param  GameRotation enable/disable Game Rotation Vector SFLP feature
 * @param  Gravity enable/disable Gravity Vector SFLP feature
 * @param  gBias enable/disable Gyroscope Bias SFLP feature
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_SFLP_Batch(bool GameRotation, bool Gravity, bool gBias)
{
  lsm6dsv_fifo_sflp_raw_t fifo_sflp;
  fifo_sflp.game_rotation = GameRotation;
  fifo_sflp.gravity = Gravity;
  fifo_sflp.gbias = gBias;
  if (lsm6dsv_fifo_sflp_batch_set(&reg_ctx, fifo_sflp)) {
    return LSM6DSV_ERROR;
  }

  /* Check if all SFLP features are disabled */
  if (!fifo_sflp.game_rotation && !fifo_sflp.gravity && !fifo_sflp.gbias) {
    /* Disable SFLP */
    if (lsm6dsv_sflp_game_rotation_set(&reg_ctx, PROPERTY_DISABLE)) {
      return LSM6DSV_ERROR;
    }
  }
  else {
    /* Enable SFLP low power */
    if (lsm6dsv_sflp_game_rotation_set(&reg_ctx, PROPERTY_ENABLE)) {
      return LSM6DSV_ERROR;
    }
  }
  return LSM6DSV_OK;
}


/**
 * @brief  Set the LSM6DSV sensor fusion low power (sflp) output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_SFLP_ODR(float odr)
{
  lsm6dsv_sflp_data_rate_t rate = odr <= 15    ? LSM6DSV_SFLP_15Hz
                                     : odr <= 30  ? LSM6DSV_SFLP_30Hz
                                     : odr <= 60  ? LSM6DSV_SFLP_60Hz
                                     : odr <= 120 ? LSM6DSV_SFLP_120Hz
                                     : odr <= 240 ? LSM6DSV_SFLP_240Hz
                                                  : LSM6DSV_SFLP_480Hz;

  return (LSM6DSVStatusTypeDef)lsm6dsv_sflp_data_rate_set(&reg_ctx, rate);
}

/**
 * @brief  Set the LSM6DSV sflp G bias
 * @param  x the gyro bias in the x axis to be set
 * @param  y the gyro bias in the y axis to be set
 * @param  z the gyro bias in the z axis to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Set_SFLP_GBIAS(float x, float y, float z)
{
  lsm6dsv_sflp_gbias_t val = {0, 0, 0};
  val.gbias_x = x;
  val.gbias_y = y;
  val.gbias_z = z;
  if (lsm6dsv_sflp_game_gbias_set(&reg_ctx, &val) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
  * @brief Reset SFLP
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSVStatusTypeDef LSM6DSV::Reset_SFLP(void)
{
  lsm6dsv_emb_func_init_a_t emb_func_init_a;

  if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_EMBED_FUNC_MEM_BANK) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_read_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INIT_A, (uint8_t *)&emb_func_init_a, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  emb_func_init_a.sflp_game_init = 1;

  if (lsm6dsv_write_reg(&reg_ctx, LSM6DSV_EMB_FUNC_INIT_A, (uint8_t *)&emb_func_init_a, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  if (lsm6dsv_mem_bank_set(&reg_ctx, LSM6DSV_MAIN_MEM_BANK) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }
  return LSM6DSV_OK;
}

/**
 * @brief  Enable the LSM6DSV block update feature
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Block_Data_Update()
{
    return (LSM6DSVStatusTypeDef)lsm6dsv_block_data_update_set(&reg_ctx, PROPERTY_ENABLE);
}

/**
 * @brief  Disable the LSM6DSV block update feature
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Block_Data_Update()
{
    return (LSM6DSVStatusTypeDef)lsm6dsv_block_data_update_set(&reg_ctx, PROPERTY_DISABLE);
}

/**
 * @brief  Enable register address automatically incremented during a multiple byte access with a serial interface.
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Enable_Auto_Increment()
{
  return (LSM6DSVStatusTypeDef)lsm6dsv_auto_increment_set(&reg_ctx, PROPERTY_ENABLE);
}

/**
 * @brief  Disable register address automatically incremented during a multiple byte access with a serial interface.
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Disable_Auto_Increment()
{
  return (LSM6DSVStatusTypeDef)lsm6dsv_auto_increment_set(&reg_ctx, PROPERTY_DISABLE);
}

/**
 * @brief Preform a full reset of the LSM6DSV
 * @param  flags lsm6dsv_reset_t flags for what to reset
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Device_Reset(LSM6DSV_Reset_t flags)
{
  if (lsm6dsv_reset_set(&reg_ctx, (lsm6dsv_reset_t)flags) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  lsm6dsv_reset_t rst;
  do {
    if (lsm6dsv_reset_get(&reg_ctx, &rst) != LSM6DSV_OK) {
      return LSM6DSV_ERROR;
    }
  } while (rst != LSM6DSV_READY);
  return LSM6DSV_OK;
}

/**
 * @brief  Get the LSM6DSV register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (lsm6dsv_read_reg(&reg_ctx, Reg, Data, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

/**
 * @brief  Set the LSM6DSV register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSVStatusTypeDef LSM6DSV::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (lsm6dsv_write_reg(&reg_ctx, Reg, &Data, 1) != LSM6DSV_OK) {
    return LSM6DSV_ERROR;
  }

  return LSM6DSV_OK;
}

int32_t LSM6DSV_io_write(void *handle, uint8_t WriteAddr, const uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LSM6DSV *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LSM6DSV_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LSM6DSV *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}

void LSM6DSV_sleep(uint32_t ms) {
    delay(ms);
}

/**
 * @brief  Converts uint16_t half-precision number into a uint32_t single-precision number.
 * @param  h half-precision number
 * @param  f pointer where the result of the conversion is written
 * @retval 0
 */
LSM6DSVStatusTypeDef LSM6DSV::npy_halfbits_to_floatbits(uint16_t h, uint32_t *f)
{
  uint16_t h_exp, h_sig;
  uint32_t f_sgn, f_exp, f_sig;

  h_exp = (h & 0x7c00u);
  f_sgn = ((uint32_t)h & 0x8000u) << 16;
  switch (h_exp) {
    case 0x0000u: /* 0 or subnormal */
      h_sig = (h & 0x03ffu);
      /* Signed zero */
      if (h_sig == 0) {
        *f = f_sgn;
        return LSM6DSV_OK;
      }
      /* Subnormal */
      h_sig <<= 1;
      while ((h_sig & 0x0400u) == 0) {
        h_sig <<= 1;
        h_exp++;
      }
      f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
      f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
      *f = f_sgn + f_exp + f_sig;
      return LSM6DSV_OK;
    case 0x7c00u: /* inf or NaN */
      /* All-ones exponent and a copy of the significand */
      *f = f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
      return LSM6DSV_OK;
    default: /* normalized */
      /* Just need to adjust the exponent and shift */
      *f = f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
      return LSM6DSV_OK;
  }
}

/**
 * @brief  Converts uint16_t half-precision number into a float single-precision number.
 * @param  h half-precision number
 * @param  f pointer where the result of the conversion is written
 * @retval 0
 */
LSM6DSVStatusTypeDef LSM6DSV::npy_half_to_float(uint16_t h, float *f)
{
  union {
    float ret;
    uint32_t retbits;
  } conv;
  npy_halfbits_to_floatbits(h, &conv.retbits);
  *f = conv.ret;
  return LSM6DSV_OK;
}

/**
 * @brief  Compute quaternions.
 * @param  quat results of the computation
 * @param  sflp raw value of the quaternions
 * @retval 0
 */
LSM6DSVStatusTypeDef LSM6DSV::sflp2q(float quat[4], uint16_t sflp[3])
{
  float sumsq = 0;

  npy_half_to_float(sflp[0], &quat[0]);
  npy_half_to_float(sflp[1], &quat[1]);
  npy_half_to_float(sflp[2], &quat[2]);

  for (uint8_t i = 0; i < 3; i++) {
    sumsq += quat[i] * quat[i];
  }

  if (sumsq > 1.0f) {
    float n = sqrtf(sumsq);
    quat[0] /= n;
    quat[1] /= n;
    quat[2] /= n;
    sumsq = 1.0f;
  }

  quat[3] = sqrtf(1.0f - sumsq);
  return LSM6DSV_OK;
}