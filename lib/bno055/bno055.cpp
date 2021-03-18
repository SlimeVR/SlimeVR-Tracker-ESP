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
* @file bno055.c
* @date 10/01/2020
* @version  2.0.6
*
*/

/*********************************************************/
/*              INCLUDES    */
/*******************************************************/
#include "bno055.h"

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif

/*! file <BNO055 >
 * brief <Sensor driver for BNO055> */

/*  STRUCTURE DEFINITIONS   */
static struct bno055_t *p_bno055;

/*   LOCAL FUNCTIONS    */

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
BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8 = BNO055_INIT_VALUE;
    u8 bno055_page_zero_u8 = BNO055_PAGE_ZERO;

    /* Array holding the Software revision id
     */
    u8 a_SW_ID_u8[BNO055_REV_ID_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };

    /* stuct parameters are assign to bno055*/
    p_bno055 = bno055;

    /* Write the default page as zero*/
    com_rslt = p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                               BNO055_PAGE_ID_REG,
                                               &bno055_page_zero_u8,
                                               BNO055_GEN_READ_WRITE_LENGTH);

    /* Read the chip id of the sensor from page
     * zero 0x00 register*/
    com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                               BNO055_CHIP_ID_REG,
                                               &data_u8,
                                               BNO055_GEN_READ_WRITE_LENGTH);
    p_bno055->chip_id = data_u8;

    /* Read the accel revision id from page
     * zero 0x01 register*/
    com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                               BNO055_ACCEL_REV_ID_REG,
                                               &data_u8,
                                               BNO055_GEN_READ_WRITE_LENGTH);
    p_bno055->accel_rev_id = data_u8;

    /* Read the mag revision id from page
     * zero 0x02 register*/
    com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                               BNO055_MAG_REV_ID_REG,
                                               &data_u8,
                                               BNO055_GEN_READ_WRITE_LENGTH);
    p_bno055->mag_rev_id = data_u8;

    /* Read the gyro revision id from page
     * zero 0x02 register*/
    com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                               BNO055_GYRO_REV_ID_REG,
                                               &data_u8,
                                               BNO055_GEN_READ_WRITE_LENGTH);
    p_bno055->gyro_rev_id = data_u8;

    /* Read the boot loader revision from page
     * zero 0x06 register*/
    com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                               BNO055_BL_REV_ID_REG,
                                               &data_u8,
                                               BNO055_GEN_READ_WRITE_LENGTH);
    p_bno055->bl_rev_id = data_u8;

    /* Read the software revision id from page
     * zero 0x04 and 0x05 register( 2 bytes of data)*/
    com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                               BNO055_SW_REV_ID_LSB_REG,
                                               a_SW_ID_u8,
                                               BNO055_LSB_MSB_READ_LENGTH);
    a_SW_ID_u8[BNO055_SW_ID_LSB] = BNO055_GET_BITSLICE(a_SW_ID_u8[BNO055_SW_ID_LSB], BNO055_SW_REV_ID_LSB);
    p_bno055->sw_rev_id =
        (u16)((((u32)((u8)a_SW_ID_u8[BNO055_SW_ID_MSB])) << BNO055_SHIFT_EIGHT_BITS) | (a_SW_ID_u8[BNO055_SW_ID_LSB]));

    /* Read the page id from the register 0x07*/
    com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                               BNO055_PAGE_ID_REG,
                                               &data_u8,
                                               BNO055_GEN_READ_WRITE_LENGTH);
    p_bno055->page_id = data_u8;

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_write_register(u8 addr_u8, u8 *data_u8, u8 len_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* Write the values of respective given register */
        com_rslt = p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr, addr_u8, data_u8, len_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_register(u8 addr_u8, u8 *data_u8, u8 len_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* Read the value from given register*/
        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr, addr_u8, data_u8, len_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_chip_id(u8 *chip_id_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8 = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the chip id*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_CHIP_ID_REG,
                                                      &data_u8,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *chip_id_u8 = data_u8;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_sw_rev_id(u16 *sw_id_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* array having the software revision id
     * data_u8[0] - LSB
     * data_u8[1] - MSB*/
    u8 data_u8[BNO055_REV_ID_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct  p_bno055 is empty*/
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value of software
             * revision id*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SW_REV_ID_LSB_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SW_ID_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SW_ID_LSB], BNO055_SW_REV_ID_LSB);
            *sw_id_u8 =
                (u16)((((u32)((u8)data_u8[BNO055_SW_ID_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SW_ID_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_page_id(u8 *page_id_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8 = BNO055_INIT_VALUE;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* Read the page id form 0x07*/
        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                  BNO055_PAGE_ID_REG,
                                                  &data_u8,
                                                  BNO055_GEN_READ_WRITE_LENGTH);
        if (com_rslt == BNO055_SUCCESS)
        {
            data_u8 = BNO055_GET_BITSLICE(data_u8, BNO055_PAGE_ID);
            *page_id_u8 = data_u8;
            p_bno055->page_id = data_u8;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(u8 page_id_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* Read the current page*/
        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                  BNO055_PAGE_ID_REG,
                                                  &data_u8r,
                                                  BNO055_GEN_READ_WRITE_LENGTH);

        /* Check condition for communication BNO055_SUCCESS*/
        if (com_rslt == BNO055_SUCCESS)
        {
            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_PAGE_ID, page_id_u8);

            /* Write the page id*/
            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                        BNO055_PAGE_ID_REG,
                                                        &data_u8r,
                                                        BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                p_bno055->page_id = page_id_u8;
            }
            p_bno055->delay_msec(20);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_rev_id(u8 *accel_rev_id_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8 = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the accel revision id */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_REV_ID_REG,
                                                      &data_u8,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_rev_id_u8 = data_u8;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_rev_id(u8 *mag_rev_id_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8 = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the mag revision id */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_REV_ID_REG,
                                                      &data_u8,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *mag_rev_id_u8 = data_u8;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_rev_id(u8 *gyro_rev_id_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8 = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the gyro revision id */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_REV_ID_REG,
                                                      &data_u8,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_rev_id_u8 = data_u8;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_bl_rev_id(u8 *bl_rev_id_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8 = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the boot loader revision id */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_BL_REV_ID_REG,
                                                      &data_u8,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *bl_rev_id_u8 = data_u8;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_x(s16 *accel_x_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the accel x value
     * data_u8[BNO055_SENSOR_DATA_LSB] - x-LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - x-MSB
     */
    u8 data_u8[BNO055_ACCEL_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the accel x axis two byte value*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_ACCEL_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_ACCEL_DATA_X_MSB_VALUEX);
            *accel_x_s16 =
                (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_DATA_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_y(s16 *accel_y_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the accel y value
     * data_u8[BNO055_SENSOR_DATA_LSB] - y-LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - y-MSB
     */
    u8 data_u8[BNO055_ACCEL_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the accel y axis two byte value*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_DATA_Y_LSB_VALUEY_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_ACCEL_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_ACCEL_DATA_Y_MSB_VALUEY);
            *accel_y_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_z(s16 *accel_z_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the accel z value
     * data_u8[BNO055_SENSOR_DATA_LSB] - z-LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - z-MSB
     */
    u8 data_u8[BNO055_ACCEL_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the accel z axis two byte value*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_DATA_Z_LSB_VALUEZ_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_ACCEL_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_ACCEL_DATA_Z_MSB_VALUEZ);
            *accel_z_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_xyz(struct bno055_accel_t *accel)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the accel xyz value
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
     */
    u8 data_u8[BNO055_ACCEL_XYZ_DATA_SIZE] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_ACCEL_XYZ_DATA_SIZE);

            /* Data X*/
            data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                        BNO055_ACCEL_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                        BNO055_ACCEL_DATA_X_MSB_VALUEX);
            accel->x =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

            /* Data Y*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                        BNO055_ACCEL_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                        BNO055_ACCEL_DATA_Y_MSB_VALUEY);
            accel->y =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

            /* Data Z*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                        BNO055_ACCEL_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                        BNO055_ACCEL_DATA_Z_MSB_VALUEZ);
            accel->z =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_x(s16 *mag_x_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the mag x value
     * data_u8[BNO055_SENSOR_DATA_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - x->MSB
     */
    u8 data_u8[BNO055_MAG_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /*Read the mag x two bytes of data */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_MAG_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_MAG_DATA_X_MSB_VALUEX);
            *mag_x_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_y(s16 *mag_y_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the mag y value
     * data_u8[BNO055_SENSOR_DATA_LSB] - y->LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - y->MSB
     */
    u8 data_u8[BNO055_MAG_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /*Read the mag y two bytes of data */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_DATA_Y_LSB_VALUEY_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_MAG_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_MAG_DATA_Y_MSB_VALUEY);
            *mag_y_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_z(s16 *mag_z_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the mag z value
     * data_u8[BNO055_SENSOR_DATA_LSB] - z->LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - z->MSB
     */
    u8 data_u8[BNO055_MAG_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_DATA_Z_LSB_VALUEZ_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);

            /*Read the mag z two bytes of data */
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_MAG_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_MAG_DATA_Z_MSB_VALUEZ);
            *mag_z_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_xyz(struct bno055_mag_t *mag)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the mag xyz value
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
     */
    u8 data_u8[BNO055_MAG_XYZ_DATA_SIZE] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /*Read the six byte value of mag xyz*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_MAG_XYZ_DATA_SIZE);

            /* Data X*/
            data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                        BNO055_MAG_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                        BNO055_MAG_DATA_X_MSB_VALUEX);
            mag->x =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

            /* Data Y*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                        BNO055_MAG_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                        BNO055_MAG_DATA_Y_MSB_VALUEY);
            mag->y =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

            /* Data Z*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                        BNO055_MAG_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                        BNO055_MAG_DATA_Z_MSB_VALUEZ);
            mag->z =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_x(s16 *gyro_x_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8[BNO055_GYRO_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the gyro 16 bit x value*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_GYRO_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_GYRO_DATA_X_MSB_VALUEX);
            *gyro_x_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_y(s16 *gyro_y_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8[BNO055_GYRO_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of gyro y */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_DATA_Y_LSB_VALUEY_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_GYRO_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_GYRO_DATA_Y_MSB_VALUEY);
            *gyro_y_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_z(s16 *gyro_z_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8[BNO055_GYRO_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the gyro z 16 bit value*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_DATA_Z_LSB_VALUEZ_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_GYRO_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_GYRO_DATA_Z_MSB_VALUEZ);
            *gyro_z_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_xyz(struct bno055_gyro_t *gyro)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the gyro xyz value
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
     */
    u8 data_u8[BNO055_GYRO_XYZ_DATA_SIZE] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the six bytes data of gyro xyz*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_GYRO_XYZ_DATA_SIZE);

            /* Data x*/
            data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                        BNO055_GYRO_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                        BNO055_GYRO_DATA_X_MSB_VALUEX);
            gyro->x =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

            /* Data y*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                        BNO055_GYRO_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                        BNO055_GYRO_DATA_Y_MSB_VALUEY);
            gyro->y =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

            /* Data z*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                        BNO055_GYRO_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                        BNO055_GYRO_DATA_Z_MSB_VALUEZ);
            gyro->z =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_h(s16 *euler_h_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the Euler h value
     * data_u8[BNO055_SENSOR_DATA_EULER_LSB] - h->LSB
     * data_u8[BNO055_SENSOR_DATA_EULER_MSB] - h->MSB
     */
    u8 data_u8[BNO055_EULER_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the eulre heading data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_EULER_H_LSB_VALUEH_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_EULER_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_LSB],
                                                                        BNO055_EULER_H_LSB_VALUEH);
            data_u8[BNO055_SENSOR_DATA_EULER_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_MSB],
                                                                        BNO055_EULER_H_MSB_VALUEH);
            *euler_h_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_EULER_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_EULER_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_r(s16 *euler_r_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the Euler r value
     * data_u8[BNO055_SENSOR_DATA_EULER_LSB] - r->LSB
     * data_u8[BNO055_SENSOR_DATA_EULER_MSB] - r->MSB
     */
    u8 data_u8[BNO055_EULER_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the Euler roll data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_EULER_R_LSB_VALUER_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_EULER_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_LSB],
                                                                        BNO055_EULER_R_LSB_VALUER);
            data_u8[BNO055_SENSOR_DATA_EULER_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_MSB],
                                                                        BNO055_EULER_R_MSB_VALUER);
            *euler_r_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_EULER_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_EULER_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_p(s16 *euler_p_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the Euler p value
     * data_u8[BNO055_SENSOR_DATA_EULER_LSB] - p->LSB
     * data_u8[BNO055_SENSOR_DATA_EULER_MSB] - p->MSB
     */
    u8 data_u8[BNO055_EULER_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the Euler p data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_EULER_P_LSB_VALUEP_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_EULER_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_LSB],
                                                                        BNO055_EULER_P_LSB_VALUEP);
            data_u8[BNO055_SENSOR_DATA_EULER_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_EULER_MSB],
                                                                        BNO055_EULER_P_MSB_VALUEP);
            *euler_p_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_EULER_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_EULER_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_hrp(struct bno055_euler_t *euler)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the Euler hrp value
     * data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB] - h->LSB
     * data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB] - h->MSB
     * data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB] - r->MSB
     * data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB] - r->MSB
     * data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB] - p->MSB
     * data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB] - p->MSB
     */
    u8 data_u8[BNO055_EULER_HRP_DATA_SIZE] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the six byte of Euler hrp data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_EULER_H_LSB_VALUEH_REG,
                                                      data_u8,
                                                      BNO055_EULER_HRP_DATA_SIZE);

            /* Data h*/
            data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB] = BNO055_GET_BITSLICE(
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB],
                BNO055_EULER_H_LSB_VALUEH);
            data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB] = BNO055_GET_BITSLICE(
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB],
                BNO055_EULER_H_MSB_VALUEH);
            euler->h =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB]));

            /* Data r*/
            data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB] = BNO055_GET_BITSLICE(
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB],
                BNO055_EULER_R_LSB_VALUER);
            data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB] = BNO055_GET_BITSLICE(
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB],
                BNO055_EULER_R_MSB_VALUER);
            euler->r =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB]));

            /* Data p*/
            data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB] = BNO055_GET_BITSLICE(
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB],
                BNO055_EULER_P_LSB_VALUEP);
            data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB] = BNO055_GET_BITSLICE(
                data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB],
                BNO055_EULER_P_MSB_VALUEP);
            euler->p =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_w(s16 *quaternion_w_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the Quaternion w value
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB] - w->LSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB] - w->MSB
     */
    u8 data_u8[BNO055_QUATERNION_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
             * of quaternion w data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_QUATERNION_DATA_W_LSB_VALUEW_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB],
                                                                             BNO055_QUATERNION_DATA_W_LSB_VALUEW);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB],
                                                                             BNO055_QUATERNION_DATA_W_MSB_VALUEW);
            *quaternion_w_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_x(s16 *quaternion_x_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the quaternion x value
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB] - x->MSB
     */
    u8 data_u8[BNO055_QUATERNION_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
             * of quaternion x data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_QUATERNION_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB],
                                                                             BNO055_QUATERNION_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB],
                                                                             BNO055_QUATERNION_DATA_X_MSB_VALUEX);
            *quaternion_x_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_y(s16 *quaternion_y_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the quaternion y value
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB] - y->LSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB] - y->MSB
     */
    u8 data_u8[BNO055_QUATERNION_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
             * of quaternion y data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_QUATERNION_DATA_Y_LSB_VALUEY_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB],
                                                                             BNO055_QUATERNION_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB],
                                                                             BNO055_QUATERNION_DATA_Y_MSB_VALUEY);
            *quaternion_y_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_z(s16 *quaternion_z_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the quaternion z value
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB] - z->LSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB] - z->MSB
     */
    u8 data_u8[BNO055_QUATERNION_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
             * of quaternion z data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_QUATERNION_DATA_Z_LSB_VALUEZ_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB],
                                                                             BNO055_QUATERNION_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB],
                                                                             BNO055_QUATERNION_DATA_Z_MSB_VALUEZ);
            *quaternion_z_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_QUATERNION_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_wxyz(struct bno055_quaternion_t *quaternion)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the quaternion wxyz value
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB] - w->LSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB] - w->MSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB] - x->MSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB] - z->MSB
     * data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB] - z->MSB
     */
    u8 data_u8[BNO055_QUATERNION_WXYZ_DATA_SIZE] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the eight byte value
             * of quaternion wxyz data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_QUATERNION_DATA_W_LSB_VALUEW_REG,
                                                      data_u8,
                                                      BNO055_QUATERNION_WXYZ_DATA_SIZE);

            /* Data W*/
            data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB] =
                BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB],
                                    BNO055_QUATERNION_DATA_W_LSB_VALUEW);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB] =
                BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB],
                                    BNO055_QUATERNION_DATA_W_MSB_VALUEW);
            quaternion->w =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB]));

            /* Data X*/
            data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB] =
                BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB],
                                    BNO055_QUATERNION_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB] =
                BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB],
                                    BNO055_QUATERNION_DATA_X_MSB_VALUEX);
            quaternion->x =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB]));

            /* Data Y*/
            data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB] =
                BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB],
                                    BNO055_QUATERNION_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB] =
                BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB],
                                    BNO055_QUATERNION_DATA_Y_MSB_VALUEY);
            quaternion->y =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB]));

            /* Data Z*/
            data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB] =
                BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB],
                                    BNO055_QUATERNION_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB] =
                BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB],
                                    BNO055_QUATERNION_DATA_Z_MSB_VALUEZ);
            quaternion->z =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_x(s16 *linear_accel_x_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the linear accel x value
     * data_u8[BNO055_SENSOR_DATA_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - x->MSB
     */
    u8 data_u8[BNO055_ACCEL_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
            * of linear accel x data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX);
            *linear_accel_x_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_y(s16 *linear_accel_y_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the linear accel y value
     * data_u8[BNO055_SENSOR_DATA_LSB] - y->LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - y->MSB
     */
    u8 data_u8[BNO055_ACCEL_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
            * of linear accel y data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY);
            *linear_accel_y_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_z(s16 *linear_accel_z_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the linear accel z value
     * data_u8[BNO055_SENSOR_DATA_LSB] - z->LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - z->MSB
     */
    u8 data_u8[BNO055_ACCEL_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
            * of linear accel z data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ);
            *linear_accel_z_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_xyz(struct bno055_linear_accel_t *linear_accel)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the linear accel xyz value
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
     */
    u8 data_u8[BNO055_ACCEL_XYZ_DATA_SIZE] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the six byte value
             *  of linear accel xyz data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_ACCEL_XYZ_DATA_SIZE);

            /* Data x*/
            data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                        BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                        BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX);
            linear_accel->x =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

            /* Data y*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                        BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                        BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY);
            linear_accel->y =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

            /* Data z*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                        BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                        BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ);
            linear_accel->z =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_x(s16 *gravity_x_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the gravity x value
     * data_u8[BNO055_SENSOR_DATA_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - x->MSB
     */
    u8 data_u8[BNO055_GRAVITY_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
             * of gravity x data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GRAVITY_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_GRAVITY_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_GRAVITY_DATA_X_MSB_VALUEX);
            *gravity_x_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_y(s16 *gravity_y_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the gravity y value
     * data_u8[BNO055_SENSOR_DATA_LSB] - y->LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - y->MSB
     */
    u8 data_u8[BNO055_GRAVITY_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
             * of gravity y data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GRAVITY_DATA_Y_LSB_VALUEY_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_GRAVITY_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_GRAVITY_DATA_Y_MSB_VALUEY);
            *gravity_y_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_z(s16 *gravity_z_s16)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the gravity z value
     * data_u8[BNO055_SENSOR_DATA_LSB] - z->LSB
     * data_u8[BNO055_SENSOR_DATA_MSB] - z->MSB
     */
    u8 data_u8[BNO055_GRAVITY_DATA_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the two byte value
             * of gravity z data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GRAVITY_DATA_Z_LSB_VALUEZ_REG,
                                                      data_u8,
                                                      BNO055_LSB_MSB_READ_LENGTH);
            data_u8[BNO055_SENSOR_DATA_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_LSB],
                                                                  BNO055_GRAVITY_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_MSB],
                                                                  BNO055_GRAVITY_DATA_Z_MSB_VALUEZ);
            *gravity_z_s16 =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_xyz(struct bno055_gravity_t *gravity)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the gravity xyz value
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] - x->LSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] - x->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] - y->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] - z->MSB
     * data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] - z->MSB
     */
    u8 data_u8[BNO055_GRAVITY_XYZ_DATA_SIZE] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the six byte value
             * of gravity xyz data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GRAVITY_DATA_X_LSB_VALUEX_REG,
                                                      data_u8,
                                                      BNO055_GRAVITY_XYZ_DATA_SIZE);

            /* Data x*/
            data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB],
                                                                        BNO055_GRAVITY_DATA_X_LSB_VALUEX);
            data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB],
                                                                        BNO055_GRAVITY_DATA_X_MSB_VALUEX);
            gravity->x =
                (s16)(((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_X_MSB]) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_X_LSB]));

            /* Data y*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB],
                                                                        BNO055_GRAVITY_DATA_Y_LSB_VALUEY);
            data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB],
                                                                        BNO055_GRAVITY_DATA_Y_MSB_VALUEY);
            gravity->y =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Y_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Y_LSB]));

            /* Data z*/
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB],
                                                                        BNO055_GRAVITY_DATA_Z_LSB_VALUEZ);
            data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB],
                                                                        BNO055_GRAVITY_DATA_Z_MSB_VALUEZ);
            gravity->z =
                (s16)((((s32)((s8)data_u8[BNO055_SENSOR_DATA_XYZ_Z_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
                      (data_u8[BNO055_SENSOR_DATA_XYZ_Z_LSB]));
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_temp_data(s8 *temp_s8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8 = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the raw temperature data */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_TEMP_REG,
                                                      &data_u8,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *temp_s8 = data_u8;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}
#ifdef  BNO055_FLOAT_ENABLE

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_x_msq(float *accel_x_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_x_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as m/s2 if the unit is in mg*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MSQ)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw x data*/
        com_rslt += bno055_read_accel_x(&reg_accel_x_s16);
        p_bno055->delay_msec(BNO055_GEN_READ_WRITE_LENGTH);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw accel x to m/s2*/
            data_f = (float)(reg_accel_x_s16 / BNO055_ACCEL_DIV_MSQ);
            *accel_x_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

/*!
 *  @brief This API is used to convert the accel x raw data
 *  to millig output as float
 *
 *  @param accel_x_f : The accel x millig data
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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_x_mg(float *accel_x_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_x_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as mg if the unit is in m/s2*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MG)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw x data*/
        com_rslt += bno055_read_accel_x(&reg_accel_x_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw accel x to m/s2*/
            data_f = (float)(reg_accel_x_s16 / BNO055_ACCEL_DIV_MG);
            *accel_x_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_y_msq(float *accel_y_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_y_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as m/s2 if the unit is in mg*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MSQ)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        com_rslt += bno055_read_accel_y(&reg_accel_y_s16);
        p_bno055->delay_msec(BNO055_GEN_READ_WRITE_LENGTH);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw accel y to m/s2*/
            data_f = (float)(reg_accel_y_s16 / BNO055_ACCEL_DIV_MSQ);
            *accel_y_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_y_mg(float *accel_y_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_y_s16 = BNO055_INIT_VALUE;
    float data = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as mg if the unit is in m/s2*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MG)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw z data*/
        com_rslt += bno055_read_accel_y(&reg_accel_y_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw accel z to mg*/
            data = (float)(reg_accel_y_s16 / BNO055_ACCEL_DIV_MG);
            *accel_y_f = data;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_z_msq(float *accel_z_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_z_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as m/s2 if the unit is in mg*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MSQ)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw z data*/
        com_rslt += bno055_read_accel_z(&reg_accel_z_s16);
        p_bno055->delay_msec(BNO055_GEN_READ_WRITE_LENGTH);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw accel z to m/s2*/
            data_f = (float)(reg_accel_z_s16 / BNO055_ACCEL_DIV_MSQ);
            *accel_z_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_z_mg(float *accel_z_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_z_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as mg if the unit is in m/s2 */
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MG)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw z data*/
        com_rslt += bno055_read_accel_z(&reg_accel_z_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw accel x to mg*/
            data_f = (float)(reg_accel_z_s16 / BNO055_ACCEL_DIV_MG);
            *accel_z_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_xyz_msq(struct bno055_accel_float_t *accel_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_accel_t reg_accel_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as m/s2 if the unit is in mg*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MSQ)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw xyz data*/
        com_rslt += bno055_read_accel_xyz(&reg_accel_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the accel raw xyz to meterpersecseq*/
            accel_xyz->x = (float)(reg_accel_xyz.x / BNO055_ACCEL_DIV_MSQ);
            accel_xyz->y = (float)(reg_accel_xyz.y / BNO055_ACCEL_DIV_MSQ);
            accel_xyz->z = (float)(reg_accel_xyz.z / BNO055_ACCEL_DIV_MSQ);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_xyz_mg(struct bno055_accel_float_t *accel_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_accel_t reg_accel_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as mg if the unit is in m/s2*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MG)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw y data*/
        com_rslt += bno055_read_accel_xyz(&reg_accel_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /*Convert the accel raw xyz to millig */
            accel_xyz->x = (float)(reg_accel_xyz.x / BNO055_ACCEL_DIV_MG);
            accel_xyz->y = (float)(reg_accel_xyz.y / BNO055_ACCEL_DIV_MG);
            accel_xyz->z = (float)(reg_accel_xyz.z / BNO055_ACCEL_DIV_MG);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_x_uT(float *mag_x_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_mag_x_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;

    /* Read raw mag x data */
    com_rslt = bno055_read_mag_x(&reg_mag_x_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw mag x to microTesla*/
        data_f = (float)(reg_mag_x_s16 / BNO055_MAG_DIV_UT);
        *mag_x_f = data_f;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_y_uT(float *mag_y_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_mag_y_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;

    /* Read raw mag y data */
    com_rslt = bno055_read_mag_y(&reg_mag_y_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw mag y to microTesla*/
        data_f = (float)(reg_mag_y_s16 / BNO055_MAG_DIV_UT);
        *mag_y_f = data_f;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_z_uT(float *mag_z_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_mag_z_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;

    /* Read raw mag z data */
    com_rslt = bno055_read_mag_z(&reg_mag_z_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw mag z to microTesla*/
        data_f = (float)(reg_mag_z_s16 / BNO055_MAG_DIV_UT);
        *mag_z_f = data_f;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_xyz_uT(struct bno055_mag_float_t *mag_xyz_data)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_mag_t mag_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };

    /* Read raw mag x data */
    com_rslt = bno055_read_mag_xyz(&mag_xyz);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert mag raw xyz to microTesla*/
        mag_xyz_data->x = (float)(mag_xyz.x / BNO055_MAG_DIV_UT);
        mag_xyz_data->y = (float)(mag_xyz.y / BNO055_MAG_DIV_UT);
        mag_xyz_data->z = (float)(mag_xyz.z / BNO055_MAG_DIV_UT);
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_x_dps(float *gyro_x_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_x_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as dps if the unit is in rps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw x data */
        com_rslt += bno055_read_gyro_x(&reg_gyro_x_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw gyro x to dps*/
            data_f = (float)(reg_gyro_x_s16 / BNO055_GYRO_DIV_DPS);
            *gyro_x_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_x_rps(float *gyro_x_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_x_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as rps if the unit is in dps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_RPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw x data */
        com_rslt += bno055_read_gyro_x(&reg_gyro_x_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw gyro x to rps*/
            data_f = (float)(reg_gyro_x_s16 / BNO055_GYRO_DIV_RPS);
            *gyro_x_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_y_dps(float *gyro_y_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_y_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as dps if the unit is in rps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw y data */
        com_rslt += bno055_read_gyro_y(&reg_gyro_y_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw gyro x to dps*/
            data_f = (float)(reg_gyro_y_s16 / BNO055_GYRO_DIV_DPS);
            *gyro_y_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_y_rps(float *gyro_y_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_y_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as rps if the unit is in dps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_RPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw y data */
        com_rslt += bno055_read_gyro_y(&reg_gyro_y_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw gyro x to rps*/
            data_f = (float)(reg_gyro_y_s16 / BNO055_GYRO_DIV_RPS);
            *gyro_y_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_z_dps(float *gyro_z_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_z_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as dps if the unit is in rps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw z data */
        com_rslt += bno055_read_gyro_z(&reg_gyro_z_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw gyro x to dps*/
            data_f = (float)(reg_gyro_z_s16 / BNO055_GYRO_DIV_DPS);
            *gyro_z_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_z_rps(float *gyro_z_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_z_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as rps if the unit is in dps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_RPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw x data */
        com_rslt += bno055_read_gyro_z(&reg_gyro_z_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw gyro x to rps*/
            data_f = (float)(reg_gyro_z_s16 / BNO055_GYRO_DIV_RPS);
            *gyro_z_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_xyz_dps(struct bno055_gyro_float_t *gyro_xyz_data)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_gyro_t gyro_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as dps if the unit is in rps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw xyz data */
        com_rslt += bno055_read_gyro_xyz(&gyro_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert gyro raw xyz to dps*/
            gyro_xyz_data->x = (float)(gyro_xyz.x / BNO055_GYRO_DIV_DPS);
            gyro_xyz_data->y = (float)(gyro_xyz.y / BNO055_GYRO_DIV_DPS);
            gyro_xyz_data->z = (float)(gyro_xyz.z / BNO055_GYRO_DIV_DPS);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_xyz_rps(struct bno055_gyro_float_t *gyro_xyz_data)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_gyro_t gyro_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as rps if the unit is in dps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_RPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw xyz data */
        com_rslt += bno055_read_gyro_xyz(&gyro_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert gyro raw xyz to rps*/
            gyro_xyz_data->x = (float)(gyro_xyz.x / BNO055_GYRO_DIV_RPS);
            gyro_xyz_data->y = (float)(gyro_xyz.y / BNO055_GYRO_DIV_RPS);
            gyro_xyz_data->z = (float)(gyro_xyz.z / BNO055_GYRO_DIV_RPS);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_h_deg(float *euler_h_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_h_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as degree if the unit is in radians */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw h data*/
        com_rslt += bno055_read_euler_h(&reg_euler_h_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler h data to degree*/
            data_f = (float)(reg_euler_h_s16 / BNO055_EULER_DIV_DEG);
            *euler_h_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_h_rad(float *euler_h_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_h_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_RAD)
    {
        /* Read the current Euler unit and set the
         * unit as radians if the unit is in degree */
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw h data*/
        com_rslt += bno055_read_euler_h(&reg_euler_h_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler h data to degree*/
            data_f = (float)(reg_euler_h_s16 / BNO055_EULER_DIV_RAD);
            *euler_h_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_r_deg(float *euler_r_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_r = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as degree if the unit is in radians */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw r data*/
        com_rslt += bno055_read_euler_r(&reg_euler_r);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler r data to degree*/
            data_f = (float)(reg_euler_r / BNO055_EULER_DIV_DEG);
            *euler_r_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_r_rad(float *euler_r_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_r_f = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as radians if the unit is in degree */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_RAD)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw r data*/
        com_rslt += bno055_read_euler_r(&reg_euler_r_f);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler r data to radians*/
            data_f = (float)(reg_euler_r_f / BNO055_EULER_DIV_RAD);
            *euler_r_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_p_deg(float *euler_p_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_p_f = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as degree if the unit is in radians */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw p data*/
        com_rslt += bno055_read_euler_p(&reg_euler_p_f);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler p data to degree*/
            data_f = (float)(reg_euler_p_f / BNO055_EULER_DIV_DEG);
            *euler_p_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_p_rad(float *euler_p_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_p_f = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as radians if the unit is in degree */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_RAD)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw r data*/
        com_rslt += bno055_read_euler_p(&reg_euler_p_f);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler r data to radians*/
            data_f = (float)(reg_euler_p_f / BNO055_EULER_DIV_RAD);
            *euler_p_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_hpr_deg(struct bno055_euler_float_t *euler_hpr)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_euler_t reg_euler = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as degree if the unit is in radians */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw hrp data*/
        com_rslt += bno055_read_euler_hrp(&reg_euler);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler hrp to degree*/
            euler_hpr->h = (float)(reg_euler.h / BNO055_EULER_DIV_DEG);
            euler_hpr->p = (float)(reg_euler.p / BNO055_EULER_DIV_DEG);
            euler_hpr->r = (float)(reg_euler.r / BNO055_EULER_DIV_DEG);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_hpr_rad(struct bno055_euler_float_t *euler_hpr)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_euler_t reg_euler = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as radians if the unit is in degree */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_RAD)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw hrp data*/
        com_rslt += bno055_read_euler_hrp(&reg_euler);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw hrp to radians */
            euler_hpr->h = (float)(reg_euler.h / BNO055_EULER_DIV_RAD);
            euler_hpr->p = (float)(reg_euler.p / BNO055_EULER_DIV_RAD);
            euler_hpr->r = (float)(reg_euler.r / BNO055_EULER_DIV_RAD);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_x_msq(float *linear_accel_x_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_linear_accel_x_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;

    /* Read the raw x of linear accel */
    com_rslt = bno055_read_linear_accel_x(&reg_linear_accel_x_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw linear accel x to m/s2*/
        data_f = (float)(reg_linear_accel_x_s16 / BNO055_LINEAR_ACCEL_DIV_MSQ);
        *linear_accel_x_f = data_f;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_y_msq(float *linear_accel_y_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_linear_accel_y = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;

    /* Read the raw y of linear accel */
    com_rslt = bno055_read_linear_accel_y(&reg_linear_accel_y);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw linear accel x to m/s2*/
        data_f = (float)(reg_linear_accel_y / BNO055_LINEAR_ACCEL_DIV_MSQ);
        *linear_accel_y_f = data_f;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_z_msq(float *linear_accel_z_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_linear_accel_z = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;

    /* Read the raw x of linear accel */
    com_rslt = bno055_read_linear_accel_z(&reg_linear_accel_z);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw linear accel z to m/s2*/
        data_f = (float)(reg_linear_accel_z / BNO055_LINEAR_ACCEL_DIV_MSQ);
        *linear_accel_z_f = data_f;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
    struct bno055_linear_accel_float_t *linear_accel_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_linear_accel_t reg_linear_accel = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };

    /* Read the raw x of linear accel */
    com_rslt = bno055_read_linear_accel_xyz(&reg_linear_accel);
    if (com_rslt == BNO055_SUCCESS)
    {
        linear_accel_xyz->x = (float)(reg_linear_accel.x / BNO055_LINEAR_ACCEL_DIV_MSQ);
        linear_accel_xyz->y = (float)(reg_linear_accel.y / BNO055_LINEAR_ACCEL_DIV_MSQ);
        linear_accel_xyz->z = (float)(reg_linear_accel.z / BNO055_LINEAR_ACCEL_DIV_MSQ);
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_x_msq(float *gravity_x_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gravity_x_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;

    /* Read raw gravity of x*/
    com_rslt = bno055_read_gravity_x(&reg_gravity_x_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw gravity x to m/s2*/
        data_f = (float)(reg_gravity_x_s16 / BNO055_GRAVITY_DIV_MSQ);
        *gravity_x_f = data_f;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_y_msq(float *gravity_y_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gravity_y_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;

    /* Read raw gravity of y*/
    com_rslt = bno055_read_gravity_y(&reg_gravity_y_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw gravity y to m/s2*/
        data_f = (float)(reg_gravity_y_s16 / BNO055_GRAVITY_DIV_MSQ);
        *gravity_y_f = data_f;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_z_msq(float *gravity_z_f)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gravity_z_s16 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;

    /* Read raw gravity of z */
    com_rslt = bno055_read_gravity_z(&reg_gravity_z_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw gravity z to m/s2*/
        data_f = (float)(reg_gravity_z_s16 / BNO055_GRAVITY_DIV_MSQ);
        *gravity_z_f = data_f;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gravity_xyz_msq(struct bno055_gravity_float_t *gravity_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_gravity_t reg_gravity_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };

    /* Read raw gravity of xyz */
    com_rslt = bno055_read_gravity_xyz(&reg_gravity_xyz);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert raw gravity xyz to meterpersecseq */
        gravity_xyz->x = (float)(reg_gravity_xyz.x / BNO055_GRAVITY_DIV_MSQ);
        gravity_xyz->y = (float)(reg_gravity_xyz.y / BNO055_GRAVITY_DIV_MSQ);
        gravity_xyz->z = (float)(reg_gravity_xyz.z / BNO055_GRAVITY_DIV_MSQ);
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_temp_fahrenheit(float *temp_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s8 reg_temp_s8 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 temp_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current temperature unit and set the
     * unit as Fahrenheit if the unit is in Celsius */
    com_rslt = bno055_get_temp_unit(&temp_unit_u8);
    if (temp_unit_u8 != BNO055_TEMP_UNIT_FAHRENHEIT)
    {
        com_rslt += bno055_set_temp_unit(BNO055_TEMP_UNIT_FAHRENHEIT);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the raw temperature data */
        com_rslt += bno055_read_temp_data(&reg_temp_s8);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw temperature data to Fahrenheit*/
            data_f = (float)(reg_temp_s8 / BNO055_TEMP_DIV_FAHRENHEIT);
            *temp_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_temp_celsius(float *temp_f)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s8 reg_temp_s8 = BNO055_INIT_VALUE;
    float data_f = BNO055_INIT_VALUE;
    u8 temp_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current temperature unit and set the
     * unit as Fahrenheit if the unit is in Celsius */
    com_rslt = bno055_get_temp_unit(&temp_unit_u8);
    if (temp_unit_u8 != BNO055_TEMP_UNIT_CELSIUS)
    {
        com_rslt += bno055_set_temp_unit(BNO055_TEMP_UNIT_CELSIUS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the raw temperature data */
        com_rslt += bno055_read_temp_data(&reg_temp_s8);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw temperature data to Fahrenheit*/
            data_f = (float)(reg_temp_s8 / BNO055_TEMP_DIV_CELSIUS);
            *temp_f = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}
#endif
#ifdef  BNO055_DOUBLE_ENABLE

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_x_msq(double *accel_x_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_x_s16 = BNO055_INIT_VALUE;
    double data_f = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as m/s2 if the unit is in mg*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MSQ)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw y data*/
        com_rslt += bno055_read_accel_x(&reg_accel_x_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw x to m/s2 */
            data_f = (double)(reg_accel_x_s16 / BNO055_ACCEL_DIV_MSQ);
            *accel_x_d = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_x_mg(double *accel_x_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_x_s16 = BNO055_INIT_VALUE;
    double data_f = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as mg if the unit is in m/s2*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MG)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw y data*/
        com_rslt += bno055_read_accel_x(&reg_accel_x_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw x to mg */
            data_f = (double)(reg_accel_x_s16 / BNO055_ACCEL_DIV_MG);
            *accel_x_d = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_y_msq(double *accel_y_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_y_s16 = BNO055_INIT_VALUE;
    double data_f = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as m/s2 if the unit is in mg*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MSQ)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw y data*/
        com_rslt += bno055_read_accel_y(&reg_accel_y_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw x to m/s2 */
            data_f = (double)(reg_accel_y_s16 / BNO055_ACCEL_DIV_MSQ);
            *accel_y_d = data_f;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_y_mg(double *accel_y_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_y_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as mg if the unit is in m/s2*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MG)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw y data*/
        com_rslt += bno055_read_accel_y(&reg_accel_y_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw y to mg */
            data_d = (double)(reg_accel_y_s16 / BNO055_ACCEL_DIV_MG);
            *accel_y_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_z_msq(double *accel_z_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_z_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as m/s2 if the unit is in mg*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MSQ)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw z data*/
        com_rslt += bno055_read_accel_z(&reg_accel_z_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw z to m/s2 */
            data_d = (double)(reg_accel_z_s16 / BNO055_ACCEL_DIV_MSQ);
            *accel_z_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_z_mg(double *accel_z_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_accel_z_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as mg if the unit is in m/s2*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MG)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw z data*/
        com_rslt += bno055_read_accel_z(&reg_accel_z_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw z to mg */
            data_d = (double)(reg_accel_z_s16 / BNO055_ACCEL_DIV_MG);
            *accel_z_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_xyz_msq(struct bno055_accel_double_t *accel_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_accel_t reg_accel_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as m/s2 if the unit is in mg*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MSQ)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw xyz data*/
        com_rslt += bno055_read_accel_xyz(&reg_accel_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw xyz to m/s2*/
            accel_xyz->x = (double)(reg_accel_xyz.x / BNO055_ACCEL_DIV_MSQ);
            accel_xyz->y = (double)(reg_accel_xyz.y / BNO055_ACCEL_DIV_MSQ);
            accel_xyz->z = (double)(reg_accel_xyz.z / BNO055_ACCEL_DIV_MSQ);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_xyz_mg(struct bno055_accel_double_t *accel_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_accel_t reg_accel_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 accel_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current accel unit and set the
     * unit as m/s2 if the unit is in mg*/
    com_rslt = bno055_get_accel_unit(&accel_unit_u8);
    if (accel_unit_u8 != BNO055_ACCEL_UNIT_MG)
    {
        com_rslt += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the accel raw xyz data*/
        com_rslt += bno055_read_accel_xyz(&reg_accel_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw xyz to mg*/
            accel_xyz->x = (double)(reg_accel_xyz.x / BNO055_ACCEL_DIV_MG);
            accel_xyz->y = (double)(reg_accel_xyz.y / BNO055_ACCEL_DIV_MG);
            accel_xyz->z = (double)(reg_accel_xyz.z / BNO055_ACCEL_DIV_MG);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_x_uT(double *mag_x_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_mag_x_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;

    /* Read raw mag x data */
    com_rslt = bno055_read_mag_x(&reg_mag_x_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert raw mag x to microTesla */
        data_d = (double)(reg_mag_x_s16 / BNO055_MAG_DIV_UT);
        *mag_x_d = data_d;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

/*!
 *  @brief This API is used to convert the mag y raw data
 *  to microTesla output as double
 *
 *  @param mag_y_d : The mag y microTesla data
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_y_uT(double *mag_y_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_mag_y_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;

    /* Read raw mag y data */
    com_rslt = bno055_read_mag_y(&reg_mag_y_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert raw mag y to microTesla */
        data_d = (double)(reg_mag_y_s16 / BNO055_MAG_DIV_UT);
        *mag_y_d = data_d;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_z_uT(double *mag_z_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_mag_z_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;

    /* Read raw mag x */
    com_rslt = bno055_read_mag_z(&reg_mag_z_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert raw mag x to microTesla */
        data_d = (double)(reg_mag_z_s16 / BNO055_MAG_DIV_UT);
        *mag_z_d = data_d;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_xyz_uT(struct bno055_mag_double_t *mag_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_mag_t reg_mag_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };

    /* Read raw mag xyz data */
    com_rslt = bno055_read_mag_xyz(&reg_mag_xyz);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert raw mag xyz to microTesla*/
        mag_xyz->x = (double)(reg_mag_xyz.x / BNO055_MAG_DIV_UT);
        mag_xyz->y = (double)(reg_mag_xyz.y / BNO055_MAG_DIV_UT);
        mag_xyz->z = (double)(reg_mag_xyz.z / BNO055_MAG_DIV_UT);
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_x_dps(double *gyro_x_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_x_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as dps if the unit is in rps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw x data */
        com_rslt += bno055_read_gyro_x(&reg_gyro_x_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw gyro x to dps */
            data_d = (double)(reg_gyro_x_s16 / BNO055_GYRO_DIV_DPS);
            *gyro_x_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_x_rps(double *gyro_x_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_x_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as rps if the unit is in dps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_RPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw x data */
        com_rslt += bno055_read_gyro_x(&reg_gyro_x_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw gyro x to rps */
            data_d = (double)(reg_gyro_x_s16 / BNO055_GYRO_DIV_RPS);
            *gyro_x_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_y_dps(double *gyro_y_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_y_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as dps if the unit is in rps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw y data */
        com_rslt += bno055_read_gyro_y(&reg_gyro_y_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw gyro y to dps */
            data_d = (double)(reg_gyro_y_s16 / BNO055_GYRO_DIV_DPS);
            *gyro_y_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_y_rps(double *gyro_y_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_y_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as rps if the unit is in dps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_RPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw y data */
        com_rslt += bno055_read_gyro_y(&reg_gyro_y_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw gyro y to rps */
            data_d = (double)(reg_gyro_y_s16 / BNO055_GYRO_DIV_RPS);
            *gyro_y_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_z_dps(double *gyro_z_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_z_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as dps if the unit is in rps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw z data */
        com_rslt += bno055_read_gyro_z(&reg_gyro_z_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw gyro z to dps */
            data_d = (double)(reg_gyro_z_s16 / BNO055_GYRO_DIV_DPS);
            *gyro_z_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_z_rps(double *gyro_z_d)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gyro_z_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as rps if the unit is in dps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_RPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw x data */
        com_rslt += bno055_read_gyro_z(&reg_gyro_z_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw gyro x to rps */
            data_d = (double)(reg_gyro_z_s16 / BNO055_GYRO_DIV_RPS);
            *gyro_z_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_xyz_dps(struct bno055_gyro_double_t *gyro_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_gyro_t reg_gyro_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as dps if the unit is in rps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_DPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw xyz data */
        com_rslt += bno055_read_gyro_xyz(&reg_gyro_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert gyro raw xyz to dps*/
            gyro_xyz->x = (double)(reg_gyro_xyz.x / BNO055_GYRO_DIV_DPS);
            gyro_xyz->y = (double)(reg_gyro_xyz.y / BNO055_GYRO_DIV_DPS);
            gyro_xyz->z = (double)(reg_gyro_xyz.z / BNO055_GYRO_DIV_DPS);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_xyz_rps(struct bno055_gyro_double_t *gyro_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_gyro_t reg_gyro_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 gyro_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current gyro unit and set the
     * unit as rps if the unit is in dps */
    com_rslt = bno055_get_gyro_unit(&gyro_unit_u8);
    if (gyro_unit_u8 != BNO055_GYRO_UNIT_RPS)
    {
        com_rslt += bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read gyro raw x data */
        com_rslt += bno055_read_gyro_xyz(&reg_gyro_xyz);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert the raw gyro xyz to rps*/
            gyro_xyz->x = (double)(reg_gyro_xyz.x / BNO055_GYRO_DIV_RPS);
            gyro_xyz->y = (double)(reg_gyro_xyz.y / BNO055_GYRO_DIV_RPS);
            gyro_xyz->z = (double)(reg_gyro_xyz.z / BNO055_GYRO_DIV_RPS);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_h_deg(double *euler_h_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_h_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as degree if the unit is in radians */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw h data*/
        com_rslt += bno055_read_euler_h(&reg_euler_h_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler h to degree */
            data_d = (double)(reg_euler_h_s16 / BNO055_EULER_DIV_DEG);
            *euler_h_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_h_rad(double *euler_h_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_h_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as radians if the unit is in degree */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_RAD)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw h data*/
        com_rslt += bno055_read_euler_h(&reg_euler_h_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler h to radians */
            data_d = (double)(reg_euler_h_s16 / BNO055_EULER_DIV_RAD);
            *euler_h_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_r_deg(double *euler_r_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_r_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as degree if the unit is in radians */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw r data*/
        com_rslt += bno055_read_euler_r(&reg_euler_r_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler r to degree */
            data_d = (double)(reg_euler_r_s16 / BNO055_EULER_DIV_DEG);
            *euler_r_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_r_rad(double *euler_r_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_r_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as radians if the unit is in degree */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_RAD)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw r data*/
        com_rslt += bno055_read_euler_r(&reg_euler_r_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler r to radians */
            data_d = (double)(reg_euler_r_s16 / BNO055_EULER_DIV_RAD);
            *euler_r_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_p_deg(double *euler_p_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_p_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as degree if the unit is in radians */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw p data*/
        com_rslt += bno055_read_euler_p(&reg_euler_p_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler p to degree*/
            data_d = (double)(reg_euler_p_s16 / BNO055_EULER_DIV_DEG);
            *euler_p_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_p_rad(double *euler_p_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_euler_p_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as radians if the unit is in degree */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_RAD)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw p data*/
        com_rslt += bno055_read_euler_p(&reg_euler_p_s16);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw p to radians*/
            data_d = (double)(reg_euler_p_s16 / BNO055_EULER_DIV_RAD);
            *euler_p_d = data_d;
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_hpr_deg(struct bno055_euler_double_t *euler_hpr)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_euler_t reg_euler = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as degree if the unit is in radians */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
    {
        com_rslt += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read Euler raw h data*/
        com_rslt += bno055_read_euler_hrp(&reg_euler);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler hrp to degree*/
            euler_hpr->h = (double)(reg_euler.h / BNO055_EULER_DIV_DEG);
            euler_hpr->p = (double)(reg_euler.p / BNO055_EULER_DIV_DEG);
            euler_hpr->r = (double)(reg_euler.r / BNO055_EULER_DIV_DEG);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_hpr_rad(struct bno055_euler_double_t *euler_hpr)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_euler_t reg_euler = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };
    u8 euler_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current Euler unit and set the
     * unit as radians if the unit is in degree */
    com_rslt = bno055_get_euler_unit(&euler_unit_u8);
    if (euler_unit_u8 != BNO055_EULER_UNIT_RAD)
    {
        com_rslt = bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the raw hrp */
        com_rslt = bno055_read_euler_hrp(&reg_euler);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw Euler hrp to radians*/
            euler_hpr->h = (double)(reg_euler.h / BNO055_EULER_DIV_RAD);
            euler_hpr->p = (double)(reg_euler.p / BNO055_EULER_DIV_RAD);
            euler_hpr->r = (double)(reg_euler.r / BNO055_EULER_DIV_RAD);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_x_msq(double *linear_accel_x_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_linear_accel_x_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;

    /* Read the raw x of linear accel */
    com_rslt = bno055_read_linear_accel_x(&reg_linear_accel_x_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw x to m/s2 */
        data_d = (double)(reg_linear_accel_x_s16 / BNO055_LINEAR_ACCEL_DIV_MSQ);
        *linear_accel_x_d = data_d;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_y_msq(double *linear_accel_y_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_linear_accel_y_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;

    /* Read the raw x of linear accel */
    com_rslt = bno055_read_linear_accel_y(&reg_linear_accel_y_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw y to m/s2 */
        data_d = (double)(reg_linear_accel_y_s16 / BNO055_LINEAR_ACCEL_DIV_MSQ);
        *linear_accel_y_d = data_d;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_z_msq(double *linear_accel_z_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_linear_accel_z_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;

    /* Read the raw x of linear accel */
    com_rslt = bno055_read_linear_accel_z(&reg_linear_accel_z_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw z to m/s2 */
        data_d = (double)(reg_linear_accel_z_s16 / BNO055_LINEAR_ACCEL_DIV_MSQ);
        *linear_accel_z_d = data_d;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
    struct bno055_linear_accel_double_t *linear_accel_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_linear_accel_t reg_linear_accel_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };

    /* Read the raw xyz of linear accel */
    com_rslt = bno055_read_linear_accel_xyz(&reg_linear_accel_xyz);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert the raw xyz of linear accel to m/s2 */
        linear_accel_xyz->x = (double)(reg_linear_accel_xyz.x / BNO055_LINEAR_ACCEL_DIV_MSQ);
        linear_accel_xyz->y = (double)(reg_linear_accel_xyz.y / BNO055_LINEAR_ACCEL_DIV_MSQ);
        linear_accel_xyz->z = (double)(reg_linear_accel_xyz.z / BNO055_LINEAR_ACCEL_DIV_MSQ);
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_x_msq(double *gravity_x_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gravity_x_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;

    /* Read raw gravity of x*/
    com_rslt = bno055_read_gravity_x(&reg_gravity_x_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert raw gravity of x to m/s2 */
        data_d = (double)(reg_gravity_x_s16 / BNO055_GRAVITY_DIV_MSQ);
        *gravity_x_d = data_d;
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_y_msq(double *gravity_y_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gravity_y_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;

    /* Read raw gravity of y */
    com_rslt = bno055_read_gravity_y(&reg_gravity_y_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* convert raw gravity of y to m/s2 */
        data_d = (double)(reg_gravity_y_s16 / BNO055_GRAVITY_DIV_MSQ);
        *gravity_y_d = data_d;
    }
    else
    {
        com_rslt += BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_z_msq(double *gravity_z_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s16 reg_gravity_z_s16 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;

    /* Read raw gravity of z */
    com_rslt = bno055_read_gravity_z(&reg_gravity_z_s16);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert raw gravity of z to m/s2 */
        data_d = (double)(reg_gravity_z_s16 / BNO055_GRAVITY_DIV_MSQ);
        *gravity_z_d = data_d;
    }
    else
    {
        com_rslt += BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gravity_xyz_msq(struct bno055_gravity_double_t *gravity_xyz)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    struct bno055_gravity_t reg_gravity_xyz = { BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE };

    /* Read raw gravity of xyz */
    com_rslt = bno055_read_gravity_xyz(&reg_gravity_xyz);
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Convert raw gravity of xyz to m/s2 */
        gravity_xyz->x = (double)(reg_gravity_xyz.x / BNO055_GRAVITY_DIV_MSQ);
        gravity_xyz->y = (double)(reg_gravity_xyz.y / BNO055_GRAVITY_DIV_MSQ);
        gravity_xyz->z = (double)(reg_gravity_xyz.z / BNO055_GRAVITY_DIV_MSQ);
    }
    else
    {
        com_rslt += BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_temp_fahrenheit(double *temp_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s8 reg_temp_s8 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 temp_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current temperature unit and set the
     * unit as Fahrenheit if the unit is in Celsius */
    com_rslt = bno055_get_temp_unit(&temp_unit_u8);
    if (temp_unit_u8 != BNO055_TEMP_UNIT_FAHRENHEIT)
    {
        com_rslt += bno055_set_temp_unit(BNO055_TEMP_UNIT_FAHRENHEIT);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the raw temperature data */
        com_rslt += bno055_read_temp_data(&reg_temp_s8);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw temperature data to Fahrenheit*/
            data_d = (double)(reg_temp_s8 / BNO055_TEMP_DIV_FAHRENHEIT);
            *temp_d = data_d;
        }
        else
        {
            com_rslt += BNO055_ERROR;
        }
    }
    else
    {
        com_rslt += BNO055_ERROR;
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_temp_celsius(double *temp_d)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    s8 reg_temp_s8 = BNO055_INIT_VALUE;
    double data_d = BNO055_INIT_VALUE;
    u8 temp_unit_u8 = BNO055_INIT_VALUE;

    /* Read the current temperature unit and set the
     * unit as Fahrenheit if the unit is in Celsius */
    com_rslt = bno055_get_temp_unit(&temp_unit_u8);
    if (temp_unit_u8 != BNO055_TEMP_UNIT_CELSIUS)
    {
        com_rslt += bno055_set_temp_unit(BNO055_TEMP_UNIT_CELSIUS);
    }
    if (com_rslt == BNO055_SUCCESS)
    {
        /* Read the raw temperature data */
        com_rslt += bno055_read_temp_data(&reg_temp_s8);
        if (com_rslt == BNO055_SUCCESS)
        {
            /* Convert raw temperature data to Fahrenheit*/
            data_d = (double)(reg_temp_s8 / BNO055_TEMP_DIV_CELSIUS);
            *temp_d = data_d;
        }
        else
        {
            com_rslt += BNO055_ERROR;
        }
    }
    else
    {
        com_rslt += BNO055_ERROR;
    }

    return com_rslt;
}
#endif

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_calib_stat(u8 *mag_calib_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, mag calib
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the mag calib stat_s8 */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_CALIB_STAT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *mag_calib_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_MAG_CALIB_STAT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_calib_stat(u8 *accel_calib_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty*/
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel calib
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the accel calib stat_s8 */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_CALIB_STAT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_calib_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_CALIB_STAT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_calib_stat(u8 *gyro_calib_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro calib
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the gyro calib status */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_CALIB_STAT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_calib_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_CALIB_STAT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_calib_stat(u8 *sys_calib_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty*/
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page,system calib
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the system calib */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SYS_CALIB_STAT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *sys_calib_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SYS_CALIB_STAT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_accel(u8 *selftest_accel_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel self test is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the accel self test */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SELFTEST_ACCEL_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *selftest_accel_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SELFTEST_ACCEL);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_mag(u8 *selftest_mag_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, self test of mag is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the mag self test */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SELFTEST_MAG_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *selftest_mag_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SELFTEST_MAG);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_gyro(u8 *selftest_gyro_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page self test of gyro is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the gyro self test */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SELFTEST_GYRO_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *selftest_gyro_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SELFTEST_GYRO);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_mcu(u8 *selftest_mcu_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page self test of micro controller
         * is available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the self test of micro controller*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SELFTEST_MCU_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *selftest_mcu_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SELFTEST_MCU);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_gyro_any_motion(u8 *gyro_any_motion_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro anymotion interrupt
         * status is available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the gyro anymotion interrupt stat_s8*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_INTR_STAT_GYRO_ANY_MOTION_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_any_motion_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_INTR_STAT_GYRO_ANY_MOTION);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to read the stat_s8 of
 *  gyro highrate interrupt from register from 0x37 bit 3
 *
 *  @param gyro_highrate_u8 : The value of gyro highrate interrupt
 *
 *     gyro_highrate_u8   |  result
 *    -------------------   | ---------------------
 *     0x00                 | indicates no interrupt triggered
 *     0x01                 | indicates interrupt triggered
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_gyro_highrate(u8 *gyro_highrate_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the gyro highrate interrupt stat_s8*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_INTR_STAT_GYRO_HIGHRATE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_INTR_STAT_GYRO_HIGHRATE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to read the stat_s8 of
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_accel_high_g(u8 *accel_high_g_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel highg is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the accel highg interrupt stat_s8 */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_INTR_STAT_ACCEL_HIGH_G_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_high_g_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_INTR_STAT_ACCEL_HIGH_G);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_accel_any_motion(u8 *accel_any_motion_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel anymotion is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the accel anymotion interrupt stat_s8 */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_INTR_STAT_ACCEL_ANY_MOTION_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_any_motion_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_INTR_STAT_ACCEL_ANY_MOTION);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to read the stat_s8 of
 *  accel nomotion/slowmotion interrupt from register from 0x37 bit 6
 *
 *  @param accel_no_motion_u8 : The value of accel
 *  nomotion/slowmotion interrupt
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_accel_no_motion(u8 *accel_no_motion_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel
        * nomotion/slowmotion interrupt
        * is available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the stat_s8 of accel
             * nomotion/slowmotion interrupt*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_INTR_STAT_ACCEL_NO_MOTION_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_no_motion_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_INTR_STAT_ACCEL_NO_MOTION);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_stat_main_clk(u8 *stat_main_clk_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, status of main clk is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the status of main clk */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SYS_MAIN_CLK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *stat_main_clk_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SYS_MAIN_CLK);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_stat_code(u8 *sys_stat_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, the status of system is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the the status of system*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SYS_STAT_CODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *sys_stat_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SYS_STAT_CODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_error_code(u8 *sys_error_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, system BNO055_ERROR code is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the system BNO055_ERROR code*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SYS_ERROR_CODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *sys_error_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SYS_ERROR_CODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_unit(u8 *accel_unit_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel unit is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the accel unit */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_UNIT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_unit_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_UNIT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_unit(u8 accel_unit_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the accel unit */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_ACCEL_UNIT_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_UNIT, accel_unit_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_ACCEL_UNIT_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_unit(u8 *gyro_unit_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro unit is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the gyro unit */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_UNIT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_unit_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_UNIT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_unit(u8 gyro_unit_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the gyro unit */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_GYRO_UNIT_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_UNIT, gyro_unit_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_GYRO_UNIT_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_unit(u8 *euler_unit_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, Euler unit is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the Euler unit */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_EULER_UNIT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *euler_unit_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_EULER_UNIT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_unit(u8 euler_unit_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the Euler unit*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_EULER_UNIT_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_EULER_UNIT, euler_unit_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_EULER_UNIT_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_tilt_unit(u8 *tilt_unit_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, chip id is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_TILT_UNIT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *tilt_unit_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_TILT_UNIT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_tilt_unit(u8 tilt_unit_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_TILT_UNIT_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_TILT_UNIT, tilt_unit_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_TILT_UNIT_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_unit(u8 *temp_unit_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, temperature unit is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the temperature unit */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_TEMP_UNIT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *temp_unit_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_TEMP_UNIT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_unit(u8 temp_unit_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the temperature unit */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_TEMP_UNIT_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_TEMP_UNIT, temp_unit_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_TEMP_UNIT_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_data_output_format(u8 *data_output_format_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, data output format is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the data output format */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_DATA_OUTPUT_FORMAT_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *data_output_format_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_DATA_OUTPUT_FORMAT);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to write the current selected orientation mode
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
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_data_output_format(u8 data_output_format_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the data output format */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_DATA_OUTPUT_FORMAT_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_DATA_OUTPUT_FORMAT, data_output_format_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_DATA_OUTPUT_FORMAT_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_operation_mode(u8 *operation_mode_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, operation mode is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of operation mode*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_OPERATION_MODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *operation_mode_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_OPERATION_MODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(u8 operation_mode_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            /* If the previous operation mode is config it is
             * directly write the operation mode */
            if (prev_opmode_u8 == BNO055_OPERATION_MODE_CONFIG)
            {
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_OPERATION_MODE_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_OPERATION_MODE, operation_mode_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_OPERATION_MODE_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);

                    /* Config mode to other
                     * operation mode switching
                     * required delay of 600ms*/
                    p_bno055->delay_msec(BNO055_MODE_SWITCHING_DELAY);
                }
            }
            else
            {
                /* If the previous operation
                 * mode is not config it is
                 * write the config mode */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_OPERATION_MODE_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_OPERATION_MODE, BNO055_OPERATION_MODE_CONFIG);
                    com_rslt +=
                        bno055_write_register(BNO055_OPERATION_MODE_REG, &data_u8r, BNO055_GEN_READ_WRITE_LENGTH);

                    /* other mode to config mode switching
                     * required delay of 20ms*/
                    p_bno055->delay_msec(BNO055_CONFIG_MODE_SWITCHING_DELAY);
                }

                /* Write the operation mode */
                if (operation_mode_u8 != BNO055_OPERATION_MODE_CONFIG)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_OPERATION_MODE_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_OPERATION_MODE, operation_mode_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_OPERATION_MODE_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);

                        /* Config mode to other
                         * operation mode switching
                         * required delay of 600ms*/
                        p_bno055->delay_msec(BNO055_MODE_SWITCHING_DELAY);
                    }
                }
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_power_mode(u8 *power_mode_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, power mode is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of power mode */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_POWER_MODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *power_mode_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_POWER_MODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_power_mode(u8 power_mode_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the value of power mode */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_POWER_MODE_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_POWER_MODE, power_mode_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_POWER_MODE_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

BNO055_RETURN_FUNCTION_TYPE bno055_sw_reset()
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, reset
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Write the value of reset */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SYS_RST_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_SYS_RST, 1);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_SYS_RST_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
                p_bno055->delay_msec(600);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to read the reset interrupt
 *  from register from 0x3F bit 6
 *  It resets all the interrupt bit and interrupt output
 *
 *  @param intr_rst_u8 : The value of reset interrupt
 *
 *    intr_rst_u8 | result
 *   ------------ |----------
 *     0x01       | BNO055_BIT_ENABLE
 *     0x00       | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_rst(u8 *intr_rst_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page,  reset interrupt is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of reset interrupt*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_INTR_RST_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *intr_rst_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_INTR_RST);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to write the reset interrupt
 *  from register from 0x3F bit 6
 *  It resets all the interrupt bit and interrupt output
 *
 *  @param intr_rst_u8 : The value of reset interrupt
 *
 *    intr_rst_u8 | result
 *   -------------- |----------
 *     0x01         | BNO055_BIT_ENABLE
 *     0x00         | BNO055_BIT_DISABLE
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_rst(u8 intr_rst_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, reset interrupt
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Write the value of reset interrupt */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_INTR_RST_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_INTR_RST, intr_rst_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_INTR_RST_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to read the clk source
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_clk_src(u8 *clk_src_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, clk source is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of clk source */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_CLK_SRC_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *clk_src_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_CLK_SRC);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_clk_src(u8 clk_src_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, clk source is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Write the value of clk source */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_CLK_SRC_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_CLK_SRC, clk_src_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_CLK_SRC_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_rst(u8 *sys_rst_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, reset system is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of reset system */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SYS_RST_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *sys_rst_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SYS_RST);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_sys_rst(u8 sys_rst_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, reset system is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Write the value of reset system */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SYS_RST_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_SYS_RST, sys_rst_u8);
                com_rslt = p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                           BNO055_SYS_RST_REG,
                                                           &data_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest(u8 *selftest_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, self test is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of self test */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SELFTEST_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *selftest_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SELFTEST);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_selftest(u8 selftest_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the value of self test */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_SELFTEST_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_SELFTEST, selftest_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SELFTEST_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_source(u8 *temp_source_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, temperature source is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of temperature source */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_TEMP_SOURCE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *temp_source_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_TEMP_SOURCE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_source(u8 temp_source_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the value of temperature source*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_TEMP_SOURCE_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_TEMP_SOURCE, temp_source_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_TEMP_SOURCE_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_axis_remap_value(u8 *remap_axis_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, axis remap is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of axis remap*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_REMAP_AXIS_VALUE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *remap_axis_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_REMAP_AXIS_VALUE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to write the axis remap value
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_axis_remap_value(u8 remap_axis_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }

            /* Write the value of axis remap */
            if (stat_s8 == BNO055_SUCCESS)
            {
                switch (remap_axis_u8)
                {
                    case BNO055_REMAP_X_Y:
                    case BNO055_REMAP_Y_Z:
                    case BNO055_REMAP_Z_X:
                    case BNO055_REMAP_X_Y_Z_TYPE0:
                    case BNO055_REMAP_X_Y_Z_TYPE1:
                    case BNO055_DEFAULT_AXIS:
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_REMAP_AXIS_VALUE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_REMAP_AXIS_VALUE, remap_axis_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_REMAP_AXIS_VALUE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                        break;
                    default:

                        /* Write the default axis remap value */
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_REMAP_AXIS_VALUE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_REMAP_AXIS_VALUE, BNO055_DEFAULT_AXIS);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_REMAP_AXIS_VALUE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                        break;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_x_sign(u8 *remap_x_sign_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, x-axis remap sign is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of x-axis remap sign */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_REMAP_X_SIGN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *remap_x_sign_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_REMAP_X_SIGN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_x_sign(u8 remap_x_sign_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the value of x-axis remap */
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_REMAP_X_SIGN_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_REMAP_X_SIGN, remap_x_sign_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_REMAP_X_SIGN_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_y_sign(u8 *remap_y_sign_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, y-axis remap sign is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of y-axis remap sign*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_REMAP_Y_SIGN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *remap_y_sign_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_REMAP_Y_SIGN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_y_sign(u8 remap_y_sign_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the value of y-axis remap sign*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_REMAP_Y_SIGN_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_REMAP_Y_SIGN, remap_y_sign_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_REMAP_Y_SIGN_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_z_sign(u8 *remap_z_sign_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, z-axis remap sign is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read the value of z-axis remap sign*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_REMAP_Z_SIGN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *remap_z_sign_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_REMAP_Z_SIGN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to write the z-axis remap
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_z_sign(u8 remap_z_sign_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write the value of z-axis remap sign*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_REMAP_Z_SIGN_REG,
                                                          &data_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_REMAP_Z_SIGN, remap_z_sign_u8);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_REMAP_Z_SIGN_REG,
                                                                &data_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix(struct bno055_sic_matrix_t  *sic_matrix)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the soft iron calibration matrix values
     * data_u8[BNO055_SOFT_IRON_CALIB_0_LSB] - sic_0->LSB
     * data_u8[BNO055_SOFT_IRON_CALIB_0_MSB] - sic_0->MSB
     * data_u8[BNO055_SOFT_IRON_CALIB_1_LSB] - sic_1->LSB
     * data_u8[BNO055_SOFT_IRON_CALIB_1_MSB] - sic_1->MSB
     * data_u8[BNO055_SOFT_IRON_CALIB_2_LSB] - sic_2->LSB
     * data_u8[BNO055_SOFT_IRON_CALIB_2_MSB] - sic_2->MSB
     * data_u8[BNO055_SOFT_IRON_CALIB_3_LSB] - sic_3->LSB
     * data_u8[BNO055_SOFT_IRON_CALIB_3_MSB] - sic_3->MSB
     * data_u8[BNO055_SOFT_IRON_CALIB_4_LSB] - sic_4->LSB
     * data_u8[BNO055_SOFT_IRON_CALIB_4_MSB] - sic_4->MSB
     * data_u8[BNO055_SOFT_IRON_CALIB_5_LSB] - sic_5->LSB
     * data_u8[BNO055_SOFT_IRON_CALIB_5_MSB] - sic_5->MSB
     * data_u8[BNO055_SOFT_IRON_CALIB_6_LSB] - sic_6->LSB
     * data_u8[BNO055_SOFT_IRON_CALIB_6_MSB] - sic_6->MSB
     * data_u8[BNO055_SOFT_IRON_CALIB_7_LSB] - sic_7->LSB
     * data_u8[BNO055_SOFT_IRON_CALIB_7_MSB] - sic_7->MSB
     * data_u8[BNO055_SOFT_IRON_CALIB_8_LSB] - sic_8->LSB
     * data_u8[BNO055_SOFT_IRON_CALIB_8_MSB] - sic_8->MSB
     */
    u8 data_u8[BNO055_SOFT_IRON_CALIBRATION_MATRIX_SIZE] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE,
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, soft iron calibration matrix is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read soft iron calibration matrix value
             * it is eighteen bytes of data */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_SIC_MATRIX_0_LSB_REG,
                                                      data_u8,
                                                      BNO055_SOFT_IRON_CALIBRATION_MATRIX_SIZE);
            if (com_rslt == BNO055_SUCCESS)
            {
                /*soft iron calibration matrix zero*/
                data_u8[BNO055_SOFT_IRON_CALIB_0_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_0_LSB],
                                                                            BNO055_SIC_MATRIX_0_LSB);
                data_u8[BNO055_SOFT_IRON_CALIB_0_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_0_MSB],
                                                                            BNO055_SIC_MATRIX_0_MSB);
                sic_matrix->sic_0 =
                    (s16)((((s32)(s8)(data_u8[BNO055_SOFT_IRON_CALIB_0_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SOFT_IRON_CALIB_0_LSB]));

                /*soft iron calibration matrix one*/
                data_u8[BNO055_SOFT_IRON_CALIB_1_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_1_LSB],
                                                                            BNO055_SIC_MATRIX_1_LSB);
                data_u8[BNO055_SOFT_IRON_CALIB_1_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_1_MSB],
                                                                            BNO055_SIC_MATRIX_1_MSB);
                sic_matrix->sic_1 =
                    (s16)((((s32)(s8)(data_u8[BNO055_SOFT_IRON_CALIB_1_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SOFT_IRON_CALIB_1_LSB]));

                /*soft iron calibration matrix two*/
                data_u8[BNO055_SOFT_IRON_CALIB_2_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_2_LSB],
                                                                            BNO055_SIC_MATRIX_2_LSB);
                data_u8[BNO055_SOFT_IRON_CALIB_2_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_2_MSB],
                                                                            BNO055_SIC_MATRIX_2_MSB);
                sic_matrix->sic_2 =
                    (s16)((((s32)(s8)(data_u8[BNO055_SOFT_IRON_CALIB_2_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SOFT_IRON_CALIB_2_LSB]));

                /*soft iron calibration matrix three*/
                data_u8[BNO055_SOFT_IRON_CALIB_3_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_3_LSB],
                                                                            BNO055_SIC_MATRIX_3_LSB);
                data_u8[BNO055_SOFT_IRON_CALIB_3_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_3_MSB],
                                                                            BNO055_SIC_MATRIX_3_LSB);
                sic_matrix->sic_3 =
                    (s16)((((s32)(s8)(data_u8[BNO055_SOFT_IRON_CALIB_3_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SOFT_IRON_CALIB_3_LSB]));

                /*soft iron calibration matrix four*/
                data_u8[BNO055_SOFT_IRON_CALIB_4_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_4_LSB],
                                                                            BNO055_SIC_MATRIX_4_LSB);
                data_u8[BNO055_SOFT_IRON_CALIB_4_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_4_MSB],
                                                                            BNO055_SIC_MATRIX_4_LSB);
                sic_matrix->sic_4 =
                    (s16)((((s32)(s8)(data_u8[BNO055_SOFT_IRON_CALIB_4_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SOFT_IRON_CALIB_4_LSB]));

                /*soft iron calibration matrix five*/
                data_u8[BNO055_SOFT_IRON_CALIB_5_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_5_LSB],
                                                                            BNO055_SIC_MATRIX_5_LSB);
                data_u8[BNO055_SOFT_IRON_CALIB_5_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_5_MSB],
                                                                            BNO055_SIC_MATRIX_5_LSB);
                sic_matrix->sic_5 =
                    (s16)((((s32)(s8)(data_u8[BNO055_SOFT_IRON_CALIB_5_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SOFT_IRON_CALIB_5_LSB]));

                /*soft iron calibration matrix six*/
                data_u8[BNO055_SOFT_IRON_CALIB_6_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_6_LSB],
                                                                            BNO055_SIC_MATRIX_6_LSB);
                data_u8[BNO055_SOFT_IRON_CALIB_6_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_6_MSB],
                                                                            BNO055_SIC_MATRIX_6_LSB);
                sic_matrix->sic_6 =
                    (s16)((((s32)(s8)(data_u8[BNO055_SOFT_IRON_CALIB_6_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SOFT_IRON_CALIB_6_LSB]));

                /*soft iron calibration matrix seven*/
                data_u8[BNO055_SOFT_IRON_CALIB_7_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_7_LSB],
                                                                            BNO055_SIC_MATRIX_7_LSB);
                data_u8[BNO055_SOFT_IRON_CALIB_7_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_7_MSB],
                                                                            BNO055_SIC_MATRIX_7_LSB);
                sic_matrix->sic_7 =
                    (s16)((((s32)(s8)(data_u8[BNO055_SOFT_IRON_CALIB_7_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SOFT_IRON_CALIB_7_LSB]));

                /*soft iron calibration matrix eight*/
                data_u8[BNO055_SOFT_IRON_CALIB_8_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_8_LSB],
                                                                            BNO055_SIC_MATRIX_8_LSB);
                data_u8[BNO055_SOFT_IRON_CALIB_8_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SOFT_IRON_CALIB_8_MSB],
                                                                            BNO055_SIC_MATRIX_8_LSB);
                sic_matrix->sic_8 =
                    (s16)((((s32)(s8)(data_u8[BNO055_SOFT_IRON_CALIB_8_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SOFT_IRON_CALIB_8_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix(struct bno055_sic_matrix_t  *sic_matrix)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data1_u8r = BNO055_INIT_VALUE;
    u8 data2_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* write soft iron calibration
                 * matrix zero value*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_SIC_MATRIX_0_LSB_REG,
                                                          &data2_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_0 & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_0_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_0_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_0_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_0 >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_0_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_0_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write soft iron calibration
                 * matrix one value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_1_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_1 & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_1_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_1_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_1_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_1 >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_1_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_1_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write soft iron calibration
                 * matrix two value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_2_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_2 & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_2_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_2_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_2_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_2 >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_2_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_2_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write soft iron calibration
                 * matrix three value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_3_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_3 & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_3_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_3_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_3_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_3 >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_3_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_3_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write soft iron calibration
                 * matrix four value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_4_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_4 & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_4_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_4_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_4_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_4 >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_4_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_4_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write soft iron calibration
                 * matrix five value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_5_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_5 & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_5_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_5_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_5_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_5 >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_5_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_5_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write soft iron calibration
                 * matrix six value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_6_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_6 & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_6_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_6_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_6_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_6 >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_6_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_6_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write soft iron calibration
                 * matrix seven value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_7_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_7 & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_7_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_7_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_7_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_7 >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_7_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_7_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write soft iron calibration
                 * matrix eight value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_8_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_8 & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_8_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_8_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_SIC_MATRIX_8_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(sic_matrix->sic_8 >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_SIC_MATRIX_8_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_SIC_MATRIX_8_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset(struct bno055_accel_offset_t  *accel_offset)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the accel offset values
     * data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB] - offset x->LSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB] - offset x->MSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB] - offset y->LSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB] - offset y->MSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB] - offset z->LSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB] - offset z->MSB
     */
    u8 data_u8[BNO055_ACCEL_OFFSET_ARRAY] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel offset is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read accel offset value it is six bytes of data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_OFFSET_X_LSB_REG,
                                                      data_u8,
                                                      BNO055_ACCEL_OFFSET_ARRAY);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Read accel x offset value*/
                data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB],
                                                                               BNO055_ACCEL_OFFSET_X_LSB);
                data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB],
                                                                               BNO055_ACCEL_OFFSET_X_MSB);
                accel_offset->x =
                    (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB]));

                /* Read accel y offset value*/
                data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB],
                                                                               BNO055_ACCEL_OFFSET_Y_LSB);
                data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB],
                                                                               BNO055_ACCEL_OFFSET_Y_MSB);
                accel_offset->y =
                    (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB]));

                /* Read accel z offset value*/
                data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB],
                                                                               BNO055_ACCEL_OFFSET_Z_LSB);
                data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB],
                                                                               BNO055_ACCEL_OFFSET_Z_MSB);
                accel_offset->z =
                    (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB]));

                /* Read accel radius value
                 * it is two bytes of data*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_ACCEL_RADIUS_LSB_REG,
                                                           data_u8,
                                                           BNO055_LSB_MSB_READ_LENGTH);

                /* Array holding the accel radius values
                 * data_u8[BNO055_OFFSET_RADIUS_LSB] - radius->LSB
                 * data_u8[BNO055_OFFSET_RADIUS_MSB] - radius->MSB
                 */
                if (com_rslt == BNO055_SUCCESS)
                {
                    data_u8[BNO055_OFFSET_RADIUS_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_OFFSET_RADIUS_LSB],
                                                                            BNO055_ACCEL_RADIUS_LSB);
                    data_u8[BNO055_OFFSET_RADIUS_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_OFFSET_RADIUS_MSB],
                                                                            BNO055_ACCEL_RADIUS_MSB);
                    accel_offset->r =
                        (s16)((((s32)(s8)(data_u8[BNO055_OFFSET_RADIUS_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                              (data_u8[BNO055_OFFSET_RADIUS_LSB]));
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset(struct bno055_accel_offset_t  *accel_offset)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data1_u8r = BNO055_INIT_VALUE;
    u8 data2_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* write accel offset x value*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_ACCEL_OFFSET_X_LSB_REG,
                                                          &data2_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(accel_offset->x & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_ACCEL_OFFSET_X_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_ACCEL_OFFSET_X_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_ACCEL_OFFSET_X_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(accel_offset->x >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_ACCEL_OFFSET_X_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_ACCEL_OFFSET_X_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write accel offset y value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_ACCEL_OFFSET_Y_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(accel_offset->y & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_ACCEL_OFFSET_Y_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_ACCEL_OFFSET_Y_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_ACCEL_OFFSET_Y_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(accel_offset->y >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_ACCEL_OFFSET_Y_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_ACCEL_OFFSET_Y_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write accel offset z value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_ACCEL_OFFSET_Z_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(accel_offset->z & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_ACCEL_OFFSET_Z_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_ACCEL_OFFSET_Z_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_ACCEL_OFFSET_Z_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(accel_offset->z >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_ACCEL_OFFSET_Z_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_ACCEL_OFFSET_Z_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /*write accel radius value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_ACCEL_RADIUS_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(accel_offset->r & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_ACCEL_RADIUS_LSB, data1_u8r);
                    com_rslt = p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                               BNO055_ACCEL_RADIUS_LSB_REG,
                                                               &data2_u8r,
                                                               BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_ACCEL_RADIUS_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(accel_offset->r >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_ACCEL_RADIUS_MSB, data1_u8r);
                    com_rslt = p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                               BNO055_ACCEL_RADIUS_MSB_REG,
                                                               &data2_u8r,
                                                               BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset(struct bno055_mag_offset_t  *mag_offset)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the mag offset values
     * data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB] - offset x->LSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB] - offset x->MSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB] - offset y->LSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB] - offset y->MSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB] - offset z->LSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB] - offset z->MSB
     */
    u8 data_u8[BNO055_MAG_OFFSET_ARRAY] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, mag offset is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read mag offset value it the six bytes of data */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_OFFSET_X_LSB_REG,
                                                      data_u8,
                                                      BNO055_MAG_OFFSET_ARRAY);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Read mag x offset value*/
                data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB],
                                                                               BNO055_MAG_OFFSET_X_LSB);
                data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB],
                                                                               BNO055_MAG_OFFSET_X_MSB);
                mag_offset->x =
                    (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB]));

                /* Read mag y offset value*/
                data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB],
                                                                               BNO055_MAG_OFFSET_Y_LSB);
                data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB],
                                                                               BNO055_MAG_OFFSET_Y_MSB);
                mag_offset->y =
                    (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB]));

                /* Read mag z offset value*/
                data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB],
                                                                               BNO055_MAG_OFFSET_Z_LSB);
                data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB],
                                                                               BNO055_MAG_OFFSET_Z_MSB);
                mag_offset->z =
                    (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB]));

                /* Read mag radius value
                 * it the two bytes of data */
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_MAG_RADIUS_LSB_REG,
                                                           data_u8,
                                                           BNO055_LSB_MSB_READ_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    /* Array holding the mag radius values
                     * data_u8[BNO055_OFFSET_RADIUS_LSB] -
                     * radius->LSB
                     * data_u8[BNO055_OFFSET_RADIUS_MSB] -
                     * radius->MSB
                     */
                    data_u8[BNO055_OFFSET_RADIUS_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_OFFSET_RADIUS_LSB],
                                                                            BNO055_MAG_RADIUS_LSB);
                    data_u8[BNO055_OFFSET_RADIUS_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_OFFSET_RADIUS_MSB],
                                                                            BNO055_MAG_RADIUS_MSB);
                    mag_offset->r =
                        (s16)((((s32)(s8)(data_u8[BNO055_OFFSET_RADIUS_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                              (data_u8[BNO055_OFFSET_RADIUS_LSB]));
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API is used to read mag offset
 *  offset form register 0x69 to 0x6A
 *
 *  @param mag_offset :  The value of mag offset and radius
 *
 *      bno055_mag_offset_t |     result
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
BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset(struct bno055_mag_offset_t *mag_offset)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data1_u8r = BNO055_INIT_VALUE;
    u8 data2_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* write Mag offset x value*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_MAG_OFFSET_X_LSB_REG,
                                                          &data2_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(mag_offset->x & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_MAG_OFFSET_X_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_MAG_OFFSET_X_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_MAG_OFFSET_X_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(mag_offset->x >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_MAG_OFFSET_X_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_MAG_OFFSET_X_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write Mag offset y value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_MAG_OFFSET_Y_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(mag_offset->y & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_MAG_OFFSET_Y_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_MAG_OFFSET_Y_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_MAG_OFFSET_Y_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(mag_offset->y >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_MAG_OFFSET_Y_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_MAG_OFFSET_Y_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write Mag offset z value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_MAG_OFFSET_Z_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(mag_offset->z & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_MAG_OFFSET_Z_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_MAG_OFFSET_Z_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_MAG_OFFSET_Z_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(mag_offset->z >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_MAG_OFFSET_Z_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_MAG_OFFSET_Z_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write Mag radius value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_MAG_RADIUS_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(mag_offset->r & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_MAG_RADIUS_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_MAG_RADIUS_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_MAG_RADIUS_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(mag_offset->r >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_MAG_RADIUS_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_MAG_RADIUS_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset(struct bno055_gyro_offset_t  *gyro_offset)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;

    /* Array holding the gyro offset values
     * data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB] - offset x->LSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB] - offset x->MSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB] - offset y->LSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB] - offset y->MSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB] - offset z->LSB
     * data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB] - offset z->MSB
     */
    u8 data_u8[BNO055_GYRO_OFFSET_ARRAY] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro offset is
         * available in the page zero*/
        if (p_bno055->page_id != BNO055_PAGE_ZERO)
        {
            /* Write the page zero*/
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ZERO))
        {
            /* Read gyro offset value it the six bytes of data*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_OFFSET_X_LSB_REG,
                                                      data_u8,
                                                      BNO055_GYRO_OFFSET_ARRAY);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Read gyro x offset value*/
                data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB],
                                                                               BNO055_GYRO_OFFSET_X_LSB);
                data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB],
                                                                               BNO055_GYRO_OFFSET_X_MSB);
                gyro_offset->x =
                    (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_OFFSET_DATA_X_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SENSOR_OFFSET_DATA_X_LSB]));

                /* Read gyro y offset value*/
                data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB],
                                                                               BNO055_GYRO_OFFSET_Y_LSB);
                data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB],
                                                                               BNO055_GYRO_OFFSET_Y_MSB);
                gyro_offset->y =
                    (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_OFFSET_DATA_Y_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SENSOR_OFFSET_DATA_Y_LSB]));

                /* Read gyro z offset value*/
                data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB],
                                                                               BNO055_GYRO_OFFSET_Z_LSB);
                data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB] = BNO055_GET_BITSLICE(data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB],
                                                                               BNO055_GYRO_OFFSET_Z_MSB);
                gyro_offset->z =
                    (s16)((((s32)(s8)(data_u8[BNO055_SENSOR_OFFSET_DATA_Z_MSB])) << (BNO055_SHIFT_EIGHT_BITS)) |
                          (data_u8[BNO055_SENSOR_OFFSET_DATA_Z_LSB]));
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset(struct bno055_gyro_offset_t  *gyro_offset)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data1_u8r = BNO055_INIT_VALUE;
    u8 data2_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* write gryo offset x value*/
                com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                          BNO055_GYRO_OFFSET_X_LSB_REG,
                                                          &data2_u8r,
                                                          BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(gyro_offset->x & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_GYRO_OFFSET_X_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_GYRO_OFFSET_X_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_GYRO_OFFSET_X_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(gyro_offset->x >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_GYRO_OFFSET_X_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_GYRO_OFFSET_X_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write gryo offset y value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_GYRO_OFFSET_Y_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(gyro_offset->y & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_GYRO_OFFSET_Y_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_GYRO_OFFSET_Y_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_GYRO_OFFSET_Y_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(gyro_offset->y >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_GYRO_OFFSET_Y_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_GYRO_OFFSET_Y_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }

                /* write gryo offset z value*/
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_GYRO_OFFSET_Z_LSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(gyro_offset->z & BNO055_SIC_HEX_0_0_F_F_DATA));
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_GYRO_OFFSET_Z_LSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_GYRO_OFFSET_Z_LSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
                com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                           BNO055_GYRO_OFFSET_Z_MSB_REG,
                                                           &data2_u8r,
                                                           BNO055_GEN_READ_WRITE_LENGTH);
                if (com_rslt == BNO055_SUCCESS)
                {
                    data1_u8r = ((s8)(gyro_offset->z >> BNO055_SHIFT_EIGHT_BITS) & BNO055_SIC_HEX_0_0_F_F_DATA);
                    data2_u8r = BNO055_SET_BITSLICE(data2_u8r, BNO055_GYRO_OFFSET_Z_MSB, data1_u8r);
                    com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                BNO055_GYRO_OFFSET_Z_MSB_REG,
                                                                &data2_u8r,
                                                                BNO055_GEN_READ_WRITE_LENGTH);
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

/********************************************************/
/************** PAGE1 Functions *********************/
/********************************************************/

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_range(u8 *accel_range_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel range is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel g range */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_RANGE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_range_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_RANGE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_range(u8 accel_range_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (accel_range_u8 < BNO055_ACCEL_RANGE)
                    {
                        /* Write the value of accel range*/
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_ACCEL_RANGE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_RANGE, accel_range_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_ACCEL_RANGE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_bw(u8 *accel_bw_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel bandwidth is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel bandwidth */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_BW_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_bw_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_BW);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_bw(u8 accel_bw_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (accel_bw_u8 < BNO055_ACCEL_GYRO_BW_RANGE)
                    {
                        /* Write the accel */
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_ACCEL_BW_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_BW, accel_bw_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_ACCEL_BW_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_power_mode(u8 *accel_power_mode_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel power mode is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel bandwidth */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_POWER_MODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_power_mode_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_POWER_MODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_power_mode(u8 accel_power_mode_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (accel_power_mode_u8 < BNO055_ACCEL_POWER_MODE_RANGE)
                    {
                        /* Write the value of accel bandwidth*/
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_ACCEL_POWER_MODE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_POWER_MODE, accel_power_mode_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_ACCEL_POWER_MODE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_data_output_rate(u8 *mag_data_output_rate_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, output data rate
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the mag output data rate*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_DATA_OUTPUT_RATE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *mag_data_output_rate_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_MAG_DATA_OUTPUT_RATE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to write the mag output data rate
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_data_output_rate(u8 mag_data_output_rate_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (mag_data_output_rate_u8 < BNO055_MAG_OUTPUT_RANGE)
                    {
                        /* Write the value of
                         * mag output data rate*/
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_MAG_DATA_OUTPUT_RATE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r,
                                                           BNO055_MAG_DATA_OUTPUT_RATE,
                                                           mag_data_output_rate_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_MAG_DATA_OUTPUT_RATE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

/*!
 *  @brief This API used to read the mag operation mode
 *  from page one register from 0x09 bit 3 to 4
 *
 *  @param mag_operation_mode_u8 : The value of mag operation mode
 *
 *  mag_operation_mode_u8  |      result
 * ------------------------- |--------------------------
 *     0x00                  | MAG_OPR_MODE_LOWPOWER
 *     0x01                  | MAG_OPR_MODE_REGULAR
 *     0x02                  | MAG_OPR_MODE_ENHANCED_REGULAR
 *     0x03                  | MAG_OPR_MODE_HIGH_ACCURACY
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_operation_mode(u8 *mag_operation_mode_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, mag operation mode is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of mag operation mode*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_OPERATION_MODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *mag_operation_mode_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_MAG_OPERATION_MODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to write the mag operation mode
 *  from page one register from 0x09 bit 3 to 4
 *
 *  @param mag_operation_mode_u8 : The value of mag operation mode
 *
 *  mag_operation_mode_u8  |      result
 * ------------------------- |--------------------------
 *     0x00                  | MAG_OPR_MODE_LOWPOWER
 *     0x01                  | MAG_OPR_MODE_REGULAR
 *     0x02                  | MAG_OPR_MODE_ENHANCED_REGULAR
 *     0x03                  | MAG_OPR_MODE_HIGH_ACCURACY
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_operation_mode(u8 mag_operation_mode_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (mag_operation_mode_u8 < BNO055_MAG_OPR_MODE_RANGE)
                    {
                        /* Write the value
                         * of mag operation mode*/
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_MAG_OPERATION_MODE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_MAG_OPERATION_MODE, mag_operation_mode_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_MAG_OPERATION_MODE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

/*!
 *  @brief This API used to read the mag power mode
 *  from page one register from 0x09 bit 4 to 6
 *
 *  @param mag_power_mode_u8 : The value of mag power mode
 *
 * mag_power_mode_u8 |   result
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_power_mode(u8 *mag_power_mode_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, mag power mode is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of mag power mode */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_POWER_MODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *mag_power_mode_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_MAG_POWER_MODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to write the mag power mode
 *  from page one register from 0x09 bit 4 to 6
 *
 *  @param mag_power_mode_u8 : The value of mag power mode
 *
 * mag_power_mode_u8 |   result
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_power_mode(u8 mag_power_mode_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (mag_power_mode_u8 < BNO055_MAG_POWER_MODE_RANGE)
                    {
                        /* Write the value of mag power mode*/
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_MAG_POWER_MODE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_MAG_POWER_MODE, mag_power_mode_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_MAG_POWER_MODE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_range(u8 *gyro_range_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro range is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro range */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_RANGE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_range_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_RANGE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_range(u8 gyro_range_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (gyro_range_u8 < BNO055_GYRO_RANGE)
                    {
                        /* Write the value of gyro range*/
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_GYRO_RANGE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_RANGE, gyro_range_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_GYRO_RANGE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_bw(u8 *gyro_bw_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro bandwidth is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_BW_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_bw_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_BW);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_bw(u8 gyro_bw_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 gyro_opmode = BNO055_INIT_VALUE;
    u8 gyro_auto_sleep_durn = BNO055_INIT_VALUE;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of gyro bandwidth */
                    if ((gyro_bw_u8 == BNO055_INIT_VALUE || gyro_bw_u8 > BNO055_INIT_VALUE) &&
                        gyro_bw_u8 < BNO055_ACCEL_GYRO_BW_RANGE)
                    {
                        switch (gyro_bw_u8)
                        {
                            case BNO055_GYRO_BW_523HZ:
                                gyro_bw_u8 = BNO055_GYRO_BW_523HZ;
                                break;
                            case BNO055_GYRO_BW_230HZ:
                                gyro_bw_u8 = BNO055_GYRO_BW_230HZ;
                                break;
                            case BNO055_GYRO_BW_116HZ:
                                gyro_bw_u8 = BNO055_GYRO_BW_116HZ;
                                break;
                            case BNO055_GYRO_BW_47HZ:
                                gyro_bw_u8 = BNO055_GYRO_BW_47HZ;
                                break;
                            case BNO055_GYRO_BW_23HZ:
                                gyro_bw_u8 = BNO055_GYRO_BW_23HZ;
                                break;
                            case BNO055_GYRO_BW_12HZ:
                                gyro_bw_u8 = BNO055_GYRO_BW_12HZ;
                                break;
                            case BNO055_GYRO_BW_64HZ:
                                gyro_bw_u8 = BNO055_GYRO_BW_64HZ;
                                break;
                            case BNO055_GYRO_BW_32HZ:
                                gyro_bw_u8 = BNO055_GYRO_BW_32HZ;
                                break;
                            default:
                                break;
                        }
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_GYRO_BW_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_BW, gyro_bw_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_GYRO_BW_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                        com_rslt = bno055_get_gyro_power_mode(&gyro_opmode);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            if (gyro_opmode == BNO055_GYRO_POWER_MODE_ADVANCE_POWERSAVE)
                            {
                                com_rslt += bno055_get_gyro_auto_sleep_durn(&gyro_auto_sleep_durn);
                                if (com_rslt == BNO055_SUCCESS)
                                {
                                    com_rslt += bno055_gyro_set_auto_sleep_durn(gyro_auto_sleep_durn, gyro_bw_u8);
                                }
                            }
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_power_mode(u8 *gyro_power_mode_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro power mode is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Write the value of gyro power mode*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_POWER_MODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_power_mode_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_POWER_MODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_power_mode(u8 gyro_power_mode_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 gyro_auto_sleep_durn = BNO055_INIT_VALUE;
    u8 gyro_bw_u8 = BNO055_INIT_VALUE;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of power mode*/
                    if ((gyro_power_mode_u8 == BNO055_INIT_VALUE || gyro_power_mode_u8 > BNO055_INIT_VALUE) &&
                        gyro_power_mode_u8 < BNO055_GYRO_RANGE)
                    {
                        switch (gyro_power_mode_u8)
                        {
                            case BNO055_GYRO_POWER_MODE_NORMAL:
                                gyro_power_mode_u8 = BNO055_GYRO_POWER_MODE_NORMAL;
                                break;
                            case BNO055_GYRO_POWER_MODE_FASTPOWERUP:
                                gyro_power_mode_u8 = BNO055_GYRO_POWER_MODE_FASTPOWERUP;
                                break;
                            case BNO055_GYRO_POWER_MODE_DEEPSUSPEND:
                                gyro_power_mode_u8 = BNO055_GYRO_POWER_MODE_DEEPSUSPEND;
                                break;
                            case BNO055_GYRO_POWER_MODE_SUSPEND:
                                gyro_power_mode_u8 = BNO055_GYRO_POWER_MODE_SUSPEND;
                                break;
                            case BNO055_GYRO_POWER_MODE_ADVANCE_POWERSAVE:
                                com_rslt = bno055_get_gyro_bw(&gyro_bw_u8);
                                com_rslt += bno055_get_gyro_auto_sleep_durn(&gyro_auto_sleep_durn);
                                if (com_rslt == BNO055_SUCCESS)
                                {
                                    com_rslt += bno055_gyro_set_auto_sleep_durn(gyro_auto_sleep_durn, gyro_bw_u8);
                                }
                                com_rslt += bno055_write_page_id(BNO055_PAGE_ONE);
                                gyro_power_mode_u8 = BNO055_GYRO_POWER_MODE_ADVANCE_POWERSAVE;
                                break;
                            default:
                                break;
                        }
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_GYRO_POWER_MODE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_POWER_MODE, gyro_power_mode_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_GYRO_POWER_MODE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleep_tmr_mode(u8 *sleep_tmr_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel sleep mode is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* read the value of accel sleep mode */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_SLEEP_MODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *sleep_tmr_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_SLEEP_MODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleep_tmr_mode(u8 sleep_tmr_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (sleep_tmr_u8 < BNO055_ACCEL_SLEEP_MODE_RANGE)
                    {
                        /*Write the value
                         * of accel sleep mode*/
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_ACCEL_SLEEP_MODE_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_SLEEP_MODE, sleep_tmr_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_ACCEL_SLEEP_MODE_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleep_durn(u8 *sleep_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel sleep duration
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel sleep duration */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_SLEEP_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *sleep_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_SLEEP_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to write the accel sleep duration
 *  from page one register from 0x0C bit 1 to 4
 *
 *  @param sleep_durn_u8 : The value of accel sleep duration
 *
 * sleep_durn_u8  |      result
 * ---------------|-----------------------------
 *     0x05       | BNO055_ACCEL_SLEEP_DURN_0_5MS
 *     0x06       | BNO055_ACCEL_SLEEP_DURN_1MS
 *     0x07       | BNO055_ACCEL_SLEEP_DURN_2MS
 *     0x08       | BNO055_ACCEL_SLEEP_DURN_4MS
 *     0x09       | BNO055_ACCEL_SLEEP_DURN_6MS
 *     0x0A       | BNO055_ACCEL_SLEEP_DURN_10MS
 *     0x0B       | BNO055_ACCEL_SLEEP_DURN_25MS
 *     0x0C       | BNO055_ACCEL_SLEEP_DURN_50MS
 *     0x0D       | BNO055_ACCEL_SLEEP_DURN_100MS
 *     0x0E       | BNO055_ACCEL_SLEEP_DURN_500MS
 *     0x0F       | BNO055_ACCEL_SLEEP_DURN_1S
 *
 *  @return results of bus communication function
 *  @retval 0 -> BNO055_SUCCESS
 *  @retval 1 -> BNO055_ERROR
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleep_durn(u8 sleep_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (sleep_durn_u8 < BNO055_ACCEL_SLEEP_DURATION_RANGE)
                    {
                        /* Write the accel
                        * sleep duration*/
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_ACCEL_SLEEP_DURN_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_SLEEP_DURN, sleep_durn_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_ACCEL_SLEEP_DURN_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_sleep_durn(u8 *sleep_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel range is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the gyro sleep duration */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_SLEEP_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *sleep_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_SLEEP_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_sleep_durn(u8 sleep_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    if (sleep_durn_u8 < BNO055_GYRO_AUTO_SLEEP_DURATION_RANGE)
                    {
                        com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                  BNO055_GYRO_SLEEP_DURN_REG,
                                                                  &data_u8r,
                                                                  BNO055_GEN_READ_WRITE_LENGTH);
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            /* Write the gyro
                             *  sleep duration */
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_SLEEP_DURN, sleep_durn_u8);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_GYRO_SLEEP_DURN_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_auto_sleep_durn(u8 *auto_sleep_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel range is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro auto sleep duration */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_AUTO_SLEEP_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *auto_sleep_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_AUTO_SLEEP_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_gyro_set_auto_sleep_durn(u8 auto_sleep_durn_u8, u8 bw)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 auto_sleep_durn_u8r;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of gyro sleep duration */
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_AUTO_SLEEP_DURN_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (auto_sleep_durn_u8 < BNO055_GYRO_AUTO_SLEEP_DURATION_RANGE)
                    {
                        switch (bw)
                        {
                            case BNO055_GYRO_BW_523HZ:
                                if (auto_sleep_durn_u8 > BNO055_GYRO_4MS_AUTOSLPDUR)
                                {
                                    auto_sleep_durn_u8r = auto_sleep_durn_u8;
                                }
                                else
                                {
                                    auto_sleep_durn_u8r = BNO055_GYRO_4MS_AUTOSLPDUR;
                                }
                                break;
                            case BNO055_GYRO_BW_230HZ:
                                if (auto_sleep_durn_u8 > BNO055_GYRO_4MS_AUTOSLPDUR)
                                {
                                    auto_sleep_durn_u8r = auto_sleep_durn_u8;
                                }
                                else
                                {
                                    auto_sleep_durn_u8r = BNO055_GYRO_4MS_AUTOSLPDUR;
                                }
                                break;
                            case BNO055_GYRO_BW_116HZ:
                                if (auto_sleep_durn_u8 > BNO055_GYRO_4MS_AUTOSLPDUR)
                                {
                                    auto_sleep_durn_u8r = auto_sleep_durn_u8;
                                }
                                else
                                {
                                    auto_sleep_durn_u8r = BNO055_GYRO_4MS_AUTOSLPDUR;
                                }
                                break;
                            case BNO055_GYRO_BW_47HZ:
                                if (auto_sleep_durn_u8 > BNO055_GYRO_5MS_AUTOSLPDUR)
                                {
                                    auto_sleep_durn_u8r = auto_sleep_durn_u8;
                                }
                                else
                                {
                                    auto_sleep_durn_u8r = BNO055_GYRO_5MS_AUTOSLPDUR;
                                }
                                break;
                            case BNO055_GYRO_BW_23HZ:
                                if (auto_sleep_durn_u8 > BNO055_GYRO_10MS_AUTOSLPDUR)
                                {
                                    auto_sleep_durn_u8r = auto_sleep_durn_u8;
                                }
                                else
                                {
                                    auto_sleep_durn_u8r = BNO055_GYRO_10MS_AUTOSLPDUR;
                                }
                                break;
                            case BNO055_GYRO_BW_12HZ:
                                if (auto_sleep_durn_u8 > BNO055_GYRO_20MS_AUTOSLPDUR)
                                {
                                    auto_sleep_durn_u8r = auto_sleep_durn_u8;
                                }
                                else
                                {
                                    auto_sleep_durn_u8r = BNO055_GYRO_20MS_AUTOSLPDUR;
                                }
                                break;
                            case BNO055_GYRO_BW_64HZ:
                                if (auto_sleep_durn_u8 > BNO055_GYRO_10MS_AUTOSLPDUR)
                                {
                                    auto_sleep_durn_u8r = auto_sleep_durn_u8;
                                }
                                else
                                {
                                    auto_sleep_durn_u8r = BNO055_GYRO_10MS_AUTOSLPDUR;
                                }
                                break;
                            case BNO055_GYRO_BW_32HZ:
                                if (auto_sleep_durn_u8 > BNO055_GYRO_20MS_AUTOSLPDUR)
                                {
                                    auto_sleep_durn_u8r = auto_sleep_durn_u8;
                                }
                                else
                                {
                                    auto_sleep_durn_u8r = BNO055_GYRO_20MS_AUTOSLPDUR;
                                }
                                break;
                            default:
                                if (auto_sleep_durn_u8 > BNO055_GYRO_4MS_AUTOSLPDUR)
                                {
                                    auto_sleep_durn_u8r = auto_sleep_durn_u8;
                                }
                                else
                                {
                                    auto_sleep_durn_u8r = BNO055_GYRO_4MS_AUTOSLPDUR;
                                }
                                break;
                        }
                        if (com_rslt == BNO055_SUCCESS)
                        {
                            data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_AUTO_SLEEP_DURN, auto_sleep_durn_u8r);
                            com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                        BNO055_GYRO_AUTO_SLEEP_DURN_REG,
                                                                        &data_u8r,
                                                                        BNO055_GEN_READ_WRITE_LENGTH);
                        }
                    }
                    else
                    {
                        com_rslt = BNO055_OUT_OF_RANGE;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_mode(u8 *sleep_mode_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page,mag sleep mode is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of mag sleep mode*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_SLEEP_MODE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *sleep_mode_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_MAG_SLEEP_MODE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_mode(u8 sleep_mode_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_MAG_SLEEP_MODE_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        /* Write the value
                         * of mag sleep mode*/
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_MAG_SLEEP_MODE, sleep_mode_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_MAG_SLEEP_MODE_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_durn(u8 *sleep_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page,mag sleep duration is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of mag sleep duration*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_MAG_SLEEP_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *sleep_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_MAG_SLEEP_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_durn(u8 sleep_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_MAG_SLEEP_DURN_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        /* Write the value of
                         *  mag sleep duration */
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_MAG_SLEEP_DURN, sleep_durn_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_MAG_SLEEP_DURN_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_gyro_any_motion(u8 *gyro_any_motion_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro anymotion interrupt mask is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro anymotion interrupt mask*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_ANY_MOTION_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_any_motion_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_INTR_MASK);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_gyro_any_motion(u8 gyro_any_motion_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel range is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Write the value of gyro anymotion interrupt mask*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_ANY_MOTION_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_INTR_MASK, gyro_any_motion_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_GYRO_ANY_MOTION_INTR_MASK_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
 *  the following API
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_gyro_highrate(u8 *gyro_highrate_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate interrupt mask is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate interrupt mask*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_INTR_MASK);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_gyro_highrate(u8 gyro_highrate_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate interrupt mask is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Write the value of gyro
                 * highrate interrupt mask*/
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_INTR_MASK, gyro_highrate_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_GYRO_HIGHRATE_INTR_MASK_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_accel_high_g(u8 *accel_high_g_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel highg interrupt mask is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel highg interrupt mask*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_HIGH_G_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_high_g_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_INTR_MASK);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_accel_high_g(u8 accel_high_g_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel highg interrupt mask is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_HIGH_G_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Write the value of accel
                 * highg interrupt mask*/
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_INTR_MASK, accel_high_g_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_ACCEL_HIGH_G_INTR_MASK_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_accel_any_motion(u8 *accel_any_motion_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel anymotion interrupt mask is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* The value of accel anymotion interrupt mask*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_ANY_MOTION_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_any_motion_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_INTR_MASK);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_accel_any_motion(u8 accel_any_motion_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel anymotion interrupt mask is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Write the value of accel anymotion interrupt mask*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_ANY_MOTION_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_INTR_MASK, accel_any_motion_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_ACCEL_ANY_MOTION_INTR_MASK_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_accel_no_motion(u8 *accel_nomotion_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel nomotion interrupt mask is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel nomotion interrupt mask*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_NO_MOTION_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_nomotion_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_NO_MOTION_INTR_MASK);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_accel_no_motion(u8 accel_nomotion_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel
         * nomotion interrupt mask is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_NO_MOTION_INTR_MASK_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Write the value of accel
                * nomotion interrupt mask*/
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_NO_MOTION_INTR_MASK, accel_nomotion_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_ACCEL_NO_MOTION_INTR_MASK_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_gyro_any_motion(u8 *gyro_any_motion_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro anymotion interrupt  is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro anymotion interrupt */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_ANY_MOTION_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_any_motion_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_INTR);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_gyro_any_motion(u8 gyro_any_motion_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro anymotion interrupt  is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Write the value of gyro anymotion interrupt */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_ANY_MOTION_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_INTR, gyro_any_motion_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_GYRO_ANY_MOTION_INTR_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_gyro_highrate(u8 *gyro_highrate_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate interrupt is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate interrupt */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_INTR);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_gyro_highrate(u8 gyro_highrate_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate interrupt is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Write the value of gyro highrate interrupt */
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_INTR, gyro_highrate_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_GYRO_HIGHRATE_INTR_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_accel_high_g(u8 *accel_high_g_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel highg interrupt  is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel highg interrupt*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_HIGH_G_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_high_g_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_INTR);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_accel_high_g(u8 accel_high_g_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel highg interrupt is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_HIGH_G_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Write the value of accel highg interrupt*/
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_INTR, accel_high_g_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_ACCEL_HIGH_G_INTR_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_accel_any_motion(u8 *accel_any_motion_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel anymotion interrupt  is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel anymotion interrupt */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_ANY_MOTION_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_any_motion_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_INTR);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_accel_any_motion(u8 accel_any_motion_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel range is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Write the value of accel anymotion interrupt */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_ANY_MOTION_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_INTR, accel_any_motion_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_ACCEL_ANY_MOTION_INTR_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_accel_no_motion(u8 *accel_nomotion_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel nomotion interrupt is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel nomotion interrupt*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_NO_MOTION_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_nomotion_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_NO_MOTION_INTR);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_accel_no_motion(u8 accel_nomotion_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page,
         *  accel nomotion interrupt is
         *  available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_NO_MOTION_INTR_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            if (com_rslt == BNO055_SUCCESS)
            {
                /* Write the value of
                 * accel nomotion interrupt */
                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_NO_MOTION_INTR, accel_nomotion_u8);
                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                            BNO055_ACCEL_NO_MOTION_INTR_REG,
                                                            &data_u8r,
                                                            BNO055_GEN_READ_WRITE_LENGTH);
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_any_motion_thres(u8 *accel_any_motion_thres_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel any motion threshold  is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel any motion threshold */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_ANY_MOTION_THRES_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_any_motion_thres_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_THRES);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_any_motion_thres(u8 accel_any_motion_thres_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_ANY_MOTION_THRES_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        /* Write the value of
                         * accel any motion threshold*/
                        data_u8r = BNO055_SET_BITSLICE(data_u8r,
                                                       BNO055_ACCEL_ANY_MOTION_THRES,
                                                       accel_any_motion_thres_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_ACCEL_ANY_MOTION_THRES_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_any_motion_durn(u8 *accel_any_motion_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel anymotion duration  is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel anymotion duration */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_ANY_MOTION_DURN_SET_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_any_motion_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_DURN_SET);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_any_motion_durn(u8 accel_any_motion_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_ANY_MOTION_DURN_SET_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        /* Write the value of
                         * accel anymotion duration*/
                        data_u8r = BNO055_SET_BITSLICE(data_u8r,
                                                       BNO055_ACCEL_ANY_MOTION_DURN_SET,
                                                       accel_any_motion_durn_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_ACCEL_ANY_MOTION_DURN_SET_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_any_motion_no_motion_axis_enable(u8 channel_u8, u8 *data_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel anymotion enable is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            switch (channel_u8)
            {
                case BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS:

                    /* Read the value of accel anymotion x enable*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_ANY_MOTION_X_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_X_AXIS);
                    break;
                case BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS:

                    /* Read the value of accel anymotion y enable*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_ANY_MOTION_Y_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_Y_AXIS);
                    break;
                case BNO055_ACCEL_ANY_MOTION_NO_MOTION_Z_AXIS:

                    /* Read the value of accel anymotion z enable*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_ANY_MOTION_Z_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_Z_AXIS);
                    break;
                default:
                    com_rslt = BNO055_OUT_OF_RANGE;
                    break;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_any_motion_no_motion_axis_enable(u8 channel_u8, u8 data_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    switch (channel_u8)
                    {
                        case BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS:

                            /* Write the value of
                             * accel anymotion x enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_ACCEL_ANY_MOTION_X_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_X_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_ACCEL_ANY_MOTION_X_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        case BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS:

                            /* Write the value of
                             * accel anymotion y enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_ACCEL_ANY_MOTION_Y_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_Y_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_ACCEL_ANY_MOTION_Y_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        case BNO055_ACCEL_ANY_MOTION_NO_MOTION_Z_AXIS:

                            /* Write the value of
                             * accel anymotion z enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_ACCEL_ANY_MOTION_Z_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_ANY_MOTION_Z_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_ACCEL_ANY_MOTION_Z_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        default:
                            com_rslt = BNO055_OUT_OF_RANGE;
                            break;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_axis_enable(u8 channel_u8, u8 *data_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel highg enable is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            switch (channel_u8)
            {
                case BNO055_ACCEL_HIGH_G_X_AXIS:

                    /* Read the value of accel x highg enable*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_HIGH_G_X_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_X_AXIS);
                    break;
                case BNO055_ACCEL_HIGH_G_Y_AXIS:

                    /* Read the value of accel y highg enable*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_HIGH_G_Y_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_Y_AXIS);
                    break;
                case BNO055_ACCEL_HIGH_G_Z_AXIS:

                    /* Read the value of accel z highg enable*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_HIGH_G_Z_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_Z_AXIS);
                    break;
                default:
                    com_rslt = BNO055_OUT_OF_RANGE;
                    break;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_axis_enable(u8 channel_u8, u8 data_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    switch (channel_u8)
                    {
                        case BNO055_ACCEL_HIGH_G_X_AXIS:

                            /* Write the value of
                             * accel x highg enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_ACCEL_HIGH_G_X_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_X_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_ACCEL_HIGH_G_X_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        case BNO055_ACCEL_HIGH_G_Y_AXIS:

                            /* Write the value of
                             * accel y highg enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_ACCEL_HIGH_G_Y_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_Y_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_ACCEL_HIGH_G_Y_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        case BNO055_ACCEL_HIGH_G_Z_AXIS:

                            /* Write the value of
                             * accel z highg enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_ACCEL_HIGH_G_Z_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_Z_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_ACCEL_HIGH_G_Z_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        default:
                            com_rslt = BNO055_OUT_OF_RANGE;
                            break;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
 *  to [highg duration  + 1] * 2 ms
 *
 *  in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_durn(u8 *accel_high_g_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel highg duration is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel highg duration*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_HIGH_G_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_high_g_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
 *  to [highg duration  + 1] * 2 ms
 *
 *  in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_durn(u8 accel_high_g_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_HIGH_G_DURN_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        /* Write the value of
                         * accel highg duration*/
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_DURN, accel_high_g_durn_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_ACCEL_HIGH_G_DURN_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_thres(u8 *accel_high_g_thres_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, highg threshold is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of highg threshold */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_HIGH_G_THRES_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_high_g_thres_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_THRES);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_thres(u8 accel_high_g_thres_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_HIGH_G_THRES_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        /* Write the value of
                         * accel highg threshold */
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_ACCEL_HIGH_G_THRES, accel_high_g_thres_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_ACCEL_HIGH_G_THRES_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_thres(u8 *accel_slow_no_motion_thres_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel slownomotion threshold is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of slownomotion threshold */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_SLOW_NO_MOTION_THRES_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_slow_no_motion_thres_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_SLOW_NO_MOTION_THRES);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_thres(u8 accel_slow_no_motion_thres_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of
                     * slownomotion threshold */
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_SLOW_NO_MOTION_THRES_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r,
                                                       BNO055_ACCEL_SLOW_NO_MOTION_THRES,
                                                       accel_slow_no_motion_thres_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_ACCEL_SLOW_NO_MOTION_THRES_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_enable(u8 *accel_slow_no_motion_en_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel slownomotion enable is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of accel slownomotion enable */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_SLOW_NO_MOTION_ENABLE_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_slow_no_motion_en_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_SLOW_NO_MOTION_ENABLE);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_enable(u8 accel_slow_no_motion_en_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_SLOW_NO_MOTION_ENABLE_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        /* Read the value of
                         * accel slownomotion enable */
                        data_u8r = BNO055_SET_BITSLICE(data_u8r,
                                                       BNO055_ACCEL_SLOW_NO_MOTION_ENABLE,
                                                       accel_slow_no_motion_en_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_ACCEL_SLOW_NO_MOTION_ENABLE_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_durn(u8 *accel_slow_no_motion_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, accel slownomotion duration is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /*read value of accel slownomotion duration*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_ACCEL_SLOW_NO_MOTION_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *accel_slow_no_motion_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_SLOW_NO_MOTION_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_durn(u8 accel_slow_no_motion_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_ACCEL_SLOW_NO_MOTION_DURN_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        /*Write the value of accel
                         * slownomotion duration*/
                        data_u8r = BNO055_SET_BITSLICE(data_u8r,
                                                       BNO055_ACCEL_SLOW_NO_MOTION_DURN,
                                                       accel_slow_no_motion_durn_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_ACCEL_SLOW_NO_MOTION_DURN_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_axis_enable(u8 channel_u8, u8 *data_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro anymotion axis is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            switch (channel_u8)
            {
                case BNO055_GYRO_ANY_MOTION_X_AXIS:

                    /* Read the gyro anymotion x enable*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_ANY_MOTION_X_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_X_AXIS);
                    break;
                case BNO055_GYRO_ANY_MOTION_Y_AXIS:

                    /* Read the gyro anymotion y enable*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_ANY_MOTION_Y_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_Y_AXIS);
                    break;
                case BNO055_GYRO_ANY_MOTION_Z_AXIS:

                    /* Read the gyro anymotion z enable*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_ANY_MOTION_Z_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_Z_AXIS);
                    break;
                default:
                    com_rslt = BNO055_OUT_OF_RANGE;
                    break;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_axis_enable(u8 channel_u8, u8 data_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    switch (channel_u8)
                    {
                        case BNO055_GYRO_ANY_MOTION_X_AXIS:

                            /* Write the gyro
                             * anymotion x enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_GYRO_ANY_MOTION_X_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_X_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_GYRO_ANY_MOTION_X_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        case BNO055_GYRO_ANY_MOTION_Y_AXIS:

                            /* Write the gyro
                             * anymotion y enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_GYRO_ANY_MOTION_Y_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_Y_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_GYRO_ANY_MOTION_Y_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        case BNO055_GYRO_ANY_MOTION_Z_AXIS:

                            /* Write the gyro
                             * anymotion z enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_GYRO_ANY_MOTION_Z_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_Z_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_GYRO_ANY_MOTION_Z_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        default:
                            com_rslt = BNO055_OUT_OF_RANGE;
                            break;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_axis_enable(u8 channel_u8, u8 *data_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate enable is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            switch (channel_u8)
            {
                case BNO055_GYRO_HIGHRATE_X_AXIS:

                    /* Read the gyro highrate x enable */
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_X_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_X_AXIS);
                    break;
                case BNO055_GYRO_HIGHRATE_Y_AXIS:

                    /* Read the gyro highrate y enable */
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_Y_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Y_AXIS);
                    break;
                case BNO055_GYRO_HIGHRATE_Z_AXIS:

                    /* Read the gyro highrate z enable */
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_Z_AXIS_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    *data_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Z_AXIS);
                    break;
                default:
                    com_rslt = BNO055_OUT_OF_RANGE;
                    break;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_axis_enable(u8 channel_u8, u8 data_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    switch (channel_u8)
                    {
                        case BNO055_GYRO_HIGHRATE_X_AXIS:

                            /* Write the value of
                             * gyro highrate x enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_GYRO_HIGHRATE_X_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_X_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_GYRO_HIGHRATE_X_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        case BNO055_GYRO_HIGHRATE_Y_AXIS:

                            /* Write the value of
                             * gyro highrate y enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_GYRO_HIGHRATE_Y_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Y_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_GYRO_HIGHRATE_Y_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        case BNO055_GYRO_HIGHRATE_Z_AXIS:

                            /* Write the value of
                             * gyro highrate z enable*/
                            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                                      BNO055_GYRO_HIGHRATE_Z_AXIS_REG,
                                                                      &data_u8r,
                                                                      BNO055_GEN_READ_WRITE_LENGTH);
                            if (com_rslt == BNO055_SUCCESS)
                            {
                                data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Z_AXIS, data_u8);
                                com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                            BNO055_GYRO_HIGHRATE_Z_AXIS_REG,
                                                                            &data_u8r,
                                                                            BNO055_GEN_READ_WRITE_LENGTH);
                            }
                            break;
                        default:
                            com_rslt = BNO055_OUT_OF_RANGE;
                            break;
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_filter(u8 *gyro_any_motion_filter_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro anymotion filter is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro anymotion filter*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_ANY_MOTION_FILTER_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_any_motion_filter_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_FILTER);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_filter(u8 gyro_any_motion_filter_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of
                     * gyro anymotion filter*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_ANY_MOTION_FILTER_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r,
                                                       BNO055_GYRO_ANY_MOTION_FILTER,
                                                       gyro_any_motion_filter_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_ANY_MOTION_FILTER_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_filter(u8 *gyro_highrate_filter_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate filter is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate filter */
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_FILTER_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_filter_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_FILTER);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_filter(u8 gyro_highrate_filter_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of
                     * gyro highrate filter*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_FILTER_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_FILTER, gyro_highrate_filter_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_FILTER_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_thres(u8 *gyro_highrate_x_thres_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate x threshold is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate threshold*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_X_THRES_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_x_thres_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_X_THRES);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_thres(u8 gyro_highrate_x_thres_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of
                     * gyro highrate x threshold*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_X_THRES_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r =
                            BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_X_THRES, gyro_highrate_x_thres_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_X_THRES_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_hyst(u8 *gyro_highrate_x_hyst_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page,gyro highrate x hysteresis is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate x hysteresis*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_X_HYST_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_x_hyst_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_X_HYST);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_hyst(u8 gyro_highrate_x_hyst_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /*Write the value of
                     * gyro highrate x hysteresis*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_X_HYST_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_X_HYST, gyro_highrate_x_hyst_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_X_HYST_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_durn(u8 *gyro_highrate_x_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate x duration is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate x duration*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_X_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_x_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_X_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_durn(u8 gyro_highrate_x_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value
                     * of gyro highrate x duration*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_X_DURN_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_X_DURN, gyro_highrate_x_durn_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_X_DURN_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_thres(u8 *gyro_highrate_y_thres_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate y threshold is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate y threshold*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_Y_THRES_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_y_thres_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Y_THRES);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_thres(u8 gyro_highrate_y_thres_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value
                     * of gyro highrate y threshold*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_Y_THRES_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r =
                            BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Y_THRES, gyro_highrate_y_thres_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_Y_THRES_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_hyst(u8 *gyro_highrate_y_hyst_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate y hysteresis is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate y hysteresis*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_Y_HYST_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_y_hyst_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Y_HYST);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_hyst(u8 gyro_highrate_y_hyst_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of
                     * gyro highrate y hysteresis*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_Y_HYST_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Y_HYST, gyro_highrate_y_hyst_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_Y_HYST_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_durn(u8 *gyro_highrate_y_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate y duration is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate y duration*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_Y_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_y_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Y_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_durn(u8 gyro_highrate_y_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value
                     * of gyro highrate y duration*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_Y_DURN_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Y_DURN, gyro_highrate_y_durn_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_Y_DURN_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_thres(u8 *gyro_highrate_z_thres_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate z threshold is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate z threshold*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_Z_THRES_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_z_thres_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Z_THRES);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_thres(u8 gyro_highrate_z_thres_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value
                     * of gyro highrate z threshold*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_Z_THRES_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r =
                            BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Z_THRES, gyro_highrate_z_thres_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_Z_THRES_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_hyst(u8 *gyro_highrate_z_hyst_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate z hysteresis is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate z hysteresis*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_Z_HYST_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_z_hyst_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Z_HYST);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_hyst(u8 gyro_highrate_z_hyst_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value
                     * of gyro highrate z hysteresis*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_Z_HYST_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Z_HYST, gyro_highrate_z_hyst_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_Z_HYST_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_durn(u8 *gyro_highrate_z_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro highrate z duration is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro highrate z duration*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_HIGHRATE_Z_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_highrate_z_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Z_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_durn(u8 gyro_highrate_z_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of
                     * gyro highrate z duration*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_HIGHRATE_Z_DURN_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_HIGHRATE_Z_DURN, gyro_highrate_z_durn_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_HIGHRATE_Z_DURN_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_thres(u8 *gyro_any_motion_thres_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page,gyro anymotion threshold is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro anymotion threshold*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_ANY_MOTION_THRES_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_any_motion_thres_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_THRES);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_thres(u8 gyro_any_motion_thres_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 pg_stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value
                     * of gyro anymotion threshold*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_ANY_MOTION_THRES_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r =
                            BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_ANY_MOTION_THRES, gyro_any_motion_thres_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_ANY_MOTION_THRES_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_slope_samples(u8 *gyro_any_motion_slope_samples_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro anymotion slope samples is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /*Read the value of gyro anymotion slope samples*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_SLOPE_SAMPLES_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_any_motion_slope_samples_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_SLOPE_SAMPLES);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

/*!
 *  @brief This API used to write gyro anymotion slope samples
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_slope_samples(u8 gyro_any_motion_slope_samples_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of
                     * gyro anymotion slope samples*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_SLOPE_SAMPLES_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r,
                                                       BNO055_GYRO_SLOPE_SAMPLES,
                                                       gyro_any_motion_slope_samples_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_SLOPE_SAMPLES_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode of
         * previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_awake_durn(u8 *gyro_awake_durn_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /*condition check for page, gyro anymotion awake duration is
         * available in the page one*/
        if (p_bno055->page_id != BNO055_PAGE_ONE)
        {
            /* Write page as one */
            stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
        }
        if ((stat_s8 == BNO055_SUCCESS) || (p_bno055->page_id == BNO055_PAGE_ONE))
        {
            /* Read the value of gyro anymotion awake duration*/
            com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                      BNO055_GYRO_AWAKE_DURN_REG,
                                                      &data_u8r,
                                                      BNO055_GEN_READ_WRITE_LENGTH);
            *gyro_awake_durn_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_AWAKE_DURN);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }

    return com_rslt;
}

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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_awake_durn(u8 gyro_awake_durn_u8)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;
    s8 pg_stat_s8 = BNO055_ERROR;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;

    /* Check the struct p_bno055 is empty */
    if (p_bno055 == NULL)
    {
        return BNO055_E_NULL_PTR;
    }
    else
    {
        /* The write operation effective only if the operation
         * mode is in config mode, this part of code is checking the
         * current operation mode and set the config mode */
        stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
        if (stat_s8 == BNO055_SUCCESS)
        {
            if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
            {
                stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
            }
            if (stat_s8 == BNO055_SUCCESS)
            {
                /* Write page as one */
                pg_stat_s8 = bno055_write_page_id(BNO055_PAGE_ONE);
                if (pg_stat_s8 == BNO055_SUCCESS)
                {
                    /* Write the value of gyro
                     *  anymotion awake duration*/
                    com_rslt = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
                                                              BNO055_GYRO_AWAKE_DURN_REG,
                                                              &data_u8r,
                                                              BNO055_GEN_READ_WRITE_LENGTH);
                    if (com_rslt == BNO055_SUCCESS)
                    {
                        data_u8r = BNO055_SET_BITSLICE(data_u8r, BNO055_GYRO_AWAKE_DURN, gyro_awake_durn_u8);
                        com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC(p_bno055->dev_addr,
                                                                    BNO055_GYRO_AWAKE_DURN_REG,
                                                                    &data_u8r,
                                                                    BNO055_GEN_READ_WRITE_LENGTH);
                    }
                }
                else
                {
                    com_rslt = BNO055_ERROR;
                }
            }
            else
            {
                com_rslt = BNO055_ERROR;
            }
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}
