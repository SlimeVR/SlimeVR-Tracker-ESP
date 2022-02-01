/*
===============================================
BMI160 accelerometer/gyroscope library for Intel(R) Curie(TM) devices.
Copyright (c) 2015 Intel Corporation.  All rights reserved.
Based on MPU6050 Arduino library provided by Jeff Rowberg as part of his
excellent I2Cdev device library: https://github.com/jrowberg/i2cdevlib
===============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
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
===============================================
*/
#include "BMI160.h"
#include "I2Cdev.h"

#define BMI160_CHIP_ID 0xD1

#define BMI160_ACCEL_POWERUP_DELAY_MS 10
#define BMI160_GYRO_POWERUP_DELAY_MS 100

/* Test the sign bit and set remaining MSBs if sign bit is set */
#define BMI160_SIGN_EXTEND(val, from) \
    (((val) & (1 << ((from) - 1))) ? (val | (((1 << (1 + (sizeof(val) << 3) - (from))) - 1) << (from))) : val)


/******************************************************************************/

class I2CdevMod : public I2Cdev {
    public:
        static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
            uint8_t b;
            if (readByte(devAddr, regAddr, &b) != 0) {
                uint8_t mask = ((1 << length) - 1) << bitStart;
                data <<= bitStart; // shift data into correct position
                data &= mask; // zero all non-important bits in data
                b &= ~(mask); // zero all important bits in existing byte
                b |= data; // combine data with existing byte
                return writeByte(devAddr, regAddr, b);
            } else {
                return false;
            }
        }
        static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout) {
            uint8_t count, b;
            if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
                uint8_t mask = ((1 << length) - 1) << bitStart;
                b &= mask;
                b >>= bitStart;
                *data = b;
            }
            return count;
        }
};

BMI160::BMI160() {};

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up).
 */
void BMI160::initialize(uint8_t addr)
{
    devAddr = addr;
    /* Issue a soft-reset to bring the device into a clean state */
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_SOFT_RESET);
    delay(1);

    /* Power up the accelerometer */
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_ACC_MODE_NORMAL);
    delay(BMI160_ACCEL_POWERUP_DELAY_MS);

    /* Power up the gyroscope */
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_GYR_MODE_NORMAL);
    delay(BMI160_GYRO_POWERUP_DELAY_MS);

    setGyroRate(BMI160_GYRO_RATE_800HZ);
    delay(1);
    setAccelRate(BMI160_ACCEL_RATE_800HZ);
    delay(1);
    setFullScaleGyroRange(BMI160_GYRO_RANGE_500);
    delay(1);
    setFullScaleAccelRange(BMI160_ACCEL_RANGE_4G);
    delay(1);
    setGyroDLPFMode(BMI160_DLPF_MODE_OSR4);
    delay(1);
    setAccelDLPFMode(BMI160_DLPF_MODE_OSR4);
    delay(1);

    /* Only PIN1 interrupts currently supported - map all interrupts to PIN1 */
    I2CdevMod::writeByte(devAddr, BMI160_RA_INT_MAP_0, 0xFF);
    I2CdevMod::writeByte(devAddr, BMI160_RA_INT_MAP_1, 0xF0);
    I2CdevMod::writeByte(devAddr, BMI160_RA_INT_MAP_2, 0x00);
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b11010001, 0xD1).
 * @return Device ID (should be 0xD1)
 * @see BMI160_RA_CHIP_ID
 */
uint8_t BMI160::getDeviceID() {
    I2CdevMod::readByte(devAddr, BMI160_RA_CHIP_ID, buffer);
    return buffer[0];
}

/** Verify the SPI connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool BMI160::testConnection()
{
    return (BMI160_CHIP_ID == getDeviceID());
}

/** Set gyroscope output data rate.
 * @param rate New output data rate
 * @see getGyroRate()
 * @see BMI160_GYRO_RATE_25HZ
 * @see BMI160_RA_GYRO_CONF
 */
void BMI160::setGyroRate(uint8_t rate) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_GYRO_CONF,
                   BMI160_GYRO_RATE_SEL_BIT,
                   BMI160_GYRO_RATE_SEL_LEN, rate);
}

/** Set accelerometer output data rate.
 * @param rate New output data rate
 * @see getAccelRate()
 * @see BMI160_RA_ACCEL_CONF
 */
void BMI160::setAccelRate(uint8_t rate) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_ACCEL_CONF,
                   BMI160_ACCEL_RATE_SEL_BIT,
                   BMI160_ACCEL_RATE_SEL_LEN, rate);
}

/** Set gyroscope digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getGyroDLPFMode()
 */
void BMI160::setGyroDLPFMode(uint8_t mode) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_GYRO_CONF,
                          BMI160_GYRO_DLPF_SEL_BIT,
                          BMI160_GYRO_DLPF_SEL_LEN, mode);
}

/** Set accelerometer digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getAccelDLPFMode()
 */
void BMI160::setAccelDLPFMode(uint8_t mode) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_ACCEL_CONF,
                          BMI160_ACCEL_DLPF_SEL_BIT,
                          BMI160_ACCEL_DLPF_SEL_LEN, mode);
}

/** Get full-scale gyroscope range.
 * The gyr_range parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 4 = +/-  125 degrees/sec
 * 3 = +/-  250 degrees/sec
 * 2 = +/-  500 degrees/sec
 * 1 = +/- 1000 degrees/sec
 * 0 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see BMI160_RA_GYRO_RANGE
 * @see BMI160GyroRange
 */
uint8_t BMI160::getFullScaleGyroRange() {
    I2CdevMod::readBits(devAddr, BMI160_RA_GYRO_RANGE,
                         BMI160_GYRO_RANGE_SEL_BIT,
                         BMI160_GYRO_RANGE_SEL_LEN, buffer);
    return buffer[0];
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleGyroRange()
 */
void BMI160::setFullScaleGyroRange(uint8_t range) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_GYRO_RANGE,
                   BMI160_GYRO_RANGE_SEL_BIT,
                   BMI160_GYRO_RANGE_SEL_LEN, range);
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 *  3 = +/- 2g
 *  5 = +/- 4g
 *  8 = +/- 8g
 * 12 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see BMI160_RA_ACCEL_RANGE
 * @see BMI160AccelRange
 */
uint8_t BMI160::getFullScaleAccelRange() {
    I2CdevMod::readBits(devAddr, BMI160_RA_ACCEL_RANGE,
                         BMI160_ACCEL_RANGE_SEL_BIT,
                         BMI160_ACCEL_RANGE_SEL_LEN, buffer);
    return buffer[0];
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 * @see BMI160AccelRange
 */
void BMI160::setFullScaleAccelRange(uint8_t range) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_ACCEL_RANGE,
                   BMI160_ACCEL_RANGE_SEL_BIT,
                   BMI160_ACCEL_RANGE_SEL_LEN, range);
}

/** Get accelerometer offset compensation enabled value.
 * @see getXAccelOffset()
 * @see BMI160_RA_OFFSET_6
 */
bool BMI160::getAccelOffsetEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_OFFSET_6,
                            BMI160_ACC_OFFSET_EN,
                            1, buffer);
    return !!buffer[0];
}

/** Set accelerometer offset compensation enabled value.
 * @see getXAccelOffset()
 * @see BMI160_RA_OFFSET_6
 */
void BMI160::setAccelOffsetEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_OFFSET_6,
                   BMI160_ACC_OFFSET_EN,
                   1, enabled ? 0x1 : 0);
}

/** Execute internal calibration to generate Accelerometer X-Axis offset value.
 * This populates the Accelerometer offset compensation value for the X-Axis only.
 * These can be retrieved using the getXAccelOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure NO movement and correct orientation of the
 * BMI160 device occurs while this auto-calibration process is active.
 * For example, to calibrate to a target of 0g on the X-axis, the BMI160 device
 * must be resting horizontally as shown in Section 5.2 of the BMI160 Data Sheet.
 *
 * To enable offset compensation, @see setAccelOffsetEnabled()
 *
 * @param target X-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
 * @see setAccelOffsetEnabled()
 * @see getXAccelOffset()
 * @see BMI160_RA_FOC_CONF
 * @see BMI160_RA_CMD
 */
void BMI160::autoCalibrateXAccelOffset(int target) {
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << BMI160_FOC_ACC_X_BIT);
    else if (target == -1)
        foc_conf = (0x2 << BMI160_FOC_ACC_X_BIT);
    else if (target == 0)
        foc_conf = (0x3 << BMI160_FOC_ACC_X_BIT);
    else
        return;  /* Invalid target value */

    I2CdevMod::writeByte(devAddr, BMI160_RA_FOC_CONF, foc_conf);
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_START_FOC);
    do {
        I2CdevMod::readBits(devAddr, BMI160_RA_STATUS,
                           BMI160_STATUS_FOC_RDY,
                           1, buffer);
        delay(1);
    } while (!buffer[0]);
}

/** Execute internal calibration to generate Accelerometer Y-Axis offset value.
 * This populates the Accelerometer offset compensation value for the Y-Axis only.
 * These can be retrieved using the getYAccelOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure NO movement and correct orientation of the
 * BMI160 device occurs while this auto-calibration process is active.
 * For example, to calibrate to a target of 0g on the Y-axis, the BMI160 device
 * must be resting horizontally as shown in Section 5.2 of the BMI160 Data Sheet.
 *
 * To enable offset compensation, @see setAccelOffsetEnabled()
 *
 * @param target Y-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
 * @see setAccelOffsetEnabled()
 * @see getYAccelOffset()
 * @see BMI160_RA_FOC_CONF
 * @see BMI160_RA_CMD
 */
void BMI160::autoCalibrateYAccelOffset(int target) {
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << BMI160_FOC_ACC_Y_BIT);
    else if (target == -1)
        foc_conf = (0x2 << BMI160_FOC_ACC_Y_BIT);
    else if (target == 0)
        foc_conf = (0x3 << BMI160_FOC_ACC_Y_BIT);
    else
        return;  /* Invalid target value */

    I2CdevMod::writeByte(devAddr, BMI160_RA_FOC_CONF, foc_conf);
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_START_FOC);
    do {
        I2CdevMod::readBits(devAddr, BMI160_RA_STATUS,
                           BMI160_STATUS_FOC_RDY,
                           1, buffer);
        delay(1);
    } while (!buffer[0]);
}

/** Execute internal calibration to generate Accelerometer Z-Axis offset value.
 * This populates the Accelerometer offset compensation value for the Z-Axis only.
 * These can be retrieved using the getZAccelOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure NO movement and correct orientation of the
 * BMI160 device occurs while this auto-calibration process is active.
 * For example, to calibrate to a target of +1g on the Z-axis, the BMI160 device
 * must be resting horizontally as shown in Section 5.2 of the BMI160 Data Sheet.
 *
 * To enable offset compensation, @see setAccelOffsetEnabled()
 *
 * @param target Z-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
 * @see setAccelOffsetEnabled()
 * @see getZAccelOffset()
 * @see BMI160_RA_FOC_CONF
 * @see BMI160_RA_CMD
 */
void BMI160::autoCalibrateZAccelOffset(int target) {
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << BMI160_FOC_ACC_Z_BIT);
    else if (target == -1)
        foc_conf = (0x2 << BMI160_FOC_ACC_Z_BIT);
    else if (target == 0)
        foc_conf = (0x3 << BMI160_FOC_ACC_Z_BIT);
    else
        return;  /* Invalid target value */

    I2CdevMod::writeByte(devAddr, BMI160_RA_FOC_CONF, foc_conf);
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_START_FOC);
    do {
        I2CdevMod::readBits(devAddr, BMI160_RA_STATUS,
                           BMI160_STATUS_FOC_RDY,
                           1, buffer);
        delay(1);
    } while (!buffer[0]);
}

/** Get offset compensation value for accelerometer X-axis data.
 * The value is represented as an 8-bit two-complement number in
 * units of 3.9mg per LSB.
 * @see BMI160_RA_OFFSET_0
 */
int8_t BMI160::getXAccelOffset() {
    I2CdevMod::readByte(devAddr, BMI160_RA_OFFSET_0, buffer);
    return buffer[0];
}

/** Set offset compensation value for accelerometer X-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateXAccelOffset().
 * @see getXAccelOffset()
 * @see BMI160_RA_OFFSET_0
 */
void BMI160::setXAccelOffset(int8_t offset) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_OFFSET_0, offset);
    getAccelerationX(); /* Read and discard the next data value */
}

/** Get offset compensation value for accelerometer Y-axis data.
 * The value is represented as an 8-bit two-complement number in
 * units of 3.9mg per LSB.
 * @see BMI160_RA_OFFSET_1
 */
int8_t BMI160::getYAccelOffset() {
    I2CdevMod::readByte(devAddr, BMI160_RA_OFFSET_1, buffer);
    return buffer[0];
}

/** Set offset compensation value for accelerometer Y-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateYAccelOffset().
 * @see getYAccelOffset()
 * @see BMI160_RA_OFFSET_1
 */
void BMI160::setYAccelOffset(int8_t offset) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_OFFSET_1, offset);
    getAccelerationY(); /* Read and discard the next data value */
}

/** Get offset compensation value for accelerometer Z-axis data.
 * The value is represented as an 8-bit two-complement number in
 * units of 3.9mg per LSB.
 * @see BMI160_RA_OFFSET_2
 */
int8_t BMI160::getZAccelOffset() {
    I2CdevMod::readByte(devAddr, BMI160_RA_OFFSET_2, buffer);
    return buffer[0];
}

/** Set offset compensation value for accelerometer Z-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateZAccelOffset().
 * @see getZAccelOffset()
 * @see BMI160_RA_OFFSET_2
 */
void BMI160::setZAccelOffset(int8_t offset) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_OFFSET_2, offset);
    getAccelerationZ(); /* Read and discard the next data value */
}

/** Get gyroscope offset compensation enabled value.
 * @see getXGyroOffset()
 * @see BMI160_RA_OFFSET_6
 */
bool BMI160::getGyroOffsetEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_OFFSET_6,
                            BMI160_GYR_OFFSET_EN,
                            1, buffer);
    return !!buffer[0];
}

/** Set gyroscope offset compensation enabled value.
 * @see getXGyroOffset()
 * @see BMI160_RA_OFFSET_6
 */
void BMI160::setGyroOffsetEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_OFFSET_6,
                      BMI160_GYR_OFFSET_EN,
                      1, enabled ? 0x1 : 0);
}

/** Execute internal calibration to generate Gyro offset values.
 * This populates the Gyro offset compensation values for all 3 axes.
 * These can be retrieved using the get[X/Y/Z]GyroOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure that NO rotation of the BMI160 device
 * occurs while this auto-calibration process is active.
 *
 * To enable offset compensation, @see setGyroOffsetEnabled()
 * @see setGyroOffsetEnabled()
 * @see getXGyroOffset()
 * @see getYGyroOffset()
 * @see getZGyroOffset()
 * @see BMI160_RA_FOC_CONF
 * @see BMI160_RA_CMD
 */
void BMI160::autoCalibrateGyroOffset() {
    uint8_t foc_conf = (1 << BMI160_FOC_GYR_EN);
    I2CdevMod::writeByte(devAddr, BMI160_RA_FOC_CONF, foc_conf);
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_START_FOC);
    do {
        I2CdevMod::readBits(devAddr, BMI160_RA_STATUS,
                           BMI160_STATUS_FOC_RDY,
                           1, buffer);
        delay(1);
    } while (!buffer[0]);
}

/** Get offset compensation value for gyroscope X-axis data.
 * The value is represented as an 10-bit two-complement number in
 * units of 0.061 degrees/s per LSB (sign-extended for int16_t type).
 * @see BMI160_RA_OFFSET_3
 * @see BMI160_RA_OFFSET_6
 */
int16_t BMI160::getXGyroOffset() {
    I2CdevMod::readByte(devAddr, BMI160_RA_OFFSET_3, buffer);
    int16_t offset = buffer[0];
    I2CdevMod::readBits(devAddr, BMI160_RA_OFFSET_6,
                     BMI160_GYR_OFFSET_X_MSB_BIT,
                     BMI160_GYR_OFFSET_X_MSB_LEN,
                     buffer);
    offset |= (int16_t)(buffer[0]) << 8;
    return BMI160_SIGN_EXTEND(offset, 10);
}

/** Set offset compensation value for gyroscope X-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateGyroOffset().
 * @see getXGyroOffset()
 * @see BMI160_RA_OFFSET_3
 * @see BMI160_RA_OFFSET_6
 */
void BMI160::setXGyroOffset(int16_t offset) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_OFFSET_3, offset);
    I2CdevMod::writeBits(devAddr, BMI160_RA_OFFSET_6,
                   BMI160_GYR_OFFSET_X_MSB_BIT,
                   BMI160_GYR_OFFSET_X_MSB_LEN, offset >> 8);
    getRotationX(); /* Read and discard the next data value */
}

/** Get offset compensation value for gyroscope Y-axis data.
 * The value is represented as an 10-bit two-complement number in
 * units of 0.061 degrees/s per LSB (sign-extended for int16_t type).
 * @see BMI160_RA_OFFSET_4
 * @see BMI160_RA_OFFSET_6
 */
int16_t BMI160::getYGyroOffset() {
    I2CdevMod::readByte(devAddr, BMI160_RA_OFFSET_4, buffer);
    int16_t offset = buffer[0];
    I2CdevMod::readBits(devAddr, BMI160_RA_OFFSET_6,
                     BMI160_GYR_OFFSET_Y_MSB_BIT,
                     BMI160_GYR_OFFSET_Y_MSB_LEN, buffer);
    offset |= (int16_t)(buffer[0]) << 8;
    return BMI160_SIGN_EXTEND(offset, 10);
}

/** Set offset compensation value for gyroscope Y-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateGyroOffset().
 * @see getYGyroOffset()
 * @see BMI160_RA_OFFSET_4
 * @see BMI160_RA_OFFSET_6
 */
void BMI160::setYGyroOffset(int16_t offset) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_OFFSET_4, offset);
    I2CdevMod::writeBits(devAddr, BMI160_RA_OFFSET_6,
                   BMI160_GYR_OFFSET_Y_MSB_BIT,
                   BMI160_GYR_OFFSET_Y_MSB_LEN, offset >> 8);
    getRotationY(); /* Read and discard the next data value */
}

/** Get offset compensation value for gyroscope Z-axis data.
 * The value is represented as an 10-bit two-complement number in
 * units of 0.061 degrees/s per LSB (sign-extended for int16_t type).
 * @see BMI160_RA_OFFSET_5
 * @see BMI160_RA_OFFSET_6
 */
int16_t BMI160::getZGyroOffset() {
    I2CdevMod::readByte(devAddr, BMI160_RA_OFFSET_5, buffer);
    int16_t offset = buffer[0];
    I2CdevMod::readBits(devAddr, BMI160_RA_OFFSET_6,
                     BMI160_GYR_OFFSET_Z_MSB_BIT,
                     BMI160_GYR_OFFSET_Z_MSB_LEN, buffer);
    offset |= (int16_t)(buffer[0]) << 8;
    return BMI160_SIGN_EXTEND(offset, 10);
}

/** Set offset compensation value for gyroscope Z-axis data.
 * This is used for applying manual calibration constants if required.
 * For auto-calibration, @see autoCalibrateGyroOffset().
 * @see getZGyroOffset()
 * @see BMI160_RA_OFFSET_5
 * @see BMI160_RA_OFFSET_6
 */
void BMI160::setZGyroOffset(int16_t offset) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_OFFSET_5, offset);
    I2CdevMod::writeBits(devAddr, BMI160_RA_OFFSET_6,
                   BMI160_GYR_OFFSET_Z_MSB_BIT,
                   BMI160_GYR_OFFSET_Z_MSB_LEN, offset >> 8);
    getRotationZ(); /* Read and discard the next data value */
}

/** Get free-fall event acceleration threshold.
 * This register configures the detection threshold for Free Fall event
 * detection. The unit of int_low_th is 1LSB = 7.81mg (min: 3.91mg). Free Fall
 * is detected when the absolute value of the accelerometer measurements for the
 * three axes are each less than the detection threshold. This condition
 * triggers the Free-Fall (low-g) interrupt if the condition is maintained for
 * the duration specified in the int_low_dur field of the INT_LOWHIGH[0]
 * register (@see BMI160_RA_INT_LOWHIGH_0)
 *
 * For more details on the Free Fall detection interrupt, see Section 2.6.7 of the
 * BMI160 Data Sheet.
 *
 * @return Current free-fall acceleration threshold value (LSB = 7.81mg, 0 = 3.91mg)
 * @see BMI160_RA_INT_LOWHIGH_1
 */
uint8_t BMI160::getFreefallDetectionThreshold() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_LOWHIGH_1, buffer);
    return buffer[0];
}

/** Set free-fall event acceleration threshold.
 * @param threshold New free-fall acceleration threshold value (LSB = 7.81mg, 0 = 3.91mg)
 * @see getFreefallDetectionThreshold()
 * @see BMI160_RA_INT_LOWHIGH_1
 */
void BMI160::setFreefallDetectionThreshold(uint8_t threshold) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_INT_LOWHIGH_1, threshold);
}

/** Get free-fall event duration threshold.
 * This register configures the duration threshold for Free Fall event
 * detection. The int_low_dur field of the INT_LOWHIGH[0] register has a unit
 * of 1 LSB = 2.5 ms (minimum 2.5ms).
 *
 * For more details on the Free Fall detection interrupt, see Section 2.6.7 of
 * the BMI160 Data Sheet.
 *
 * @return Current free-fall duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
 * @see BMI160_RA_INT_LOWHIGH_0
 */
uint8_t BMI160::getFreefallDetectionDuration() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_LOWHIGH_0, buffer);
    return buffer[0];
}

/** Set free-fall event duration threshold.
 * @param duration New free-fall duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
 * @see getFreefallDetectionDuration()
 * @see BMI160_RA_INT_LOWHIGH_0
 */
void BMI160::setFreefallDetectionDuration(uint8_t duration) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_INT_LOWHIGH_0, duration);
}

/** Get shock event acceleration threshold.
 * This register configures the detection threshold for Shock event
 * detection. The unit of threshold is dependent on the accelerometer
 * sensitivity range (@see getFullScaleAccelRange()):
 *
 * <pre>
 * Full Scale Range | LSB Resolution
 * -----------------+----------------
 * +/- 2g           |  7.81 mg/LSB (0 =  3.91mg)
 * +/- 4g           | 15.63 mg/LSB (0 =  7.81mg)
 * +/- 8g           | 31.25 mg/LSB (0 = 15.63mg)
 * +/- 16g          | 62.50 mg/LSB (0 = 31.25mg)
 * </pre>
 *
 * Shock is detected when the absolute value of the accelerometer measurements
 * for any of the three axes exceeds the detection threshold. This condition
 * triggers the Shock (high-g) interrupt if the condition is maintained without
 * a sign-change for the duration specified in the int_high_dur field of the
 * INT_LOWHIGH[3] register (@see BMI160_RA_INT_LOWHIGH_3).
 *
 * For more details on the Shock (high-g) detection interrupt, see Section 2.6.8 of the
 * BMI160 Data Sheet.
 *
 * @return Current shock acceleration threshold value
 * @see BMI160_RA_INT_LOWHIGH_4
 */
uint8_t BMI160::getShockDetectionThreshold() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_LOWHIGH_4, buffer);
    return buffer[0];
}

/** Set shock event acceleration threshold.
 * @param threshold New shock acceleration threshold value
 * @see getShockDetectionThreshold()
 * @see BMI160_RA_INT_LOWHIGH_4
 */
void BMI160::setShockDetectionThreshold(uint8_t threshold) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_INT_LOWHIGH_4, threshold);
}

/** Get shock event duration threshold.
 * This register configures the duration threshold for Shock event
 * detection. The int_high_dur field of the INT_LOWHIGH[3] register has a unit
 * of 1 LSB = 2.5 ms (minimum 2.5ms).
 *
 * For more details on the Shock detection interrupt, see Section 2.6.8 of
 * the BMI160 Data Sheet.
 *
 * @return Current shock duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
 * @see BMI160_RA_INT_LOWHIGH_3
 */
uint8_t BMI160::getShockDetectionDuration() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_LOWHIGH_3, buffer);
    return buffer[0];
}

/** Set free-fall event duration threshold.
 * @param duration New free-fall duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
 * @see getFreefallDetectionDuration()
 * @see BMI160_RA_INT_LOWHIGH_3
 */
void BMI160::setShockDetectionDuration(uint8_t duration) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_INT_LOWHIGH_3, duration);
}

/** Get Step Detection mode.
 * Returns an enum value which corresponds to current mode
 * 0 = Normal Mode
 * 1 = Sensitive Mode
 * 2 = Robust Mode
 * 3 = Unkown Mode
 * For more details on the Step Detection, see Section
 * 2.11.37 of the BMI160 Data Sheet.
 *
 * @return Current configuration of the step detector
 * @see BMI160_RA_STEP_CONF_0
 * @see BMI160_RA_STEP_CONF_1
 */
uint8_t BMI160::getStepDetectionMode() {
    uint8_t ret_step_conf0, ret_min_step_buf;

    I2CdevMod::readByte(devAddr, BMI160_RA_STEP_CONF_0, buffer);
    ret_step_conf0 = buffer[0];
    I2CdevMod::readByte(devAddr, BMI160_RA_STEP_CONF_1, buffer);
    ret_min_step_buf = buffer[0];

    if ((ret_step_conf0 == BMI160_RA_STEP_CONF_0_NOR) && (ret_min_step_buf == BMI160_RA_STEP_CONF_1_NOR))
        return BMI160_STEP_MODE_NORMAL;
    else if ((ret_step_conf0 == BMI160_RA_STEP_CONF_0_SEN) && (ret_min_step_buf == BMI160_RA_STEP_CONF_1_SEN))
	return BMI160_STEP_MODE_SENSITIVE;
    else if ((ret_step_conf0 == BMI160_RA_STEP_CONF_0_ROB) && (ret_min_step_buf == BMI160_RA_STEP_CONF_1_ROB))
        return BMI160_STEP_MODE_ROBUST;
    else
        return BMI160_STEP_MODE_UNKNOWN;
}

/** Set Step Detection mode.
 * Sets the step detection mode to one of 3 predefined sensitivity settings:
 *
 *  @see BMI160_STEP_MODE_NORMAL (Recommended for most applications)
 *  @see BMI160_STEP_MODE_SENSITIVE
 *  @see BMI160_STEP_MODE_ROBUST
 *
 * Please refer to Section 2.11.37 of the BMI160 Data Sheet for more information
 * on Step Detection configuration.
 *
 * @return Set Step Detection mode
 * @see BMI160_RA_STEP_CONF_0
 * @see BMI160_RA_STEP_CONF_1
 * @see BMI160StepMode
 */
void BMI160::setStepDetectionMode(BMI160StepMode mode) {
    uint8_t step_conf0, min_step_buf;

    /* Applying pre-defined values suggested in data-sheet Section 2.11.37 */
    switch (mode) {
    case BMI160_STEP_MODE_NORMAL:
        step_conf0 = 0x15;
        min_step_buf = 0x3;
        break;
    case BMI160_STEP_MODE_SENSITIVE:
        step_conf0 = 0x2D;
        min_step_buf = 0x0;
        break;
    case BMI160_STEP_MODE_ROBUST:
        step_conf0 = 0x1D;
        min_step_buf = 0x7;
        break;
    default:
        /* Unrecognised mode option */
        return;
    };

    I2CdevMod::writeByte(devAddr, BMI160_RA_STEP_CONF_0, step_conf0);
    I2CdevMod::writeBits(devAddr, BMI160_RA_STEP_CONF_1,
                   BMI160_STEP_BUF_MIN_BIT,
                   BMI160_STEP_BUF_MIN_LEN, min_step_buf);
}


/** Get Step Counter enabled status.
 * Once enabled and configured correctly (@see setStepDetectionMode()), the
 * BMI160 will increment a counter for every step detected by the accelerometer.
 * To retrieve the current step count, @see getStepCount().
 *
 * For more details on the Step Counting feature, see Section
 * 2.7 of the BMI160 Data Sheet.
 *
 * @return Current Step Counter enabled status
 * @see BMI160_RA_STEP_CONF_1
 * @see BMI160_STEP_CNT_EN_BIT
 */
bool BMI160::getStepCountEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_STEP_CONF_1,
                     BMI160_STEP_CNT_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set Step Counter enabled status.
 *
 * @return Set Step Counter enabled
 * @see getStepCountEnabled()
 * @see BMI160_RA_STEP_CONF_1
 * @see BMI160_STEP_CNT_EN_BIT
 */
void BMI160::setStepCountEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_STEP_CONF_1,
                      BMI160_STEP_CNT_EN_BIT,
                      1, enabled ? 0x1 : 0);
}


/** Get current number of detected step movements (Step Count).
 * Returns a step counter which is incremented when step movements are detected
 * (assuming Step Detection mode and Step Counter are configured/enabled).
 *
 * @return Number of steps as an unsigned 16-bit integer
 * @see setStepCountEnabled()
 * @see setStepDetectionMode()
 * @see BMI160_RA_STEP_CNT_L
 */
uint16_t BMI160::getStepCount() {
    I2CdevMod::readBytes(devAddr, BMI160_RA_STEP_CNT_L, 2, buffer);
    return (((uint16_t)buffer[1]) << 8) | buffer[0];
}

/** Resets the current number of detected step movements (Step Count) to 0.
 *
 * @see getStepCount()
 * @see BMI160_RA_CMD
 */
void BMI160::resetStepCount() {
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_STEP_CNT_CLR);
}

/** Get motion detection event acceleration threshold.
 * This register configures the detection threshold for Motion interrupt
 * generation in the INT_MOTION[1] register. The unit of threshold is
 * dependent on the accelerometer sensitivity range (@see
 * getFullScaleAccelRange()):
 *
 * <pre>
 * Full Scale Range | LSB Resolution
 * -----------------+----------------
 * +/- 2g           |  3.91 mg/LSB
 * +/- 4g           |  7.81 mg/LSB
 * +/- 8g           | 15.63 mg/LSB
 * +/- 16g          | 31.25 mg/LSB
 * </pre>
 *
 * Motion is detected when the difference between the absolute value of
 * consecutive accelerometer measurements for the 3 axes exceeds this Motion
 * detection threshold. This condition triggers the Motion interrupt if the
 * condition is maintained for the sample count interval specified in the
 * int_anym_dur field of the INT_MOTION[0] register (@see BMI160_RA_INT_MOTION_0)
 *
 * The Motion interrupt will indicate the axis and polarity of detected motion
 * in INT_STATUS[2] (@see BMI160_RA_INT_STATUS_2).
 *
 * For more details on the Motion detection interrupt, see Section 2.6.1 of the
 * BMI160 Data Sheet.
 *
 * @return Current motion detection acceleration threshold value
 * @see getMotionDetectionDuration()
 * @see BMI160_RA_INT_MOTION_1
 */
uint8_t BMI160::getMotionDetectionThreshold() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_MOTION_1, buffer);
    return buffer[0];
}

/** Set motion detection event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value
 * @see getMotionDetectionThreshold()
 * @see BMI160_RA_INT_MOTION_1
 */
void BMI160::setMotionDetectionThreshold(uint8_t threshold) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_INT_MOTION_1, threshold);
}

/** Get motion detection event duration threshold.
 * This register configures the duration counter threshold for Motion interrupt
 * generation, as a number of consecutive samples (from 1-4). The time
 * between samples depends on the accelerometer Output Data Rate
 * (@see getAccelRate()).
 *
 * The Motion detection interrupt is triggered when the difference between
 * samples exceeds the Any-Motion interrupt threshold for the number of
 * consecutive samples specified here.
 *
 * For more details on the Motion detection interrupt, see Section 2.6.1 of the
 * BMI160 Data Sheet.
 *
 * @return Current motion detection duration threshold value (#samples [1-4])
 * @see getMotionDetectionThreshold()
 * @see BMI160_RA_INT_MOTION_0
 */
uint8_t BMI160::getMotionDetectionDuration() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_MOTION_0,
                     BMI160_ANYMOTION_DUR_BIT,
                     BMI160_ANYMOTION_DUR_LEN, buffer);
    return 1 + buffer[0];
}

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (#samples [1-4])
 * @see getMotionDetectionDuration()
 * @see BMI160_RA_INT_MOTION_0
 */
void BMI160::setMotionDetectionDuration(uint8_t samples) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_MOTION_0,
                   BMI160_ANYMOTION_DUR_BIT,
                   BMI160_ANYMOTION_DUR_LEN, samples - 1);
}

/** Get zero motion detection event acceleration threshold.
 * This register configures the detection threshold for Zero Motion interrupt
 * generation in the INT_MOTION[1] register. The unit of threshold is
 * dependent on the accelerometer sensitivity range
 * (@see getFullScaleAccelRange()) as follows:
 *
 * <pre>
 * Full Scale Range | LSB Resolution
 * -----------------+----------------
 * +/- 2g           |  3.91 mg/LSB
 * +/- 4g           |  7.81 mg/LSB
 * +/- 8g           | 15.63 mg/LSB
 * +/- 16g          | 31.25 mg/LSB
 * </pre>
 *
 * Zero Motion is detected when the difference between the value of
 * consecutive accelerometer measurements for each axis remains smaller than
 * this Motion detection threshold. This condition triggers the Zero Motion
 * interrupt if the condition is maintained for a time duration
 * specified in the int_slo_no_mot_dur field of the INT_MOTION[0] register (@see
 * BMI160_RA_INT_MOTION_0), and clears the interrupt when the condition is
 * then absent for the same duration.
 *
 * For more details on the Zero Motion detection interrupt, see Section 2.6.9 of
 * the BMI160 Data Sheet.
 *
 * @return Current zero motion detection acceleration threshold value
 * @see getZeroMotionDetectionDuration()
 * @see BMI160_RA_INT_MOTION_2
 */
uint8_t BMI160::getZeroMotionDetectionThreshold() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_MOTION_2, buffer);
    return buffer[0];
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value
 * @see getZeroMotionDetectionThreshold()
 * @see BMI160_RA_INT_MOTION_2
 */
void BMI160::setZeroMotionDetectionThreshold(uint8_t threshold) {
    I2CdevMod::writeByte(devAddr, BMI160_RA_INT_MOTION_2, threshold);
}

/** Get zero motion detection event duration threshold.
 * This register configures the duration time for Zero Motion interrupt
 * generation. A time range between 1.28s and 430.08s can be selected, but the
 * granularity of the timing reduces as the duration increases:
 *
 * <pre>
 * Duration           | Granularity
 * -------------------+----------------
 * [1.28 - 20.48]s    |  1.28s
 * [25.6 - 102.4]s    |  5.12s
 * [112.64 - 430.08]s | 10.24s
 * </pre>
 *
 * The Zero Motion interrupt is triggered when the Zero Motion condition is
 * maintained for the duration specified in this register.
 *
 * For more details on the Zero Motion detection interrupt, see Section 2.6.9 of
 * the BMI160 Data Sheet.
 *
 * @return Current zero motion detection duration threshold value
 *         @see BMI160ZeroMotionDuration for a list of possible values
 * @see getZeroMotionDetectionThreshold()
 * @see BMI160_RA_INT_MOTION_0
 * @see BMI160ZeroMotionDuration
 */
uint8_t BMI160::getZeroMotionDetectionDuration() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_MOTION_0,
                     BMI160_NOMOTION_DUR_BIT,
                     BMI160_NOMOTION_DUR_LEN, buffer);
    return buffer[0];
}

/** Set zero motion detection event duration threshold.
 *
 * This must be called at least once to enable zero-motion detection.
 *
 * @param duration New zero motion detection duration threshold value
 *        @see BMI160ZeroMotionDuration for a list of valid values
 * @see getZeroMotionDetectionDuration()
 * @see BMI160_RA_INT_MOTION_0
 * @see BMI160ZeroMotionDuration
 */
void BMI160::setZeroMotionDetectionDuration(uint8_t duration) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_MOTION_0,
                   BMI160_NOMOTION_DUR_BIT,
                   BMI160_NOMOTION_DUR_LEN, duration);
}

/** Get Tap event acceleration threshold.
 * This register configures the detection threshold for Tap event
 * detection. The threshold is expressed a 5-bit unsigned integer.
 * The unit of threshold is dependent on the accelerometer
 * sensitivity range (@see getFullScaleAccelRange()):
 *
 * <pre>
 * Full Scale Range | LSB Resolution
 * -----------------+----------------
 * +/- 2g           |  62.5 mg/LSB (0 =  31.25mg)
 * +/- 4g           | 125.0 mg/LSB (0 =  62.5mg)
 * +/- 8g           | 250.0 mg/LSB (0 = 125.0mg)
 * +/- 16g          | 500.0 mg/LSB (0 = 250.0mg)
 * </pre>
 *
 * A Tap is detected as a shock event which exceeds the detection threshold for
 * a specified duration.  A threshold between 0.7g and 1.5g in the 2g
 * measurement range is suggested for typical tap detection applications.
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current shock acceleration threshold value
 * @see BMI160_RA_INT_TAP_1
 */
uint8_t BMI160::getTapDetectionThreshold() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_TAP_1,
                     BMI160_TAP_THRESH_BIT,
                     BMI160_TAP_THRESH_LEN, buffer);
    return buffer[0];
}

/** Set tap event acceleration threshold.
 * @param threshold New tap acceleration threshold value
 * @see getTapDetectionThreshold()
 * @see BMI160_RA_INT_TAP_1
 */
void BMI160::setTapDetectionThreshold(uint8_t threshold) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_TAP_1,
                   BMI160_TAP_THRESH_BIT,
                   BMI160_TAP_THRESH_LEN, threshold);
}

/** Get tap shock detection duration.
 * This register configures the duration for a tap event generation.
 *
 * The time will be returned as a 1-bit boolean, with the following
 * values (@see BMI160TapShockDuration)
 *
 * <pre>
 * duration specifier | duration threshold
 * -------------------+----------------
 *  0b0               |  50ms
 *  0b1               |  75ms
 * </pre>
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current tap detection duration threshold value
 * @see BMI160_RA_INT_TAP_0
 * @see BMI160TapShockDuration
 */
bool BMI160::getTapShockDuration() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_TAP_0,
                     BMI160_TAP_SHOCK_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set tap shock detection event duration threshold.
 *
 * @param units New tap detection duration threshold value
 * @see getTapShockDetectionDuration()
 * @see BMI160_RA_INT_TAP_0
 */
void BMI160::setTapShockDuration(bool duration) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_TAP_0,
                   BMI160_TAP_SHOCK_BIT,
                   1, duration ? 0x1 : 0);
}

/** Get tap quiet duration threshold.
 * This register configures the quiet duration for double-tap event detection.
 *
 * The time will be returned as a 1-bit boolean, with the following
 * values (@see BMI160TapQuietDuration)
 *
 * <pre>
 * duration specifier | duration threshold
 * -------------------+----------------
 *  0b0               |  30ms
 *  0b1               |  20ms
 * </pre>
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current tap quiet detection duration threshold value
 * @see BMI160_RA_INT_TAP_0
 * @see BMI160TapQuietDuration
 */
bool BMI160::getTapQuietDuration() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_TAP_0,
                     BMI160_TAP_QUIET_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set tap quiet duration threshold.
 *
 * @param units New tap detection duration threshold value
 * @see getTapQuietDuration()
 * @see BMI160_RA_INT_TAP_0
 */
void BMI160::setTapQuietDuration(bool duration) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_TAP_0,
                   BMI160_TAP_QUIET_BIT,
                   1, duration ? 0x1 : 0);
}

/** Get double-tap detection time window length.
 * This register configures the length of time window between 2 tap events for
 * double-tap event generation.
 *
 * The time will be returned as a 3-bit unsigned integer, with the following
 * values (@see BMI160DoubleTapDuration)
 *
 * <pre>
 * duration specifier | length of time window
 * -------------------+----------------
 *  0b000             |  50ms
 *  0b001             | 100ms
 *  0b010             | 150ms
 *  0b011             | 200ms
 *  0b100             | 250ms
 *  0b101             | 375ms
 *  0b110             | 500ms
 *  0b111             | 700ms
 * </pre>
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current double-tap detection time window threshold value
 * @see BMI160_RA_INT_TAP_0
 * @see BMI160DoubleTapDuration
 */
uint8_t BMI160::getDoubleTapDetectionDuration() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_TAP_0,
                     BMI160_TAP_DUR_BIT,
                     BMI160_TAP_DUR_LEN, buffer);
    return buffer[0];
}

/** Set double-tap detection event duration threshold.
 *
 * @param duration New double-tap detection time window threshold value
 * @see getDoubleTapDetectionDuration()
 * @see BMI160_RA_INT_TAP_0
 */
void BMI160::setDoubleTapDetectionDuration(uint8_t duration) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_TAP_0,
                   BMI160_TAP_DUR_BIT,
                   BMI160_TAP_DUR_LEN, duration);
}

/** Get Free Fall interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_EN_1
 * @see BMI160_LOW_G_EN_BIT
 **/
bool BMI160::getIntFreefallEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_EN_1,
                     BMI160_LOW_G_EN_BIT,
                     BMI160_LOW_G_EN_LEN, buffer);
    return !!buffer[0];
}

/** Set Free Fall interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see BMI160_RA_INT_EN_1
 * @see BMI160_LOW_G_EN_BIT
 **/
void BMI160::setIntFreefallEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_EN_1,
                   BMI160_LOW_G_EN_BIT,
                   BMI160_LOW_G_EN_LEN, enabled ? 0x1 : 0);
}

/** Get Shock interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_EN_1
 * @see BMI160_HIGH_G_EN_BIT
 **/
bool BMI160::getIntShockEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_EN_1,
                     BMI160_HIGH_G_EN_BIT,
                     BMI160_HIGH_G_EN_LEN, buffer);
    return !!buffer[0];
}

/** Set Shock interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntShockEnabled()
 * @see BMI160_RA_INT_EN_1
 * @see BMI160_HIGH_G_EN_BIT
 **/
void BMI160::setIntShockEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_EN_1,
                   BMI160_HIGH_G_EN_BIT,
                   BMI160_HIGH_G_EN_LEN, enabled ? 0x7 : 0x0);
}

/** Get Step interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_EN_2
 * @see BMI160_STEP_EN_BIT
 **/
bool BMI160::getIntStepEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_EN_2,
                     BMI160_STEP_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set Step interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntStepEnabled()
 * @see BMI160_RA_INT_EN_2
 * @see BMI160_STEP_EN_BIT
 **/
void BMI160::setIntStepEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_EN_2,
                   BMI160_STEP_EN_BIT,
                   1, enabled ? 0x1 : 0x0);
}

/** Get Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_EN_0
 * @see BMI160_ANYMOTION_EN_BIT
 **/
bool BMI160::getIntMotionEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_EN_0,
                     BMI160_ANYMOTION_EN_BIT,
                     BMI160_ANYMOTION_EN_LEN, buffer);
    return !!buffer[0];
}

/** Set Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntMotionEnabled()
 * @see BMI160_RA_INT_EN_0
 * @see BMI160_ANYMOTION_EN_BIT
 **/
void BMI160::setIntMotionEnabled(bool enabled) {
    /* Enable for all 3 axes */
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_EN_0,
                   BMI160_ANYMOTION_EN_BIT,
                   BMI160_ANYMOTION_EN_LEN, enabled ? 0x7 : 0x0);
}

/** Get Zero Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_EN_2
 * @see BMI160_NOMOTION_EN_BIT
 **/
bool BMI160::getIntZeroMotionEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_EN_2,
                     BMI160_NOMOTION_EN_BIT,
                     BMI160_NOMOTION_EN_LEN, buffer);
    return !!buffer[0];
}

/** Set Zero Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntZeroMotionEnabled()
 * @see BMI160_RA_INT_EN_2
 * @see BMI160_NOMOTION_EN_BIT
 * @see BMI160_RA_INT_MOTION_3
 **/
void BMI160::setIntZeroMotionEnabled(bool enabled) {
    if (enabled) {
        /* Select No-Motion detection mode */
        I2CdevMod::writeBits(devAddr, BMI160_RA_INT_MOTION_3,
                       BMI160_NOMOTION_SEL_BIT,
                       BMI160_NOMOTION_SEL_LEN, 0x1);
    }
    /* Enable for all 3 axes */
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_EN_2,
                   BMI160_NOMOTION_EN_BIT,
                   BMI160_NOMOTION_EN_LEN, enabled ? 0x7 : 0x0);
}

/** Get Tap Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_EN_0
 * @see BMI160_S_TAP_EN_BIT
 **/
bool BMI160::getIntTapEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_EN_0,
                     BMI160_S_TAP_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set Tap Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntTapEnabled()
 * @see BMI160_RA_INT_EN_0
 * @see BMI160_S_TAP_EN_BIT
 **/
void BMI160::setIntTapEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_EN_0,
                   BMI160_S_TAP_EN_BIT,
                   1, enabled ? 0x1 : 0);
}

/** Get Tap Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_EN_0
 * @see BMI160_D_TAP_EN_BIT
 **/
bool BMI160::getIntDoubleTapEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_EN_0,
                     BMI160_D_TAP_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set Tap Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntTapEnabled()
 * @see BMI160_RA_INT_EN_0
 * @see BMI160_D_TAP_EN_BIT
 **/
void BMI160::setIntDoubleTapEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_EN_0,
                   BMI160_D_TAP_EN_BIT,
                   1, enabled ? 0x1 : 0);
}

/** Get FIFO Buffer Full interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_EN_1
 * @see BMI160_FFULL_EN_BIT
 **/
bool BMI160::getIntFIFOBufferFullEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_EN_1,
                     BMI160_FFULL_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set FIFO Buffer Full interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFIFOBufferFullEnabled()
 * @see BMI160_RA_INT_EN_1
 * @see BMI160_FFULL_EN_BIT
 **/
void BMI160::setIntFIFOBufferFullEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_EN_1,
                   BMI160_FFULL_EN_BIT,
                   1, enabled ? 0x1 : 0x0);
}

/** Get Data Ready interrupt enabled setting.
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_EN_1
 * @see BMI160_DRDY_EN_BIT
 */
bool BMI160::getIntDataReadyEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_EN_1,
                     BMI160_DRDY_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see BMI160_RA_INT_EN_1
 * @see BMI160_DRDY_EN_BIT
 */
void BMI160::setIntDataReadyEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_EN_1,
                   BMI160_DRDY_EN_BIT,
                   1, enabled ? 0x1 : 0x0);
}

/** Get accelerometer FIFO enabled value.
 * When set to 1, this bit enables accelerometer data samples to be
 * written into the FIFO buffer.
 * @return Current accelerometer FIFO enabled value
 * @see BMI160_RA_FIFO_CONFIG_1
 */
bool BMI160::getAccelFIFOEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_FIFO_CONFIG_1,
                     BMI160_FIFO_ACC_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set accelerometer FIFO enabled value.
 * @param enabled New accelerometer FIFO enabled value
 * @see getAccelFIFOEnabled()
 * @see BMI160_RA_FIFO_CONFIG_1
 */
void BMI160::setAccelFIFOEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_FIFO_CONFIG_1,
                   BMI160_FIFO_ACC_EN_BIT,
                   1, enabled ? 0x1 : 0);
}

/** Get gyroscope FIFO enabled value.
 * When set to 1, this bit enables gyroscope data samples to be
 * written into the FIFO buffer.
 * @return Current gyroscope FIFO enabled value
 * @see BMI160_RA_FIFO_CONFIG_1
 */
bool BMI160::getGyroFIFOEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_FIFO_CONFIG_1,
                     BMI160_FIFO_GYR_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set gyroscope FIFO enabled value.
 * @param enabled New gyroscope FIFO enabled value
 * @see getGyroFIFOEnabled()
 * @see BMI160_RA_FIFO_CONFIG_1
 */
void BMI160::setGyroFIFOEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_FIFO_CONFIG_1,
                   BMI160_FIFO_GYR_EN_BIT,
                   1, enabled ? 0x1 : 0);
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer.
 *
 * In "headerless" FIFO mode, it is directly proportional to the number of
 * samples available given the set of sensor data bound to be stored in the
 * FIFO. See @ref getFIFOHeaderModeEnabled().
 *
 * @return Current FIFO buffer size
 * @see BMI160_RA_FIFO_LENGTH_0
 */
uint16_t BMI160::getFIFOCount() {
    I2CdevMod::readBytes(devAddr, BMI160_RA_FIFO_LENGTH_0, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Reset the FIFO.
 * This command clears all data in the FIFO buffer.  It is recommended
 * to invoke this after reconfiguring the FIFO.
 *
 * @see BMI160_RA_CMD
 * @see BMI160_CMD_FIFO_FLUSH
 */
void BMI160::resetFIFO() {
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_FIFO_FLUSH);
}

/** Reset the Interrupt controller.
 * This command clears interrupt status registers and latched interrupts.
 *
 * @see BMI160_RA_CMD
 * @see BMI160_CMD_FIFO_FLUSH
 */
void BMI160::resetInterrupt() {
    I2CdevMod::writeByte(devAddr, BMI160_RA_CMD, BMI160_CMD_INT_RESET);
}

/** Get FIFO Header-Mode enabled status.
 * When this bit is set to 0, the FIFO header-mode is disabled, and frames
 * read from the FIFO will be headerless (raw sensor data only).
 * When this bit is set to 1, the FIFO header-mode is enabled, and frames
 * read from the FIFO will include headers.
 *
 * For more information on the FIFO modes and data formats, please refer
 * to Section 2.5 of the BMI160 Data Sheet.
 *
 * @return Current FIFO Header-Mode enabled status
 * @see BMI160_RA_FIFO_CONFIG_1
 * @see BMI160_FIFO_HEADER_EN_BIT
 */
bool BMI160::getFIFOHeaderModeEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_FIFO_CONFIG_1,
                     BMI160_FIFO_HEADER_EN_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Set FIFO Header-Mode enabled status.
 * @param enabled New FIFO Header-Mode enabled status
 * @see getFIFOHeaderModeEnabled()
 * @see BMI160_RA_FIFO_CONFIG_1
 * @see BMI160_FIFO_HEADER_EN_BIT
 */
void BMI160::setFIFOHeaderModeEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_FIFO_CONFIG_1,
                   BMI160_FIFO_HEADER_EN_BIT,
                   1, enabled ? 0x1 : 0);
}

/** Get data frames from FIFO buffer.
 * This register is used to read and write data frames from the FIFO buffer.
 * Data is written to the FIFO in order of DATA register number (from lowest
 * to highest) corresponding to the FIFO data sources enabled (@see
 * getGyroFIFOEnabled() and getAccelFIFOEnabled()).
 *
 * The data frame format depends on the enabled data sources and also on
 * the FIFO header-mode setting (@see getFIFOHeaderModeEnabled()).
 *
 * It is strongly recommended, where possible, to read whole frames from the
 * FIFO.  Partially-read frames will be repeated until fully read out.
 *
 * If the FIFO buffer has filled to the point where subsequent writes may
 * cause data loss, the status bit ffull_int is automatically set to 1. This bit
 * is located in INT_STATUS[1]. When the FIFO buffer has overflowed, the oldest
 * data will be lost and new data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return a magic number
 * (@see BMI160_FIFO_DATA_INVALID) until new data is available. The user should
 * check FIFO_LENGTH to ensure that the FIFO buffer is not read when empty (see
 * @getFIFOCount()).
 *
 * @return Data frames from FIFO buffer
 */
void BMI160::getFIFOBytes(uint8_t *data, uint16_t length) {
    if (length) {
        I2CdevMod::readBytes(devAddr, BMI160_RA_FIFO_DATA, length, data);
    }
}

/** Get full set of interrupt status bits from INT_STATUS[0] register.
 * Interrupts are typically cleared automatically.
 * Please refer to the BMI160 Data Sheet for more information.
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_0
 */
uint8_t BMI160::getIntStatus0() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_0, buffer);
    return buffer[0];
}

/** Get full set of interrupt status bits from INT_STATUS[1] register.
 * Interrupts are typically cleared automatically.
 * Please refer to the BMI160 Data Sheet for more information.
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_1
 */
uint8_t BMI160::getIntStatus1() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_1, buffer);
    return buffer[0];
}

/** Get full set of interrupt status bits from INT_STATUS[2] register.
 * Interrupts are typically cleared automatically.
 * Please refer to the BMI160 Data Sheet for more information.
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_2
 */
uint8_t BMI160::getIntStatus2() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    return buffer[0];
}

/** Get full set of interrupt status bits from INT_STATUS[3] register.
 * Interrupts are typically cleared automatically.
 * Please refer to the BMI160 Data Sheet for more information.
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_3
 */
uint8_t BMI160::getIntStatus3() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_3, buffer);
    return buffer[0];
}

/** Get Free Fall interrupt status.
 * This bit automatically sets to 1 when a Free Fall condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Free-Fall (Low-G) detection interrupt, see Section
 * 2.6.7 of the BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_1
 * @see BMI160_LOW_G_INT_BIT
 */
bool BMI160::getIntFreefallStatus() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_STATUS_1,
                     BMI160_LOW_G_INT_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Get Tap Detection interrupt status.
 * This bit automatically sets to 1 when a Tap Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_0
 * @see BMI160_S_TAP_INT_BIT
 */
bool BMI160::getIntTapStatus() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_STATUS_0,
                     BMI160_S_TAP_INT_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Get Double-Tap Detection interrupt status.
 * This bit automatically sets to 1 when a Double-Tap Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Double-Tap detection interrupt, see Section 2.6.4 of the
 * BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_0
 * @see BMI160_D_TAP_INT_BIT
 */
bool BMI160::getIntDoubleTapStatus() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_STATUS_0,
                     BMI160_D_TAP_INT_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Get Shock interrupt status.
 * This bit automatically sets to 1 when a Shock (High-G) Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Shock (High-G) detection interrupt, see Section
 * 2.6.8 of the BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_1
 * @see BMI160_HIGH_G_INT_BIT
 */
bool BMI160::getIntShockStatus() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_STATUS_1,
                     BMI160_HIGH_G_INT_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Check if shock interrupt was triggered by negative X-axis motion
 * @return Shock detection status
 * @see BMI160_RA_INT_STATUS_3
 * @see BMI160_HIGH_G_SIGN_BIT
 * @see BMI160_HIGH_G_1ST_X_BIT
 */
bool BMI160::getXNegShockDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_3, buffer);
    uint8_t status = buffer[0];
    return !!((status & (1 << BMI160_HIGH_G_SIGN_BIT)) &&
              (status & (1 << BMI160_HIGH_G_1ST_X_BIT)));
}

/** Check if shock interrupt was triggered by positive X-axis motion
 * @return Shock detection status
 * @see BMI160_RA_INT_STATUS_3
 * @see BMI160_HIGH_G_SIGN_BIT
 * @see BMI160_HIGH_G_1ST_X_BIT
 */
bool BMI160::getXPosShockDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_3, buffer);
    uint8_t status = buffer[0];
    return !!(!(status & (1 << BMI160_HIGH_G_SIGN_BIT)) &&
              (status & (1 << BMI160_HIGH_G_1ST_X_BIT)));
}

/** Check if shock interrupt was triggered by negative Y-axis motion
 * @return Shock detection status
 * @see BMI160_RA_INT_STATUS_3
 * @see BMI160_HIGH_G_SIGN_BIT
 * @see BMI160_HIGH_G_1ST_Y_BIT
 */
bool BMI160::getYNegShockDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_3, buffer);
    uint8_t status = buffer[0];
    return !!((status & (1 << BMI160_HIGH_G_SIGN_BIT)) &&
              (status & (1 << BMI160_HIGH_G_1ST_Y_BIT)));
}

/** Check if shock interrupt was triggered by positive Y-axis motion
 * @return Shock detection status
 * @see BMI160_RA_INT_STATUS_3
 * @see BMI160_HIGH_G_SIGN_BIT
 * @see BMI160_HIGH_G_1ST_Y_BIT
 */
bool BMI160::getYPosShockDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_3, buffer);
    uint8_t status = buffer[0];
    return !!(!(status & (1 << BMI160_HIGH_G_SIGN_BIT)) &&
              (status & (1 << BMI160_HIGH_G_1ST_Y_BIT)));
}

/** Check if shock interrupt was triggered by negative Z-axis motion
 * @return Shock detection status
 * @see BMI160_RA_INT_STATUS_3
 * @see BMI160_HIGH_G_SIGN_BIT
 * @see BMI160_HIGH_G_1ST_Z_BIT
 */
bool BMI160::getZNegShockDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_3, buffer);
    uint8_t status = buffer[0];
    return !!((status & (1 << BMI160_HIGH_G_SIGN_BIT)) &&
              (status & (1 << BMI160_HIGH_G_1ST_Z_BIT)));
}

/** Check if shock interrupt was triggered by positive Z-axis motion
 * @return Shock detection status
 * @see BMI160_RA_INT_STATUS_3
 * @see BMI160_HIGH_G_SIGN_BIT
 * @see BMI160_HIGH_G_1ST_Z_BIT
 */
bool BMI160::getZPosShockDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_3, buffer);
    uint8_t status = buffer[0];
    return !!(!(status & (1 << BMI160_HIGH_G_SIGN_BIT)) &&
              (status & (1 << BMI160_HIGH_G_1ST_Z_BIT)));
}

/** Get Step interrupt status.
 * This bit automatically sets to 1 when a Step Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Step detection interrupt, see Section
 * 2.6.3 of the BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_0
 * @see BMI160_STEP_INT_BIT
 */
bool BMI160::getIntStepStatus() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_STATUS_0,
                     BMI160_STEP_INT_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Get Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Motion Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Motion detection interrupt, see Section 2.6.1 of the
 * BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_0
 * @see BMI160_ANYMOTION_INT_BIT
 */
bool BMI160::getIntMotionStatus() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_STATUS_0,
                     BMI160_ANYMOTION_INT_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Check if motion interrupt was triggered by negative X-axis motion
 * @return Motion detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_ANYMOTION_SIGN_BIT
 * @see BMI160_ANYMOTION_1ST_X_BIT
 */
bool BMI160::getXNegMotionDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!((status & (1 << BMI160_ANYMOTION_SIGN_BIT)) &&
              (status & (1 << BMI160_ANYMOTION_1ST_X_BIT)));
}

/** Check if motion interrupt was triggered by positive X-axis motion
 * @return Motion detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_ANYMOTION_SIGN_BIT
 * @see BMI160_ANYMOTION_1ST_X_BIT
 */
bool BMI160::getXPosMotionDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!(!(status & (1 << BMI160_ANYMOTION_SIGN_BIT)) &&
              (status & (1 << BMI160_ANYMOTION_1ST_X_BIT)));
}

/** Check if motion interrupt was triggered by negative Y-axis motion
 * @return Motion detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_ANYMOTION_SIGN_BIT
 * @see BMI160_ANYMOTION_1ST_Y_BIT
 */
bool BMI160::getYNegMotionDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!((status & (1 << BMI160_ANYMOTION_SIGN_BIT)) &&
              (status & (1 << BMI160_ANYMOTION_1ST_Y_BIT)));
}

/** Check if motion interrupt was triggered by positive Y-axis motion
 * @return Motion detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_ANYMOTION_SIGN_BIT
 * @see BMI160_ANYMOTION_1ST_Y_BIT
 */
bool BMI160::getYPosMotionDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!(!(status & (1 << BMI160_ANYMOTION_SIGN_BIT)) &&
              (status & (1 << BMI160_ANYMOTION_1ST_Y_BIT)));
}

/** Check if motion interrupt was triggered by negative Z-axis motion
 * @return Motion detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_ANYMOTION_SIGN_BIT
 * @see BMI160_ANYMOTION_1ST_Z_BIT
 */
bool BMI160::getZNegMotionDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!((status & (1 << BMI160_ANYMOTION_SIGN_BIT)) &&
              (status & (1 << BMI160_ANYMOTION_1ST_Z_BIT)));
}

/** Check if motion interrupt was triggered by positive Z-axis motion
 * @return Motion detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_ANYMOTION_SIGN_BIT
 * @see BMI160_ANYMOTION_1ST_Z_BIT
 */
bool BMI160::getZPosMotionDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!(!(status & (1 << BMI160_ANYMOTION_SIGN_BIT)) &&
              (status & (1 << BMI160_ANYMOTION_1ST_Z_BIT)));
}

/** Check if tap interrupt was triggered by negative X-axis tap
 * @return Tap detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_TAP_SIGN_BIT
 * @see BMI160_TAP_1ST_X_BIT
 */
bool BMI160::getXNegTapDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!((status & (1 << BMI160_TAP_SIGN_BIT)) &&
              (status & (1 << BMI160_TAP_1ST_X_BIT)));
}

/** Check if tap interrupt was triggered by positive X-axis tap
 * @return Tap detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_TAP_SIGN_BIT
 * @see BMI160_TAP_1ST_X_BIT
 */
bool BMI160::getXPosTapDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!(!(status & (1 << BMI160_TAP_SIGN_BIT)) &&
              (status & (1 << BMI160_TAP_1ST_X_BIT)));
}

/** Check if tap interrupt was triggered by negative Y-axis tap
 * @return Tap detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_TAP_SIGN_BIT
 * @see BMI160_TAP_1ST_Y_BIT
 */
bool BMI160::getYNegTapDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!((status & (1 << BMI160_TAP_SIGN_BIT)) &&
              (status & (1 << BMI160_TAP_1ST_Y_BIT)));
}

/** Check if tap interrupt was triggered by positive Y-axis tap
 * @return Tap detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_TAP_SIGN_BIT
 * @see BMI160_TAP_1ST_Y_BIT
 */
bool BMI160::getYPosTapDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!(!(status & (1 << BMI160_TAP_SIGN_BIT)) &&
              (status & (1 << BMI160_TAP_1ST_Y_BIT)));
}

/** Check if tap interrupt was triggered by negative Z-axis tap
 * @return Tap detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_TAP_SIGN_BIT
 * @see BMI160_TAP_1ST_Z_BIT
 */
bool BMI160::getZNegTapDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!((status & (1 << BMI160_TAP_SIGN_BIT)) &&
              (status & (1 << BMI160_TAP_1ST_Z_BIT)));
}

/** Check if tap interrupt was triggered by positive Z-axis tap
 * @return Tap detection status
 * @see BMI160_RA_INT_STATUS_2
 * @see BMI160_TAP_SIGN_BIT
 * @see BMI160_TAP_1ST_Z_BIT
 */
bool BMI160::getZPosTapDetected() {
    I2CdevMod::readByte(devAddr, BMI160_RA_INT_STATUS_2, buffer);
    uint8_t status = buffer[0];
    return !!(!(status & (1 << BMI160_TAP_SIGN_BIT)) &&
              (status & (1 << BMI160_TAP_1ST_Z_BIT)));
}

/** Get Zero Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Zero Motion Detection condition
 * is present, and clears when the condition is no longer present.
 *
 * For more details on the Motion detection interrupt, see Section 2.6.9 of the
 * BMI160 Data Sheet.
 *
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_1
 * @see BMI160_NOMOTION_INT_BIT
 */
bool BMI160::getIntZeroMotionStatus() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_STATUS_1,
                     BMI160_NOMOTION_INT_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Get FIFO Buffer Full interrupt status.
 * This bit automatically sets to 1 when a FIFO Full condition has been
 * generated. The bit clears to 0 when the FIFO is not full.
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_1
 * @see BMI160_FFULL_INT_BIT
 */
bool BMI160::getIntFIFOBufferFullStatus() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_STATUS_1,
                     BMI160_FFULL_INT_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Get Data Ready interrupt status.
 * This bit automatically sets to 1 when a Data Ready interrupt has been
 * generated. The bit clears to 0 after the data registers have been read.
 * @return Current interrupt status
 * @see BMI160_RA_INT_STATUS_1
 * @see BMI160_FFULL_INT_BIT
 */
bool BMI160::getIntDataReadyStatus() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_STATUS_1,
                     BMI160_DRDY_INT_BIT,
                     1, buffer);
    return !!buffer[0];
}

/** Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * @return Current interrupt mode (0=active-high, 1=active-low)
 * @see BMI160_RA_INT_OUT_CTRL
 * @see BMI160_INT1_LVL
 */
bool BMI160::getInterruptMode() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_OUT_CTRL,
                     BMI160_INT1_LVL,
                     1, buffer);
    return !buffer[0];
}

/** Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see BMI160_RA_INT_OUT_CTRL
 * @see BMI160_INT1_LVL
 */
void BMI160::setInterruptMode(bool mode) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_OUT_CTRL,
                   BMI160_INT1_LVL,
                   1, mode ? 0x0 : 0x1);
}

/** Get interrupt drive mode.
 * Will be set 0 for push-pull, 1 for open-drain.
 * @return Current interrupt drive mode (0=push-pull, 1=open-drain)
 * @see BMI160_RA_INT_OUT_CTRL
 * @see BMI160_INT1_OD
 */
bool BMI160::getInterruptDrive() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_OUT_CTRL,
                     BMI160_INT1_OD,
                     1, buffer);
    return !!buffer[0];
}

/** Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_OPEN_BIT
 */
void BMI160::setInterruptDrive(bool drive) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_OUT_CTRL,
                   BMI160_INT1_OD,
                   1, drive ? 0x1 : 0x0);
}

/** Get interrupt latch mode.  The following options are available:
 *
 * <pre>
 * Latch Mode    | Interrupt Latching
 * --------------+-------------------------
 * 0             | non-latched
 * 1             | temporary, 312.5us pulse
 * 2             | temporary,   625us pulse
 * 3             | temporary,  1.25ms pulse
 * 4             | temporary,   2.5ms pulse
 * 5             | temporary,     5ms pulse
 * 6             | temporary,    10ms pulse
 * 7             | temporary,    20ms pulse
 * 8             | temporary,    40ms pulse
 * 9             | temporary,    80ms pulse
 * 10            | temporary,   160ms pulse
 * 11            | temporary,   320ms pulse
 * 12            | temporary,   640ms pulse
 * 13            | temporary,  1.28s pulse
 * 14            | temporary,  2.56s pulse
 * 15            | latched until cleared (@see resetInterrupt())
 * </pre>
 *
 * Note that latching does not apply to the following interrupt sources:
 * - Data Ready
 * - Orientation (including Flat) detection
 *
 * @return Current latch mode
 * @see BMI160_RA_INT_LATCH
 * @see BMI160InterruptLatchMode
 */
uint8_t BMI160::getInterruptLatch() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_LATCH,
                         BMI160_LATCH_MODE_BIT,
                         BMI160_LATCH_MODE_LEN, buffer);
    return buffer[0];
}

/** Set interrupt latch mode.
 * @param latch New latch mode
 * @see getInterruptLatch()
 * @see BMI160_RA_INT_LATCH
 * @see BMI160InterruptLatchMode
 */
void BMI160::setInterruptLatch(uint8_t mode) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_LATCH,
                   BMI160_LATCH_MODE_BIT,
                   BMI160_LATCH_MODE_LEN, mode);
}

/** Get interrupt enabled status.
 * @return Current interrupt enabled status
 * @see BMI160_RA_INT_OUT_CTRL
 * @see BMI160_INT1_OUTPUT_EN
 **/
bool BMI160::getIntEnabled() {
    I2CdevMod::readBits(devAddr, BMI160_RA_INT_OUT_CTRL,
                     BMI160_INT1_OUTPUT_EN,
                     1, buffer);
    return !!buffer[0];
}

/** Set interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see BMI160_RA_INT_OUT_CTRL
 * @see BMI160_INT1_OUTPUT_EN
 **/
void BMI160::setIntEnabled(bool enabled) {
    I2CdevMod::writeBits(devAddr, BMI160_RA_INT_OUT_CTRL,
                   BMI160_INT1_OUTPUT_EN,
                   1, enabled ? 0x1 : 0);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see BMI160_RA_GYRO_X_L
 */
void BMI160::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    I2CdevMod::readBytes(devAddr, BMI160_RA_GYRO_X_L, 12, buffer);
    *gx = (((int16_t)buffer[1])  << 8) | buffer[0];
    *gy = (((int16_t)buffer[3])  << 8) | buffer[2];
    *gz = (((int16_t)buffer[5])  << 8) | buffer[4];
    *ax = (((int16_t)buffer[7])  << 8) | buffer[6];
    *ay = (((int16_t)buffer[9])  << 8) | buffer[8];
    *az = (((int16_t)buffer[11]) << 8) | buffer[10];
}

/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Output Data Rate
 * as configured by @see getAccelRate()
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Output Data Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale configured by
 * @setFullScaleAccelRange. For each full scale setting, the accelerometers'
 * sensitivity per LSB is shown in the table below:
 *
 * <pre>
 * Full Scale Range | LSB Sensitivity
 * -----------------+----------------
 * +/- 2g           | 8192 LSB/mg
 * +/- 4g           | 4096 LSB/mg
 * +/- 8g           | 2048 LSB/mg
 * +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see BMI160_RA_ACCEL_X_L
 */
void BMI160::getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    I2CdevMod::readBytes(devAddr, BMI160_RA_ACCEL_X_L, 6, buffer);
    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_ACCEL_X_L
 */
int16_t BMI160::getAccelerationX() {
    I2CdevMod::readBytes(devAddr, BMI160_RA_ACCEL_X_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_ACCEL_Y_L
 */
int16_t BMI160::getAccelerationY() {
    I2CdevMod::readBytes(devAddr, BMI160_RA_ACCEL_Y_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_ACCEL_Z_L
 */
int16_t BMI160::getAccelerationZ() {
    I2CdevMod::readBytes(devAddr, BMI160_RA_ACCEL_Z_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get current internal temperature as a signed 16-bit integer.
 *  The resolution is typically 1/2^9 degrees Celcius per LSB, at an
 *  offset of 23 degrees Celcius.  For example:
 *
 * <pre>
 * Value    | Temperature
 * ---------+----------------
 * 0x7FFF   | 87 - 1/2^9 degrees C
 * ...      | ...
 * 0x0000   | 23 degrees C
 * ...      | ...
 * 0x8001   | -41 + 1/2^9 degrees C
 * 0x8000   | Invalid
 *
 * @return Temperature reading in 16-bit 2's complement format
 * @see BMI160_RA_TEMP_L
 */
int16_t BMI160::getTemperature() {
    I2CdevMod::readBytes(devAddr, BMI160_RA_TEMP_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Output Data Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale configured by
 * @setFullScaleGyroRange(). For each full scale setting, the gyroscopes'
 * sensitivity per LSB is shown in the table below:
 *
 * <pre>
 * Full Scale Range   | LSB Sensitivity
 * -------------------+----------------
 * +/- 125  degrees/s | 262.4 LSB/deg/s
 * +/- 250  degrees/s | 131.2 LSB/deg/s
 * +/- 500  degrees/s | 65.5  LSB/deg/s
 * +/- 1000 degrees/s | 32.8  LSB/deg/s
 * +/- 2000 degrees/s | 16.4  LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see BMI160_RA_GYRO_X_L
 */
void BMI160::getRotation(int16_t* x, int16_t* y, int16_t* z) {
    I2CdevMod::readBytes(devAddr, BMI160_RA_GYRO_X_L, 6, buffer);
    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_GYRO_X_L
 */
int16_t BMI160::getRotationX() {
    I2CdevMod::readBytes(devAddr, BMI160_RA_GYRO_X_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_GYRO_Y_L
 */
int16_t BMI160::getRotationY() {
    I2CdevMod::readBytes(devAddr, BMI160_RA_GYRO_Y_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_GYRO_Z_L
 */
int16_t BMI160::getRotationZ() {
    I2CdevMod::readBytes(devAddr, BMI160_RA_GYRO_Z_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Read a BMI160 register directly.
 * @param reg register address
 * @return 8-bit register value
 */
uint8_t BMI160::getRegister(uint8_t reg) {
    I2CdevMod::readByte(devAddr, reg, buffer);
    return buffer[0];
}

/** Write a BMI160 register directly.
 * @param reg register address
 * @param data 8-bit register value
 */
void BMI160::setRegister(uint8_t reg, uint8_t data) {
    I2CdevMod::writeByte(devAddr, reg, data);
}