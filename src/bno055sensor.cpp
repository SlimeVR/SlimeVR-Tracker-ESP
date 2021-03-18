/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

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
#include "sensor.h"
#include <i2cscan.h>
#include <I2Cdev.h>
#include "udpclient.h"
#include "defines.h"

namespace {
    void signalAssert() {
        for(int i = 0; i < 200; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }
    }

    void BNO055_delay_msek(u32 msek)
    {
       delay(msek);
    }

    s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {
        if(I2Cdev::readBytes(dev_addr, reg_addr, cnt, reg_data) < 0)
            return BNO055_ERROR;
        return BNO055_SUCCESS;
    }

    s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {
        if(I2Cdev::writeBytes(dev_addr, reg_addr, cnt, reg_data))
            return BNO055_SUCCESS;
        return BNO055_ERROR;
    }
}

void BNO055Sensor::motionSetup(DeviceConfig * config) {
    Wire.setClock(100000);
    delay(1000);
    uint8_t addr = BNO055_I2C_ADDR1;
    imu.dev_addr = addr;
    if(!I2CSCAN::isI2CExist(addr)) {
        addr = BNO055_I2C_ADDR2;
        imu.dev_addr = addr;
        if(!I2CSCAN::isI2CExist(addr)) {
            Serial.println("Can't find I2C device on addr 0x28 or 0x29, scanning for all I2C devices and returning");
            I2CSCAN::scani2cports();
            signalAssert();
            return;
        }
    }
    imu.delay_msec = BNO055_delay_msek;
    imu.bus_read = BNO055_I2C_bus_read;
    imu.bus_write = BNO055_I2C_bus_write;

    s8 comres;
    comres = bno055_init(&imu);
    delay(500);
    if(comres == BNO055_ERROR) {
        Serial.print("Can't init BNO055, got: ");
        Serial.println(comres);
    } else {
        Serial.print("Initialized BNO055, chip id ");
        u8 chipId;
        bno055_read_chip_id(&chipId);
        Serial.print(chipId, HEX);
        u16 softwareRev;
        bno055_read_sw_rev_id(&softwareRev);
        Serial.print(", softwared rev ");
        Serial.print(softwareRev, HEX);
        bno055_read_accel_rev_id(&chipId);
        Serial.print(", accel id ");
        Serial.print(chipId, HEX);
        bno055_read_mag_rev_id(&chipId);
        Serial.print(", mag id ");
        Serial.print(chipId, HEX);
        bno055_read_gyro_rev_id(&chipId);
        Serial.print(", gyro id ");
        Serial.print(chipId, HEX);
        bno055_read_bl_rev_id(&chipId);
        Serial.print(", bootloader rev ");
        Serial.println(chipId, HEX);
    }

    bno055_sw_reset();
    delay(500);
    bno055_set_clk_src(BNO055_BIT_ENABLE);

    comres = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    if(comres == BNO055_ERROR) {
        Serial.print("Can't set BNO055 power mode, got: ");
        Serial.println(comres);
    }
    delay(500);

    Serial.println("Started BNO055");
    Serial.println("Reading POST test results...");
    u8 selfTestResult;
    Serial.print("Accel self-test result: ");
    bno055_get_selftest_accel(&selfTestResult);
    Serial.println(selfTestResult, HEX);
    Serial.print("Gyro self-test result: ");
    bno055_get_selftest_gyro(&selfTestResult);
    Serial.println(selfTestResult, HEX);
    Serial.print("Mag self-test result: ");
    bno055_get_selftest_mag(&selfTestResult);
    Serial.println(selfTestResult, HEX);
    Serial.print("MCU self-test result: ");
    bno055_get_selftest_mcu(&selfTestResult);
    Serial.println(selfTestResult, HEX);

    Serial.println("Enabling BIST self-test...");
    bno055_set_selftest(BNO055_BIT_ENABLE);
    Serial.print("Gyro self-test result: ");
    bno055_get_selftest_gyro(&selfTestResult);
    Serial.println(selfTestResult, HEX);
    Serial.print("Accel self-test result: ");
    bno055_get_selftest_accel(&selfTestResult);
    Serial.println(selfTestResult, HEX);
    Serial.print("Mag self-test result: ");
    bno055_get_selftest_mag(&selfTestResult);
    Serial.println(selfTestResult, HEX);
    Serial.print("MCU self-test result: ");
    bno055_get_selftest_mcu(&selfTestResult);
    Serial.println(selfTestResult, HEX);
    bno055_set_selftest(BNO055_BIT_DISABLE);

    Serial.print("Calibration status of all sensors: ");
    bno055_get_sys_calib_stat(&selfTestResult);
    Serial.print(selfTestResult, HEX);
    bno055_get_gyro_calib_stat(&selfTestResult);
    Serial.print(selfTestResult, HEX);
    bno055_get_accel_calib_stat(&selfTestResult);
    Serial.print(selfTestResult, HEX);
    bno055_get_mag_calib_stat(&selfTestResult);
    Serial.println(selfTestResult, HEX);

    bno055_set_intr_rst(BNO055_BIT_DISABLE);
    delay(100);
    comres = bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
    if(comres == BNO055_ERROR) {
        Serial.print("Can't set BNO055 operation mode, got: ");
        Serial.println(comres);
    }
    delay(500);
    
    Serial.print("Current operation mode: ");
    bno055_get_operation_mode(&selfTestResult);
    Serial.println(selfTestResult, HEX);

    Serial.print("System status: ");
    bno055_get_sys_stat_code(&selfTestResult);
    Serial.println(selfTestResult, HEX);
    Serial.print("System error: ");
    bno055_get_sys_error_code(&selfTestResult);
    Serial.println(selfTestResult, HEX);

    /*Serial.println("Gonna dump page 0, fuck it...");
    u8 page[127];
    imu.bus_read(imu.dev_addr, 0, page, 127);
    Serial.print("Raw register data: ");
    for(int i = 0; i < sizeof(page); ++i) {
        Serial.print(i, HEX);
        Serial.print(": ");
        Serial.print(page[i], HEX);
        Serial.print(", ");
        page[i] = 0;
    }
    Serial.println();
    Serial.println("Gonna dump page 1, fuck it...");
    bno055_write_page_id(1);
    imu.bus_read(imu.dev_addr, 0, page, 127);
    Serial.print("Raw register data: ");
    for(int i = 0; i < sizeof(page); ++i) {
        Serial.print(i, HEX);
        Serial.print(": ");
        Serial.print(page[i], HEX);
        Serial.print(", ");
    }
    Serial.println();
    bno055_write_page_id(0);*/
}

void BNO055Sensor::motionLoop() {
    s8 comres = bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
    if(comres == BNO055_SUCCESS) {
        comres  = bno055_read_quaternion_wxyz(&rawQuat);
        if(comres == BNO055_SUCCESS) {
            quaternion.x = ((double) rawQuat.x) / 16384.0;
            quaternion.y = ((double) rawQuat.y) / 16384.0;
            quaternion.z = ((double) rawQuat.z) / 16384.0;
            quaternion.w = ((double) rawQuat.w) / 16384.0;
            quaternion *= sensorOffset;
            newData = true;
        }
    }
    if(comres != BNO055_SUCCESS) {
        Serial.print("Can't read BNO055 data, got: ");
        Serial.println(comres);
    }
}

void BNO055Sensor::sendData() {
    u8 selfTestResult;
    if(newData) {
        newData = false;
        sendQuat(&quaternion, PACKET_ROTATION);
        #ifdef FULL_DEBUG
            Serial.print("Quaternion: ");
            Serial.print(quaternion.x);
            Serial.print(",");
            Serial.print(quaternion.y);
            Serial.print(",");
            Serial.print(quaternion.z);
            Serial.print(",");
            Serial.print(quaternion.w);
            Serial.print(" / ");
            Serial.print(rawQuat.x);
            Serial.print(",");
            Serial.print(rawQuat.y);
            Serial.print(",");
            Serial.print(rawQuat.z);
            Serial.print(",");
            Serial.println(rawQuat.w);

            u8 data_u8[BNO055_QUATERNION_WXYZ_DATA_SIZE];
            imu.bus_read(imu.dev_addr, BNO055_QUATERNION_DATA_W_LSB_VALUEW_REG, data_u8, BNO055_QUATERNION_WXYZ_DATA_SIZE);
            Serial.print("Raw register data: ");
            for(int i = 0; i < sizeof(data_u8); ++i)
                Serial.print(data_u8[i], HEX);
            Serial.println();

            Serial.print("System status: ");
            bno055_get_sys_stat_code(&selfTestResult);
            Serial.println(selfTestResult, HEX);
            Serial.print("System error: ");
            bno055_get_sys_error_code(&selfTestResult);
            Serial.println(selfTestResult, HEX);
        #endif
    }
}

void BNO055Sensor::startCalibration(int calibrationType) {

}