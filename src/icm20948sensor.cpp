/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain, S.J. Remington

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

#include <ICM_20948.h>
#include "sensor.h"
#include "udpclient.h"
#include "defines.h"
#include "calibration.h"
#include <i2cscan.h>
#include "ledstatus.h"
#include <EEPROM.h> // for 8266, save the current bias values to eeprom

#define BIAS_DEBUG false

namespace {
    void signalAssert() {
        for (int i = 0; i < 200; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }
    }

    void sendResetReason(uint8_t reason, uint8_t sensorId) {
        sendByte(reason, sensorId, PACKET_RESET_REASON);
    }
}

// seconds after previous save (from start) when calibration (DMP Bias) data will be saved to NVS. Increments through the list then stops; to prevent unwelcome eeprom wear.
int bias_save_periods[] = { 120, 180, 300, 600, 600 }; // 2min + 3min + 5min + 10min + 10min (no more saves after 30min)
const uint8_t number_i2c_addr = 1;
uint8_t poss_addresses[number_i2c_addr] = {0X69}; // 0X68

// #ifndef USE_6_AXIS
//     #define USE_6_AXIS true
// #endif

// #ifndef ENABLE_TAP
//     #define ENABLE_TAP false
// #endif

void ICM20948Sensor::i2c_scan() { // Basically obsolete but kept for when adding > 2 external 
    uint8_t error;

    for (uint8_t add_int = 0; add_int < number_i2c_addr; add_int++ )
    {
        Serial.printf("Scanning 0x%02X for slave... ", poss_addresses[add_int]);
        Wire.beginTransmission(poss_addresses[add_int]);
        error = Wire.endTransmission();
        if (error == 0){          
              Serial.print("Found at address. ");       
            if (poss_addresses[add_int] == 0x69 || poss_addresses[add_int] == 0x68){
                  Serial.println("\t Address is ICM.");
                addr = poss_addresses[add_int];
                ICM_found = true; 
            }
        }
    }
}

void ICM20948Sensor::save_bias(bool repeat) { 
    #if defined(SAVE_BIAS) && SAVE_BIAS && ESP8266
        int8_t count;
        int32_t bias_a[3], bias_g[3], bias_m[3];

        imu.GetBiasGyroX(&bias_g[0]);
        imu.GetBiasGyroY(&bias_g[1]);
        imu.GetBiasGyroZ(&bias_g[2]);

        imu.GetBiasAccelX(&bias_a[0]);
        imu.GetBiasAccelY(&bias_a[1]);
        imu.GetBiasAccelZ(&bias_a[2]);

        imu.GetBiasCPassX(&bias_m[0]);
        imu.GetBiasCPassY(&bias_m[1]);
        imu.GetBiasCPassZ(&bias_m[2]);

        bool gyro_set  = bias_g[0] && bias_g[1] && bias_g[2];
        bool accel_set = bias_a[0] && bias_a[1] && bias_a[2];
        bool CPass_set = bias_m[0] && bias_m[1] && bias_m[2];

        EEPROM.begin(4096); // max memory usage = 4096
        EEPROM.get(addr + 100, count); // 1st imu counter in EEPROM addr: 0x69+100=205, 2nd addr: 0x68+100=204
        Serial.printf("[0x%02X] EEPROM position: %d, count: %d \n", addr, addr + 100, count);
        if (count < 0 || count > 42) {
            count = (int)auxiliary; // 1st imu counter is even number, 2nd is odd
        } else if (repeat) {
            count++;
        }
        EEPROM.put(addr + 100, count);
        Serial.printf("[0x%02X] bias gyro  save(%d): [%d, %d, %d] \n", addr, count * 12, bias_g[0], bias_g[1], bias_g[2]);
        Serial.printf("[0x%02X] bias accel save(%d): [%d, %d, %d] \n", addr, count * 12, bias_a[0], bias_a[1], bias_a[2]);
        Serial.printf("[0x%02X] bias CPass save(%d): [%d, %d, %d] \n\n", addr, count * 12, bias_m[0], bias_m[1], bias_m[2]);
        if (gyro_set) {
            EEPROM.put(1024 + (count * 12), bias_g); // 1024 ~ 2008
        }
        if (accel_set) {
            EEPROM.put(2046 + (count * 12), bias_a); // 2046 ~ 3030
        }
        if (CPass_set) {
            EEPROM.put(3072 + (count * 12), bias_m); // 3072 ~ 4056
        }
        EEPROM.end(); // save and end
        if (repeat) {
            bias_save_counter++;
            // Possible: Could make it repeat the final timer value if any of the biases are still 0. Save strategy could be improved.
            if (sizeof(bias_save_periods) != bias_save_counter) {
                timer.in(bias_save_periods[bias_save_counter] * 1000, [](void *arg) -> bool { ((ICM20948Sensor*)arg)->save_bias(true); return false; }, this);
            }
        }
    #endif
}

void ICM20948Sensor::setupICM20948(bool auxiliary, uint8_t addr) {
    this->addr = addr;
    this->auxiliary = auxiliary;
    this->sensorOffset = {Quat(Vector3(0, 0, 1), ((int)auxiliary) == 0 ? IMU_ROTATION : SECOND_IMU_ROTATION)};
    if(auxiliary)
    {
        Serial.println("2nd IMU setup");
        second_imu = addr;
    }
    else
    {
        Serial.println("1st IMU setup");
        first_imu = addr;
    }
}

void ICM20948Sensor::motionSetup() {
    #ifdef FULL_DEBUG
        imu.enableDebugging(Serial);
    #endif
    if (imu.begin(Wire, addr) != ICM_20948_Stat_Ok) {
        Serial.print("[ERR] IMU ICM20948: Can't connect to ");
        Serial.println(IMU_NAME);
        signalAssert();
        return;
    }

    // Configure imu setup and load any stored bias values
    // poss_addresses[0] = addr;
    // i2c_scan();
    // Serial.println("Scan completed");
    if(imu.initializeDMP() == ICM_20948_Stat_Ok)
    {
        Serial.print("DMP initialized"); 
        Serial.println(IMU_NAME);
    }
    else
    {
        Serial.print("[ERR] DMP Failed to initialize"); 
        Serial.println(IMU_NAME);
        return;
    }

    if (USE_6_AXIS)
    {
        Serial.printf("[0x%02X] use 6-axis...\n", addr);
        if(imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok)
        {
            Serial.print("Enabled DMP Senor for Game Rotation Vector");
            Serial.println(IMU_NAME);
        }
        else
        {
            Serial.println("[ERR] Enabling DMP Senor for Game Rotation Vector Failed");
            Serial.println(IMU_NAME);
            return; 
        }
    }
    else
    {
        Serial.printf("[0x%02X] use 9-axis...\n", addr);
        if(imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok)
        {
            Serial.print("Enabled DMP Senor for Sensor Orientation");
            Serial.println(IMU_NAME);
        }
        else
        {
            Serial.print("[ERR] Enabling DMP Senor Orientation Failed");
            Serial.println(IMU_NAME);
            return; 
        }
    }

    

    // Might need to set up other DMP functions later, just Quad6/Quad9 for now

    if (USE_6_AXIS)
    {
        if(imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok)
        {
            Serial.print("Set Quat6 to Max frequency");
            Serial.println(IMU_NAME);
        }
        else
        {
            Serial.print("[ERR] Failed to Set Quat6 to Max frequency");
            Serial.println(IMU_NAME);
            return;
        }
    }
    else
    {
        if(imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok)
        {
            Serial.print("Set Quat9 to Max frequency");
            Serial.println(IMU_NAME);
        }
        else
        {
            Serial.print("[ERR] Failed to Set Quat9 to Max frequency");
            Serial.println(IMU_NAME);
            return;
        }
    }

    // Enable the FIFO
    if(imu.enableFIFO() == ICM_20948_Stat_Ok)
    {
        Serial.print("FIFO Enabled");
        Serial.println(IMU_NAME);
    }
    else
    {
        Serial.print("[ERR] FIFO Enabling Failed");
        Serial.println(IMU_NAME);
        return;
    }

    // Enable the DMP
    if(imu.enableDMP() == ICM_20948_Stat_Ok)
    {
        Serial.print("DMP Enabled");
        Serial.println(IMU_NAME);
    }
    else
    {
        Serial.print("[ERR] DMP Enabling Failed");
        Serial.println(IMU_NAME);
        return;
    }

    // Reset DMP
    if(imu.resetDMP() == ICM_20948_Stat_Ok)
    {
        Serial.print("Reset DMP");
        Serial.println(IMU_NAME);
    }
    else
    {
        Serial.print("[ERR] Failed to reset DMP");
        Serial.println(IMU_NAME);
        return;
    }

    // Reset FIFO
    if(imu.resetFIFO() == ICM_20948_Stat_Ok)
    {
        Serial.print("Reset FIFO");
        Serial.println(IMU_NAME);
    }
    else
    {
        Serial.print("[ERR] Failed to reset FIFO");
        Serial.println(IMU_NAME);
        return;
    }

    #if defined(LOAD_BIAS) && LOAD_BIAS && ESP8266
        int8_t count;
        int32_t bias_g[3], bias_a[3], bias_m[3];
        count = 0;
        EEPROM.begin(4096); // max memory usage = 4096
        EEPROM.get(addr + 100, count); // 1st imu counter in EEPROM addr: 0x69+100=205, 2nd addr: 0x68+100=204
        Serial.printf("[0x%02X] EEPROM position: %d, count: %d \n", addr, addr + 100, count);
        if (count < 0 || count > 42) {
            count = (int)auxiliary; // 1st imu counter is even number, 2nd is odd
            EEPROM.put(addr + 100, count);
        }
        EEPROM.get(1024 + (count * 12), bias_g); // 1024 ~ 2008
        EEPROM.get(2046 + (count * 12), bias_a); // 2046 ~ 3030
        EEPROM.get(3072 + (count * 12), bias_m); // 3072 ~ 4056
        EEPROM.end();
        Serial.printf("[0x%02X] EEPROM gyro  get(%d): [%d, %d, %d] \n", addr, count * 12, bias_g[0], bias_g[1], bias_g[2]);
        Serial.printf("[0x%02X] EEPROM accel get(%d): [%d, %d, %d] \n", addr, count * 12, bias_a[0], bias_a[1], bias_a[2]);
        Serial.printf("[0x%02X] EEPROM CPass get(%d): [%d, %d, %d] \n\n", addr, count * 12, bias_m[0], bias_m[1], bias_m[2]);

        imu.SetBiasGyroX(bias_g[0]);
        imu.SetBiasGyroY(bias_g[1]);
        imu.SetBiasGyroZ(bias_g[2]);

        imu.SetBiasAccelX(bias_a[0]);
        imu.SetBiasAccelY(bias_a[1]);
        imu.SetBiasAccelZ(bias_a[2]);

        imu.SetBiasCPassX(bias_m[0]);
        imu.SetBiasCPassY(bias_m[1]);
        imu.SetBiasCPassZ(bias_m[2]);
    #else
        if(BIAS_DEBUG)
        {
            int bias_a[3], bias_g[3], bias_m[3];
            
            imu.GetBiasGyroX(&bias_g[0]);
            imu.GetBiasGyroY(&bias_g[1]);
            imu.GetBiasGyroZ(&bias_g[2]);

            imu.GetBiasAccelX(&bias_a[0]);
            imu.GetBiasAccelY(&bias_a[1]);
            imu.GetBiasAccelZ(&bias_a[2]);

            imu.GetBiasCPassX(&bias_m[0]);
            imu.GetBiasCPassY(&bias_m[1]);
            imu.GetBiasCPassZ(&bias_m[2]);

            Serial.print("Starting Gyro Bias is ");
            Serial.print(bias_g[0]);
            Serial.print(",");
            Serial.print(bias_g[1]);
            Serial.print(",");
            Serial.println(bias_g[2]);
            Serial.print("Starting Accel Bias is ");
            Serial.print(bias_a[0]);
            Serial.print(",");
            Serial.print(bias_a[1]);
            Serial.print(",");
            Serial.println(bias_a[2]);
            Serial.print("Starting CPass Bias is ");
            Serial.print(bias_m[0]);
            Serial.print(",");
            Serial.print(bias_m[1]);
            Serial.print(",");
            Serial.println(bias_m[2]);

            //Sets all bias to 90
            bias_g[0] = 90;
            bias_g[1] = 90;
            bias_g[2] = 90;
            bias_a[0] = 90;
            bias_a[1] = 90;
            bias_a[2] = 90;
            bias_m[0] = 90;
            bias_m[1] = 90;
            bias_m[2] = 90;

            //Sets all bias to 0 in memory
            imu.SetBiasGyroX(bias_g[0]);
            imu.SetBiasGyroY(bias_g[1]);
            imu.SetBiasGyroZ(bias_g[2]);

            imu.SetBiasAccelX(bias_a[0]);
            imu.SetBiasAccelY(bias_a[1]);
            imu.SetBiasAccelZ(bias_a[2]);

            imu.SetBiasCPassX(bias_m[0]);
            imu.SetBiasCPassY(bias_m[1]);
            imu.SetBiasCPassZ(bias_m[2]);

            //Sets all bias to 0
            bias_g[0] = 0;
            bias_g[1] = 0;
            bias_g[2] = 0;
            bias_a[0] = 0;
            bias_a[1] = 0;
            bias_a[2] = 0;
            bias_m[0] = 0;
            bias_m[1] = 0;
            bias_m[2] = 0;

            //Reloads all bias from memory
            imu.GetBiasGyroX(&bias_g[0]);
            imu.GetBiasGyroY(&bias_g[1]);
            imu.GetBiasGyroZ(&bias_g[2]);

            imu.GetBiasAccelX(&bias_a[0]);
            imu.GetBiasAccelY(&bias_a[1]);
            imu.GetBiasAccelZ(&bias_a[2]);

            imu.GetBiasCPassX(&bias_m[0]);
            imu.GetBiasCPassY(&bias_m[1]);
            imu.GetBiasCPassZ(&bias_m[2]);

            Serial.println("All set bias should be 90");

            Serial.print("Set Gyro Bias is ");
            Serial.print(bias_g[0]);
            Serial.print(",");
            Serial.print(bias_g[1]);
            Serial.print(",");
            Serial.println(bias_g[2]);
            Serial.print("Set Accel Bias is ");
            Serial.print(bias_a[0]);
            Serial.print(",");
            Serial.print(bias_a[1]);
            Serial.print(",");
            Serial.println(bias_a[2]);
            Serial.print("Set CPass Bias is ");
            Serial.print(bias_m[0]);
            Serial.print(",");
            Serial.print(bias_m[1]);
            Serial.print(",");
            Serial.println(bias_m[2]);
        }
    #endif

    lastData = millis();
    working = true;

    #if ESP8266 && defined(SAVE_BIAS)
        timer.in(bias_save_periods[0] * 1000, [](void *arg) -> bool { ((ICM20948Sensor*)arg)->save_bias(true); return false; }, this);
    #endif
}


void ICM20948Sensor::motionLoop() {
    timer.tick();
    if(imu.dataReady())
    {
        lastReset = -1;
        lastData = millis();
    }
    

    if (lastData + 1000 < millis()) {
        working = false;
        lastData = millis();        
        Serial.print("[ERR] Sensor timeout ");
        Serial.println(addr);
    }
}

void ICM20948Sensor::sendData() { 
    ICM_20948_Status_e readStatus = imu.readDMPdataFromFIFO(&dmpData);
        if(readStatus == ICM_20948_Stat_Ok)
        {
            if (USE_6_AXIS)
            {
                if ((dmpData.header & DMP_header_bitmap_Quat6) > 0)
                {
                    // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
                    // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
                    // The quaternion data is scaled by 2^30.
                    // Scale to +/- 1
                    double q1 = ((double)dmpData.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)dmpData.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)dmpData.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                    quaternion.w = q0;
                    quaternion.x = q1;
                    quaternion.y = q2;
                    quaternion.z = q3;
                    quaternion *= sensorOffset; //imu rotation
                    sendRotationData(&quaternion, DATA_TYPE_NORMAL, 0, auxiliary, PACKET_ROTATION_DATA);
                }
            }
            else
            {
                if ((dmpData.header & DMP_header_bitmap_Quat9) > 0)
                {
                    // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
                    // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
                    // The quaternion data is scaled by 2^30.
                    // Scale to +/- 1
                    double q1 = ((double)dmpData.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)dmpData.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)dmpData.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                    quaternion.w = q0;
                    quaternion.x = q1;
                    quaternion.y = q2;
                    quaternion.z = q3;
                    quaternion *= sensorOffset; //imu rotation
                    sendRotationData(&quaternion, DATA_TYPE_NORMAL, dmpData.Quat9.Data.Accuracy, auxiliary, PACKET_ROTATION_DATA);
                }
            }
        }
        else
        {
            Serial.print("[ERR] Failed read DMP FIFO data ");
            Serial.println(IMU_NAME);
            Serial.println(imu.statusString(readStatus));
        }
}

void ICM20948Sensor::startCalibration(int calibrationType) {
    // 20948 does continuous calibration

    // TODO If ESP32, manually force a new save
    // #ifdef ESP32
    //     save_bias(false);
    // #endif
    // If 8266, save the current bias values to eeprom
    #ifdef ESP8266
        // Types are int, device config saves float - need to save and load like mpu6050 does
        save_bias(false);
    #endif
}

// initializeDMP is a weak function. Let's overwrite it so we can increase the sample rate
ICM_20948_Status_e ICM_20948::initializeDMP(void)
{
  // First, let's check if the DMP is available
  if (_device._dmp_firmware_available != true)
  {
    debugPrint(F("ICM_20948::startupDMP: DMP is not available. Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    return ICM_20948_Stat_DMPNotSupported;
  }

  ICM_20948_Status_e  worstResult = ICM_20948_Stat_Ok;

#if defined(ICM_20948_USE_DMP)

  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
  // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

  ICM_20948_Status_e  result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful

  // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  // 0: use I2C_SLV0
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
  result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true); if (result > worstResult) worstResult = result;
  //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
  result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single); if (result > worstResult) worstResult = result;

  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  result = setBank(3); if (result > worstResult) worstResult = result; // Select Bank 3
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register  

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  result = setClockSource(ICM_20948_Clock_Auto); if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  result = setBank(0); if (result > worstResult) worstResult = result;                               // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;

  // Disable the FIFO
  result = enableFIFO(false); if (result > worstResult) worstResult = result;

  // Disable the DMP
  result = enableDMP(false); if (result > worstResult) worstResult = result;

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                         // gpm2
                         // gpm4
                         // gpm8
                         // gpm16
  myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                         // dps250
                         // dps500
                         // dps1000
                         // dps2000
  result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  result = enableDLPF(ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t zero = 0;
  result = write(AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  result = write(AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;

  // Turn off data ready interrupt through INT_ENABLE_1
  result = intEnableRawDataReady(false); if (result > worstResult) worstResult = result;

  // Reset FIFO through FIFO_RST
  result = resetFIFO(); if (result > worstResult) worstResult = result;

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  //mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.g = 4; // 225Hz
  mySmplrt.a = 4; // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  result = loadDMPFirmware(); if (result > worstResult) worstResult = result;

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fix = 0x48;
  result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

  // Set the Single FIFO Priority Select register to 0xE4
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = setGyroSF(19, 3); if (result > worstResult) worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

#endif

  return worstResult;
}
