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

