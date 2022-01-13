/*
Based on Demo's fork
*/
//Spelling mistakes are probally mine - Threads

#include <ICM_20948.h>
#include "sensor.h"
#include "udpclient.h"
#include "defines.h"
#include "calibration.h"
#include <i2cscan.h>
#include "ledstatus.h"

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
#if defined(SAVE_BIAS)
  if (SAVE_BIAS) 
  {
    int bias_a[3], bias_g[3], bias_m[3];
    
    imu.GetBiasGyroX(&bias_g[0]);
    imu.GetBiasGyroY(&bias_g[1]);
    imu.GetBiasGyroZ(&bias_g[2]);

    Serial.print("Starting Gyro Bias is ");
    Serial.print(bias_g[0]);
    Serial.print(",");
    Serial.print(bias_g[1]);
    Serial.print(",");
    Serial.println(bias_g[2]);


    imu.GetBiasAccelX(&bias_a[0]);
    imu.GetBiasAccelY(&bias_a[1]);
    imu.GetBiasAccelZ(&bias_a[2]);

    Serial.print("Starting Accel Bias is ");
    Serial.print(bias_a[0]);
    Serial.print(",");
    Serial.print(bias_a[1]);
    Serial.print(",");
    Serial.println(bias_a[2]);

    imu.GetBiasCPassX(&bias_m[0]);
    imu.GetBiasCPassY(&bias_m[1]);
    imu.GetBiasCPassZ(&bias_m[2]);

    Serial.print("Starting CPass Bias is ");
    Serial.print(bias_m[0]);
    Serial.print(",");
    Serial.print(bias_m[1]);
    Serial.print(",");
    Serial.println(bias_m[2]);

    // bool accel_set = bias_a[0] && bias_a[1] && bias_a[2];
    // bool gyro_set = bias_g[0] && bias_g[1] && bias_g[2];
    // bool mag_set = bias_m[0] && bias_m[1] && bias_m[2];
    
    // if (accel_set) {
    //   // Save accel
    //   prefs.putInt(auxiliary ? "ba01" : "ba00", bias_a[0]);
    //   prefs.putInt(auxiliary ? "ba11" : "ba10", bias_a[1]);
    //   prefs.putInt(auxiliary ? "ba21" : "ba20", bias_a[2]);

    // #ifdef FULL_DEBUG
    //         Serial.println("Wrote Accel Bias");
    // #endif
    // }
    
    // if (gyro_set) {
    //   // Save gyro
    //   prefs.putInt(auxiliary ? "bg01" : "bg00", bias_g[0]);
    //   prefs.putInt(auxiliary ? "bg11" : "bg10", bias_g[1]);
    //   prefs.putInt(auxiliary ? "bg21" : "bg20", bias_g[2]);

    // #ifdef FULL_DEBUG
    //     Serial.println("Wrote Gyro Bias");
    // #endif
    // }

    // if (mag_set) {
    //   // Save mag
    //   prefs.putInt(auxiliary ? "bm01" : "bm00", bias_m[0]);
    //   prefs.putInt(auxiliary ? "bm11" : "bm10", bias_m[1]);
    //   prefs.putInt(auxiliary ? "bm21" : "bm20", bias_m[2]);

    // #ifdef FULL_DEBUG
    //     Serial.println("Wrote Mag Bias");
    // #endif
    // }    
  }  

    // if (repeat) {
    //     bias_save_counter++;
    //     // Possible: Could make it repeat the final timer value if any of the biases are still 0. Save strategy could be improved.
    //     if (sizeof(bias_save_periods) != bias_save_counter)
    //     {
    //         timer.in(bias_save_periods[bias_save_counter] * 1000, [](void *arg) -> bool { ((ICM20948Sensor*)arg)->save_bias(true); return false; }, this);
    //     }
    // }
#endif
}

void ICM20948Sensor::setupICM20948(bool auxiliary, uint8_t addr) {
    this->addr = addr;
    this->auxiliary = auxiliary;
    this->sensorOffset = {Quat(Vector3(0, 0, 1), ((int)auxiliary) == 0 ? IMU_ROTATION : SECOND_IMU_ROTATION)};
}

void ICM20948Sensor::motionSetup() {

    if (imu.begin(Wire, ad0_Val) != ICM_20948_Stat_Ok) {
        Serial.print("[ERR] IMU ICM20948: Can't connect to ");
        Serial.println(IMU_NAME);
        signalAssert();
        return;
    }

    // Configure imu setup and load any stored bias values
    poss_addresses[0] = addr;
    i2c_scan();
    Serial.println("Scan completed");
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
    if(imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok)
    {
        Serial.print("Enabled DMP Senor for Game Rotation Vector");
        Serial.println(IMU_NAME);
    }
    else
    {
        {
        Serial.print("[ERR] Enabling DMP Senor for Game Rotation Vector Failed");
        Serial.println(IMU_NAME);
        return; 
    }
    }

    if(imu.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok)
    {
        Serial.print("Enabled DMP Senor for Raw Gyro");
        Serial.println(IMU_NAME);
    }
    else
    {
        Serial.print("[ERR] Enabling DMP Senor for Raw Gyro Failed");
        Serial.println(IMU_NAME);
        return;
    }
    if(imu.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok)
    {
        Serial.print("Enabled DMP Senor for Raw Acc");
        Serial.println(IMU_NAME);
    }
    else
    {
        Serial.print("[ERR] Enabling DMP Senor for Raw Acc Failed");
        Serial.println(IMU_NAME);
        return;
    }
    if(imu.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok)
    {
        Serial.print("Enabled DMP Senor for Magnetic Field Uncalibrated");
        Serial.println(IMU_NAME);
    }
    else
    {
        Serial.print("[ERR] Enabling DMP Senor for Magnetic Field Uncalibrated Failed");
        Serial.println(IMU_NAME);
        return;
    }
    
    //TODO I have no idea exactly what we need, so I have included everything, but we really should trim the un

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    // if(imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
    // if(imu.setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
    // if(imu.setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
    // if(imu.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
    // if(imu.setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
    // if(imu.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz

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



    lastData = millis();
    working = true;
}


//I don't think there is any need for a motionloop with the libary
void ICM20948Sensor::motionLoop() {

    if(imu.dataReady())
    {
        Serial.println("IMU Data ready, timeout Reset");
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
    
    Serial.println("Check if my FIFO Data Availiable");
    if((imu.status == ICM_20948_Stat_FIFOMoreDataAvail))
    {
        Serial.println("FIFO Data Availiable");
        Serial.println("Attempt to read DMP Data");
        if(imu.readDMPdataFromFIFO(&dmpData) == ICM_20948_Stat_Ok)
        {
            Serial.println("Reading DMP Data");
            if ((dmpData.header & DMP_header_bitmap_Quat9) > 0)
            {
                Serial.println("Converting Rotation Data");
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
                Serial.println("Sending Rotation Data to Server");
                sendRotationData(&quaternion, DATA_TYPE_NORMAL, dmpData.Quat9.Data.Accuracy, auxiliary, PACKET_ROTATION_DATA);
            }
        }
        else
        {
            //This may lead to instability, or not
            Serial.print("[ERR] Failed read DMP FIFO data");
            Serial.println(IMU_NAME);
        }
    }
}

void ICM20948Sensor::startCalibration(int calibrationType) {
    // 20948 does continuous calibration and no need to use for ESP32 as it auto saves bias values

    // If ESP32, manually force a new save
    // #ifdef ESP32
    //     save_bias(false);
    // #endif
    // TODO: If 8266, save the current bias values to eeprom
    //#ifdef 8266
    // Types are int, device config saves float - need to save and load like mpu6050 does
    //#endif
}
