/*
Based on Demo's fork
*/

#include "sensor.h"
#include "udpclient.h"
#include "calibration.h"

// seconds after previous save (from start) when calibration (DMP Bias) data will be saved to NVS. Increments through the list then stops; to prevent unwelcome eeprom wear.
int bias_save_periods[] = { 120, 180, 300, 600, 600 }; // 2min + 3min + 5min + 10min + 10min (no more saves after 30min)
const uint8_t number_i2c_addr = 1;
uint8_t poss_addresses[number_i2c_addr] = {0X69}; // 0X68

#ifndef USE_6_AXIS
    #define USE_6_AXIS true
#endif

#ifndef ENABLE_TAP
    #define ENABLE_TAP false
#endif

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
                ICM_address = poss_addresses[add_int];
                ICM_found = true; 
            }
        }
    }
}

void ICM20948Sensor::save_bias(bool repeat) { 
#if ESP32 && defined(SAVE_BIAS)
  if (SAVE_BIAS) {
    int bias_a[3], bias_g[3], bias_m[3];
    
    icm20948.SetBiasGyroX(ICM_20948_Device_t * pdev, &bias_g[0])

    icm20948.getGyroBias(bias_g);
    icm20948.getMagBias(bias_m);
    icm20948.getAccelBias(bias_a);

    bool accel_set = bias_a[0] && bias_a[1] && bias_a[2];
    bool gyro_set = bias_g[0] && bias_g[1] && bias_g[2];
    bool mag_set = bias_m[0] && bias_m[1] && bias_m[2];
         
    #ifdef FULL_DEBUG
      Serial.println("bias gyro result:");
      Serial.println(bias_g[0]); 
      Serial.println(bias_g[1]);
      Serial.println(bias_g[2]);
      Serial.println("end gyro");  
    
      Serial.println("bias accel result:");  
      Serial.println(bias_a[0]); 
      Serial.println(bias_a[1]);
      Serial.println(bias_a[2]);
      Serial.println("end accel");   
    
      Serial.println("bias mag result:");
      Serial.println(bias_m[0]); 
      Serial.println(bias_m[1]);
      Serial.println(bias_m[2]);
      Serial.println("end mag"); 
    #endif
    
    if (accel_set) {
      // Save accel
      prefs.putInt(auxiliary ? "ba01" : "ba00", bias_a[0]);
      prefs.putInt(auxiliary ? "ba11" : "ba10", bias_a[1]);
      prefs.putInt(auxiliary ? "ba21" : "ba20", bias_a[2]);

    #ifdef FULL_DEBUG
            Serial.println("Wrote Accel Bias");
    #endif
    }
    
    if (gyro_set) {
      // Save gyro
      prefs.putInt(auxiliary ? "bg01" : "bg00", bias_g[0]);
      prefs.putInt(auxiliary ? "bg11" : "bg10", bias_g[1]);
      prefs.putInt(auxiliary ? "bg21" : "bg20", bias_g[2]);

    #ifdef FULL_DEBUG
        Serial.println("Wrote Gyro Bias");
    #endif
    }

    if (mag_set) {
      // Save mag
      prefs.putInt(auxiliary ? "bm01" : "bm00", bias_m[0]);
      prefs.putInt(auxiliary ? "bm11" : "bm10", bias_m[1]);
      prefs.putInt(auxiliary ? "bm21" : "bm20", bias_m[2]);

    #ifdef FULL_DEBUG
        Serial.println("Wrote Mag Bias");
    #endif
    }    
  }  

    if (repeat) {
        bias_save_counter++;
        // Possible: Could make it repeat the final timer value if any of the biases are still 0. Save strategy could be improved.
        if (sizeof(bias_save_periods) != bias_save_counter)
        {
            timer.in(bias_save_periods[bias_save_counter] * 1000, [](void *arg) -> bool { ((ICM20948Sensor*)arg)->save_bias(true); return false; }, this);
        }
    }

#endif // ifdef ESP32
}

void ICM20948Sensor::setupICM20948(bool auxiliary, uint8_t addr) {
    this->addr = addr;
    this->auxiliary = auxiliary;
    this->sensorOffset = {Quat(Vector3(0, 0, 1), ((int)auxiliary) == 0 ? IMU_ROTATION : SECOND_IMU_ROTATION)};
    this->tapDetector = TapDetector(3, [](){}); // Tripple tap

    icmSettings = {
        .i2c_speed = I2C_SPEED,              // i2c clock speed
        .i2c_address = 0x69,                 // i2c address (0x69 / 0x68)
        .is_SPI = false,                     // Enable SPI, if disable use i2c
        .cs_pin = 10,                        // SPI chip select pin
        .spi_speed = 7000000,                // SPI clock speed in Hz, max speed is 7MHz
        .mode = 1,                           // 0 = low power mode, 1 = high performance mode
        .enable_gyroscope = false,           // Enables gyroscope output
        .enable_accelerometer = ENABLE_TAP,  // Enables accelerometer output
        .enable_magnetometer = false,        // Enables magnetometer output // Enables quaternion output
        .enable_gravity = false,             // Enables gravity vector output
        .enable_linearAcceleration = false,  // Enables linear acceleration output
        .enable_quaternion6 = USE_6_AXIS,    // Enables quaternion 6DOF output
        .enable_quaternion9 = !USE_6_AXIS,   // Enables quaternion 9DOF output
        .enable_har = false,                 // Enables activity recognition
        .enable_steps = false,               // Enables step counter
        .enable_step_detector = false,       // Probably not working
        .gyroscope_frequency = 1,            // Max frequency = 225, min frequency = 1
        .accelerometer_frequency = 200,      // Max frequency = 225, min frequency = 1
        .magnetometer_frequency = 1,         // Max frequency = 70, min frequency = 1 
        .gravity_frequency = 1,              // Max frequency = 225, min frequency = 1
        .linearAcceleration_frequency = 1,   // Max frequency = 225, min frequency = 1
        .quaternion6_frequency = 150,        // Max frequency = 225, min frequency = 50
        .quaternion9_frequency = 150,        // Max frequency = 225, min frequency = 50
        .har_frequency = 50,                 // Max frequency = 225, min frequency = 50  
        .steps_frequency = 50,               // Max frequency = 225, min frequency = 50
        .step_detector_frequency = 100,      // Max frequency = 225, min frequency = 50
    };
}

void ICM20948Sensor::motionSetup() {

    // Configure imu setup and load any stored bias values
    #ifdef ESP32
        prefs.begin("ICM20948", false);  
    #endif
    poss_addresses[0] = addr;
    i2c_scan();
    Serial.println("Scan completed");
    if (ICM_found) {      
        icmSettings.i2c_address = ICM_address;
        int rc = icm20948.init(icmSettings);
        if (rc == 0) {
        #if ESP32 && defined(LOAD_BIAS)
            if (LOAD_BIAS) {    
                int32_t bias_a[3], bias_g[3], bias_m[3];

                bias_a[0] = prefs.getInt(auxiliary ? "ba01" : "ba00", 0);
                bias_a[1] = prefs.getInt(auxiliary ? "ba11" : "ba10", 0);
                bias_a[2] = prefs.getInt(auxiliary ? "ba21" : "ba20", 0);
                
                bias_g[0] = prefs.getInt(auxiliary ? "bg01" : "bg00", 0);
                bias_g[1] = prefs.getInt(auxiliary ? "bg11" : "bg10", 0);
                bias_g[2] = prefs.getInt(auxiliary ? "bg21" : "bg20", 0);
                
                bias_m[0] = prefs.getInt(auxiliary ? "bm01" : "bm00", 0);
                bias_m[1] = prefs.getInt(auxiliary ? "bm11" : "bm10", 0);
                bias_m[2] = prefs.getInt(auxiliary ? "bm21" : "bm20", 0);
                
                icm20948.setGyroBias(bias_g);
                icm20948.setMagBias(bias_m);
                icm20948.setAccelBias(bias_a);
                
            #ifdef FULL_DEBUG
                Serial.print("read accel ");
                Serial.print(bias_a[0]);
                Serial.print(" - ");
                Serial.print(bias_a[1]);
                Serial.print(" - ");
                Serial.println(bias_a[2]);
            
                Serial.print("read gyro ");
                Serial.print(bias_g[0]);
                Serial.print(" - ");
                Serial.print(bias_g[1]);
                Serial.print(" - ");
                Serial.println(bias_g[2]);
            
                Serial.print("read mag ");
                Serial.print(bias_m[0]);
                Serial.print(" - ");
                Serial.print(bias_m[1]);
                Serial.print(" - ");
                Serial.println(bias_m[2]);      
                
                Serial.println("Actual loaded biases: ");
                save_bias(false); // tests that the values were written successfully
            #endif
            }             
        #endif // ifdef ESP32

            lastData = millis();
            working = true;
            ICM_init = true;

            #ifdef ESP32
                timer.in(bias_save_periods[0] * 1000, [](void *arg) -> bool { ((ICM20948Sensor*)arg)->save_bias(true); return false; }, this); 
            #endif
        }
        else { // Some error on init
            ICM_init = false;
            Serial.println("ICM init error");
        }
    } 
    else {
        Serial.println("No ICM found");
    }
}

void ICM20948Sensor::motionLoop() {
    if (ICM_found && ICM_init) {   
    #ifdef ESP32
        timer.tick();
    #endif
        int r = icm20948.task();

        if (r == 0) {
            if (icm20948.quat6DataIsReady())
            {
                icm20948.readQuat6Data(&quaternion.w, &quaternion.x, &quaternion.y, &quaternion.z);
                quaternion *= sensorOffset; // May prefer to use icm20948 mount matrix option?
                newData = true;
                lastData = millis();
            }
            else if (icm20948.quat9DataIsReady())
            {
                icm20948.readQuat9Data(&quaternion.w, &quaternion.x, &quaternion.y, &quaternion.z);
                quaternion *= sensorOffset; // May prefer to use icm20948 mount matrix option?
                newData = true;   
                lastData = millis();
            }
            
            if (icm20948.accelDataIsReady())
            {
                float x, y, z;
                icm20948.readAccelData(&x, &y, &z);        
                
                newTap |= tapDetector.update(z);
            }    
        }
        else { // Some err reported
            // Should do something here
            //Serial.println(r);
        }  
    }

    if (lastData + 1000 < millis()) {
        working = false;
        lastData = millis();        
        Serial.print("[ERR] Sensor timeout ");
        Serial.println(addr);
    }
}

void ICM20948Sensor::sendData() { 
    if (newData) {
        newData = false;
        sendRotationData(&quaternion, DATA_TYPE_NORMAL, 0, auxiliary, PACKET_ROTATION_DATA);
#ifdef FULL_DEBUG
            //Serial.print("[DBG] Quaternion: ");
            //Serial.print(quaternion.x);
            //Serial.print(",");
            //Serial.print(quaternion.y);
            //Serial.print(",");
            //Serial.print(quaternion.z);
            //Serial.print(",");
            //Serial.println(quaternion.w);
#endif
    }
    
    if (newTap) {
        sendByte(1, auxiliary, PACKET_TAP);
        newTap = false;
    }
}

void ICM20948Sensor::startCalibration(int calibrationType) {
    // 20948 does continuous calibration and no need to use for ESP32 as it auto saves bias values

    // If ESP32, manually force a new save
    #ifdef ESP32
        save_bias(false);
    #endif
    // TODO: If 8266, save the current bias values to eeprom
    //#ifdef 8266
    // Types are int, device config saves float - need to save and load like mpu6050 does
    //#endif
}


// TODO:
// Base lib is modified to work with >1 imu - seems ok but could do with more verification
// Getting more drift than expected, possible causes: my calibration is off, my hardware, the imu itself or library / i2c implementation?
// confirm imu orientation matrix
// make 8266 save and load config
// make it use Eiren i2c scanning?
// make it work with interupts?
// no reset properly enabled
