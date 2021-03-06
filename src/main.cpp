//
// MPU-9250 Mahony AHRS  S.J. Remington 3/2020
// last update 12/17/2020
// added full matrix calibration for accel and mag

// ***Standard orientation defined by gyro/accel: X North Y West Z Up***

// VERY VERY IMPORTANT!
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work, and the gyro offset must be determned.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for calibration, use the companion programs MPU9250_cal and Magneto 1.2 from sailboatinstruments.blogspot.com
//
// For correcting the data, below I use the diagonal element of matrix A and ignore
// the off diagonal components. If those terms are large, (most likely only for the magnetometer)
// add them in to the corrections in function get_MPU_scaled()
//
// This version must be compiled with library routines in subfolder "libs"

#include "Wire.h"
#include "ota.h"
#include "sensor.h"
#include "configuration.h"
#include "udpclient.h"
#include "defines.h"
#include "credentials.h"

#if IMU == IMU_BNO080
    BNO080Sensor sensor{};
#elif IMU == IMU_MPU9250
    MPU9250Sensor sensor{};
#elif IMU == IMU_MPU6500
    MPU6050Sensor sensor{};
#endif
DeviceConfig config{};

bool isCalibrating = false;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long last_battery_sample = 0;

void setConfig(DeviceConfig newConfig)
{
    config = newConfig;
    saveConfig(&config);
}

void commandRecieved(int command, void * const commandData, int commandDataLength)
{
    switch (command)
    {
    case COMMAND_CALLIBRATE:
        isCalibrating = true;
        break;
    case COMMAND_SEND_CONFIG:
        sendConfig(&config, PACKET_CONFIG);
        break;
    case COMMAND_BLINK:
        blinking = true;
        blinkStart = now_ms;
        break;
    }
}

void processBlinking();

void setup()
{
    // Glow diode while loading
    pinMode(LOADING_LED, OUTPUT);
    pinMode(CALIBRATING_LED, OUTPUT);
    digitalWrite(CALIBRATING_LED, HIGH);
    digitalWrite(LOADING_LED, LOW);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.flush();
    Wire.begin(D2, D1);
    Wire.setClockStretchLimit(4000);
    Serial.begin(serialBaudRate);
    while (!Serial)
        ; // wait for connection

    // Load default calibration values and set up callbacks
    config.calibration = {
        {   49.22,    1.26, -203.52},
        {{  1.01176, -0.00048,  0.00230},
        { -0.00048,  1.00366, -0.00535},
        {  0.00230, -0.00535,  0.99003}},
        {   24.95,   81.77,  -10.36},
        {{  1.44652,  0.02739, -0.02186},
        {  0.02739,  1.48749, -0.00547},
        { -0.02186, -0.00547,  1.42441}},
        {   22.97,    13.56,   -90.65}};
    if (hasConfigStored())
    {
        loadConfig(&config);
    }
    setConfigRecievedCallback(setConfig);
    setCommandRecievedCallback(commandRecieved);

    sensor.motionSetup(&config);

    // Don't start if not connected to MPU
    /*while(!accelgyro.testConnection()) {
        Serial.print("Can't communicate with MPU9250, response ");
        Serial.println(accelgyro.getDeviceID(), HEX);
        delay(500);
    }
    Serial.println("Connected to MPU9250");
    //*/

    setUpWiFi(&config);
    otaSetup(otaPassword);
    digitalWrite(LOADING_LED, HIGH);
}

// AHRS loop

void loop()
{
    otaUpdate();
    clientUpdate();
    if(isConnected())
    {
        if (isCalibrating)
        {
            sensor.startCalibration(0);
            isCalibrating = false;
        }
        sensor.motionLoop();
        // Send updates
        now_ms = millis();
        if (now_ms - last_ms >= samplingRateInMillis)
        {
            last_ms = now_ms;
            processBlinking();

            sensor.sendData();
        }
        if(now_ms - last_battery_sample >= batterySampleRate) {
            last_battery_sample = now_ms;
            float battery = ((float) analogRead(A0)) / 1024.0 * 14.0;
            sendFloat(battery, PACKET_BATTERY_LEVEL);
        }
    }
}

void processBlinking() {
    if (blinking)
    {
        if (blinkStart + sensorIdTime < now_ms)
        {
            blinking = false;
            digitalWrite(LOADING_LED, HIGH);
        }
        else
        {
            int t = (now_ms - blinkStart) / sensorIdInterval;
            if(t % 2) {
                digitalWrite(LOADING_LED, LOW);
            } else {
                digitalWrite(LOADING_LED, HIGH);
            }
        }
        
    }
}