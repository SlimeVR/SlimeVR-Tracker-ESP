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

#include "Wire.h"
#include "ota.h"
#include "sensor.h"
#include "configuration.h"
#include "wifihandler.h"
#include "udpclient.h"
#include "defines.h"
#include "credentials.h"
#include <i2cscan.h>
#include "serialcommands.h"

#if IMU == IMU_BNO080 || IMU == IMU_BNO085
    BNO080Sensor sensor{};
    #if defined(SECOND_IMU) && SECOND_IMU && defined(PIN_IMU_INT_2)
        #define HAS_SECOND_IMU true
        BNO080Sensor sensor2{};
    #endif
#elif IMU == IMU_BNO055
    BNO055Sensor sensor{};
#elif IMU == IMU_MPU9250
    MPU9250Sensor sensor{};
#elif IMU == IMU_MPU6500 || IMU == IMU_MPU6050
    MPU6050Sensor sensor{};
#else
    #error Unsupported IMU
#endif
#ifndef HAS_SECOND_IMU
    EmptySensor sensor2{};
#endif
DeviceConfig config{};

bool isCalibrating = false;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long last_battery_sample = 0;
bool secondImuActive = false;

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
int I2C_ClearBus();

void setup()
{
    //wifi_set_sleep_type(NONE_SLEEP_T);
    // Glow diode while loading
    pinMode(LOADING_LED, OUTPUT);
    pinMode(CALIBRATING_LED, OUTPUT);
    digitalWrite(CALIBRATING_LED, HIGH);
    digitalWrite(LOADING_LED, LOW);
    
    Serial.begin(serialBaudRate);
    setUpSerialCommands();

    I2C_ClearBus(); // Make sure the bus isn't suck when reseting ESP without powering it down

    // join I2C bus
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
#ifdef ESP8266
    Wire.setClockStretchLimit(150000L); // Default streatch limit 150mS
#endif
    Wire.setClock(400000);

    if (hasConfigStored())
    {
        loadConfig(&config);
    }
    
    setConfigRecievedCallback(setConfig);
    setCommandRecievedCallback(commandRecieved);
    // Wait for IMU to boot
    delay(500);
    
    // Currently only second BNO08X is supported
#if IMU == IMU_BNO080 || IMU == IMU_BNO085
    #ifdef HAS_SECOND_IMU
        uint8_t first = I2CSCAN::pickDevice(BNO_ADDR_1, BNO_ADDR_2, true);
        uint8_t second = I2CSCAN::pickDevice(BNO_ADDR_2, BNO_ADDR_1, false);
        if(first != second) {
            sensor.setupBNO080(false, first, PIN_IMU_INT);
            sensor2.setupBNO080(true, second, PIN_IMU_INT_2);
            secondImuActive = true;
        } else {
            sensor.setupBNO080(false, first, PIN_IMU_INT);
        }
    #else
    sensor.setupBNO080(false, I2CSCAN::pickDevice(BNO_ADDR_1, BNO_ADDR_2, true), PIN_IMU_INT);
    #endif
#endif

    sensor.motionSetup(&config);
#ifdef HAS_SECOND_IMU
    if(secondImuActive)
        sensor2.motionSetup(&config);
#endif

    setUpWiFi(&config);
    otaSetup(otaPassword);
    digitalWrite(LOADING_LED, HIGH);
}

// AHRS loop

void loop()
{
    serialCommandsUpdate();
    wifiUpkeep();
    otaUpdate();
    clientUpdate(&sensor, &sensor2);
    if (isCalibrating)
    {
        sensor.startCalibration(0);
        //sensor2.startCalibration(0);
        isCalibrating = false;
    }
#ifndef UPDATE_IMU_UNCONNECTED
        if(isConnected()) {
#endif
    sensor.motionLoop();
#ifdef HAS_SECOND_IMU
    sensor2.motionLoop();
#endif
#ifndef UPDATE_IMU_UNCONNECTED
        }
#endif
    // Send updates
    now_ms = millis();
    if (now_ms - last_ms >= samplingRateInMillis)
    {
        last_ms = now_ms;
        processBlinking();
    }
#ifndef SEND_UPDATES_UNCONNECTED
    if(isConnected()) {
#endif
        sensor.sendData();
#ifdef HAS_SECOND_IMU
        sensor2.sendData();
#endif
#ifndef SEND_UPDATES_UNCONNECTED
    }
#endif
#ifdef PIN_BATTERY_LEVEL
    if(now_ms - last_battery_sample >= batterySampleRate) {
        last_battery_sample = now_ms;
        float battery = ((float) analogRead(PIN_BATTERY_LEVEL)) * batteryADCMultiplier;
        sendFloat(battery, PACKET_BATTERY_LEVEL);
    }
#endif

#ifdef SENSOR_LOOP_SLEEP_MILLIS
    delay(SENSOR_LOOP_SLEEP_MILLIS);
#endif
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

/**
 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 * From: http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
 */
int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}