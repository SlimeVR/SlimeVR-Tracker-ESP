/*
  Hardware setup:
  MPU9250 Breakout --------- Adafruit Huzzah
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- Pin4
  SCL ----------------------- Pin5
  GND ---------------------- GND

  EEPROM
  Vcc to 3.3V
  GND to GND
  AO, A1, A2 to GND  (on 24LC256 this gives an i2c slave address as 1010000 which is 0x50)
  SDA/SCL to Pin4 and Pin5 of Adafruit Huzzah, respectively
*/

///////////////////////////////////////////////////////////////////
//Debug information
///////////////////////////////////////////////////////////////////
#define serialDebug false // Set to true to get Serial output for debugging
#define serialBaudRate 115200

///////////////////////////////////////////////////////////////////
//Determines how often we sample and send data
///////////////////////////////////////////////////////////////////
#define samplingRateInMillis 10

///////////////////////////////////////////////////////////////////
//Setup for the Magnetometer
///////////////////////////////////////////////////////////////////
#define useFullCalibrationMatrix true

#define sensorIdTime 1000
#define sensorIdInterval 100