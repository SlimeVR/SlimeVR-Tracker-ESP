# SlimeVR Tracker firmware for ESP

Firmware for ESP8266 / ESP32 microcontrollers and different IMU sensors to use them as a vive-like trackers in VR.

Requires [SlimeVR Server](https://github.com/SlimeVR/SlimeVR-Server) to work with SteamVR and resolve pose. Should be compatible with [owoTrack](https://github.com/abb128/owo-track-driver), but is not guaranteed.

## Compatibility

Compatible and tested with these IMUs (select during compilation):
* BNO085, BNO086
  * Using any fusion in internal DMP. Best results with ARVR Stabilized Game Rotation Vector or ARVR Stabilized Rotation Vector if in good magnetic environment
* BNO080
  * Using any fusion in internal DMP. Doesn't have BNO085's ARVR stabilization, but still gives good results.
* BNO055
  * Should be roughly equal BNO080, but cheaper. Not tested thoroughly, please report your results on Discord if you're willing to try.
* MPU-9250
  * Using Mahony sensor fusion of Gyroscope, Magnetometer and Accelerometer, requires good magnetic environment.
  * NOTE: Currently can't be calibrated due to lack of proper commands from the server. Work in progress. Can be ran as MPU-6050 below w/o magnetometer.
* MPU-6500 & MPU-6050
  * Using internal DMP to fuse Gyroscope and Accelerometer, can be used with MPU-9250 to use it without accelerometer, can drift substantially.
  * NOTE: Currently the MPU will auto calibrate when powered on. You *must* place it on the ground and *DO NOT* move it until calibration is complete (for a few seconds). **LED on the ESP will blink 5 times after calibration is over.**

Firmware can work with both ESP8266 and ESP32. Please edit defines.h and set your pinout properly according to how you connected the IMU.
