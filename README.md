# SlimeVR Tracker ESP firmware

Firmware for ESP8266 microcontroller and different IMU sensors to use them as a vive-like trackers in VR.

Requires [SlimeVR Server](https://github.com/SlimeVR/SlimeVR-Server) to work with Steamvr and resolve pose.


## Compatibility

Compatible and tested with these IMUs (select during compilation):
* BNO085
  * Using any fusion in internal DMP. Best results with ARVR Stabilized Game Rotation Vector or ARVR Stabilized Rotation Vector if in good magnetic environment
* MPU-9250
  * Using Mahony sensor fusion of Gyroscope, Magnetometer and Accelerometer, requires good magnetic environment
* MPU-6500
  * Using internal DMP to fuse Gyroscope and Accelerometer, can be used with MPU-9250, can drift substantially
* MPU-6050
  * Same as MPU-6500
