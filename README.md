# SlimeVR Tracker firmware for ESP

Firmware for ESP8266 / ESP32 microcontrollers and different IMU sensors to use them as a vive-like trackers in VR.

Requires [SlimeVR Server](https://github.com/SlimeVR/SlimeVR-Server) to work with SteamVR and resolve pose. Should be compatible with [owoTrack](https://github.com/abb128/owo-track-driver), but is not guaranteed.

## Configuration

Firmware configuration is located in the `defines.h` file. For more information on how to configure your firmware, refer to the [Configuring the firmware project section of SlimeVR documentation](https://docs.slimevr.dev/firmware/configuring-project.html).

## Compatibility

The following IMUs and their corresponding `IMU` values are supported by the firmware:
* BNO085 & BNO086 (IMU_BNO085)
  * Using any fusion in internal DMP. Best results with ARVR Stabilized Game Rotation Vector or ARVR Stabilized Rotation Vector if in good magnetic environment.
* BNO080 (IMU_BNO080)
  * Using any fusion in internal DMP. Doesn't have BNO085's ARVR stabilization, but still gives good results.
* MPU-6500 (IMU_MPU6500) & MPU-6050 (IMU_MPU6050)
  * Using internal DMP to fuse Gyroscope and Accelerometer. Can drift substantially.
  * NOTE: Currently the MPU will auto calibrate when powered on. You *must* place it on the ground and *DO NOT* move it until calibration is complete (for a few seconds). **LED on the ESP will blink 5 times after calibration is over.**
* BNO055 (IMU_BNO055)
  * Performs much worse than BNO080, despite having the same hardware. Not recommended for use.
* MPU-9250 (IMU_MPU9250)
  * Using Mahony sensor fusion of Gyroscope, Magnetometer and Accelerometer, requires good magnetic environment.
  * See *Sensor calibration* below for info on calibrating this sensor.
  * Specify `IMU_MPU6500` in your `defines.h` to use without magentometer in 6DoF mode.
  * Experimental support!
* BMI160 (IMU_BMI160)
  * Using Mahony sensor fusion of Gyroscope and Accelerometer
  * See *Sensor calibration* below for info on calibrating this sensor.
  * Experimental support!
* ICM-20948 (IMU_ICM20948)
  * Using fision in internal DMP for 6Dof or 9DoF, 9DoF mode requires good magnetic environment.
  * Comment out `USE_6DOF` in `debug.h` for 9DoF mode.
  * Experimental support!

Firmware can work with both ESP8266 and ESP32. Please edit `defines.h` and set your pinout properly according to how you connected the IMU.

## Sensor calibration

*It is generally recommended to turn trackers on and let them lay down on a flat surface for a few seconds.** This will calibrate them better.

**Some trackers require special calibration steps on startup:**
* MPU-9250, BMI160
  * Turn them on with chip facing down. Flip up and put on a surface for a couple of seconds, the LED will light up.
  * After a few blinks, the LED will light up again
  * Slowly rotate the tracker in an 8-motion facing different derections for about 30 seconds, while LED is blinking
  * LED will turn off when calibration is complete
  * You don't have to calibrate next time you power it off, calibration values will be saved for the next use

## Uploading On Linux

Follow the instructions in this link [Platformio](https://docs.platformio.org/en/latest//faq.html#platformio-udev-rules), this should solve any permission denied errors

## Contributions

By contributing to this project you are placing all your code under MIT or less restricting licenses, and you certify that the code you have used is compatible with those licenses or is authored by you. If you're doing so on your work time, you certify that your employer is okay with this.

For an explanation on how to contribute, see [`CONTRIBUTING.md`](CONTRIBUTING.md)
