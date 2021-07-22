
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

* Work in progress. Should be roughly equal BNO080, but cheaper

* MPU-9250

* Using Mahony sensor fusion of Gyroscope, Magnetometer and Accelerometer, requires good magnetic environment

* MPU-6500

* Using internal DMP to fuse Gyroscope and Accelerometer, can be used with MPU-9250, can drift substantially

* MPU-6050

* Same as MPU-6500

  

Firmware can work with both ESP8266 and ESP32. Please edit defines.h and set your pinout properly according to how you connected the IMU.

## How to build and upload

Prerequisites:

 - Download and install VSCode IDE https://code.visualstudio.com/
 - if your board is on CH430 USB To UART download driver http://www.wch-ic.com/downloads/CH341SER_ZIP.html
 - If your board is on CP210x USB to UART download driver https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
 - Install Platform.io to VSCode using this guide https://platformio.org/install/ide?install=vscode

Building and uploading

You need to execute this command in the folder where you want this project.

    git clone https://github.com/adigyran/SlimeVR-Tracker-ESP

Open project in VSCode and change platformio.ini and defines.h contents according to the pinout and board type

Locate the alien head button in left menu and press "Build"

![enter image description here](https://i.imgur.com/aAQUQoC.png)
 
Connect the tracker by USB and press Upload
