# InvenSense Digital Motion Processor (DMP™)

## What is the Digital Motion Processor (DMP™)?

In version 1.2 of this library, we added _partial_ support for the InvenSense Digital Motion Processor (DMP™). The DMP is firmware which runs on the
ICM-20948 and which "offloads computation of motion processing algorithms from the host processor, improving system power performance".

"The DMP enables ultra-low power run-time and background calibration of the accelerometer, gyroscope, and compass, maintaining optimal performance of
the sensor data for both physical and virtual sensors generated through sensor fusion."

The DMP allows the accelerometer, gyro and magnetometer data to be combined (fused) so that Quaternion data can be produced.

The DMP firmware binary has been available for quite some time. It is included in InvenSense's "MotionLink" and "Embedded Motion Driver (eMD)" examples
which can be downloaded from the InvenSense Developers Corner. However, the code is opaque and difficult to follow.

Users like [@ericalbers](https://github.com/ericalbers/ICM20948_DMP_Arduino) and [@ZaneL](https://github.com/ZaneL/Teensy-ICM-20948) have ported the
InvenSense example code to the Arduino environment previously. We are grateful to Eric and Zane as their code allowed us to reverse-engineer some of the
ICM-20948 configuration steps.

We are also grateful to InvenSense themselves for sharing with us a _confidential & proprietary_ document called "_Application Note: Programming Sequence for
ICM-20648 DMP Hardware Function_". InvenSense admit that the document is not complete and have asked us not to share it openly.

The InvenSense document and the bus traffic we captured using Zane's port have allowed us to add _partial_ support for the DMP to this library, using our
own functions. We say _partial_ because, at the time of writing, our library does not support: activity recognition, step counting, pick-up and tap-detection.
It does however support:
- Raw and calibrated accelerometer, gyro and compass data and accuracy
- 6-axis and 9-axis Quaternion data (including Game Rotation Vector data)
- Geomagnetic Rotation Vector data
- and [more...](#which-dmp-features-are-currently-supported)

We have added [five new examples](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/master/examples/Arduino) to show how to configure the DMP and read:
9-axis Quaternion data; 6-axis Quaternion converted to Euler angles (roll, pitch & yaw); raw accelerometer data.

## Is DMP support enabled by default?

No. The DMP occupies 14kBytes of program memory and so, to allow the library to continue to run on processors with limited memory, DMP support is disabled by default.

You can enable it by editing the file called ```ICM_20948_C.h``` and uncommenting [line 29](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/src/util/ICM_20948_C.h#L29):

Change:

```
//#define ICM_20948_USE_DMP
```

to:

```
#define ICM_20948_USE_DMP
```

You will find ```ICM_20948_C.h``` in the library _src\util_ folder. If you are using Windows, you will find it in _Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util_.

## How is the DMP loaded and started?

In version 1.2.5 we added a new helper function named ```initializeDMP```. This is a weak function which you can overwrite e.g. if you want to change the sample rate
(see [Example10](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/examples/Arduino/Example10_DMP_FastMultipleSensors/Example10_DMP_FastMultipleSensors.ino) for details).
```initializeDMP``` does most of the heavy lifting for you: it downloads the DMP firmware; and configures all of the registers with the appropriate values. The only things
you need to do manually are: select which DMP sensors to enable; reset and start both FIFO and DMP.

The DMP firmware is loaded into the ICM-20948's processor memory space via three special Bank 0 registers:
- **AGB0_REG_MEM_START_ADDR** (0x7C) - the address which AGB0_REG_MEM_R_W reads from or writes to (it auto-increments after each read or write)
- **AGB0_REG_MEM_R_W** (0x7D) - the memory read/write register
- **AGB0_REG_MEM_BANK_SEL** (0x7E) - the memory bank select. The complete read/write address is: (AGB0_REG_MEM_BANK_SEL * 256) + AGB0_REG_MEM_START_ADDR

The firmware binary (14290 or 14301 Bytes) is written into processor memory starting at address 0x90. ```loadDMPFirmware``` automatically breaks the code up into 256 byte blocks and increments
**AGB0_REG_MEM_BANK_SEL** during the writing.

Before the DMP is enabled, the 16-bit register **AGB2_REG_PRGM_START_ADDRH** (Bank 2, 0x50) needs to be loaded with the program start address. ```setDMPstartAddress``` does this for you.

The DMP is enabled or reset by setting bits in the Bank 0 register **AGB0_REG_USER_CTRL** (0x03). ```enableDMP``` and ```resetDMP``` do this for you.

The helper functions ```readDMPmems``` and ```writeDMPmems``` will let you read and write data directly from the DMP memory space.

## How do I access the DMP data?

The DMP data is returned via the FIFO (First In First Out). ```readDMPdataFromFIFO``` checks if any data is present in the FIFO (by calling ```getFIFOcount``` which reads the 16-bit register
**AGB0_REG_FIFO_COUNT_H** (0x70)). If data is present, it is copied into a ```icm_20948_DMP_data_t``` struct.

```readDMPdataFromFIFO``` will return:
- ```ICM_20948_Stat_FIFONoDataAvail``` if no data or incomplete data is available
- ```ICM_20948_Stat_Ok``` if a valid frame was read
- ```ICM_20948_Stat_FIFOMoreDataAvail``` if a valid frame was read _and_ the FIFO contains more (unread) data
- ```ICM_20948_Stat_FIFOIncompleteData``` if a frame was present in the FIFO but it was incomplete

You can examine the 16-bit ```icm_20948_DMP_data_t data.header``` to see what data the frame contained. ```data.header``` is a bit field; each bit indicates what data is present:
- **DMP_header_bitmap_Compass_Calibr** (0x0020)
- **DMP_header_bitmap_Gyro_Calibr** (0x0040)
- **DMP_header_bitmap_Geomag** (0x0100)
- **DMP_header_bitmap_PQuat6** (0x0200)
- **DMP_header_bitmap_Quat9** (0x0400)
- **DMP_header_bitmap_Quat6** (0x0800)
- **DMP_header_bitmap_ALS** (0x1000)
- **DMP_header_bitmap_Compass** (0x2000)
- **DMP_header_bitmap_Gyro** (0x4000)
- **DMP_header_bitmap_Accel** (0x8000)

**DMP_header_bitmap_Header2** (0x0008) indicates if any secondary data was included. If the **DMP_header_bitmap_Header2** bit is set, the frame also contained one or more of:
- **DMP_header2_bitmap_Compass_Accuracy** (0x1000)
- **DMP_header2_bitmap_Gyro_Accuracy** (0x2000)
- **DMP_header2_bitmap_Accel_Accuracy** (0x4000)

## Which DMP features are currently supported?

All of the following _should_ work, but we have not tested them all:

```
INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)
```

## What changes did you make in v1.2.5?

In v1.2.5 we added some critical missing configuration steps:
- We use I2C_SLV0 and I2C_SLV1 to request the magnetometer data and trigger the next Single Measurement. We no longer use the 100Hz continuous mode for the DMP
- We now read ten bytes of data from the magnetometer, starting at register 0x03; instead of reading nine bytes, starting at register 0x10
  - Register 0x03 is reserved and the other nine registers are undocumented. They appear to contain the raw magnetometer reading in big-endian format (instead of little-endian)
  - We had to dig deep into InvenSense's Icm20948AuxCompassAkm.c to find this out...
- We configure the I2C Master ODR which reduces the magnetometer read rate from a silly 1100Hz to a sensible 69Hz
  - We had to monitor the Aux I2C pins and study the AK09916 traffic to figure this out...

The DMP configuration code was becoming so verbose that we decided to move it into its own function called ```initializeDMP```. This is a weak function which you can overwrite
e.g. if you want to change the sample rate
(see [Example10](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/examples/Arduino/Example10_DMP_FastMultipleSensors/Example10_DMP_FastMultipleSensors.ino) for details).
```initializeDMP``` does most of the heavy lifting for you: it downloads the DMP firmware; and configures all of the registers with the appropriate values. The only things
you need to do manually are: select which DMP sensors to enable; reset and start both FIFO and DMP. Please see the revised DMP examples for more details.

## What changes did you make in v1.2.6?

The final piece of the DMP puzzle turned out to be that the accel and gyro must not be in low power mode when using the DMP. The InvenSense code puts the I2C Controller,
the accel and the gyro into low power mode initially; our library did the same thing. But we had missed that the code then changes its mind and leaves only the I2C Controller
in low power mode. This subtle detail is what was preventing the DMP from working correctly. (It took us _weeks_ to find this!)

## Where are the DMP registers defined?

You will find the definitions in ```ICM_20948_DMP.h```.

That file also includes the definition for the ```icm_20948_DMP_data_t``` struct which is loaded with DMP data from the FIFO.

```const int``` declarations (including the DMP firmware image) are in ```ICM_20948_C.c```

## Can the DMP generate interrupts?

Yes it can, but you might find that they are not fully supported as we have not tested them. The main functions you will need to experiment with are ```intEnableDMP``` and ```enableDMPSensorInt```.

## How is the DMP data rate set?

It is a _combination_ of the raw sensor rate (set by ```setSampleRate```) and the multiple DMP Output Data Rate (ODR) registers
(set by ```setDMPODRrate```). There are other settings that need to be changed to match the sample rate too.
Please see [examples 9 & 10](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/master/examples/Arduino) for more details.

The DMP is capable of outputting multiple sensor data at different rates to the FIFO.

## Can I contribute to this library?

Absolutely! Please see [CONTRIBUTING.md](./CONTRIBUTING.md) for further details.

## Can I see the full DMP configuration captured from @ZaneL 's Teensy-ICM-20948 code?

Brace yourself. Here it is. ZaneL's code defaults to using the Game Rotation Vector (Quat6) at 225Hz and - by default - does not start the magnetometer.
So you won't see the I2C Controller configuration traffic here.

- **...** indicates where I've omitted some of the bus transactions (e.g. the bulk of the firmware upload)

```
SPI Traffic captured using ZaneL's Teensy-ICM-20948 wrapper for the InvenSense Nucleo example:
https://github.com/ZaneL/Teensy-ICM-20948

QuaternionAnimation (Quat6):
  .enable_gyroscope = false,      // Enables gyroscope output
  .enable_accelerometer = false,  // Enables accelerometer output
  .enable_magnetometer = false,   // Enables magnetometer output
  .enable_quaternion = true,     // Enables quaternion output
  .gyroscope_frequency = 1,      // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,  // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,   // Max frequency = 70, min frequency = 1
  .quaternion_frequency = 225     // Max frequency = 225, min frequency = 50




617669-617713 SPI: COPI data: 80 : Read Bank 0 Reg 0x00 WHO_AM_I
617711-617755 SPI: CIPO data: EA : ICM_20948

617952-617994 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
617994-618036 SPI: COPI data: 01 : Clock Source Auto (Best Available)

618291-618333 SPI: COPI data: 83 : Read Bank 0 Reg 0x03 USER_CTRL
618333-618375 SPI: COPI data: 00

618426-618468 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
618468-618510 SPI: COPI data: 10 : I2C_IF_DIS Reset I2C peripheral module and put the serial interface in SPI mode only

618560-618602 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
618602-618644 SPI: COPI data: 47 : Disable pressure sensor. Disable all gyroscope axes

618695-618737 SPI: COPI data: 80 : Read Bank 0 Reg 0x00 WHO_AM_I
618737-618779 SPI: CIPO data: EA : ICM_20948

618830-618872 SPI: COPI data: 05 : Write Bank 0 Reg 0x05 LP_CONFIG
618872-618914 SPI: COPI data: 70 : Operate I2C controller, accel and gyro in duty cycled mode

618963-619007 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
619005-619049 SPI: COPI data: 10 : I2C_IF_DIS Reset I2C peripheral module and put the serial interface in SPI mode only

<Load DMP firmware starts here>

619098-619140 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
619140-619182 SPI: COPI data: 90 : Memory Address 0x90 : Start of the DMP

619225-619269 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
619267-619311 SPI: COPI data: 00 : DMP firmware (16 bytes)
619310-619352 SPI: COPI data: 01
619352-619394 SPI: COPI data: 00
619394-619436 SPI: COPI data: 00
619436-619478 SPI: COPI data: 00
619478-619520 SPI: COPI data: 00
619520-619562 SPI: COPI data: 00
619562-619604 SPI: COPI data: 00
619604-619646 SPI: COPI data: 00
619646-619688 SPI: COPI data: 00
619688-619730 SPI: COPI data: 00
619730-619772 SPI: COPI data: 00
619772-619814 SPI: COPI data: 00
619814-619856 SPI: COPI data: 00
619856-619898 SPI: COPI data: 00
619898-619940 SPI: COPI data: 00

619988-620032 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
620030-620074 SPI: COPI data: A0 : Memory Address 0xA0 : DMP

620116-620158 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
620158-620200 SPI: COPI data: 00 : DMP firmware (16 bytes)
620200-620242 SPI: COPI data: 05
620242-620284 SPI: COPI data: 00
620284-620326 SPI: COPI data: 00
620326-620370 SPI: COPI data: 00
620368-620412 SPI: COPI data: 05
620410-620454 SPI: COPI data: 00
620452-620496 SPI: COPI data: 00
620494-620538 SPI: COPI data: 00
620536-620580 SPI: COPI data: 05
620578-620622 SPI: COPI data: 00
620620-620664 SPI: COPI data: 01
620662-620706 SPI: COPI data: 00
620704-620748 SPI: COPI data: 05
620746-620790 SPI: COPI data: 00
620788-620832 SPI: COPI data: FF

...

625336-625380 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
625378-625422 SPI: COPI data: 01 : Bank 0x01

625464-625508 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
625506-625550 SPI: COPI data: 00 : Memory Address 0x00 : DMP

625592-625634 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
625634-625676 SPI: COPI data: 00 : DMP firmware (16 bytes)
625676-625718 SPI: COPI data: 00
625718-625760 SPI: COPI data: 03
625760-625802 SPI: COPI data: 84
625802-625844 SPI: COPI data: 00
625844-625886 SPI: COPI data: 00
625886-625928 SPI: COPI data: 9C
625928-625970 SPI: COPI data: 40
625970-626012 SPI: COPI data: 00
626012-626054 SPI: COPI data: 00
626054-626096 SPI: COPI data: 00
626096-626138 SPI: COPI data: 00
626138-626180 SPI: COPI data: 04
626180-626222 SPI: COPI data: 00
626222-626264 SPI: COPI data: 00
626264-626306 SPI: COPI data: 00

...

1416262-1416304 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
1416304-1416346 SPI: COPI data: 38 : Bank 0x38

1416390-1416432 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
1416432-1416474 SPI: COPI data: 00 : Memory Address 0x00 : DMP

1416518-1416560 SPI: COPI data: 7D : DMP firmware (16 bytes)
1416560-1416602 SPI: COPI data: DA
1416602-1416644 SPI: COPI data: DE
1416644-1416686 SPI: COPI data: F5
1416686-1416728 SPI: COPI data: E2
1416728-1416770 SPI: COPI data: F0
1416770-1416812 SPI: COPI data: D9
1416812-1416854 SPI: COPI data: F8
1416854-1416896 SPI: COPI data: D8
1416896-1416938 SPI: COPI data: A9
1416938-1416980 SPI: COPI data: F3
1416980-1417022 SPI: COPI data: F9
1417022-1417064 SPI: COPI data: F9
1417064-1417106 SPI: COPI data: F9
1417106-1417148 SPI: COPI data: F9
1417148-1417190 SPI: COPI data: F9
1417190-1417232 SPI: COPI data: F9

...

1421735-1421777 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
1421777-1421819 SPI: COPI data: 60 : Memory Address 0x60 : DMP

1421862-1421906 SPI: COPI data: 7D : DMP firmware (13 bytes)
1421904-1421948 SPI: COPI data: F3
1421946-1421990 SPI: COPI data: CD
1421988-1422032 SPI: COPI data: F2
1422030-1422074 SPI: COPI data: CA
1422072-1422116 SPI: COPI data: D0
1422114-1422158 SPI: COPI data: F3
1422156-1422200 SPI: COPI data: CB
1422198-1422242 SPI: COPI data: F2
1422240-1422284 SPI: COPI data: C8
1422282-1422326 SPI: COPI data: D0
1422324-1422368 SPI: COPI data: F3
1422366-1422410 SPI: COPI data: C9
1422408-1422452 SPI: COPI data: E0

<Verify DMP firmware starts here>

1422500-1422542 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
1422542-1422584 SPI: COPI data: 00 : Bank 0x00

1422628-1422672 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
1422670-1422714 SPI: COPI data: 90 : Memory Address 0x90 : Start of the DMP

1422756-1422800 SPI: COPI data: FD : Read Bank 0 Reg 0x7D Memory Read/Write
1422798-1422842 SPI: CIPO data: 00 : DMP firmware (16 bytes)
1422840-1422884 SPI: CIPO data: 01
1422882-1422926 SPI: CIPO data: 00
1422924-1422968 SPI: CIPO data: 00
1422966-1423010 SPI: CIPO data: 00
1423008-1423052 SPI: CIPO data: 00
1423050-1423094 SPI: CIPO data: 00
1423092-1423136 SPI: CIPO data: 00
1423134-1423178 SPI: CIPO data: 00
1423176-1423220 SPI: CIPO data: 00
1423218-1423262 SPI: CIPO data: 00
1423260-1423304 SPI: CIPO data: 00
1423302-1423346 SPI: CIPO data: 00
1423344-1423388 SPI: CIPO data: 00
1423386-1423430 SPI: CIPO data: 00
1423428-1423472 SPI: CIPO data: 00

1423529-1423571 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
1423571-1423613 SPI: COPI data: A0 : Memory Address 0xA0 : DMP

1423657-1423701 SPI: COPI data: FD : Read Bank 0 Reg 0x7D Memory Read/Write
1423699-1423743 SPI: CIPO data: 00 : DMP firmware (16 bytes)
1423741-1423785 SPI: CIPO data: 05
1423783-1423827 SPI: CIPO data: 00
1423825-1423869 SPI: CIPO data: 00
1423867-1423911 SPI: CIPO data: 00
1423909-1423953 SPI: CIPO data: 05
1423951-1423995 SPI: CIPO data: 00
1423993-1424037 SPI: CIPO data: 00
1424035-1424079 SPI: CIPO data: 00
1424077-1424121 SPI: CIPO data: 05
1424119-1424163 SPI: CIPO data: 00
1424161-1424205 SPI: CIPO data: 01
1424203-1424247 SPI: CIPO data: 00
1424245-1424289 SPI: CIPO data: 05
1424287-1424331 SPI: CIPO data: 00
1424329-1424373 SPI: CIPO data: FF

...

2228746-2228790 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2228788-2228832 SPI: COPI data: 38 : Bank 0x38

2228874-2228917 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2228917-2228959 SPI: COPI data: 00 : Memory Address 0x00 : DMP

2229003-2229045 SPI: COPI data: FD : Read Bank 0 Reg 0x7D Memory Read/Write
2229045-2229087 SPI: CIPO data: DA : DMP firmware (16 bytes)
2229087-2229129 SPI: CIPO data: DE
2229129-2229171 SPI: CIPO data: F5
2229171-2229213 SPI: CIPO data: E2
2229213-2229255 SPI: CIPO data: F0
2229255-2229297 SPI: CIPO data: D9
2229297-2229339 SPI: CIPO data: F8
2229339-2229381 SPI: CIPO data: D8
2229381-2229423 SPI: CIPO data: A9
2229423-2229465 SPI: CIPO data: F3
2229465-2229507 SPI: CIPO data: F9
2229507-2229549 SPI: CIPO data: F9
2229549-2229591 SPI: CIPO data: F9
2229591-2229633 SPI: CIPO data: F9
2229633-2229675 SPI: CIPO data: F9
2229675-2229717 SPI: CIPO data: F9

...

2234280-2234322 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2234322-2234364 SPI: COPI data: 60 : Memory Address 0x60 : DMP

2234408-2234450 SPI: COPI data: FD : Read Bank 0 Reg 0x7D Memory Read/Write
2234450-2234492 SPI: CIPO data: F3 : DMP firmware (13 bytes)
2234492-2234534 SPI: CIPO data: CD
2234534-2234576 SPI: CIPO data: F2
2234576-2234618 SPI: CIPO data: CA
2234618-2234660 SPI: CIPO data: D0
2234660-2234702 SPI: CIPO data: F3
2234702-2234744 SPI: CIPO data: CB
2234744-2234786 SPI: CIPO data: F2
2234786-2234828 SPI: CIPO data: C8
2234828-2234870 SPI: CIPO data: D0
2234870-2234912 SPI: CIPO data: F3
2234912-2234954 SPI: CIPO data: C9
2234954-2234996 SPI: CIPO data: E0

<DMP has been verified>

2235055-2235097 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2235097-2235139 SPI: CIPO data: 00

2235183-2235225 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2235225-2235267 SPI: COPI data: 20 : Select Bank 2

2235311-2235355 SPI: COPI data: 50 : Write Bank 2 Reg 0x50 PRGM_START_ADDRH
2235353-2235397 SPI: COPI data: 10 : Magic Number 0x1000
2235395-2235439 SPI: COPI data: 00

2235487-2235531 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2235529-2235573 SPI: CIPO data: 20 : Bank 2 is selected

2235615-2235657 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2235657-2235699 SPI: COPI data: 00 : Select Bank 0

2235743-2235787 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2235785-2235829 SPI: COPI data: 00 : Bank 0x00

2235871-2235915 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2235913-2235957 SPI: COPI data: 40 : DATA_OUT_CTL1 (4 * 16 + 0)

2235999-2236041 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
2236041-2236083 SPI: COPI data: 00 : Set DATA_OUT_CTL1 to 0x0000
2236083-2236125 SPI: COPI data: 00

2236173-2236217 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2236215-2236259 SPI: COPI data: 42 : DATA_OUT_CTL2 (4 * 16 + 2)

2236301-2236343 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
2236343-2236385 SPI: COPI data: 00 : Set DATA_OUT_CTL2 to 0x0000
2236385-2236427 SPI: COPI data: 00

2236475-2236519 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2236517-2236561 SPI: COPI data: 4C : DATA_INTR_CTL (4 * 16 + 12)

2236603-2236645 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
2236645-2236687 SPI: COPI data: 00 : Set DATA_INTR_CTL to 0x0000
2236687-2236729 SPI: COPI data: 00

2236777-2236821 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2236819-2236863 SPI: COPI data: 4E : MOTION_EVENT_CTL (4 * 16 + 14)

2236905-2236947 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
2236947-2236989 SPI: COPI data: 00 : Set MOTION_EVENT_CTL to 0x0000
2236989-2237031 SPI: COPI data: 00

2237079-2237123 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2237121-2237165 SPI: COPI data: 8A : DATA_RDY_STATUS (8 * 16 + 10)

2237208-2237250 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
2237250-2237292 SPI: COPI data: 00 : Set DATA_RDY_STATUS to 0x0000
2237292-2237334 SPI: COPI data: 00

2237383-2237425 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2237425-2237467 SPI: COPI data: 01 : Bank 0x01

2237511-2237553 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2237553-2237595 SPI: COPI data: FE : FIFO_WATERMARK (31 * 16 + 14)

2237639-2237681 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
2237681-2237723 SPI: COPI data: 03 : Set FIFO_WATERMARK to 0x0320 = 800
2237723-2237765 SPI: COPI data: 20

2237814-2237856 SPI: COPI data: 10 : Write Bank 0 Reg 0x10 INT_ENABLE
2237856-2237898 SPI: COPI data: 02 : DMP_INT1_EN : Enable DMP interrupt on pin 1

2237947-2237991 SPI: COPI data: 12 : Write Bank 0 Reg 0x12 INT_ENABLE_2
2237989-2238033 SPI: COPI data: 01 : Enable interrupt for FIFO overflow for peripheral 0

2238082-2238124 SPI: COPI data: 26 : Write Bank 0 Reg 0x26 Single FIFO Priority Select
2238124-2238166 SPI: COPI data: E4 : ???

2238216-2238260 SPI: COPI data: F5 : Read Bank 0 Reg 0x75 HW Fix Disable
2238258-2238302 SPI: CIPO data: 40 : ICM_20948 returns 0x40

2238352-2238394 SPI: COPI data: 75 : Write Bank 0 Reg 0x75 HW Fix Disable
2238394-2238436 SPI: COPI data: 48 : ???

2238486-2238528 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2238528-2238570 SPI: CIPO data: 00 : Bank 0 is selected

2238614-2238656 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2238656-2238698 SPI: COPI data: 20 : Select Bank 2

2238742-2238786 SPI: COPI data: 00 : Write Bank 2 Reg 0x00 GYRO_SMPLRT_DIV
2238784-2238828 SPI: COPI data: 13 : Set GYRO_SMPLRT_DIV to 0x13 = 19

2238876-2238918 SPI: COPI data: 10 : Write Bank 2 Reg 0x10 ACCEL_SMPLRT_DIV_1
2238918-2238960 SPI: COPI data: 00 : Set ACCEL_SMPLRT_DIV_1 to 0x0013 = 19
2238960-2239002 SPI: COPI data: 13

2239052-2239096 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2239094-2239138 SPI: CIPO data: 20 : Bank 2 is selected

2239180-2239224 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2239222-2239266 SPI: COPI data: 00 : Select Bank 0

2239309-2239351 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2239351-2239393 SPI: COPI data: 03 : Bank 0x03

2239437-2239479 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2239479-2239521 SPI: COPI data: 0A : BAC_RATE (48 * 16 + 10)

2239564-2239608 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
2239606-2239650 SPI: COPI data: 00 : Set BAC_RATE to 0x0000
2239648-2239692 SPI: COPI data: 00

2239740-2239784 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2239782-2239826 SPI: COPI data: 08 : B2S_RATE (48 * 16 + 8)

2239868-2239910 SPI: COPI data: 7D : Write Bank 0 Reg 0x7D Memory Read/Write
2239910-2239952 SPI: COPI data: 00 : Set B2S_RATE to 0x0000
2239952-2239994 SPI: COPI data: 00

2240041-2240085 SPI: COPI data: 76 : Write Bank 0 Reg 0x76 FIFO_CFG
2240083-2240127 SPI: COPI data: 00 : Set FIFO_CFG to 0x00

2240173-2240215 SPI: COPI data: 68 : Write Bank 0 Reg 0x68 FIFO_RST
2240215-2240257 SPI: COPI data: 1F : Set FIFO_RST to 0x1F (Reset the FIFOs for all five peripherals)

2240305-2240349 SPI: COPI data: 68 : Write Bank 0 Reg 0x68 FIFO_RST
2240347-2240391 SPI: COPI data: 1E : Set FIFO_RST to 0x1E (Reset the FIFOs except peripheral 0)

2240438-2240480 SPI: COPI data: 66 : Write Bank 0 Reg 0x68 FIFO_EN_1
2240480-2240522 SPI: COPI data: 00 : Set FIFO_EN_1 to 0x00

2240570-2240614 SPI: COPI data: 67 : Write Bank 0 Reg 0x68 FIFO_EN_2
2240612-2240656 SPI: COPI data: 00 : Set FIFO_EN_2 to 0x00

2240702-2240746 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2240744-2240788 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2240834-2240878 SPI: COPI data: 07 : Write Bank 0 Reg 0x06 PWR_MGMT_2
2240876-2240920 SPI: COPI data: 7F : Set PWR_MGMT_2 : Disable pressure; accel (all axes); gyro (all axes)

2240966-2241008 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2241008-2241050 SPI: COPI data: 61 : Set PWR_MGMT_1 : Sleep mode; Turn on low power; Auto clock best available

<wait 175us>

2241319-2241363 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2241361-2241405 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

<wait 175us>

2241652-2241694 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2241694-2241736 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

<wait 175us>

2241981-2242025 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2242023-2242067 SPI: CIPO data: 00 : Bank 2 is selected

2242109-2242151 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2242151-2242193 SPI: COPI data: 30 : Select Bank 3

2242237-2242279 SPI: COPI data: 05 : Write Bank 3 Reg 0x05 I2C Peripheral 0 Control
2242279-2242321 SPI: COPI data: 00 : Set I2C_PERIPH0_CTRL to 0x00

2242370-2242412 SPI: COPI data: 09 : Write Bank 3 Reg 0x09 I2C Peripheral 1 Control
2242412-2242454 SPI: COPI data: 00 : Set I2C_PERIPH1_CTRL to 0x00

2242503-2242545 SPI: COPI data: 0D : Write Bank 3 Reg 0x0D I2C Peripheral 2 Control
2242545-2242587 SPI: COPI data: 00 : Set I2C_PERIPH2_CTRL to 0x00

2242636-2242680 SPI: COPI data: 11 : Write Bank 3 Reg 0x11 I2C Peripheral 3 Control
2242678-2242722 SPI: COPI data: 00 : Set I2C_PERIPH3_CTRL to 0x00

2242770-2242812 SPI: COPI data: 01 : Write Bank 3 Reg 0x01 I2C Master Control
2242812-2242854 SPI: COPI data: 10 : Set I2C_MST_CTRL to 0x10 : I2C_MST_P_NSR use stop between reads

2242903-2242945 SPI: COPI data: 00 : Write Bank 3 Reg 0x00 I2C Master ODR Config
2242945-2242987 SPI: COPI data: 04 : Set I2C_MST_ODR_CONFIG to 0x00 : ODR = 1.1kHz

2243042-2243086 SPI: COPI data: 03 : Write Bank 3 Reg 0x00 I2C Peripheral 0 Address
2243084-2243128 SPI: COPI data: 8C : Set I2C_PERIPH0_ADDR to 0x8C (Read 0x0C AK09916 Magnetometer)

2243177-2243221 SPI: COPI data: 04 : Write Bank 3 Reg 0x00 I2C Peripheral 0 Register
2243219-2243263 SPI: COPI data: 00 : Point to AK09916 Magnetometer register 0

2243312-2243354 SPI: COPI data: 05 : Write Bank 3 Reg 0x05 I2C Peripheral 0 Control
2243354-2243396 SPI: COPI data: 81 : Set I2C_PERIPH0_CTRL to 0x81: Enable reading from peripheral; one byte

2243447-2243489 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2243489-2243531 SPI: CIPO data: 30 : Bank 3 is selected

2243574-2243618 SPI: COPI data: 7F : Write Bank 3 Reg 0x7F REG_BANK_SEL
2243616-2243660 SPI: COPI data: 00 : Select Bank 0

2243702-2243746 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2243744-2243788 SPI: COPI data: 30 : Set USER_CTRL to 0x30 : Enable I2C Controller module; Reset I2C Peripheral module (SPI mode only)

<wait 60ms>

2364091-2364135 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2364133-2364177 SPI: COPI data: 10 : Set USER_CTRL to 0x10 : Reset I2C Peripheral module (SPI mode only)

2364226-2364268 SPI: COPI data: BB : Read Bank 0 Reg 0x3B PERIPH_SENS_DATA_00
2364268-2364310 SPI: CIPO data: 48 : Magnetometer returns 0x48 (Who I Am)

2364361-2364403 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2364403-2364445 SPI: COPI data: 00 : Bank 0 is selected

2364489-2364531 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2364531-2364573 SPI: COPI data: 30 : Select Bank 3

2364617-2364659 SPI: COPI data: 05 : Write Bank 3 Reg 0x05 I2C Peripheral 0 Control
2364659-2364701 SPI: COPI data: 00 : Set I2C_PERIPH0_CTRL to 0x00

2364753-2364797 SPI: COPI data: 07 : Write Bank 3 Reg 0x07 I2C Peripheral 1 Address
2364795-2364839 SPI: COPI data: 0C : Set I2C_PERIPH1_ADDR to 0x0C (Write 0x0C AK09916 Magnetometer)

2364888-2364930 SPI: COPI data: 08 : Write Bank 3 Reg 0x05 I2C Peripheral 1 Register
2364930-2364972 SPI: COPI data: 31 : Point to AK09916 Magnetometer register 0x31 (CNTL2)

2365022-2365064 SPI: COPI data: 0A : Write Bank 3 Reg 0x05 I2C Peripheral 1 Data Out
2365064-2365108 SPI: COPI data: 00

2365157-2365199 SPI: COPI data: 09 : Write Bank 3 Reg 0x05 I2C Peripheral 1 Control
2365199-2365241 SPI: COPI data: 81 : Enable reading from this peripheral; one byte

2365291-2365335 SPI: COPI data: FF : Read Bank 3 Reg 0x7F REG_BANK_SEL
2365333-2365377 SPI: CIPO data: 30 : Bank 3 is selected

2365419-2365461 SPI: COPI data: 7F : Write Bank 3 Reg 0x7F REG_BANK_SEL
2365461-2365503 SPI: COPI data: 00 : Select Bank 0

2365547-2365589 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2365589-2365631 SPI: COPI data: 30 : Set USER_CTRL to 0x30 : Enable I2C Controller module; Reset I2C Peripheral module (SPI mode only)

<wait 60ms>

2485926-2485968 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2485968-2486010 SPI: COPI data: 10 : Set USER_CTRL to 0x10 : Reset I2C Peripheral module (SPI mode only)

2486059-2486103 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2486101-2486145 SPI: COPI data: 00 : Bank 0 is selected

2486187-2486229 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2486229-2486271 SPI: COPI data: 30 : Select Bank 3

2486315-2486357 SPI: COPI data: 09 : Write Bank 3 Reg 0x05 I2C Peripheral 1 Control
2486357-2486399 SPI: COPI data: 00 : Set I2C_PERIPH1_CTRL to 0x00

2486449-2486491 SPI: COPI data: 05 : Write Bank 3 Reg 0x05 I2C Peripheral 0 Control
2486491-2486533 SPI: COPI data: 00 : Set I2C_PERIPH0_CTRL to 0x00

2486582-2486626 SPI: COPI data: 09 : Write Bank 3 Reg 0x05 I2C Peripheral 1 Control
2486624-2486668 SPI: COPI data: 00 : Set I2C_PERIPH1_CTRL to 0x00

<Compass mounting matrix configuration>

2486716-2486760 SPI: COPI data: FF : Read Bank 3 Reg 0x7F REG_BANK_SEL
2486758-2486802 SPI: CIPO data: 30 : Bank 3 is selected

2486844-2486888 SPI: COPI data: 7F : Write Bank 3 Reg 0x7F REG_BANK_SEL
2486886-2486930 SPI: COPI data: 00 : Select Bank 0

2486972-2487016 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2487014-2487058 SPI: COPI data: 10 : Set USER_CTRL to 0x10 : Reset I2C Peripheral module (SPI mode only)

2487144-2487188 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2487186-2487230 SPI: COPI data: 01 : Bank 0x01

2487273-2487315 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2487315-2487357 SPI: COPI data: 70 : CPASS_MTX_00 (23 * 16 + 0)

2487400-2487444 SPI: COPI data: 7D : Set Compass Mount Matrix 00 to 0x09999999
2487442-2487486 SPI: COPI data: 09
2487484-2487528 SPI: COPI data: 99
2487526-2487570 SPI: COPI data: 99
2487568-2487612 SPI: COPI data: 99

2487660-2487702 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2487702-2487744 SPI: COPI data: 74 : CPASS_MTX_01 (23 * 16 + 4)

2487788-2487830 SPI: COPI data: 7D : Set Compass Mount Matrix 01 to 0x00000000
2487830-2487872 SPI: COPI data: 00
2487872-2487914 SPI: COPI data: 00
2487914-2487956 SPI: COPI data: 00
2487956-2487998 SPI: COPI data: 00

2488048-2488090 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2488090-2488132 SPI: COPI data: 78 : CPASS_MTX_02 (23 * 16 + 8)

2488175-2488219 SPI: COPI data: 7D : Set Compass Mount Matrix 02 to 0x00000000
2488217-2488261 SPI: COPI data: 00
2488259-2488303 SPI: COPI data: 00
2488301-2488345 SPI: COPI data: 00
2488343-2488387 SPI: COPI data: 00

2488435-2488477 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2488477-2488519 SPI: COPI data: 7C : CPASS_MTX_10 (23 * 16 + 12)

2488563-2488605 SPI: COPI data: 7D : Set Compass Mount Matrix 10 to 0x00000000
2488605-2488647 SPI: COPI data: 00
2488647-2488689 SPI: COPI data: 00
2488689-2488731 SPI: COPI data: 00
2488731-2488773 SPI: COPI data: 00

2488822-2488866 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2488864-2488907 SPI: COPI data: 80 : CPASS_MTX_11 (24 * 16)

2488950-2488994 SPI: COPI data: 7D : Set Compass Mount Matrix 11 to 0xF6666667
2488992-2489036 SPI: COPI data: F6
2489034-2489078 SPI: COPI data: 66
2489076-2489120 SPI: COPI data: 66
2489118-2489162 SPI: COPI data: 67

2489210-2489254 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2489252-2489296 SPI: COPI data: 84 : CPASS_MTX_12 (24 * 16 + 4)

2489338-2489380 SPI: COPI data: 7D : Set Compass Mount Matrix 12 to 0x00000000
2489380-2489422 SPI: COPI data: 00
2489422-2489464 SPI: COPI data: 00
2489464-2489506 SPI: COPI data: 00
2489506-2489548 SPI: COPI data: 00

2489598-2489640 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2489640-2489682 SPI: COPI data: 88 : CPASS_MTX_20 (24 * 16 + 8)

2489725-2489769 SPI: COPI data: 7D : Set Compass Mount Matrix 20 to 0x00000000
2489767-2489811 SPI: COPI data: 00
2489809-2489853 SPI: COPI data: 00
2489851-2489895 SPI: COPI data: 00
2489893-2489937 SPI: COPI data: 00

2489985-2490029 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2490027-2490071 SPI: COPI data: 8C : CPASS_MTX_21 (24 * 16 + 12)

2490113-2490155 SPI: COPI data: 7D : Set Compass Mount Matrix 21 to 0x00000000
2490155-2490197 SPI: COPI data: 00
2490197-2490239 SPI: COPI data: 00
2490239-2490281 SPI: COPI data: 00
2490281-2490323 SPI: COPI data: 00

2490373-2490415 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2490415-2490457 SPI: COPI data: 90 : CPASS_MTX_22 (25 * 16)

2490500-2490544 SPI: COPI data: 7D : Set Compass Mount Matrix 22 to 0xF6666667
2490542-2490586 SPI: COPI data: F6
2490584-2490628 SPI: COPI data: 66
2490626-2490670 SPI: COPI data: 66
2490668-2490712 SPI: COPI data: 67

2490759-2490801 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2490801-2490843 SPI: COPI data: 21 : Set PWR_MGMT_1 : Auto clock best available

<wait 175us>

2491026-2491070 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2491068-2491112 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

<wait 175us>

<B2S mounting matrix configuration>

2491360-2491402 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2491402-2491444 SPI: COPI data: 0D : Bank 0x0D

2491488-2491530 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2491530-2491572 SPI: COPI data: 00 : B2S_MTX_00 (208 * 16)

2491616-2491658 SPI: COPI data: 7D : Set B2S Mount Matrix 00 to 0x40000000
2491658-2491700 SPI: COPI data: 40
2491700-2491742 SPI: COPI data: 00
2491742-2491784 SPI: COPI data: 00
2491784-2491826 SPI: COPI data: 00

2491871-2491915 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2491913-2491957 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2492005-2492047 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2492047-2492089 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

<wait 175us>

2492335-2492377 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2492377-2492419 SPI: COPI data: 04 : B2S_MTX_01 (208 * 16 + 4)

2492462-2492506 SPI: COPI data: 7D : Set B2S Mount Matrix 01 to 0x00000000
2492504-2492548 SPI: COPI data: 00
2492546-2492590 SPI: COPI data: 00
2492588-2492632 SPI: COPI data: 00
2492630-2492674 SPI: COPI data: 00

2492718-2492762 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2492760-2492804 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2492851-2492895 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2492893-2492937 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

2493182-2493224 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2493224-2493266 SPI: COPI data: 08 : B2S_MTX_02 (208 * 16 + 8)

2493310-2493352 SPI: COPI data: 7D : Set B2S Mount Matrix 01 to 0x00000000
2493352-2493394 SPI: COPI data: 00
2493394-2493436 SPI: COPI data: 00
2493436-2493478 SPI: COPI data: 00
2493478-2493520 SPI: COPI data: 00

2493565-2493609 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2493607-2493651 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2493698-2493742 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2493740-2493784 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

2494029-2494071 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2494071-2494113 SPI: COPI data: 0C : B2S_MTX_10 (208 * 16 + 12)

2494156-2494200 SPI: COPI data: 7D : Set B2S Mount Matrix 10 to 0x00000000
2494198-2494242 SPI: COPI data: 00
2494240-2494284 SPI: COPI data: 00
2494282-2494326 SPI: COPI data: 00
2494324-2494368 SPI: COPI data: 00

2494412-2494456 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2494454-2494498 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2494545-2494589 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2494587-2494631 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

2494876-2494920 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2494918-2494962 SPI: COPI data: 10 : B2S_MTX_11 (209 * 16)

2495004-2495046 SPI: COPI data: 7D : Set B2S Mount Matrix 11 to 0x40000000
2495046-2495088 SPI: COPI data: 40
2495088-2495130 SPI: COPI data: 00
2495130-2495172 SPI: COPI data: 00
2495172-2495214 SPI: COPI data: 00

2495260-2495304 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2495302-2495346 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2495393-2495437 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2495435-2495479 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

2495723-2495767 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2495765-2495809 SPI: COPI data: 14 : B2S_MTX_12 (209 * 16 + 4)

2495851-2495893 SPI: COPI data: 7D : Set B2S Mount Matrix 12 to 0x00000000
2495893-2495935 SPI: COPI data: 00
2495935-2495977 SPI: COPI data: 00
2495977-2496019 SPI: COPI data: 00
2496019-2496061 SPI: COPI data: 00

2496107-2496149 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2496149-2496191 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2496240-2496282 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2496282-2496324 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

2496570-2496614 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2496612-2496656 SPI: COPI data: 18 : B2S_MTX_20 (209 * 16 + 8)

2496698-2496740 SPI: COPI data: 7D : Set B2S Mount Matrix 20 to 0x00000000
2496740-2496782 SPI: COPI data: 00
2496782-2496824 SPI: COPI data: 00
2496824-2496866 SPI: COPI data: 00
2496866-2496908 SPI: COPI data: 00

2496954-2496996 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2496996-2497038 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2497087-2497129 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2497129-2497171 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

2497417-2497461 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2497459-2497503 SPI: COPI data: 1C : B2S_MTX_21 (209 * 16 + 12)

2497545-2497587 SPI: COPI data: 7D : Set B2S Mount Matrix 21 to 0x00000000
2497587-2497629 SPI: COPI data: 00
2497629-2497671 SPI: COPI data: 00
2497671-2497713 SPI: COPI data: 00
2497713-2497755 SPI: COPI data: 00

2497801-2497843 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2497843-2497885 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2497934-2497976 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2497976-2498018 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

2498264-2498308 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2498306-2498350 SPI: COPI data: 20 : B2S_MTX_22 (210 * 16)

2498392-2498434 SPI: COPI data: 7D : Set B2S Mount Matrix 22 to 0x40000000
2498434-2498476 SPI: COPI data: 40
2498476-2498518 SPI: COPI data: 00
2498518-2498560 SPI: COPI data: 00
2498560-2498602 SPI: COPI data: 00

<Repeat of B2S mounting matirx configuration>

2498648-2498690 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2498690-2498732 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2498917-2498959 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2498959-2499001 SPI: COPI data: 01 : Set PWR_MGMT_1 : Auto clock best available

2499251-2499295 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2499293-2499337 SPI: COPI data: 00 : B2S_MTX_00 (208 * 16)

2499379-2499421 SPI: COPI data: 7D : Set B2S Mount Matrix 00 to 0x40000000
2499421-2499463 SPI: COPI data: 40
2499463-2499505 SPI: COPI data: 00
2499505-2499547 SPI: COPI data: 00
2499547-2499589 SPI: COPI data: 00

2499635-2499677 SPI: COPI data: 06
2499677-2499719 SPI: COPI data: 21

2499768-2499810 SPI: COPI data: 06
2499810-2499852 SPI: COPI data: 01

2500098-2500140 SPI: COPI data: 7C
2500140-2500182 SPI: COPI data: 04

2500226-2500268 SPI: COPI data: 7D
2500268-2500310 SPI: COPI data: 00
2500310-2500352 SPI: COPI data: 00
2500352-2500394 SPI: COPI data: 00
2500394-2500436 SPI: COPI data: 00

2500482-2500524 SPI: COPI data: 06
2500524-2500566 SPI: COPI data: 21

2500615-2500657 SPI: COPI data: 06
2500657-2500699 SPI: COPI data: 01

2500946-2500988 SPI: COPI data: 7C
2500988-2501030 SPI: COPI data: 08

2501073-2501117 SPI: COPI data: 7D
2501115-2501159 SPI: COPI data: 00
2501157-2501201 SPI: COPI data: 00
2501199-2501243 SPI: COPI data: 00
2501241-2501285 SPI: COPI data: 00

2501330-2501372 SPI: COPI data: 06
2501372-2501414 SPI: COPI data: 21

2501463-2501505 SPI: COPI data: 06
2501505-2501547 SPI: COPI data: 01

2501793-2501835 SPI: COPI data: 7C
2501835-2501877 SPI: COPI data: 0C

2501920-2501964 SPI: COPI data: 7D
2501962-2502006 SPI: COPI data: 00
2502004-2502048 SPI: COPI data: 00
2502046-2502090 SPI: COPI data: 00
2502088-2502132 SPI: COPI data: 00

2502176-2502220 SPI: COPI data: 06
2502218-2502262 SPI: COPI data: 21

2502309-2502353 SPI: COPI data: 06
2502351-2502395 SPI: COPI data: 01

2502640-2502682 SPI: COPI data: 7C
2502682-2502724 SPI: COPI data: 10

2502767-2502811 SPI: COPI data: 7D
2502809-2502853 SPI: COPI data: 40
2502851-2502895 SPI: COPI data: 00
2502893-2502937 SPI: COPI data: 00
2502935-2502979 SPI: COPI data: 00

2503023-2503065 SPI: COPI data: 06
2503065-2503107 SPI: COPI data: 21

2503156-2503198 SPI: COPI data: 06
2503198-2503240 SPI: COPI data: 01

2503487-2503529 SPI: COPI data: 7C
2503529-2503571 SPI: COPI data: 14

2503614-2503658 SPI: COPI data: 7D
2503656-2503700 SPI: COPI data: 00
2503698-2503742 SPI: COPI data: 00
2503740-2503784 SPI: COPI data: 00
2503782-2503826 SPI: COPI data: 00

2503870-2503914 SPI: COPI data: 06
2503912-2503956 SPI: COPI data: 21

2504003-2504047 SPI: COPI data: 06
2504045-2504089 SPI: COPI data: 01

2504333-2504376 SPI: COPI data: 7C
2504376-2504418 SPI: COPI data: 18

2504461-2504505 SPI: COPI data: 7D
2504503-2504547 SPI: COPI data: 00
2504545-2504589 SPI: COPI data: 00
2504587-2504631 SPI: COPI data: 00
2504629-2504673 SPI: COPI data: 00

2504717-2504759 SPI: COPI data: 06
2504759-2504801 SPI: COPI data: 21

2504850-2504894 SPI: COPI data: 06
2504892-2504936 SPI: COPI data: 01

2505181-2505223 SPI: COPI data: 7C
2505223-2505265 SPI: COPI data: 1C

2505308-2505352 SPI: COPI data: 7D
2505350-2505394 SPI: COPI data: 00
2505392-2505436 SPI: COPI data: 00
2505434-2505478 SPI: COPI data: 00
2505476-2505520 SPI: COPI data: 00

2505564-2505608 SPI: COPI data: 06
2505606-2505650 SPI: COPI data: 21

2505697-2505741 SPI: COPI data: 06
2505739-2505783 SPI: COPI data: 01

2506027-2506071 SPI: COPI data: 7C
2506069-2506113 SPI: COPI data: 20

2506155-2506197 SPI: COPI data: 7D
2506197-2506239 SPI: COPI data: 40
2506239-2506281 SPI: COPI data: 00
2506281-2506323 SPI: COPI data: 00
2506323-2506365 SPI: COPI data: 00

<Repeat (2) of B2S mounting matirx configuration>

2506411-2506453 SPI: COPI data: 06
2506453-2506495 SPI: COPI data: 21

2506680-2506722 SPI: COPI data: 06
2506722-2506764 SPI: COPI data: 01

2507011-2507053 SPI: COPI data: 7C
2507053-2507095 SPI: COPI data: 00

2507138-2507182 SPI: COPI data: 7D
2507180-2507224 SPI: COPI data: 40
2507222-2507266 SPI: COPI data: 00
2507264-2507308 SPI: COPI data: 00
2507306-2507350 SPI: COPI data: 00

2507394-2507438 SPI: COPI data: 06
2507436-2507479 SPI: COPI data: 21

2507528-2507570 SPI: COPI data: 06
2507570-2507612 SPI: COPI data: 01

2507858-2507900 SPI: COPI data: 7C
2507900-2507942 SPI: COPI data: 04

2507985-2508029 SPI: COPI data: 7D
2508027-2508071 SPI: COPI data: 00
2508069-2508113 SPI: COPI data: 00
2508111-2508155 SPI: COPI data: 00
2508153-2508197 SPI: COPI data: 00

2508241-2508285 SPI: COPI data: 06
2508283-2508327 SPI: COPI data: 21

2508374-2508418 SPI: COPI data: 06
2508416-2508460 SPI: COPI data: 01

2508705-2508747 SPI: COPI data: 7C
2508747-2508789 SPI: COPI data: 08

2508832-2508876 SPI: COPI data: 7D
2508874-2508918 SPI: COPI data: 00
2508916-2508960 SPI: COPI data: 00
2508958-2509002 SPI: COPI data: 00
2509000-2509044 SPI: COPI data: 00

2509088-2509132 SPI: COPI data: 06
2509130-2509174 SPI: COPI data: 21

2509222-2509264 SPI: COPI data: 06
2509264-2509306 SPI: COPI data: 01

2509552-2509594 SPI: COPI data: 7C
2509594-2509636 SPI: COPI data: 0C

2509680-2509722 SPI: COPI data: 7D
2509722-2509764 SPI: COPI data: 00
2509764-2509806 SPI: COPI data: 00
2509806-2509848 SPI: COPI data: 00
2509848-2509890 SPI: COPI data: 00

2509935-2509979 SPI: COPI data: 06
2509977-2510021 SPI: COPI data: 21

2510068-2510112 SPI: COPI data: 06
2510110-2510154 SPI: COPI data: 01

2510399-2510441 SPI: COPI data: 7C
2510441-2510483 SPI: COPI data: 10

2510526-2510570 SPI: COPI data: 7D
2510568-2510612 SPI: COPI data: 40
2510610-2510654 SPI: COPI data: 00
2510652-2510696 SPI: COPI data: 00
2510694-2510738 SPI: COPI data: 00

2510782-2510826 SPI: COPI data: 06
2510824-2510868 SPI: COPI data: 21

2510915-2510957 SPI: COPI data: 06
2510957-2510999 SPI: COPI data: 01

2511249-2511291 SPI: COPI data: 7C
2511291-2511333 SPI: COPI data: 14

2511377-2511419 SPI: COPI data: 7D
2511419-2511461 SPI: COPI data: 00
2511461-2511503 SPI: COPI data: 00
2511503-2511545 SPI: COPI data: 00
2511545-2511587 SPI: COPI data: 00

2511633-2511675 SPI: COPI data: 06
2511675-2511717 SPI: COPI data: 21

2511766-2511808 SPI: COPI data: 06
2511808-2511850 SPI: COPI data: 01

2512096-2512138 SPI: COPI data: 7C
2512138-2512180 SPI: COPI data: 18

2512224-2512266 SPI: COPI data: 7D
2512266-2512308 SPI: COPI data: 00
2512308-2512350 SPI: COPI data: 00
2512350-2512392 SPI: COPI data: 00
2512392-2512434 SPI: COPI data: 00

2512479-2512523 SPI: COPI data: 06
2512521-2512565 SPI: COPI data: 21

2512613-2512655 SPI: COPI data: 06
2512655-2512697 SPI: COPI data: 01

2512944-2512986 SPI: COPI data: 7C
2512986-2513028 SPI: COPI data: 1C

2513071-2513115 SPI: COPI data: 7D
2513113-2513157 SPI: COPI data: 00
2513155-2513199 SPI: COPI data: 00
2513197-2513241 SPI: COPI data: 00
2513239-2513283 SPI: COPI data: 00

2513327-2513371 SPI: COPI data: 06
2513369-2513413 SPI: COPI data: 21

2513460-2513504 SPI: COPI data: 06
2513502-2513546 SPI: COPI data: 01

2513791-2513833 SPI: COPI data: 7C
2513833-2513875 SPI: COPI data: 20

2513918-2513962 SPI: COPI data: 7D
2513960-2514004 SPI: COPI data: 40
2514002-2514046 SPI: COPI data: 00
2514044-2514088 SPI: COPI data: 00
2514086-2514130 SPI: COPI data: 00

<Repeat (3) of B2S mounting matirx configuration>

2514174-2514218 SPI: COPI data: 06
2514216-2514260 SPI: COPI data: 21

2514443-2514485 SPI: COPI data: 06
2514485-2514527 SPI: COPI data: 01

2514773-2514815 SPI: COPI data: 7C
2514815-2514857 SPI: COPI data: 00

2514901-2514943 SPI: COPI data: 7D
2514943-2514985 SPI: COPI data: 40
2514985-2515027 SPI: COPI data: 00
2515027-2515069 SPI: COPI data: 00
2515069-2515111 SPI: COPI data: 00

2515156-2515200 SPI: COPI data: 06
2515198-2515242 SPI: COPI data: 21

2515290-2515332 SPI: COPI data: 06
2515332-2515374 SPI: COPI data: 01

2515620-2515662 SPI: COPI data: 7C
2515662-2515704 SPI: COPI data: 04

2515748-2515790 SPI: COPI data: 7D
2515790-2515832 SPI: COPI data: 00
2515832-2515874 SPI: COPI data: 00
2515874-2515916 SPI: COPI data: 00
2515916-2515958 SPI: COPI data: 00

2516004-2516046 SPI: COPI data: 06
2516046-2516088 SPI: COPI data: 21

2516137-2516179 SPI: COPI data: 06
2516179-2516221 SPI: COPI data: 01

2516467-2516509 SPI: COPI data: 7C
2516509-2516551 SPI: COPI data: 08

2516594-2516638 SPI: COPI data: 7D
2516636-2516680 SPI: COPI data: 00
2516678-2516722 SPI: COPI data: 00
2516720-2516763 SPI: COPI data: 00
2516763-2516805 SPI: COPI data: 00

2516851-2516893 SPI: COPI data: 06
2516893-2516935 SPI: COPI data: 21

2516984-2517026 SPI: COPI data: 06
2517026-2517068 SPI: COPI data: 01

2517318-2517360 SPI: COPI data: 7C
2517360-2517402 SPI: COPI data: 0C

2517445-2517489 SPI: COPI data: 7D
2517487-2517531 SPI: COPI data: 00
2517529-2517573 SPI: COPI data: 00
2517571-2517615 SPI: COPI data: 00
2517613-2517657 SPI: COPI data: 00

2517701-2517743 SPI: COPI data: 06
2517743-2517787 SPI: COPI data: 21

2517834-2517878 SPI: COPI data: 06
2517876-2517920 SPI: COPI data: 01

2518164-2518208 SPI: COPI data: 7C
2518206-2518250 SPI: COPI data: 10

2518292-2518334 SPI: COPI data: 7D
2518334-2518376 SPI: COPI data: 40
2518376-2518418 SPI: COPI data: 00
2518418-2518460 SPI: COPI data: 00
2518460-2518502 SPI: COPI data: 00

2518548-2518590 SPI: COPI data: 06
2518590-2518632 SPI: COPI data: 21

2518681-2518723 SPI: COPI data: 06
2518723-2518765 SPI: COPI data: 01

2519012-2519054 SPI: COPI data: 7C
2519054-2519096 SPI: COPI data: 14

2519140-2519182 SPI: COPI data: 7D
2519182-2519224 SPI: COPI data: 00
2519224-2519266 SPI: COPI data: 00
2519266-2519308 SPI: COPI data: 00
2519308-2519350 SPI: COPI data: 00

2519396-2519438 SPI: COPI data: 06
2519438-2519480 SPI: COPI data: 21

2519529-2519571 SPI: COPI data: 06
2519571-2519613 SPI: COPI data: 01

2519859-2519903 SPI: COPI data: 7C
2519901-2519945 SPI: COPI data: 18

2519987-2520029 SPI: COPI data: 7D
2520029-2520071 SPI: COPI data: 00
2520071-2520113 SPI: COPI data: 00
2520113-2520155 SPI: COPI data: 00
2520155-2520197 SPI: COPI data: 00

2520243-2520285 SPI: COPI data: 06
2520285-2520327 SPI: COPI data: 21

2520376-2520418 SPI: COPI data: 06
2520418-2520460 SPI: COPI data: 01

2520706-2520748 SPI: COPI data: 7C
2520748-2520790 SPI: COPI data: 1C

2520834-2520876 SPI: COPI data: 7D
2520876-2520918 SPI: COPI data: 00
2520918-2520960 SPI: COPI data: 00
2520960-2521002 SPI: COPI data: 00
2521002-2521044 SPI: COPI data: 00

2521090-2521132 SPI: COPI data: 06
2521132-2521174 SPI: COPI data: 21

2521223-2521265 SPI: COPI data: 06
2521265-2521307 SPI: COPI data: 01

2521553-2521597 SPI: COPI data: 7C
2521595-2521639 SPI: COPI data: 20

2521681-2521723 SPI: COPI data: 7D
2521723-2521765 SPI: COPI data: 40
2521765-2521807 SPI: COPI data: 00
2521807-2521849 SPI: COPI data: 00
2521849-2521891 SPI: COPI data: 00

<Repeat (4) of B2S mounting matirx configuration>

2521937-2521979 SPI: COPI data: 06
2521979-2522021 SPI: COPI data: 21

2522340-2522382 SPI: COPI data: 06
2522382-2522424 SPI: COPI data: 01

2522670-2522712 SPI: COPI data: 7C
2522712-2522754 SPI: COPI data: 00

2522798-2522840 SPI: COPI data: 7D
2522840-2522882 SPI: COPI data: 40
2522882-2522924 SPI: COPI data: 00
2522924-2522966 SPI: COPI data: 00
2522966-2523008 SPI: COPI data: 00

2523054-2523096 SPI: COPI data: 06
2523096-2523138 SPI: COPI data: 21

2523187-2523229 SPI: COPI data: 06
2523229-2523271 SPI: COPI data: 01

2523517-2523559 SPI: COPI data: 7C
2523559-2523601 SPI: COPI data: 04

2523645-2523687 SPI: COPI data: 7D
2523687-2523729 SPI: COPI data: 00
2523729-2523771 SPI: COPI data: 00
2523771-2523813 SPI: COPI data: 00
2523813-2523855 SPI: COPI data: 00

2523900-2523944 SPI: COPI data: 06
2523942-2523985 SPI: COPI data: 21

2524034-2524076 SPI: COPI data: 06
2524076-2524118 SPI: COPI data: 01

2524364-2524406 SPI: COPI data: 7C
2524406-2524448 SPI: COPI data: 08

2524491-2524535 SPI: COPI data: 7D
2524533-2524577 SPI: COPI data: 00
2524575-2524619 SPI: COPI data: 00
2524617-2524661 SPI: COPI data: 00
2524659-2524703 SPI: COPI data: 00

2524747-2524791 SPI: COPI data: 06
2524789-2524833 SPI: COPI data: 21

2524880-2524922 SPI: COPI data: 06
2524922-2524964 SPI: COPI data: 01

2525214-2525256 SPI: COPI data: 7C
2525256-2525298 SPI: COPI data: 0C

2525342-2525384 SPI: COPI data: 7D
2525384-2525426 SPI: COPI data: 00
2525426-2525468 SPI: COPI data: 00
2525468-2525510 SPI: COPI data: 00
2525510-2525552 SPI: COPI data: 00

2525598-2525640 SPI: COPI data: 06
2525640-2525682 SPI: COPI data: 21

2525731-2525773 SPI: COPI data: 06
2525773-2525815 SPI: COPI data: 01

2526061-2526103 SPI: COPI data: 7C
2526103-2526145 SPI: COPI data: 10

2526189-2526231 SPI: COPI data: 7D
2526231-2526273 SPI: COPI data: 40
2526273-2526315 SPI: COPI data: 00
2526315-2526357 SPI: COPI data: 00
2526357-2526399 SPI: COPI data: 00

2526444-2526488 SPI: COPI data: 06
2526444-2526488 SPI: CIPO data: 00
2526486-2526530 SPI: COPI data: 21
2526486-2526530 SPI: CIPO data: 00

2526577-2526621 SPI: COPI data: 06
2526577-2526621 SPI: CIPO data: 00
2526619-2526663 SPI: COPI data: 01
2526619-2526663 SPI: CIPO data: 00

2526908-2526952 SPI: COPI data: 7C
2526950-2526994 SPI: COPI data: 14

2527036-2527080 SPI: COPI data: 7D
2527078-2527122 SPI: COPI data: 00
2527120-2527164 SPI: COPI data: 00
2527164-2527206 SPI: COPI data: 00
2527206-2527248 SPI: COPI data: 00

2527294-2527336 SPI: COPI data: 06
2527336-2527378 SPI: COPI data: 21

2527427-2527469 SPI: COPI data: 06
2527469-2527511 SPI: COPI data: 01

2527757-2527799 SPI: COPI data: 7C
2527799-2527841 SPI: COPI data: 18

2527885-2527927 SPI: COPI data: 7D
2527927-2527969 SPI: COPI data: 00
2527969-2528011 SPI: COPI data: 00
2528011-2528053 SPI: COPI data: 00
2528053-2528095 SPI: COPI data: 00

2528141-2528183 SPI: COPI data: 06
2528183-2528225 SPI: COPI data: 21

2528274-2528316 SPI: COPI data: 06
2528316-2528358 SPI: COPI data: 01

2528604-2528646 SPI: COPI data: 7C
2528646-2528688 SPI: COPI data: 1C

2528732-2528774 SPI: COPI data: 7D
2528774-2528816 SPI: COPI data: 00
2528816-2528858 SPI: COPI data: 00
2528858-2528900 SPI: COPI data: 00
2528900-2528942 SPI: COPI data: 00

2528987-2529031 SPI: COPI data: 06
2529029-2529073 SPI: COPI data: 21

2529120-2529164 SPI: COPI data: 06
2529164-2529206 SPI: COPI data: 01

2529453-2529495 SPI: COPI data: 7C
2529495-2529537 SPI: COPI data: 20

2529580-2529624 SPI: COPI data: 7D
2529622-2529666 SPI: COPI data: 40
2529664-2529708 SPI: COPI data: 00
2529706-2529750 SPI: COPI data: 00
2529748-2529792 SPI: COPI data: 00

2529836-2529878 SPI: COPI data: 06
2529878-2529920 SPI: COPI data: 21

<wait 1ms>

2531860-2531902 SPI: COPI data: 06
2531902-2531944 SPI: COPI data: 01

2532190-2532232 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2532232-2532274 SPI: COPI data: 00 : Bank 0 is selected

2532317-2532361 SPI: COPI data: 7F : Write Bank 3 Reg 0x7F REG_BANK_SEL
2532359-2532403 SPI: COPI data: 20 : Select Bank 2

2532446-2532488 SPI: COPI data: 94 : Read Bank 2 Reg 0x14 ACCEL_CONFIG
2532488-2532530 SPI: CIPO data: 01 : Accel DLPF is enabled; +/-2g; 246Hz (3dB); 1125Hz

2532578-2532620 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2532620-2532662 SPI: CIPO data: 20 : Bank 2 is selected

2532706-2532748 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2532748-2532790 SPI: COPI data: 00 : Select Bank 0

2532834-2532876 SPI: COPI data: 06
2532876-2532918 SPI: COPI data: 21

2532967-2533009 SPI: COPI data: 06
2533009-2533051 SPI: COPI data: 01

<wait 125us>

2533301-2533343 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2533343-2533385 SPI: CIPO data: 00 : Bank 0 is selected

2533428-2533472 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2533470-2533514 SPI: COPI data: 20 : Select Bank 2

2533556-2533598 SPI: COPI data: 14 : Write Bank 2 Reg 0x14 ACCEL_CONFIG
2533598-2533640 SPI: COPI data: 02 : ACCEL_FCHOICE 0 (Bypass Accel DLPF); +/- 4g; 4500Hz

2533687-2533729 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2533729-2533771 SPI: CIPO data: 20 : Bank 2 is selected

2533814-2533858 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2533856-2533900 SPI: COPI data: 00 : Select Bank 0

2533942-2533986 SPI: COPI data: 06
2533984-2534028 SPI: COPI data: 21

2534075-2534119 SPI: COPI data: 06
2534117-2534161 SPI: COPI data: 01

2534405-2534449 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2534447-2534491 SPI: CIPO data: 00 : Bank 0 is selected

2534533-2534575 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2534575-2534617 SPI: COPI data: 20 : Select Bank 2

2534662-2534704 SPI: COPI data: 95 : Read Bank 2 Reg 0x15 ACCEL_CONFIG_2
2534704-2534746 SPI: CIPO data: 00 : DEC3_CFG: 0 (Averages: 1)

2534794-2534836 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2534836-2534878 SPI: CIPO data: 20 : Bank 2 is selected

2534921-2534965 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2534963-2535007 SPI: COPI data: 00 : Select Bank 0

2535049-2535091 SPI: COPI data: 06
2535091-2535133 SPI: COPI data: 21

2535182-2535226 SPI: COPI data: 06
2535224-2535268 SPI: COPI data: 01

2535512-2535556 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2535554-2535598 SPI: CIPO data: 00 : Bank 0 is selected

2535640-2535682 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2535682-2535724 SPI: COPI data: 20 : Select Bank 2

2535768-2535810 SPI: COPI data: 15 : Write Bank 2 Reg 0x15 ACCEL_CONFIG_2
2535810-2535852 SPI: COPI data: 00 : DEC3_CFG: 0 (Averages: 1)

2535899-2535941 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2535941-2535983 SPI: CIPO data: 20 : Bank 2 is selected

2536026-2536070 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2536068-2536112 SPI: COPI data: 00 : Select Bank 0

2536154-2536196 SPI: COPI data: 06
2536196-2536238 SPI: COPI data: 21

2536288-2536332 SPI: COPI data: 06
2536331-2536373 SPI: COPI data: 01

2536618-2536662 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2536660-2536704 SPI: COPI data: 01 : Bank 0x01

2536746-2536790 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2536788-2536832 SPI: COPI data: E0 : ACC_SCALE (30 * 16)

2536874-2536916 SPI: COPI data: 7D : Set ACC_SCALE to 0x04000000
2536916-2536958 SPI: COPI data: 04
2536958-2537000 SPI: COPI data: 00
2537000-2537042 SPI: COPI data: 00
2537042-2537084 SPI: COPI data: 00

2537130-2537172 SPI: COPI data: 06
2537172-2537214 SPI: COPI data: 21

2537264-2537308 SPI: COPI data: 06
2537306-2537350 SPI: COPI data: 01

2537594-2537638 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2537636-2537680 SPI: COPI data: 04 : Bank 0x04

2537722-2537766 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2537764-2537808 SPI: COPI data: F4 : ACC_SCALE2 (79 * 16 + 4)

2537850-2537892 SPI: COPI data: 7D : Set ACC_SCALE2 to 0x00040000
2537892-2537934 SPI: COPI data: 00
2537934-2537976 SPI: COPI data: 04
2537976-2538018 SPI: COPI data: 00
2538018-2538060 SPI: COPI data: 00

2538106-2538148 SPI: COPI data: 06
2538148-2538190 SPI: COPI data: 21

2538241-2538285 SPI: COPI data: 06
2538283-2538327 SPI: COPI data: 01

2538571-2538615 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2538613-2538657 SPI: CIPO data: 00 : Bank 0 is selected

2538699-2538741 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2538741-2538783 SPI: COPI data: 20 : Select Bank 2

2538828-2538872 SPI: COPI data: 94 : Read Bank 2 Reg 0x14 ACCEL_CONFIG
2538870-2538914 SPI: CIPO data: 02 : ACCEL_FCHOICE 0 (Bypass Accel DLPF); +/- 4g; 4500Hz

2538960-2539004 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2539002-2539046 SPI: CIPO data: 20 : Bank 2 is selected

2539088-2539130 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2539130-2539172 SPI: COPI data: 00 : Select Bank 0

2539216-2539258 SPI: COPI data: 06
2539258-2539300 SPI: COPI data: 21

2539349-2539393 SPI: COPI data: 06
2539391-2539435 SPI: COPI data: 01

2539679-2539723 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2539721-2539765 SPI: CIPO data: 00 : Bank 0 is selected

2539807-2539849 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2539849-2539891 SPI: COPI data: 20 : Select Bank 2

2539935-2539977 SPI: COPI data: 14 : Write Bank 2 Reg 0x14 ACCEL_CONFIG
2539977-2540019 SPI: COPI data: 02 : ACCEL_FCHOICE 0 (Bypass Accel DLPF); +/- 4g; 4500Hz

2540066-2540108 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2540108-2540150 SPI: CIPO data: 20 : Bank 2 is selected

2540193-2540237 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2540235-2540279 SPI: COPI data: 00 : Select Bank 0

2540321-2540363 SPI: COPI data: 06
2540363-2540405 SPI: COPI data: 21

2540454-2540498 SPI: COPI data: 06
2540496-2540540 SPI: COPI data: 01

2540784-2540828 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2540826-2540870 SPI: CIPO data: 00 : Bank 0 is selected

2540912-2540954 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2540954-2540996 SPI: COPI data: 20 : Select Bank 2

2541040-2541084 SPI: COPI data: 95 : Read Bank 2 Reg 0x15 ACCEL_CONFIG_2
2541082-2541126 SPI: CIPO data: 00 : DEC3_CFG: 0 (Averages: 1)

2541172-2541216 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2541214-2541258 SPI: CIPO data: 20 : Bank 2 is selected

2541300-2541342 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2541342-2541384 SPI: CIPO data: 00 : Select Bank 0

2541428-2541470 SPI: COPI data: 06
2541470-2541512 SPI: COPI data: 21

2541561-2541605 SPI: COPI data: 06
2541603-2541647 SPI: COPI data: 01

2541891-2541933 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2541933-2541975 SPI: CIPO data: 00 : Bank 0 is selected

2542019-2542061 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2542061-2542103 SPI: COPI data: 20 : Select Bank 2

2542147-2542189 SPI: COPI data: 15 : Write Bank 2 Reg 0x15 ACCEL_CONFIG_2
2542189-2542231 SPI: COPI data: 00 : DEC3_CFG: 0 (Averages: 1)

2542277-2542321 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2542319-2542363 SPI: CIPO data: 20 : Bank 2 is selected

2542405-2542447 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2542447-2542489 SPI: CIPO data: 00 : Select Bank 0

2542533-2542575 SPI: COPI data: 06
2542575-2542617 SPI: COPI data: 21

2542667-2542711 SPI: COPI data: 06
2542709-2542753 SPI: COPI data: 01

2542998-2543040 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2543040-2543082 SPI: COPI data: 01 : Bank 0x01

2543126-2543168 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2543168-2543210 SPI: COPI data: E0 : ACC_SCALE (30 * 16)

2543254-2543296 SPI: COPI data: 7D : Set ACC_SCALE to 0x04000000
2543296-2543338 SPI: COPI data: 04
2543338-2543380 SPI: COPI data: 00
2543380-2543422 SPI: COPI data: 00
2543422-2543464 SPI: COPI data: 00

2543510-2543552 SPI: COPI data: 06
2543552-2543594 SPI: COPI data: 21

2543644-2543688 SPI: COPI data: 06
2543686-2543730 SPI: COPI data: 01

2543974-2544016 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2544016-2544058 SPI: COPI data: 04 : Bank 0x04

2544102-2544144 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2544144-2544186 SPI: COPI data: F4 : ACC_SCALE2 (79 * 16 + 4)

2544230-2544272 SPI: COPI data: 7D : Set ACC_SCALE2 to 0x00040000
2544272-2544314 SPI: COPI data: 00
2544314-2544356 SPI: COPI data: 04
2544356-2544398 SPI: COPI data: 00
2544398-2544440 SPI: COPI data: 00

2544486-2544528 SPI: COPI data: 06
2544528-2544570 SPI: COPI data: 21

2544621-2544665 SPI: COPI data: 06
2544663-2544707 SPI: COPI data: 01

2544952-2544994 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2544994-2545036 SPI: CIPO data: 00 : Bank 0 is selected

2545080-2545122 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2545122-2545164 SPI: COPI data: 20 : Select Bank 2

2545212-2545254 SPI: COPI data: 81 : Read Bank 2 Reg 0x01 GYRO_CONFIG_1
2545254-2545296 SPI: CIPO data: 01 : Enable gyro DLPF; +/-250dps; 9000 Hz (i.e. the reset value)

2545344-2545386 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2545386-2545428 SPI: CIPO data: 20 : Bank 2 is selected

2545472-2545514 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2545514-2545556 SPI: COPI data: 00 : Select Bank 0

2545600-2545642 SPI: COPI data: 06
2545642-2545684 SPI: COPI data: 21

2545733-2545777 SPI: COPI data: 06
2545775-2545819 SPI: COPI data: 01

2546063-2546105 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2546105-2546147 SPI: CIPO data: 00 : Bank 0 is selected

2546191-2546233 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2546233-2546275 SPI: COPI data: 20 : Select Bank 2

2546319-2546363 SPI: COPI data: 01 : Write Bank 2 Reg 0x01 GYRO_CONFIG_1
2546361-2546405 SPI: COPI data: 07 : Enable gyro DLPF; +/- 2000dps

2546450-2546494 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2546492-2546536 SPI: CIPO data: 20 : Bank 2 is selected

2546578-2546620 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2546620-2546662 SPI: COPI data: 00 : Select Bank 0

2546706-2546748 SPI: COPI data: 06
2546748-2546790 SPI: COPI data: 21

2546839-2546883 SPI: COPI data: 06
2546881-2546925 SPI: COPI data: 01

2547169-2547211 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2547211-2547253 SPI: CIPO data: 00 : Bank 0 is selected

2547297-2547339 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2547339-2547381 SPI: COPI data: 20 : Select Bank 2

2547425-2547469 SPI: COPI data: 82 : Read Bank 2 Reg 0x02 GYRO_CONFIG_2
2547467-2547511 SPI: CIPO data: 00 : 1x averaging (i.e. reset value)

2547557-2547601 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2547599-2547643 SPI: CIPO data: 20 : Bank 2 is selected

2547685-2547729 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2547727-2547771 SPI: COPI data: 00 : Select Bank 0

2547813-2547855 SPI: COPI data: 06
2547855-2547897 SPI: COPI data: 21

2547947-2547989 SPI: COPI data: 06
2547989-2548031 SPI: COPI data: 01

2548277-2548319 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2548319-2548361 SPI: CIPO data: 00 : Bank 0 is selected

2548405-2548447 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2548447-2548489 SPI: COPI data: 20 : Select Bank 2

2548532-2548576 SPI: COPI data: 02 : Write Bank 2 Reg 0x02 GYRO_CONFIG_2
2548574-2548618 SPI: COPI data: 00 : 1x Averaging

2548663-2548705 SPI: COPI data: FF : Read Bank 2 Reg 0x7F REG_BANK_SEL
2548705-2548749 SPI: CIPO data: 20 : Bank 2 is selected

2548791-2548833 SPI: COPI data: 7F : Write Bank 2 Reg 0x7F REG_BANK_SEL
2548833-2548875 SPI: COPI data: 00 : Select Bank 0

2548919-2548961 SPI: COPI data: 06
2548961-2549003 SPI: COPI data: 21

2549053-2549095 SPI: COPI data: 06
2549095-2549137 SPI: COPI data: 01

2549386-2549430 SPI: COPI data: FF : Read Bank 0 Reg 0x7F REG_BANK_SEL
2549428-2549472 SPI: CIPO data: 00 : Bank 0 is selected

2549514-2549556 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2549556-2549598 SPI: COPI data: 10 : Select Bank 1

2549643-2549685 SPI: COPI data: A8 : Read Bank 1 Reg 0x28 TIMEBASE_CORRECTION_PLL
2549685-2549727 SPI: CIPO data: 09 : +9% ? (or maybe +9/127 %)

2549775-2549817 SPI: COPI data: FF : Read Bank 1 Reg 0x7F REG_BANK_SEL
2549817-2549859 SPI: CIPO data: 10 : Bank 1 is selected

2549902-2549946 SPI: COPI data: 7F : Write Bank 1 Reg 0x7F REG_BANK_SEL
2549944-2549988 SPI: COPI data: 00 : Select Bank 0

2550030-2550074 SPI: COPI data: 06
2550072-2550116 SPI: COPI data: 21

2550174-2550216 SPI: COPI data: 06
2550216-2550258 SPI: COPI data: 01

2550504-2550546 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2550546-2550588 SPI: COPI data: 01 : Bank 0x01

2550632-2550674 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2550674-2550716 SPI: COPI data: 30 : GYRO_SF (19 * 16)

2550760-2550802 SPI: COPI data: 7D : Set GYRO_SF to 0x276FBC37
2550802-2550844 SPI: COPI data: 27
2550844-2550886 SPI: COPI data: 6F
2550886-2550928 SPI: COPI data: BC
2550928-2550970 SPI: COPI data: 37

2551015-2551059 SPI: COPI data: 06
2551057-2551101 SPI: COPI data: 21

2551150-2551194 SPI: COPI data: 06
2551192-2551236 SPI: COPI data: 01

2551480-2551524 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2551522-2551566 SPI: COPI data: 04 : Bank 0x04

2551608-2551652 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2551650-2551694 SPI: COPI data: 8C : GYRO_FULLSCALE (72 * 16 + 12)

2551736-2551778 SPI: COPI data: 7D : Set GYRO_FULLSCALE to 0x10000000
2551778-2551822 SPI: COPI data: 10
2551820-2551864 SPI: COPI data: 00
2551862-2551906 SPI: COPI data: 00
2551904-2551948 SPI: COPI data: 00

2551992-2552034 SPI: COPI data: 06
2552034-2552076 SPI: COPI data: 21

2552128-2552170 SPI: COPI data: 06
2552170-2552212 SPI: COPI data: 01

2552458-2552500 SPI: COPI data: FF
2552500-2552542 SPI: CIPO data: 00

2552585-2552629 SPI: COPI data: 7F : Write Bank 0 Reg 0x7F REG_BANK_SEL
2552627-2552671 SPI: COPI data: 20 : Select Bank 2

2552714-2552756 SPI: COPI data: 81 : Read Bank 2 Reg 0x01 GYRO_CONFIG_1
2552756-2552798 SPI: CIPO data: 07 : Enable gyro DLPF; +/- 2000dps

2552846-2552890 SPI: COPI data: FF
2552888-2552932 SPI: CIPO data: 20

2552974-2553016 SPI: COPI data: 7F
2553016-2553058 SPI: COPI data: 00

2553102-2553144 SPI: COPI data: 06
2553144-2553186 SPI: COPI data: 21

2553236-2553278 SPI: COPI data: 06
2553278-2553320 SPI: COPI data: 01

2553566-2553608 SPI: COPI data: FF
2553608-2553650 SPI: CIPO data: 00

2553693-2553737 SPI: COPI data: 7F
2553735-2553779 SPI: COPI data: 20

2553822-2553864 SPI: COPI data: 01 : Write Bank 2 Reg 0x01 GYRO_CONFIG_1
2553864-2553906 SPI: COPI data: 07 : Enable gyro DLPF; +/- 2000dps; 196.6Hz (3dB)

2553953-2553995 SPI: COPI data: FF
2553995-2554037 SPI: CIPO data: 20

2554081-2554123 SPI: COPI data: 7F
2554123-2554165 SPI: COPI data: 00

2554208-2554252 SPI: COPI data: 06
2554250-2554294 SPI: COPI data: 21

2554342-2554384 SPI: COPI data: 06
2554384-2554426 SPI: COPI data: 01

2554671-2554715 SPI: COPI data: FF
2554713-2554757 SPI: CIPO data: 00

2554799-2554841 SPI: COPI data: 7F
2554841-2554883 SPI: COPI data: 20

2554928-2554970 SPI: COPI data: 82 : Read Bank 2 Reg 0x02 GYRO_CONFIG_2
2554970-2555012 SPI: CIPO data: 00 : 1x averaging

2555060-2555102 SPI: COPI data: FF
2555102-2555144 SPI: CIPO data: 20

2555188-2555230 SPI: COPI data: 7F
2555230-2555272 SPI: COPI data: 00

2555316-2555358 SPI: COPI data: 06
2555358-2555400 SPI: COPI data: 21

2555450-2555492 SPI: COPI data: 06
2555492-2555534 SPI: COPI data: 01

2555779-2555823 SPI: COPI data: FF
2555821-2555865 SPI: CIPO data: 00

2555907-2555951 SPI: COPI data: 7F
2555949-2555993 SPI: COPI data: 20

2556035-2556077 SPI: COPI data: 02 : Write Bank 2 Reg 0x02 GYRO_CONFIG_2
2556077-2556119 SPI: COPI data: 00 : 1x averaging

2556166-2556208 SPI: COPI data: FF
2556208-2556250 SPI: CIPO data: 20

2556293-2556337 SPI: COPI data: 7F
2556335-2556379 SPI: COPI data: 00

2556421-2556465 SPI: COPI data: 06
2556463-2556507 SPI: COPI data: 21

2556567-2556609 SPI: COPI data: 06
2556609-2556651 SPI: COPI data: 01

2556898-2556940 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2556940-2556982 SPI: COPI data: 8C : GYRO_FULLSCALE (72 * 16 + 12)

2557026-2557068 SPI: COPI data: 7D : Set GYRO_FULLSCALE to 0x10000000
2557068-2557110 SPI: COPI data: 10
2557110-2557152 SPI: COPI data: 00
2557152-2557194 SPI: COPI data: 00
2557194-2557236 SPI: COPI data: 00

2557282-2557324 SPI: COPI data: 06
2557324-2557366 SPI: COPI data: 21

2557417-2557461 SPI: COPI data: 06
2557459-2557503 SPI: COPI data: 01

2557747-2557791 SPI: COPI data: FF
2557789-2557833 SPI: CIPO data: 00

2557875-2557917 SPI: COPI data: 7F
2557917-2557959 SPI: COPI data: 20

2558004-2558046 SPI: COPI data: 81 : Read Bank 2 Reg 0x01 GYRO_CONFIG_1
2558046-2558088 SPI: CIPO data: 07 : Enable gyro DLPF; +/- 2000dps; 196.6Hz (3dB)

2558136-2558178 SPI: COPI data: FF
2558178-2558220 SPI: CIPO data: 20

2558263-2558307 SPI: COPI data: 7F
2558305-2558349 SPI: COPI data: 00

2558391-2558435 SPI: COPI data: 06
2558433-2558477 SPI: COPI data: 21

2558525-2558567 SPI: COPI data: 06
2558567-2558609 SPI: COPI data: 01

2558855-2558897 SPI: COPI data: FF
2558897-2558939 SPI: CIPO data: 00

2558982-2559026 SPI: COPI data: 7F
2559024-2559067 SPI: COPI data: 20

2559111-2559153 SPI: COPI data: 01 : Write Bank 2 Reg 0x01 GYRO_CONFIG_1
2559153-2559195 SPI: COPI data: 07 : Enable gyro DLPF; +/- 2000dps; 196.6Hz (3dB)

2559242-2559286 SPI: COPI data: FF
2559284-2559328 SPI: CIPO data: 20

2559370-2559412 SPI: COPI data: 7F
2559412-2559454 SPI: COPI data: 00

2559498-2559540 SPI: COPI data: 06
2559540-2559582 SPI: COPI data: 21

2559631-2559673 SPI: COPI data: 06
2559673-2559715 SPI: COPI data: 01

2559961-2560003 SPI: COPI data: FF
2560003-2560045 SPI: CIPO data: 00

2560089-2560131 SPI: COPI data: 7F
2560131-2560173 SPI: COPI data: 20

2560217-2560261 SPI: COPI data: 82 : Read Bank 2 Reg 0x02 GYRO_CONFIG_2
2560259-2560303 SPI: CIPO data: 00 : 1x averaging

2560349-2560393 SPI: COPI data: FF
2560391-2560435 SPI: CIPO data: 20

2560477-2560519 SPI: COPI data: 7F
2560519-2560561 SPI: COPI data: 00

2560605-2560647 SPI: COPI data: 06
2560647-2560689 SPI: COPI data: 21

2560739-2560781 SPI: COPI data: 06
2560781-2560823 SPI: COPI data: 01

2561070-2561112 SPI: COPI data: FF
2561112-2561154 SPI: CIPO data: 00

2561198-2561240 SPI: COPI data: 7F
2561240-2561282 SPI: COPI data: 20

2561325-2561369 SPI: COPI data: 02 : Write Bank 2 Reg 0x02 GYRO_CONFIG_2
2561367-2561411 SPI: COPI data: 00 : 1x averaging

2561456-2561498 SPI: COPI data: FF
2561498-2561540 SPI: CIPO data: 20

2561584-2561626 SPI: COPI data: 7F
2561626-2561668 SPI: COPI data: 00

2561712-2561754 SPI: COPI data: 06
2561754-2561796 SPI: COPI data: 21

2561857-2561899 SPI: COPI data: 06
2561899-2561941 SPI: COPI data: 01

2562187-2562231 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2562229-2562273 SPI: COPI data: 8C : GYRO_FULLSCALE (72 * 16 + 12)

2562315-2562357 SPI: COPI data: 7D : Set GYRO_FULLSCALE to 0x10000000
2562357-2562399 SPI: COPI data: 10
2562399-2562441 SPI: COPI data: 00
2562441-2562483 SPI: COPI data: 00
2562483-2562525 SPI: COPI data: 00

2562571-2562613 SPI: COPI data: 06
2562613-2562655 SPI: COPI data: 21

2562803-2562845 SPI: COPI data: FF
2562845-2562887 SPI: CIPO data: 00

2562931-2562973 SPI: COPI data: 7F
2562973-2563015 SPI: COPI data: 00

2563059-2563101 SPI: COPI data: 05 : Write Bank 0 Reg 0x05 LP_CONFIG
2563101-2563143 SPI: COPI data: 40 : Operate _only_ I2C master in duty cycled mode. ODR is determined by I2C_MST_ODR_CONFIG register.

2563414-2563458 SPI: COPI data: 06
2563456-2563500 SPI: COPI data: 01

2563744-2563786 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2563786-2563828 SPI: COPI data: 10 : Reset I2C peripheral module and put the serial interface in SPI mode only

2563877-2563919 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
2563919-2563961 SPI: COPI data: 47 : Disable pressure sensor and all gyro axes

2564010-2564052 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2564052-2564094 SPI: COPI data: D0 : DMP Enable; FIFO Enable; Reset I2C peripheral module and put the serial interface in SPI mode only

2564154-2564198 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2564196-2564240 SPI: COPI data: 00 : Bank 0x00

2564282-2564326 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2564324-2564368 SPI: COPI data: 40 : DATA_OUT_CTL1 (4 * 16)

2564410-2564452 SPI: COPI data: 7D : Set DATA_OUT_CTL1 to 0x0000
2564452-2564494 SPI: COPI data: 00
2564494-2564536 SPI: COPI data: 00

2564587-2564629 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2564629-2564671 SPI: COPI data: 4C : DATA_INTR_CTL (4 * 16 + 12)

2564714-2564758 SPI: COPI data: 7D : Set DATA_INTR_CTL to 0x0000
2564756-2564800 SPI: COPI data: 00
2564798-2564842 SPI: COPI data: 00

2564893-2564935 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2564935-2564977 SPI: COPI data: 42 : DATA_OUT_CTL2 (4 * 16 + 2)

2565020-2565064 SPI: COPI data: 7D : Set DATA_OUT_CTL2 to 0x0000
2565062-2565106 SPI: COPI data: 00
2565104-2565148 SPI: COPI data: 00

2565199-2565243 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2565241-2565285 SPI: COPI data: 4E : MOTION_EVENT_CTL (4 * 16 + 14)

2565327-2565369 SPI: COPI data: 7D : Set MOTION_EVENT_CTL to 0x0000
2565369-2565411 SPI: COPI data: 00
2565411-2565453 SPI: COPI data: 00

2565549-2565593 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
2565591-2565635 SPI: COPI data: 7F : Disable pressure; all accel axes; all gyro axes

2565681-2565723 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2565723-2565765 SPI: COPI data: 41 : Sleep; Auto clock

2566015-2566057 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2566057-2566099 SPI: COPI data: 01 : Auto clock

2566346-2566388 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
2566388-2566430 SPI: COPI data: 7F : Disable pressure; all accel axes; all gyro axes

2566481-2566523 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2566523-2566565 SPI: COPI data: 8A : DATA_RDY_STATUS (8 * 16 + 10)

2566609-2566651 SPI: COPI data: 7D : Set DATA_RDY_STATUS to 0x0000
2566651-2566693 SPI: COPI data: 00
2566693-2566735 SPI: COPI data: 00

2566783-2566825 SPI: COPI data: 06
2566825-2566867 SPI: COPI data: 21

2566921-2566963 SPI: COPI data: 06
2566963-2567005 SPI: COPI data: 01

2567254-2567296 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2567296-2567338 SPI: COPI data: D0 : DMP Enable; FIFO Enable; Reset I2C peripheral module and put the serial interface in SPI mode only

2567387-2567429 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
2567429-2567471 SPI: COPI data: 47 : Disable pressure; all gyro axes

2567520-2567562 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2567562-2567604 SPI: COPI data: D0 : DMP Enable; FIFO Enable; Reset I2C peripheral module and put the serial interface in SPI mode only

2567665-2567707 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2567707-2567749 SPI: COPI data: 40 : DATA_OUT_CTL1 (4 * 16)

2567793-2567835 SPI: COPI data: 7D : Set DATA_OUT_CTL1 to 0x0000
2567835-2567877 SPI: COPI data: 00
2567877-2567919 SPI: COPI data: 00

2567969-2568013 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2568011-2568055 SPI: COPI data: 4C : DATA_INTR_CTL (4 * 16 + 12)

2568097-2568139 SPI: COPI data: 7D : Set DATA_INTR_CTL to 0x0000
2568139-2568181 SPI: COPI data: 00
2568181-2568223 SPI: COPI data: 00

2568275-2568319 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2568317-2568360 SPI: COPI data: 42 : DATA_OUT_CTL2 (4 * 16 + 2)

2568403-2568447 SPI: COPI data: 7D : Set DATA_OUT_CTL2 to 0x0000
2568445-2568489 SPI: COPI data: 00
2568487-2568531 SPI: COPI data: 00

2568582-2568624 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2568624-2568666 SPI: COPI data: 4E : MOTION_EVENT_CTL (4 * 16 + 14)

2568710-2568752 SPI: COPI data: 7D : Set MOTION_EVENT_CTL to 0x0000
2568752-2568794 SPI: COPI data: 00
2568794-2568836 SPI: COPI data: 00

2568932-2568974 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
2568974-2569016 SPI: COPI data: 7F : Disable pressure; all accel axes; all gyro axes

2569063-2569105 SPI: COPI data: 06 : Write Bank 0 Reg 0x07 PWR_MGMT_1
2569105-2569147 SPI: COPI data: 41 : Sleep; Auto clock

2569401-2569443 SPI: COPI data: 06 : Write Bank 0 Reg 0x07 PWR_MGMT_1
2569443-2569485 SPI: COPI data: 01 : Auto clock

2569732-2569776 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
2569774-2569818 SPI: COPI data: 7F : Disable pressure; all accel axes; all gyro axes

2569867-2569911 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2569909-2569953 SPI: COPI data: 8A : DATA_RDY_STATUS (8 * 16 + 10)

2569995-2570037 SPI: COPI data: 7D : Set DATA_RDY_STATUS to 0x0000
2570037-2570079 SPI: COPI data: 00
2570079-2570121 SPI: COPI data: 00

2570169-2570211 SPI: COPI data: 06
2570211-2570253 SPI: COPI data: 21

2570307-2570349 SPI: COPI data: 06
2570349-2570391 SPI: COPI data: 01

2570637-2570679 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2570679-2570721 SPI: COPI data: D0 : DMP Enable; FIFO Enable; Reset I2C peripheral module and put the serial interface in SPI mode only

2570770-2570812 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
2570812-2570854 SPI: COPI data: 47 : Disable pressure; all gyro axes

2570902-2570946 SPI: COPI data: 03 : Write Bank 0 Reg 0x03 USER_CTRL
2570944-2570988 SPI: COPI data: D0 : DMP Enable; FIFO Enable; Reset I2C peripheral module and put the serial interface in SPI mode only

2571047-2571089 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2571089-2571131 SPI: COPI data: 40 : DATA_OUT_CTL1 (4 * 16)

2571176-2571218 SPI: COPI data: 7D : Set DATA_OUT_CTL1 to 0x0808 (Quat6 + Header 2)
2571218-2571260 SPI: COPI data: 08
2571260-2571302 SPI: COPI data: 08

2571353-2571395 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2571395-2571437 SPI: COPI data: 4C : DATA_INTR_CTL (4 * 16 + 12)

2571481-2571523 SPI: COPI data: 7D : Set DATA_INTR_CTL to 0x0808
2571523-2571565 SPI: COPI data: 08
2571565-2571607 SPI: COPI data: 08

2571659-2571703 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2571701-2571745 SPI: COPI data: 42 : DATA_OUT_CTL2 (4 * 16 + 2)

2571787-2571829 SPI: COPI data: 7D : Set DATA_OUT_CTL2 to 0x0000
2571829-2571871 SPI: COPI data: 00
2571871-2571913 SPI: COPI data: 00

2571966-2572008 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2572008-2572050 SPI: COPI data: 4E : MOTION_EVENT_CTL (4 * 16 + 14)

2572093-2572137 SPI: COPI data: 7D : Set MOTION_EVENT_CTL to 0x0300 (Gyro_Calibr + Accel_Calibr) 
2572135-2572179 SPI: COPI data: 03
2572177-2572221 SPI: COPI data: 00

2572310-2572352 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2572352-2572394 SPI: COPI data: 01 : Bank 0x01

2572438-2572480 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2572480-2572522 SPI: COPI data: 0C : ACCEL_ONLY_GAIN (16 * 16 + 12)

2572565-2572609 SPI: COPI data: 7D : Set ACCEL_ONLY_GAIN to 0x00E8BA2E (225Hz)
2572607-2572651 SPI: COPI data: 00
2572649-2572693 SPI: COPI data: E8
2572691-2572735 SPI: COPI data: BA
2572733-2572777 SPI: COPI data: 2E

2572827-2572869 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2572869-2572911 SPI: COPI data: 05 : Bank 0x05

2572954-2572998 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2572996-2573040 SPI: COPI data: B0 : ACCEL_ALPHA_VAR (91 * 16)

2573082-2573124 SPI: COPI data: 7D : Set ACCEL_ALPHA_VAR to 0x3D27D27D (225Hz)
2573124-2573166 SPI: COPI data: 3D
2573166-2573208 SPI: COPI data: 27
2573208-2573250 SPI: COPI data: D2
2573250-2573292 SPI: COPI data: 7D

2573342-2573384 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2573384-2573426 SPI: COPI data: C0 : ACCEL_A_VAR (92 * 16)

2573470-2573512 SPI: COPI data: 7D : Set ACCEL_A_VAR to 0x02D82D83 (225Hz)
2573512-2573554 SPI: COPI data: 02
2573554-2573596 SPI: COPI data: D8
2573596-2573638 SPI: COPI data: 2D
2573638-2573680 SPI: COPI data: 83

2573729-2573773 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2573771-2573815 SPI: COPI data: E4 : ACCEL_CAL_INIT (94 * 16 + 4)

2573857-2573899 SPI: COPI data: 7D : Set ACCEL_CAL_INIT to 0x0000
2573899-2573941 SPI: COPI data: 00
2573941-2573983 SPI: COPI data: 00

2574034-2574078 SPI: COPI data: FF
2574076-2574120 SPI: CIPO data: 00

2574162-2574204 SPI: COPI data: 7F
2574204-2574246 SPI: COPI data: 20

2574291-2574333 SPI: COPI data: 10 : Set Bank 2 Reg 0x10 ACCEL_SMPLRT_DIV_1 to 0x0004
2574333-2574375 SPI: COPI data: 00
2574375-2574417 SPI: COPI data: 04

2574476-2574518 SPI: COPI data: 00 : Set Bank 2 Reg 0x00 GYRO_SMPLRT_DIV to 0x04 (ODR = 1100/5 = 220Hz)
2574518-2574562 SPI: COPI data: 04

2574620-2574662 SPI: COPI data: FF
2574662-2574704 SPI: CIPO data: 20

2574747-2574791 SPI: COPI data: 7F
2574789-2574833 SPI: COPI data: 00

2574877-2574919 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2574919-2574961 SPI: COPI data: 00 : Bank 0x00

2575005-2575047 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2575047-2575089 SPI: COPI data: AC : ODR_QUAT6 (10 * 16 + 12)

2575132-2575176 SPI: COPI data: 7D : Set ODR_QUAT6 to 0x0000
2575174-2575218 SPI: COPI data: 00
2575216-2575260 SPI: COPI data: 00

2575323-2575367 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2575365-2575409 SPI: COPI data: 01 : Bank 0x01

2575451-2575495 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2575493-2575537 SPI: COPI data: 30 : GYRO_SF (19 * 16)

2575579-2575623 SPI: COPI data: 7D : Set GYRO_SF to 0x09DBEF0D (225Hz)
2575621-2575665 SPI: COPI data: 09
2575663-2575707 SPI: COPI data: DB
2575705-2575749 SPI: COPI data: EF
2575747-2575791 SPI: COPI data: 0D

2575840-2575884 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
2575882-2575926 SPI: COPI data: 40 : Disable pressure (all accel and gyro axes enabled)

2575975-2576017 SPI: COPI data: 7E : Write Bank 0 Reg 0x7E Memory Bank Select
2576017-2576059 SPI: COPI data: 00 : Bank 0x00

2576103-2576145 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2576145-2576187 SPI: COPI data: 8A : DATA_RDY_STATUS (8 * 16 + 10)

2576230-2576274 SPI: COPI data: 7D : Set DATA_RDY_STATUS to 0x0003 (gyro and accel samples)
2576272-2576316 SPI: COPI data: 00
2576314-2576358 SPI: COPI data: 03

2576404-2576448 SPI: COPI data: 06
2576446-2576490 SPI: COPI data: 21

2576554-2576596 SPI: COPI data: 06
2576596-2576638 SPI: COPI data: 01

2576885-2576927 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2576927-2576969 SPI: COPI data: 40 : DATA_OUT_CTL1 (4 * 16)

2577012-2577056 SPI: COPI data: 7D : Set DATA_OUT_CTL1 to 0x0808
2577054-2577098 SPI: COPI data: 08
2577096-2577140 SPI: COPI data: 08

2577189-2577233 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2577231-2577275 SPI: COPI data: 4C : DATA_INTR_CTL (4 * 16 + 12)

2577317-2577359 SPI: COPI data: 7D : Set DATA_INTR_CTL to 0x0808
2577359-2577401 SPI: COPI data: 08
2577401-2577443 SPI: COPI data: 08

2577495-2577539 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2577537-2577581 SPI: COPI data: 42 : DATA_OUT_CTL2 (4 * 16 + 2)

2577623-2577667 SPI: COPI data: 7D : Set DATA_OUT_CTL2 to 0x0000
2577665-2577709 SPI: COPI data: 00
2577707-2577751 SPI: COPI data: 00

2577802-2577844 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2577844-2577886 SPI: COPI data: 4E : MOTION_EVENT_CTL (4 * 16 + 14)

2577930-2577972 SPI: COPI data: 7D : Set MOTION_EVENT_CTL to 0x0300 (Gyro_Calibr + Accel_Calibr)
2577972-2578014 SPI: COPI data: 03
2578014-2578056 SPI: COPI data: 00

2578157-2578199 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2578199-2578241 SPI: COPI data: AC : ODR_QUAT6 (10 * 16 + 12)

2578285-2578327 SPI: COPI data: 7D : Set ODR_QUAT6 to 0x0000
2578327-2578369 SPI: COPI data: 00
2578369-2578411 SPI: COPI data: 00

2578476-2578520 SPI: COPI data: 07 : Write Bank 0 Reg 0x07 PWR_MGMT_2
2578518-2578562 SPI: COPI data: 40 : Disable pressure (all accel and gyro axes are enabled)

2578611-2578655 SPI: COPI data: 7C : Write Bank 0 Reg 0x7C Memory Start Address
2578653-2578697 SPI: COPI data: 8A : DATA_RDY_STATUS (8 * 16 + 10)

2578739-2578781 SPI: COPI data: 7D : Set DATA_RDY_STATUS to 0x0003 (gyro + accel)
2578781-2578823 SPI: COPI data: 00
2578823-2578865 SPI: COPI data: 03

2578913-2578955 SPI: COPI data: 06 : Write Bank 0 Reg 0x06 PWR_MGMT_1
2578955-2578997 SPI: COPI data: 21 : Set PWR_MGMT_1 : Turn on low power; Auto clock best available

2579049-2579091 SPI: COPI data: 99 : Read Bank 0 Reg 0x19 INT_STATUS
2579091-2579133 SPI: CIPO data: 88 : Reserved + Wake_On_Motion Int

2579181-2579225 SPI: COPI data: 98 : Read Bank 0 Reg 0x18 DMP_INT_STATUS
2579223-2579267 SPI: CIPO data: 00

2579320-2579362 SPI: COPI data: 99 : Read Bank 0 Reg 0x19 INT_STATUS
2579362-2579404 SPI: CIPO data: 00

2579452-2579496 SPI: COPI data: 98 : Read Bank 0 Reg 0x18 DMP_INT_STATUS
2579494-2579538 SPI: CIPO data: 00

...

...

2652310-2652352 SPI: COPI data: 99 : Read Bank 0 Reg 0x19 INT_STATUS
2652352-2652394 SPI: CIPO data: 02 : DMP_INT1 – Indicates the DMP has generated INT1 interrupt.

2652442-2652484 SPI: COPI data: 98 : Read Bank 0 Reg 0x18 DMP_INT_STATUS
2652484-2652526 SPI: CIPO data: 01

2652583-2652625 SPI: COPI data: F0 : Read Bank 0 Reg 0x70 FIFO_COUNTH
2652625-2652667 SPI: CIPO data: 00 : FIFO contains 16 bytes
2652667-2652709 SPI: CIPO data: 10

2652759-2652801 SPI: COPI data: F2 : Read Bank 0 Reg 0x72 FIFO_R_W
2652801-2652843 SPI: CIPO data: 08 : Header: Quat6
2652843-2652885 SPI: CIPO data: 00 : Header:
2652885-2652927 SPI: CIPO data: FF : Quat6: Q1
2652927-2652969 SPI: CIPO data: FF : Quat6: Q1
2652969-2653011 SPI: CIPO data: AE : Quat6: Q1
2653011-2653053 SPI: CIPO data: 9B : Quat6: Q1
2653053-2653095 SPI: CIPO data: 00 : Quat6: Q2
2653095-2653137 SPI: CIPO data: 00 : Quat6: Q2
2653137-2653179 SPI: CIPO data: 4D : Quat6: Q2
2653179-2653221 SPI: CIPO data: 83 : Quat6: Q2
2653221-2653263 SPI: CIPO data: 00 : Quat6: Q3
2653263-2653305 SPI: CIPO data: 00 : Quat6: Q3
2653305-2653347 SPI: CIPO data: 00 : Quat6: Q3
2653347-2653389 SPI: CIPO data: 00 : Quat6: Q3
2653389-2653431 SPI: CIPO data: 00 : Footer: Gyro count
2653431-2653473 SPI: CIPO data: 01 : Footer: Gyro count

2653606-2653648 SPI: COPI data: 99
2653648-2653690 SPI: CIPO data: 00

2653739-2653781 SPI: COPI data: 98
2653781-2653823 SPI: CIPO data: 00

...

```
