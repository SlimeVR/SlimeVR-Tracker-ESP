/*

This file contains a useful c translation of the DMP register map

*/

#ifndef _ICM_20948_DMP_H_
#define _ICM_20948_DMP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#define DMP_START_ADDRESS ((unsigned short)0x1000)
#define DMP_MEM_BANK_SIZE 256
#define DMP_LOAD_START 0x90

#define CFG_FIFO_SIZE (4222)

// AGB0_REG_DMP_INT_STATUS bit definitions
#define BIT_WAKE_ON_MOTION_INT 0x08
#define BIT_MSG_DMP_INT 0x0002
#define BIT_MSG_DMP_INT_0 0x0100 // CI Command

#define BIT_MSG_DMP_INT_2 0x0200 // CIM Command - SMD
#define BIT_MSG_DMP_INT_3 0x0400 // CIM Command - Pedometer

#define BIT_MSG_DMP_INT_4 0x1000 // CIM Command - Pedometer binning
#define BIT_MSG_DMP_INT_5 0x2000 // CIM Command - Bring To See Gesture
#define BIT_MSG_DMP_INT_6 0x4000 // CIM Command - Look To See Gesture

// Appendix I: DMP register addresses

// data output control
#define DATA_OUT_CTL1 (4 * 16) // 16-bit: Data output control 1 register : configure DMP to output required data
#define DATA_OUT_CTL2 (4 * 16 + 2) // 16-bit: Data output control 2 register : configure the BM, accel/gyro/compass accuracy and gesture such as Pick-up
#define DATA_INTR_CTL (4 * 16 + 12) // 16-bit: Determines which sensors can generate interrupt according to bit map defined for DATA_OUT_CTL1
#define FIFO_WATERMARK (31 * 16 + 14) // 16-bit: DMP will send FIFO interrupt if FIFO count > FIFO watermark. FIFO watermark is set to 80% of actual FIFO size by default

// motion event control
#define MOTION_EVENT_CTL (4 * 16 + 14) // 16-bit: configure DMP for Android L and Invensense specific features

// indicates to DMP which sensors are available
/*	1: gyro samples available
	2: accel samples available
	8: secondary compass samples available	*/
#define DATA_RDY_STATUS (8 * 16 + 10) // 16-bit: indicates to DMP which sensors are available

// batch mode
#define BM_BATCH_CNTR (27 * 16) // 32-bit: Batch counter
#define BM_BATCH_THLD (19 * 16 + 12) // 32-bit: Batch mode threshold
#define BM_BATCH_MASK (21 * 16 + 14) // 16-bit

// sensor output data rate: all 16-bit
#define ODR_ACCEL (11 * 16 + 14) // ODR_ACCEL Register for accel ODR
#define ODR_GYRO (11 * 16 + 10) // ODR_GYRO Register for gyro ODR
#define ODR_CPASS (11 * 16 + 6) // ODR_CPASS Register for compass ODR
#define ODR_ALS (11 * 16 + 2) // ODR_ALS Register for ALS ODR
#define ODR_QUAT6 (10 * 16 + 12) // ODR_QUAT6 Register for 6-axis quaternion ODR
#define ODR_QUAT9 (10 * 16 + 8) // ODR_QUAT9 Register for 9-axis quaternion ODR
#define ODR_PQUAT6 (10 * 16 + 4) // ODR_PQUAT6 Register for 6-axis pedometer quaternion ODR
#define ODR_GEOMAG (10 * 16 + 0) // ODR_GEOMAG Register for Geomag rv ODR
#define ODR_PRESSURE (11 * 16 + 12) // ODR_PRESSURE Register for pressure ODR
#define ODR_GYRO_CALIBR (11 * 16 + 8) // ODR_GYRO_CALIBR Register for calibrated gyro ODR
#define ODR_CPASS_CALIBR (11 * 16 + 4) // ODR_CPASS_CALIBR Register for calibrated compass ODR

// sensor output data rate counter: all 16-bit
#define ODR_CNTR_ACCEL (9 * 16 + 14) // ODR_CNTR_ACCEL Register for accel ODR counter
#define ODR_CNTR_GYRO (9 * 16 + 10) // ODR_CNTR_GYRO Register for gyro ODR counter
#define ODR_CNTR_CPASS (9 * 16 + 6) // ODR_CNTR_CPASS Register for compass ODR counter
#define ODR_CNTR_ALS (9 * 16 + 2) // ODR_CNTR_ALS Register for ALS ODR counter
#define ODR_CNTR_QUAT6 (8 * 16 + 12) // ODR_CNTR_QUAT6 Register for 6-axis quaternion ODR counter
#define ODR_CNTR_QUAT9 (8 * 16 + 8) // ODR_CNTR_QUAT9 Register for 9-axis quaternion ODR counter
#define ODR_CNTR_PQUAT6 (8 * 16 + 4) // ODR_CNTR_PQUAT6 Register for 6-axis pedometer quaternion ODR counter
#define ODR_CNTR_GEOMAG (8 * 16 + 0) // ODR_CNTR_GEOMAG Register for Geomag rv ODR counter
#define ODR_CNTR_PRESSURE (9 * 16 + 12) // ODR_CNTR_PRESSURE Register for pressure ODR counter
#define ODR_CNTR_GYRO_CALIBR (9 * 16 + 8) // ODR_CNTR_GYRO_CALIBR Register for calibrated gyro ODR counter
#define ODR_CNTR_CPASS_CALIBR (9 * 16 + 4) // ODR_CNTR_CPASS_CALIBR Register for calibrated compass ODR counter

// mounting matrix: all 32-bit
#define CPASS_MTX_00 (23 * 16) // Compass mount matrix and scale
#define CPASS_MTX_01 (23 * 16 + 4) // Compass mount matrix and scale
#define CPASS_MTX_02 (23 * 16 + 8) // Compass mount matrix and scale
#define CPASS_MTX_10 (23 * 16 + 12) // Compass mount matrix and scale
#define CPASS_MTX_11 (24 * 16) // Compass mount matrix and scale
#define CPASS_MTX_12 (24 * 16 + 4) // Compass mount matrix and scale
#define CPASS_MTX_20 (24 * 16 + 8) // Compass mount matrix and scale
#define CPASS_MTX_21 (24 * 16 + 12) // Compass mount matrix and scale
#define CPASS_MTX_22 (25 * 16) // Compass mount matrix and scale

// bias calibration: all 32-bit
// The biases are 32-bits in chip frame in hardware unit scaled by:
// 2^12 (FSR 4g) for accel, 2^15 for gyro, in uT scaled by 2^16 for compass.
#define GYRO_BIAS_X (139 * 16 + 4)
#define GYRO_BIAS_Y (139 * 16 + 8)
#define GYRO_BIAS_Z (139 * 16 + 12)
#define ACCEL_BIAS_X (110 * 16 + 4)
#define ACCEL_BIAS_Y (110 * 16 + 8)
#define ACCEL_BIAS_Z (110 * 16 + 12)
#define CPASS_BIAS_X (126 * 16 + 4)
#define CPASS_BIAS_Y (126 * 16 + 8)
#define CPASS_BIAS_Z (126 * 16 + 12)

#define GYRO_ACCURACY (138 * 16 + 2)
#define GYRO_BIAS_SET (138 * 16 + 6)
#define GYRO_LAST_TEMPR (134 * 16)
#define GYRO_SLOPE_X (78 * 16 + 4)
#define GYRO_SLOPE_Y (78 * 16 + 8)
#define GYRO_SLOPE_Z (78 * 16 + 12)

// parameters for accel calibration
#define ACCEL_ACCURACY (97 * 16)
#define ACCEL_CAL_RESET (77 * 16)
#define ACCEL_VARIANCE_THRESH (93 * 16)
#define ACCEL_CAL_RATE (94 * 16 + 4) // 16-bit: 0 (225Hz, 112Hz, 56Hz)
#define ACCEL_PRE_SENSOR_DATA (97 * 16 + 4)
#define ACCEL_COVARIANCE (101 * 16 + 8)
#define ACCEL_ALPHA_VAR (91 * 16) // 32-bit: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
#define ACCEL_A_VAR (92 * 16) // 32-bit: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
#define ACCEL_CAL_INIT (94 * 16 + 2)
#define ACCEL_CAL_SCALE_COVQ_IN_RANGE (194 * 16)
#define ACCEL_CAL_SCALE_COVQ_OUT_RANGE (195 * 16)
#define ACCEL_CAL_TEMPERATURE_SENSITIVITY (194 * 16 + 4)
#define ACCEL_CAL_TEMPERATURE_OFFSET_TRIM (194 * 16 + 12)

#define CPASS_ACCURACY (37 * 16)
#define CPASS_BIAS_SET (34 * 16 + 14)
#define MAR_MODE (37 * 16 + 2)
#define CPASS_COVARIANCE (115 * 16)
#define CPASS_COVARIANCE_CUR (118 * 16 + 8)
#define CPASS_REF_MAG_3D (122 * 16)
#define CPASS_CAL_INIT (114 * 16)
#define CPASS_EST_FIRST_BIAS (113 * 16)
#define MAG_DISTURB_STATE (113 * 16 + 2)
#define CPASS_VAR_COUNT (112 * 16 + 6)
#define CPASS_COUNT_7 (87 * 16 + 2)
#define CPASS_MAX_INNO (124 * 16)
#define CPASS_BIAS_OFFSET (113 * 16 + 4)
#define CPASS_CUR_BIAS_OFFSET (114 * 16 + 4)
#define CPASS_PRE_SENSOR_DATA (87 * 16 + 4)

// Compass Cal params to be adjusted according to sampling rate
#define CPASS_TIME_BUFFER (112 * 16 + 14)
#define CPASS_RADIUS_3D_THRESH_ANOMALY (112 * 16 + 8)

#define CPASS_STATUS_CHK (25 * 16 + 12)

// gains
#define ACCEL_FB_GAIN (34 * 16)
#define ACCEL_ONLY_GAIN (16 * 16 + 12) // 32-bit: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
#define GYRO_SF (19 * 16) // 32-bit: gyro scaling factor

// 9-axis
#define MAGN_THR_9X (80 * 16)
#define MAGN_LPF_THR_9X (80 * 16 + 8)
#define QFB_THR_9X (80 * 16 + 12)

// DMP running counter
#define DMPRATE_CNTR (18 * 16 + 4)

// pedometer
#define PEDSTD_BP_B (49 * 16 + 12)
#define PEDSTD_BP_A4 (52 * 16)
#define PEDSTD_BP_A3 (52 * 16 + 4)
#define PEDSTD_BP_A2 (52 * 16 + 8)
#define PEDSTD_BP_A1 (52 * 16 + 12)
#define PEDSTD_SB (50 * 16 + 8)
#define PEDSTD_SB_TIME (50 * 16 + 12)
#define PEDSTD_PEAKTHRSH (57 * 16 + 8)
#define PEDSTD_TIML (50 * 16 + 10)
#define PEDSTD_TIMH (50 * 16 + 14)
#define PEDSTD_PEAK (57 * 16 + 4)
#define PEDSTD_STEPCTR (54 * 16)
#define PEDSTD_STEPCTR2 (58 * 16 + 8)
#define PEDSTD_TIMECTR (60 * 16 + 4)
#define PEDSTD_DECI (58 * 16)
#define PEDSTD_SB2 (60 * 16 + 14)
#define STPDET_TIMESTAMP (18 * 16 + 8)
#define PEDSTEP_IND (19 * 16 + 4)
#define PED_Y_RATIO (17 * 16 + 0)

// SMD
#define SMD_VAR_TH (141 * 16 + 12)
#define SMD_VAR_TH_DRIVE (143 * 16 + 12)
#define SMD_DRIVE_TIMER_TH (143 * 16 + 8)
#define SMD_TILT_ANGLE_TH (179 * 16 + 12)
#define BAC_SMD_ST_TH (179 * 16 + 8)
#define BAC_ST_ALPHA4 (180 * 16 + 12)
#define BAC_ST_ALPHA4A (176 * 16 + 12)

// Wake on Motion
#define WOM_ENABLE (64 * 16 + 14)
#define WOM_STATUS (64 * 16 + 6)
#define WOM_THRESHOLD_DMP (64 * 16) // Renamed by PaulZC to avoid duplication with the Bank 2 Reg 0x13
#define WOM_CNTR_TH (64 * 16 + 12)

// Activity Recognition
#define BAC_RATE (48 * 16 + 10)
#define BAC_STATE (179 * 16 + 0)
#define BAC_STATE_PREV (179 * 16 + 4)
#define BAC_ACT_ON (182 * 16 + 0)
#define BAC_ACT_OFF (183 * 16 + 0)
#define BAC_STILL_S_F (177 * 16 + 0)
#define BAC_RUN_S_F (177 * 16 + 4)
#define BAC_DRIVE_S_F (178 * 16 + 0)
#define BAC_WALK_S_F (178 * 16 + 4)
#define BAC_SMD_S_F (178 * 16 + 8)
#define BAC_BIKE_S_F (178 * 16 + 12)
#define BAC_E1_SHORT (146 * 16 + 0)
#define BAC_E2_SHORT (146 * 16 + 4)
#define BAC_E3_SHORT (146 * 16 + 8)
#define BAC_VAR_RUN (148 * 16 + 12)
#define BAC_TILT_INIT (181 * 16 + 0)
#define BAC_MAG_ON (225 * 16 + 0)
#define BAC_PS_ON (74 * 16 + 0)
#define BAC_BIKE_PREFERENCE (173 * 16 + 8)
#define BAC_MAG_I2C_ADDR (229 * 16 + 8)
#define BAC_PS_I2C_ADDR (75 * 16 + 4)
#define BAC_DRIVE_CONFIDENCE (144 * 16 + 0)
#define BAC_WALK_CONFIDENCE (144 * 16 + 4)
#define BAC_SMD_CONFIDENCE (144 * 16 + 8)
#define BAC_BIKE_CONFIDENCE (144 * 16 + 12)
#define BAC_STILL_CONFIDENCE (145 * 16 + 0)
#define BAC_RUN_CONFIDENCE (145 * 16 + 4)
#define BAC_MODE_CNTR (150 * 16)
#define BAC_STATE_T_PREV (185 * 16 + 4)
#define BAC_ACT_T_ON (184 * 16 + 0)
#define BAC_ACT_T_OFF (184 * 16 + 4)
#define BAC_STATE_WRDBS_PREV (185 * 16 + 8)
#define BAC_ACT_WRDBS_ON (184 * 16 + 8)
#define BAC_ACT_WRDBS_OFF (184 * 16 + 12)
#define BAC_ACT_ON_OFF (190 * 16 + 2)
#define PREV_BAC_ACT_ON_OFF (188 * 16 + 2)
#define BAC_CNTR (48 * 16 + 2)

// Flip/Pick-up
#define FP_VAR_ALPHA (245 * 16 + 8)
#define FP_STILL_TH (246 * 16 + 4)
#define FP_MID_STILL_TH (244 * 16 + 8)
#define FP_NOT_STILL_TH (246 * 16 + 8)
#define FP_VIB_REJ_TH (241 * 16 + 8)
#define FP_MAX_PICKUP_T_TH (244 * 16 + 12)
#define FP_PICKUP_TIMEOUT_TH (248 * 16 + 8)
#define FP_STILL_CONST_TH (246 * 16 + 12)
#define FP_MOTION_CONST_TH (240 * 16 + 8)
#define FP_VIB_COUNT_TH (242 * 16 + 8)
#define FP_STEADY_TILT_TH (247 * 16 + 8)
#define FP_STEADY_TILT_UP_TH (242 * 16 + 12)
#define FP_Z_FLAT_TH_MINUS (243 * 16 + 8)
#define FP_Z_FLAT_TH_PLUS (243 * 16 + 12)
#define FP_DEV_IN_POCKET_TH (76 * 16 + 12)
#define FP_PICKUP_CNTR (247 * 16 + 4)
#define FP_RATE (240 * 16 + 12)

// Gyro FSR
#define GYRO_FULLSCALE (72 * 16 + 12)

// Accel FSR
// The DMP scales accel raw data internally to align 1g as 2^25.
// To do this and output hardware unit again as configured FSR, write 0x4000000 to ACC_SCALE DMP register, and write 0x40000 to ACC_SCALE2 DMP register.
#define ACC_SCALE (30 * 16 + 0) // 32-bit: Write accel scaling value for internal use
#define ACC_SCALE2 (79 * 16 + 4) // 32-bit: Write accel scaling down value

// EIS authentication
#define EIS_AUTH_INPUT (160 * 16 + 4)
#define EIS_AUTH_OUTPUT (160 * 16 + 0)

// B2S
#define B2S_RATE (48 * 16 + 8)

// B2S mounting matrix
#define B2S_MTX_00 (208 * 16)
#define B2S_MTX_01 (208 * 16 + 4)
#define B2S_MTX_02 (208 * 16 + 8)
#define B2S_MTX_10 (208 * 16 + 12)
#define B2S_MTX_11 (209 * 16)
#define B2S_MTX_12 (209 * 16 + 4)
#define B2S_MTX_20 (209 * 16 + 8)
#define B2S_MTX_21 (209 * 16 + 12)
#define B2S_MTX_22 (210 * 16)

// Dmp3 orientation parameters (Q30) initialization
#define Q0_QUAT6 (33 * 16 + 0)
#define Q1_QUAT6 (33 * 16 + 4)
#define Q2_QUAT6 (33 * 16 + 8)
#define Q3_QUAT6 (33 * 16 + 12)

  enum DMP_ODR_Registers
  {
    DMP_ODR_Reg_Accel = ODR_ACCEL,              // ODR_ACCEL Register for accel ODR
    DMP_ODR_Reg_Gyro = ODR_GYRO,                // ODR_GYRO Register for gyro ODR
    DMP_ODR_Reg_Cpass = ODR_CPASS,              // ODR_CPASS Register for compass ODR
    DMP_ODR_Reg_ALS = ODR_ALS,                  // ODR_ALS Register for ALS ODR
    DMP_ODR_Reg_Quat6 = ODR_QUAT6,              // ODR_QUAT6 Register for 6-axis quaternion ODR
    DMP_ODR_Reg_Quat9 = ODR_QUAT9,              // ODR_QUAT9 Register for 9-axis quaternion ODR
    DMP_ODR_Reg_PQuat6 = ODR_PQUAT6,            // ODR_PQUAT6 Register for 6-axis pedometer quaternion ODR
    DMP_ODR_Reg_Geomag = ODR_GEOMAG,            // ODR_GEOMAG Register for Geomag RV ODR
    DMP_ODR_Reg_Pressure = ODR_PRESSURE,        // ODR_PRESSURE Register for pressure ODR
    DMP_ODR_Reg_Gyro_Calibr = ODR_GYRO_CALIBR,  // ODR_GYRO_CALIBR Register for calibrated gyro ODR
    DMP_ODR_Reg_Cpass_Calibr = ODR_CPASS_CALIBR // ODR_CPASS_CALIBR Register for calibrated compass ODR
  };

  /** @brief Sensor identifier for control function
 */
  enum inv_icm20948_sensor
  {
    INV_ICM20948_SENSOR_ACCELEROMETER = 0,
    INV_ICM20948_SENSOR_GYROSCOPE,
    INV_ICM20948_SENSOR_RAW_ACCELEROMETER,
    INV_ICM20948_SENSOR_RAW_GYROSCOPE,
    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED,
    INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON,
    INV_ICM20948_SENSOR_STEP_DETECTOR,
    INV_ICM20948_SENSOR_STEP_COUNTER,
    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR,
    INV_ICM20948_SENSOR_ROTATION_VECTOR,
    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD,
    INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION,
    INV_ICM20948_SENSOR_FLIP_PICKUP,
    INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR,
    INV_ICM20948_SENSOR_GRAVITY,
    INV_ICM20948_SENSOR_LINEAR_ACCELERATION,
    INV_ICM20948_SENSOR_ORIENTATION,
    INV_ICM20948_SENSOR_B2S,
    INV_ICM20948_SENSOR_RAW_MAGNETOMETER,
    INV_ICM20948_SENSOR_MAX,
  };

  /* enum for android sensor*/
  enum ANDROID_SENSORS
  {
    ANDROID_SENSOR_META_DATA = 0,               // 0
    ANDROID_SENSOR_ACCELEROMETER,               // 1
    ANDROID_SENSOR_GEOMAGNETIC_FIELD,           // 2
    ANDROID_SENSOR_ORIENTATION,                 // 3
    ANDROID_SENSOR_GYROSCOPE,                   // 4
    ANDROID_SENSOR_LIGHT,                       // 5
    ANDROID_SENSOR_PRESSURE,                    // 6
    ANDROID_SENSOR_TEMPERATURE,                 // 7
    ANDROID_SENSOR_WAKEUP_PROXIMITY,            // 8
    ANDROID_SENSOR_GRAVITY,                     // 9
    ANDROID_SENSOR_LINEAR_ACCELERATION,         // 10
    ANDROID_SENSOR_ROTATION_VECTOR,             // 11
    ANDROID_SENSOR_HUMIDITY,                    // 12
    ANDROID_SENSOR_AMBIENT_TEMPERATURE,         // 13
    ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, // 14
    ANDROID_SENSOR_GAME_ROTATION_VECTOR,        // 15
    ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,      // 16
    ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,   // 17
    ANDROID_SENSOR_STEP_DETECTOR,               // 18
    ANDROID_SENSOR_STEP_COUNTER,                // 19
    ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, // 20
    ANDROID_SENSOR_HEART_RATE,                  // 21
    ANDROID_SENSOR_PROXIMITY,                   // 22

    ANDROID_SENSOR_WAKEUP_ACCELEROMETER,               // 23
    ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,              // 24
    ANDROID_SENSOR_WAKEUP_ORIENTATION,                 // 25
    ANDROID_SENSOR_WAKEUP_GYROSCOPE,                   // 26
    ANDROID_SENSOR_WAKEUP_LIGHT,                       // 27
    ANDROID_SENSOR_WAKEUP_PRESSURE,                    // 28
    ANDROID_SENSOR_WAKEUP_GRAVITY,                     // 29
    ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,         // 30
    ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,             // 31
    ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,           // 32
    ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,         // 33
    ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED, // 34
    ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,        // 35
    ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,      // 36
    ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,               // 37
    ANDROID_SENSOR_WAKEUP_STEP_COUNTER,                // 38
    ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR, // 39
    ANDROID_SENSOR_WAKEUP_HEART_RATE,                  // 40
    ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,               // 41
    ANDROID_SENSOR_RAW_ACCELEROMETER,                  // 42
    ANDROID_SENSOR_RAW_GYROSCOPE,                      // 43
    ANDROID_SENSOR_NUM_MAX,                            // 44

    ANDROID_SENSOR_B2S,                    // 45
    ANDROID_SENSOR_FLIP_PICKUP,            // 46
    ANDROID_SENSOR_ACTIVITY_CLASSIFICATON, // 47
    ANDROID_SENSOR_SCREEN_ROTATION,        // 48
    SELF_TEST,                             // 49
    SETUP,                                 // 50
    GENERAL_SENSORS_MAX                    // 51
  };

// Determines which base sensor needs to be on based upon ANDROID_SENSORS 0-31
#define INV_NEEDS_ACCEL_MASK ((1L << 1) | (1L << 3) | (1L << 9) | (1L << 10) | (1L << 11) | (1L << 15) | (1L << 17) | (1L << 18) | (1L << 19) | (1L << 20) | (1L << 23) | (1L << 25) | (1L << 29) | (1L << 30) | (1L << 31))
#define INV_NEEDS_GYRO_MASK ((1L << 3) | (1L << 4) | (1L << 9) | (1L << 10) | (1L << 11) | (1L << 15) | (1L << 16) | (1L << 25) | (1L << 26) | (1L << 29) | (1L << 30) | (1L << 31))
#define INV_NEEDS_COMPASS_MASK ((1L << 2) | (1L << 3) | (1L << 11) | (1L << 14) | (1L << 20) | (1L << 24) | (1L << 25) | (1L << 31))
#define INV_NEEDS_PRESSURE ((1L << 6) | (1L << 28))

// Determines which base sensor needs to be on based upon ANDROID_SENSORS 32-
#define INV_NEEDS_ACCEL_MASK1 ((1L << 3) | (1L << 5) | (1L << 6) | (1L << 7) | (1L << 9) | (1L << 10)) // I.e. 35, 37, 38, 39, 41, 42
#define INV_NEEDS_GYRO_MASK1 ((1L << 3) | (1L << 4) | (1L << 11)) // I.e. 35, 36, 43
#define INV_NEEDS_COMPASS_MASK1 ((1L << 2) | (1L << 7)) // I.e. 34 and 39

  enum DMP_Data_Ready_Status_Register_Bits
  {
    DMP_Data_ready_Gyro = 0x0001,             // Gyro samples available
    DMP_Data_ready_Accel = 0x0002,            // Accel samples available
    DMP_Data_ready_Secondary_Compass = 0x0008 // Secondary compass samples available
  };

  enum DMP_Data_Output_Control_1_Register_Bits
  {
    DMP_Data_Output_Control_1_Step_Ind_0 = 0x0001,     // Pedometer Step Indicator Bit 0
    DMP_Data_Output_Control_1_Step_Ind_1 = 0x0002,     // Pedometer Step Indicator Bit 1
    DMP_Data_Output_Control_1_Step_Ind_2 = 0x0004,     // Pedometer Step Indicator Bit 2
    DMP_Data_Output_Control_1_Header2 = 0x0008,        // Header 2
    DMP_Data_Output_Control_1_Step_Detector = 0x0010,  // Pedometer Step Detector
    DMP_Data_Output_Control_1_Compass_Calibr = 0x0020, // 32-bit calibrated compass
    DMP_Data_Output_Control_1_Gyro_Calibr = 0x0040,    // 32-bit calibrated gyro
    DMP_Data_Output_Control_1_Pressure = 0x0080,       // 16-bit Pressure
    DMP_Data_Output_Control_1_Geomag = 0x0100,         // 32-bit Geomag rv + heading accuracy
    DMP_Data_Output_Control_1_PQuat6 = 0x0200,         // 16-bit pedometer quaternion
    DMP_Data_Output_Control_1_Quat9 = 0x0400,          // 32-bit 9-axis quaternion + heading accuracy
    DMP_Data_Output_Control_1_Quat6 = 0x0800,          // 32-bit 6-axis quaternion
    DMP_Data_Output_Control_1_ALS = 0x1000,            // 16-bit ALS
    DMP_Data_Output_Control_1_Compass = 0x2000,        // 16-bit compass
    DMP_Data_Output_Control_1_Gyro = 0x4000,           // 16-bit gyro
    DMP_Data_Output_Control_1_Accel = 0x8000           // 16-bit accel
  };

  enum DMP_Data_Output_Control_2_Register_Bits
  {
    DMP_Data_Output_Control_2_Secondary_On_Off = 0x0040,
    DMP_Data_Output_Control_2_Activity_Recognition_BAC = 0x0080,
    DMP_Data_Output_Control_2_Batch_Mode_Enable = 0x0100,
    DMP_Data_Output_Control_2_Pickup = 0x0400,
    DMP_Data_Output_Control_2_Fsync_Detection = 0x0800,
    DMP_Data_Output_Control_2_Compass_Accuracy = 0x1000,
    DMP_Data_Output_Control_2_Gyro_Accuracy = 0x2000,
    DMP_Data_Output_Control_2_Accel_Accuracy = 0x4000
  };

  enum DMP_Motion_Event_Control_Register_Bits
  {
    DMP_Motion_Event_Control_Activity_Recog_Pedom_Accel = 0x0002, // Activity Recognition / Pedometer accel only
    DMP_Motion_Event_Control_Bring_Look_To_See = 0x0004,
    DMP_Motion_Event_Control_Geomag = 0x0008, // Geomag rv
    DMP_Motion_Event_Control_Pickup = 0x0010,
    DMP_Motion_Event_Control_BTS = 0x0020,
    DMP_Motion_Event_Control_9axis = 0x0040,
    DMP_Motion_Event_Control_Compass_Calibr = 0x0080,
    DMP_Motion_Event_Control_Gyro_Calibr = 0x0100,
    DMP_Motion_Event_Control_Accel_Calibr = 0x0200,
    DMP_Motion_Event_Control_Significant_Motion_Det = 0x0800,
    DMP_Motion_Event_Control_Tilt_Interrupt = 0x1000,
    DMP_Motion_Event_Control_Pedometer_Interrupt = 0x2000,
    DMP_Motion_Event_Control_Activity_Recog_Pedom = 0x4000,
    DMP_Motion_Event_Control_BAC_Wearable = 0x8000
  };

  enum DMP_Header_Bitmap
  {
    DMP_header_bitmap_Header2 = 0x0008,
    DMP_header_bitmap_Step_Detector = 0x0010,
    DMP_header_bitmap_Compass_Calibr = 0x0020,
    DMP_header_bitmap_Gyro_Calibr = 0x0040,
    DMP_header_bitmap_Pressure = 0x0080,
    DMP_header_bitmap_Geomag = 0x0100,
    DMP_header_bitmap_PQuat6 = 0x0200,
    DMP_header_bitmap_Quat9 = 0x0400,
    DMP_header_bitmap_Quat6 = 0x0800,
    DMP_header_bitmap_ALS = 0x1000,
    DMP_header_bitmap_Compass = 0x2000,
    DMP_header_bitmap_Gyro = 0x4000,
    DMP_header_bitmap_Accel = 0x8000
  };

  enum DMP_Header2_Bitmap
  {
    DMP_header2_bitmap_Secondary_On_Off = 0x0040,
    DMP_header2_bitmap_Activity_Recog = 0x0080,
    DMP_header2_bitmap_Pickup = 0x0400,
    DMP_header2_bitmap_Fsync = 0x0800,
    DMP_header2_bitmap_Compass_Accuracy = 0x1000,
    DMP_header2_bitmap_Gyro_Accuracy = 0x2000,
    DMP_header2_bitmap_Accel_Accuracy = 0x4000
  };

  typedef struct // DMP Activity Recognition data
  {
    uint8_t Drive : 1;
    uint8_t Walk : 1;
    uint8_t Run : 1;
    uint8_t Bike : 1;
    uint8_t Tilt : 1;
    uint8_t Still : 1;
    uint8_t reserved : 2;
  } icm_20948_DMP_Activity_t;

  typedef struct // DMP Secondary On/Off data
  {
    uint16_t Gyro_Off : 1;
    uint16_t Gyro_On : 1;
    uint16_t Compass_Off : 1;
    uint16_t Compass_On : 1;
    uint16_t Proximity_Off : 1;
    uint16_t Proximity_On : 1;
    uint16_t reserved : 10;
  } icm_20948_DMP_Secondary_On_Off_t;

#define icm_20948_DMP_Header_Bytes 2
#define icm_20948_DMP_Header2_Bytes 2
#define icm_20948_DMP_Raw_Accel_Bytes 6
#define icm_20948_DMP_Raw_Gyro_Bytes 6
#define icm_20948_DMP_Gyro_Bias_Bytes 6
#define icm_20948_DMP_Compass_Bytes 6
#define icm_20948_DMP_ALS_Bytes 8
#define icm_20948_DMP_Quat6_Bytes 12
#define icm_20948_DMP_Quat9_Bytes 14
// <-- lcm20948MPUFifoControl.c suggests icm_20948_DMP_Step_Detector_Bytes comes here <--
#define icm_20948_DMP_PQuat6_Bytes 6
#define icm_20948_DMP_Geomag_Bytes 14
#define icm_20948_DMP_Pressure_Bytes 6
#define icm_20948_DMP_Gyro_Calibr_Bytes 12 // lcm20948MPUFifoControl.c suggests icm_20948_DMP_Gyro_Calibr_Bytes is not supported?
#define icm_20948_DMP_Compass_Calibr_Bytes 12
#define icm_20948_DMP_Step_Detector_Bytes 4 // See note above
#define icm_20948_DMP_Accel_Accuracy_Bytes 2
#define icm_20948_DMP_Gyro_Accuracy_Bytes 2
#define icm_20948_DMP_Compass_Accuracy_Bytes 2
#define icm_20948_DMP_Fsync_Detection_Bytes 2 // lcm20948MPUFifoControl.c suggests icm_20948_DMP_Fsync_Detection_Bytes is not supported?
#define icm_20948_DMP_Pickup_Bytes 2
#define icm_20948_DMP_Activity_Recognition_Bytes 6
#define icm_20948_DMP_Secondary_On_Off_Bytes 2
#define icm_20948_DMP_Footer_Bytes 2
#define icm_20948_DMP_Maximum_Bytes 14 // The most bytes we will attempt to read from the FIFO in one go

  typedef struct
  {
    uint16_t header;
    uint16_t header2;
    union
    {
      uint8_t Bytes[icm_20948_DMP_Raw_Accel_Bytes];
      struct
      {
        int16_t X;
        int16_t Y;
        int16_t Z;
      } Data;
    } Raw_Accel;
    union
    {
      uint8_t Bytes[icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes];
      struct
      {
        int16_t X;
        int16_t Y;
        int16_t Z;
        int16_t BiasX;
        int16_t BiasY;
        int16_t BiasZ;
      } Data;
    } Raw_Gyro;
    union
    {
      uint8_t Bytes[icm_20948_DMP_Compass_Bytes];
      struct
      {
        int16_t X;
        int16_t Y;
        int16_t Z;
      } Data;
    } Compass;
    uint8_t ALS[icm_20948_DMP_ALS_Bytes]; // Byte[0]: Dummy, Byte[2:1]: Ch0DATA, Byte[4:3]: Ch1DATA, Byte[6:5]: PDATA, Byte[7]: Dummy
    // The 6-Axis and 9-axis Quaternion outputs each consist of 12 bytes of data.
    // These 12 bytes in turn consists of three 4-byte elements.
    // 9-axis quaternion data and Geomag rv is always followed by 2-bytes of heading accuracy, hence the size of Quat9 and Geomag data size in the FIFO is 14 bytes.
    // Quaternion data for both cases is cumulative/integrated values.
    // For a given quaternion Q, the ordering of its elements is {Q1, Q2, Q3}.
    // Each element is represented using Big Endian byte order.
    // Q0 value is computed from this equation: Q20 + Q21 + Q22 + Q23 = 1.
    // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
    // The quaternion data is scaled by 2^30.
    union
    {
      uint8_t Bytes[icm_20948_DMP_Quat6_Bytes];
      struct
      {
        int32_t Q1;
        int32_t Q2;
        int32_t Q3;
      } Data;
    } Quat6;
    union
    {
      uint8_t Bytes[icm_20948_DMP_Quat9_Bytes];
      struct
      {
        int32_t Q1;
        int32_t Q2;
        int32_t Q3;
        int16_t Accuracy;
      } Data;
    } Quat9;
    union
    {
      uint8_t Bytes[icm_20948_DMP_PQuat6_Bytes];
      struct
      {
        int16_t Q1;
        int16_t Q2;
        int16_t Q3;
      } Data;
    } PQuat6;
    union
    {
      uint8_t Bytes[icm_20948_DMP_Geomag_Bytes];
      struct
      {
        int32_t Q1;
        int32_t Q2;
        int32_t Q3;
        int16_t Accuracy;
      } Data;
    } Geomag;
    uint8_t Pressure[6]; // Byte [2:0]: Pressure data, Byte [5:3]: Temperature data
    union
    {
      uint8_t Bytes[icm_20948_DMP_Gyro_Calibr_Bytes];
      struct
      {
        int32_t X;
        int32_t Y;
        int32_t Z;
      } Data;
    } Gyro_Calibr; // Hardware unit scaled by 2^15
    union
    {
      uint8_t Bytes[icm_20948_DMP_Compass_Calibr_Bytes];
      struct
      {
        int32_t X;
        int32_t Y;
        int32_t Z;
      } Data;
    } Compass_Calibr;             // The unit is uT scaled by 2^16
    uint32_t Pedometer_Timestamp; // Timestamp as DMP cycle
    uint16_t Accel_Accuracy;      // The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
    uint16_t Gyro_Accuracy;       // The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
    uint16_t Compass_Accuracy;    // The accuracy is expressed as 0~3. The lowest is 0 and 3 is the highest.
    uint16_t Fsync_Delay_Time;    // The data is delay time between Fsync event and the 1st ODR event after Fsync event.
    uint16_t Pickup;              // The value “2” indicates pick up is detected.
    // Activity Recognition data
    // The data include Start and End states, and timestamp as DMP cycle.
    // Byte [0]: State-Start, Byte [1]: State-End, Byte [5:2]: timestamp.
    // The states are expressed as below.
    // Drive: 0x01
    // Walk: 0x02
    // Run: 0x04
    // Bike: 0x08
    // Tilt: 0x10
    // Still: 0x20
    union
    {
      uint8_t Bytes[icm_20948_DMP_Activity_Recognition_Bytes];
      struct
      {
        icm_20948_DMP_Activity_t State_Start;
        icm_20948_DMP_Activity_t State_End;
        uint32_t Timestamp;
      } Data;
    } Activity_Recognition;
    // Secondary On/Off data
    // BAC algorithm requires sensors on/off through FIFO data to detect activities effectively and save power.
    // The driver is expected to control sensors accordingly.
    // The data indicates which sensor and on or off as below.
    // Gyro Off: 0x01
    // Gyro On: 0x02
    // Compass Off: 0x04
    // Compass On: 0x08
    // Proximity Off: 0x10
    // Proximity On: 0x20
    union
    {
      uint8_t Bytes[icm_20948_DMP_Secondary_On_Off_Bytes];
      icm_20948_DMP_Secondary_On_Off_t Sensors;
    } Secondary_On_Off;
    uint16_t Footer; // Gyro count?
  } icm_20948_DMP_data_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ICM_20948_REGISTERS_H_ */
