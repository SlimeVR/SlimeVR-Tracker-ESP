#ifndef MPU6050_NODMP_CONFIG_H
#define MPU6050_NODMP_CONFIG_H

// !!! No-DMP MPU6050 Config !!!

// Calibration method choice: Smooth rotation or keeping the tracker still in 6 orientations
// (for details refer to defines_bmi160.h)
#define MPU6050_ACCEL_CALIBRATION_METHOD ACCEL_CALIBRATION_METHOD_6POINT

// Whether to calibrate accelerometer or not
// Setting to false will skip this step during calibration
// (old calibration values will be preserved)
#define MPU6050_CALIBRATE_ACCEL true

// Whether to calibrate gyro or not
// Setting to false will skip this step during calibration
// (old calibration values will be preserved)
#define MPU6050_CALIBRATE_GYRO true

// Whether to use Temperature Calibration
// (disabling doesn't erase old calibration values)
#define MPU6050_USE_TEMPCAL true


#endif
