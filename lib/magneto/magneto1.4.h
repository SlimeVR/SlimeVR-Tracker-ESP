#ifndef __MAGENTO_1_4__
#define __MAGENTO_1_4__

// In https://github.com/jremington/MPU-9250-AHRS
// magneto 1.4 magnetometer/accelerometer calibration code
// from http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// tested and works with Code::Blocks 10.05 through 20.03

// Command line version slightly modified from original, sjames_remington@gmail.com
// Now includes option to reject outliers in units of sigma, deviation of data vector length
// from mean values of the sample. Suggest using 2 as the rejection criterion

// comma separated ASCII input data file expected, three columns x, y, z

void CalculateCalibration(float *buf, int sampleCount, float BAinv[4][3]);

#endif // __MAGENTO_1_4__