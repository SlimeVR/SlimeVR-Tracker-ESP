#ifndef __MAGENTO_1_4__
#define __MAGENTO_1_4__

/* Copyright (C) 2013 www.sailboatinstruments.blogspot.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// In https://github.com/jremington/MPU-9250-AHRS
// magneto 1.4 magnetometer/accelerometer calibration code
// from http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html

/// @brief Accelerometer/Magnetometer calibration assistant
class MagnetoCalibration {
    public:
        /// @brief Updates calibration with the sample (x, y, z)
        void sample(double x, double y, double z);

        /// @brief Outputs the calibration matrix corresponding to all previous samples
        /// 
        /// This does not consume/destroy the calibration struct, allowing for the possibility of
        /// continuously updating the calibration. That said, there may be limitations on how many
        /// samples are useful to collect, due to the limitations of finite-storage numbers.
        ///
        /// @param BAinv 
        void current_calibration(float BAinv[4][3]);

    private:
        // internal 10x10 matrix representing the 10x<sample count> matrix multiplied by its transpose,
        // resulting in a 10x10 symmetric matrix
        double ata[100] = {0.0};
        double norm_sum = 0.0;
        double sample_count = 0.0;
};

#endif // __MAGENTO_1_4__