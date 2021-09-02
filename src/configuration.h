/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifndef _OWO_CONFIG_H_
#define _OWO_CONFIG_H_

struct CalibrationConfig {
    //acel offsets and correction matrix
    float A_B[3];
    float A_Ainv[3][3];
    // mag offsets and correction matrix
    float M_B[3];
    float M_Ainv[3][3];
    //raw offsets, determined for gyro at rest
    float G_off[3];
};

struct DeviceConfig {
    CalibrationConfig calibration;
    int deviceId;
    int deviceMode;
    char wifissid[50];
    char wifipass[50];
};

DeviceConfig * const getConfigPtr();
void setConfig(const DeviceConfig & config);
void saveConfig();

#endif // _OWO_CONFIG_H_