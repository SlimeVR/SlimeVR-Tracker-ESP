#ifndef _OWO_CONFIG_H_
#define _OWO_CONFIG_H_

#define LOADING_LED LED_BUILTIN
#define CALIBRATING_LED LED_BUILTIN

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
};

void initializeConfig();
bool hasConfigStored();
void loadConfig(DeviceConfig * cfg);
void saveConfig(DeviceConfig * const cfg);

#endif // _OWO_CONFIG_H_