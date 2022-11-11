/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 S.J. Remington & SlimeVR contributors

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
#include "icm20948sensor.h"
#include "calibration.h"
#include <i2cscan.h>
#include "network/network.h"
#include "GlobalVars.h"

// seconds after previous save (from start) when calibration (DMP Bias) data will be saved to NVS. Increments through the list then stops; to prevent unwelcome eeprom wear.
int bias_save_periods[] = { 120, 180, 300, 600, 600 }; // 2min + 3min + 5min + 10min + 10min (no more saves after 30min)

#define ACCEL_SENSITIVITY_4G 8192.0f

// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE_4G = ((32768. / ACCEL_SENSITIVITY_4G) / 32768.) * EARTH_GRAVITY;

void ICM20948Sensor::motionSetup()
{
    connectSensor();
    startDMP();
    loadCalibration();
    startMotionLoop();
    startCalibrationAutoSave();
}

void ICM20948Sensor::motionLoop()
{
#if ENABLE_INSPECTION
    {
        (void)imu.getAGMT();

        float rX = imu.gyrX();
        float rY = imu.gyrY();
        float rZ = imu.gyrZ();

        float aX = imu.accX();
        float aY = imu.accY();
        float aZ = imu.accZ();

        float mX = imu.magX();
        float mY = imu.magY();
        float mZ = imu.magZ();

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, mX, mY, mZ, 255);
    }
#endif

    timer.tick();

    readFIFOToEnd();
    readRotation();
    checkSensorTimeout();
}

void ICM20948Sensor::readFIFOToEnd()
{
    ICM_20948_Status_e readStatus = imu.readDMPdataFromFIFO(&dmpDataTemp);

    #ifdef DEBUG_SENSOR
    {
        m_Logger.trace("e0x%02x", readStatus);
    }
    #endif

    if(readStatus == ICM_20948_Stat_Ok)
    {
        dmpData = dmpDataTemp;
        readFIFOToEnd();
    }
}

void ICM20948Sensor::sendData()
{
    if(newData && lastDataSent + 7 < millis())
    {
        lastDataSent = millis();
        newData = false;

        #if(USE_6_AXIS)
        {
            Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, 0, sensorId);
        }
        #else
        {
            Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, dmpData.Quat9.Data.Accuracy, sensorId);
        }
        #endif

        #if SEND_ACCELERATION
        {
            Network::sendAccel(acceleration, sensorId);
        }
        #endif
    }
}

void ICM20948Sensor::startCalibration(int calibrationType)
{
    // 20948 does continuous calibration
    saveCalibration(false);
}

void ICM20948Sensor::startCalibrationAutoSave()
{
    #if SAVE_BIAS
    timer.in(bias_save_periods[0] * 1000, [](void *arg) -> bool { ((ICM20948Sensor*)arg)->saveCalibration(true); return false; }, this);
    #endif
}

void ICM20948Sensor::startDMP()
{
    if(imu.initializeDMP() == ICM_20948_Stat_Ok)
    {
        m_Logger.debug("DMP initialized");
    }
    else
    {
       m_Logger.fatal("Failed to initialize DMP");
        return;
    }

    #if(USE_6_AXIS)
    {
        m_Logger.debug("Using 6 axis configuration");
        if(imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok)
        {
            m_Logger.debug("Enabled DMP sensor for game rotation vector");
        }
        else
        {
            m_Logger.fatal("Failed to enable DMP sensor for game rotation vector");
            return;
        }
    }
    #else
    {
        m_Logger.debug("Using 9 axis configuration");
        if(imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok)
        {
            m_Logger.debug("Enabled DMP sensor for sensor orientation");
        }
        else
        {
            m_Logger.fatal("Failed to enable DMP sensor orientation");
            return;
        }
    }
    #endif

    #if(SEND_ACCELERATION)
    if (imu.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok)
    {
        m_Logger.debug("Enabled DMP sensor for accelerometer");
    }
    else
    {
        m_Logger.fatal("Failed to enable DMP sensor for accelerometer");
        return;
    }
    #endif

    // Might need to set up other DMP functions later, just Quad6/Quad9/Accel for now

    #if(USE_6_AXIS)
    {
        if(imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok)
        {
            m_Logger.debug("Set Quat6 to 100Hz frequency");
        }
        else
        {
           m_Logger.fatal("Failed to set Quat6 to 100Hz frequency");
            return;
        }
    }
    #else
    {
        if(imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok)
        {
            m_Logger.debug("Set Quat9 to 100Hz frequency");
        }
        else
        {
           m_Logger.fatal("Failed to set Quat9 to 100Hz frequency");
            return;
        }
    }
    #endif

    #if(SEND_ACCELERATION)
    if (this->imu.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok)
    {
        this->m_Logger.debug("Set Accel to 100Hz frequency");
    }
    else
    {
        this->m_Logger.fatal("Failed to set Accel to 100Hz frequency");
        return;
    }
    #endif

    // Enable the FIFO
    if(imu.enableFIFO() == ICM_20948_Stat_Ok)
    {
        m_Logger.debug("FIFO Enabled");
    }
    else
    {
       m_Logger.fatal("Failed to enable FIFO");
        return;
    }

    // Enable the DMP
    if(imu.enableDMP() == ICM_20948_Stat_Ok)
    {
        m_Logger.debug("DMP Enabled");
    }
    else
    {
       m_Logger.fatal("Failed to enable DMP");
        return;
    }

    // Reset DMP
    if(imu.resetDMP() == ICM_20948_Stat_Ok)
    {
        m_Logger.debug("Reset DMP");
    }
    else
    {
       m_Logger.fatal("Failed to reset DMP");
        return;
    }

    // Reset FIFO
    if(imu.resetFIFO() == ICM_20948_Stat_Ok)
    {
        m_Logger.debug("Reset FIFO");
    }
    else
    {
       m_Logger.fatal("Failed to reset FIFO");
        return;
    }
}

void ICM20948Sensor::connectSensor()
{
    #ifdef DEBUG_SENSOR
        imu.enableDebugging(Serial);
    #endif
    // SparkFun_ICM-20948_ArduinoLibrary only supports 0x68 or 0x69 via boolean, if something else throw a error
    boolean isOnSecondAddress = false;

    if (addr == 0x68) {
        isOnSecondAddress = false;
    } else if (addr == 0x69){
        isOnSecondAddress = true;
    } else {
        m_Logger.fatal("I2C Address not supported by ICM20948 library: 0x%02x", addr);
        return;
    }

    ICM_20948_Status_e imu_err = imu.begin(Wire, isOnSecondAddress);
    if (imu_err != ICM_20948_Stat_Ok) {
        m_Logger.fatal("Can't connect to ICM20948 at address 0x%02x, error code: 0x%02x", addr, imu_err);
        ledManager.pattern(50, 50, 200);
        return;
    }
}

void ICM20948Sensor::startMotionLoop()
{
    lastData = millis();
    working = true;
}

void ICM20948Sensor::checkSensorTimeout()
{
    if(lastData + 1000 < millis()) {
        working = false;
        lastData = millis();
        m_Logger.error("Sensor timeout I2C Address 0x%02x", addr);
        Network::sendError(1, this->sensorId);
    }
}

void ICM20948Sensor::readRotation()
{
    #if(USE_6_AXIS)
    {
        if ((dmpData.header & DMP_header_bitmap_Quat6) > 0)
        {
            // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
            // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
            // The quaternion data is scaled by 2^30.
            // Scale to +/- 1
            double q1 = ((double)dmpData.Quat6.Data.Q1) / DMPNUMBERTODOUBLECONVERTER; // Convert to double. Divide by 2^30
            double q2 = ((double)dmpData.Quat6.Data.Q2) / DMPNUMBERTODOUBLECONVERTER; // Convert to double. Divide by 2^30
            double q3 = ((double)dmpData.Quat6.Data.Q3) / DMPNUMBERTODOUBLECONVERTER; // Convert to double. Divide by 2^30
            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
            quaternion.w = q0;
            quaternion.x = q1;
            quaternion.y = q2;
            quaternion.z = q3;

            #if SEND_ACCELERATION
            calculateAccelerationWithoutGravity(&quaternion);
            #endif

            quaternion *= sensorOffset; //imu rotation

            #if ENABLE_INSPECTION
            {
                Network::sendInspectionFusedIMUData(sensorId, quaternion);
            }
            #endif

            newData = true;
            lastData = millis();
        }
    }
    #else
    {
        if((dmpData.header & DMP_header_bitmap_Quat9) > 0)
        {
            // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
            // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
            // The quaternion data is scaled by 2^30.
            // Scale to +/- 1
            double q1 = ((double)dmpData.Quat9.Data.Q1) / DMPNUMBERTODOUBLECONVERTER; // Convert to double. Divide by 2^30
            double q2 = ((double)dmpData.Quat9.Data.Q2) / DMPNUMBERTODOUBLECONVERTER; // Convert to double. Divide by 2^30
            double q3 = ((double)dmpData.Quat9.Data.Q3) / DMPNUMBERTODOUBLECONVERTER; // Convert to double. Divide by 2^30
            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
            quaternion.w = q0;
            quaternion.x = q1;
            quaternion.y = q2;
            quaternion.z = q3;

            #if SEND_ACCELERATION
            calculateAccelerationWithoutGravity(&quaternion);
            #endif

            quaternion *= sensorOffset; //imu rotation

            #if ENABLE_INSPECTION
            {
                Network::sendInspectionFusedIMUData(sensorId, quaternion);
            }
            #endif

            newData = true;
            lastData = millis();
        }
    }
    #endif
}

void ICM20948Sensor::saveCalibration(bool repeat)
{
    #if(!SAVE_BIAS)
    {
        return;
    }
    #endif
    #ifdef DEBUG_SENSOR
    m_Logger.trace("Saving Bias");
    #endif

    imu.GetBiasGyroX(&m_Calibration.G[0]);
    imu.GetBiasGyroY(&m_Calibration.G[1]);
    imu.GetBiasGyroZ(&m_Calibration.G[2]);

    imu.GetBiasAccelX(&m_Calibration.A[0]);
    imu.GetBiasAccelY(&m_Calibration.A[1]);
    imu.GetBiasAccelZ(&m_Calibration.A[2]);

    #if !USE_6_AXIS
    imu.GetBiasCPassX(&m_Calibration.C[0]);
    imu.GetBiasCPassY(&m_Calibration.C[1]);
    imu.GetBiasCPassZ(&m_Calibration.C[2]);
    #endif

    #ifdef DEBUG_SENSOR
    m_Logger.trace("Gyrometer bias:     [%d, %d, %d]", UNPACK_VECTOR_ARRAY(m_Calibration.G));
    m_Logger.trace("Accelerometer bias: [%d, %d, %d]", UNPACK_VECTOR_ARRAY(m_Calibration.A));
        #if !USE_6_AXIS
    m_Logger.trace("Compass bias:       [%d, %d, %d]", UNPACK_VECTOR_ARRAY(m_Calibration.C));
        #endif
    #endif

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::ICM20948;
    calibration.data.icm20948 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    if (repeat) {
        bias_save_counter++;
        // Possible: Could make it repeat the final timer value if any of the biases are still 0. Save strategy could be improved.
        if (sizeof(bias_save_periods) != bias_save_counter) {
            timer.in(
                bias_save_periods[bias_save_counter] * 1000,
                [](void* arg) -> bool {
                    ((ICM20948Sensor*)arg)->saveCalibration(true);
                    return false;
                },
                this);
        }
    }
}

void ICM20948Sensor::loadCalibration()
{
    #if(!LOAD_BIAS)
    {
        return;
    }
    #endif

    SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
    // If no compatible calibration data is found, the calibration data will just be zero-ed out
    switch (sensorCalibration.type) {
    case SlimeVR::Configuration::CalibrationConfigType::ICM20948:
        m_Calibration = sensorCalibration.data.icm20948;
        break;

    case SlimeVR::Configuration::CalibrationConfigType::NONE:
        m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
        m_Logger.info("Calibration is advised");
        break;

    default:
        m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
        m_Logger.info("Calibration is advised");
    }

    #ifdef DEBUG_SENSOR
    m_Logger.trace("Gyrometer bias:     [%d, %d, %d]", UNPACK_VECTOR_ARRAY(m_Calibration.G));
    m_Logger.trace("Accelerometer bias: [%d, %d, %d]", UNPACK_VECTOR_ARRAY(m_Calibration.A));
    #if !USE_6_AXIS
    m_Logger.trace("Compass bias:       [%d, %d, %d]", UNPACK_VECTOR_ARRAY(m_Calibration.C));
    #endif
    #endif

    imu.SetBiasGyroX(m_Calibration.G[0]);
    imu.SetBiasGyroY(m_Calibration.G[1]);
    imu.SetBiasGyroZ(m_Calibration.G[2]);

    imu.SetBiasAccelX(m_Calibration.A[0]);
    imu.SetBiasAccelY(m_Calibration.A[1]);
    imu.SetBiasAccelZ(m_Calibration.A[2]);

    #if !USE_6_AXIS
    imu.SetBiasCPassX(m_Calibration.C[0]);
    imu.SetBiasCPassY(m_Calibration.C[1]);
    imu.SetBiasCPassZ(m_Calibration.C[2]);
    #endif
}

void ICM20948Sensor::calculateAccelerationWithoutGravity(Quat *quaternion)
{
    #if SEND_ACCELERATION
    {
        if((dmpData.header & DMP_header_bitmap_Accel) > 0)
        {
            this->acceleration[0] = (float)this->dmpData.Raw_Accel.Data.X;
            this->acceleration[1] = (float)this->dmpData.Raw_Accel.Data.Y;
            this->acceleration[2] = (float)this->dmpData.Raw_Accel.Data.Z;

            // get the component of the acceleration that is gravity
            float gravity[3];
            gravity[0] = 2 * ((-quaternion->x) * (-quaternion->z) - quaternion->w * quaternion->y);
            gravity[1] = -2 * (quaternion->w * (-quaternion->x) + quaternion->y * (-quaternion->z));
            gravity[2] = quaternion->w * quaternion->w - quaternion->x * quaternion->x - quaternion->y * quaternion->y + quaternion->z * quaternion->z;

            // subtract gravity from the acceleration vector
            this->acceleration[0] -= gravity[0] * ACCEL_SENSITIVITY_4G;
            this->acceleration[1] -= gravity[1] * ACCEL_SENSITIVITY_4G;
            this->acceleration[2] -= gravity[2] * ACCEL_SENSITIVITY_4G;

            // finally scale the acceleration values to mps2
            this->acceleration[0] *= ASCALE_4G;
            this->acceleration[1] *= ASCALE_4G;
            this->acceleration[2] *= ASCALE_4G;
        }
    }
    #endif
}

//You need to override the library's initializeDMP to change some settings
#if OVERRIDEDMPSETUP
// initializeDMP is a weak function. Let's overwrite it so we can increase the sample rate
ICM_20948_Status_e ICM_20948::initializeDMP(void)
{
    // First, let's check if the DMP is available
    if (_device._dmp_firmware_available != true)
    {
    debugPrint(F("ICM_20948::startupDMP: DMP is not available. Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    return ICM_20948_Stat_DMPNotSupported;
    }

    ICM_20948_Status_e  worstResult = ICM_20948_Stat_Ok;

    #if defined(ICM_20948_USE_DMP)

    // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
    // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

    ICM_20948_Status_e  result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful

    // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
    // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
    // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
    // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
    //
    // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
    // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
    // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
    // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
    // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
    //
    // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
    // 0: use I2C_SLV0
    // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
    // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
    // 10: we read 10 bytes each cycle
    // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
    result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true); if (result > worstResult) worstResult = result;
    //
    // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
    // 1: use I2C_SLV1
    // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
    // AK09916_REG_CNTL2: we start writing here (0x31)
    // 1: not sure why, but the write does not happen if this is set to zero
    // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
    // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
    // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
    result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single); if (result > worstResult) worstResult = result;

    // Set the I2C Master ODR configuration
    // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
    // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
    //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
    //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
    //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
    // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
    // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
    result = setBank(3); if (result > worstResult) worstResult = result; // Select Bank 3
    uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
    result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register

    // Configure clock source through PWR_MGMT_1
    // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    result = setClockSource(ICM_20948_Clock_Auto); if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

    // Enable accel and gyro sensors through PWR_MGMT_2
    // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
    result = setBank(0); if (result > worstResult) worstResult = result;                               // Select Bank 0
    uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
    result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register

    // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
    // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
    result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;

    // Disable the FIFO
    result = enableFIFO(false); if (result > worstResult) worstResult = result;

    // Disable the DMP
    result = enableDMP(false); if (result > worstResult) worstResult = result;

    // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
    // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
    myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                            // gpm2
                            // gpm4
                            // gpm8
                            // gpm16
    myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                            // dps250
                            // dps500
                            // dps1000
                            // dps2000
    result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;

    // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
    // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
    // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
    result = enableDLPF(ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;

    // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
    // If we see this interrupt, we'll need to reset the FIFO
    //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

    // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
    // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
    result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
    uint8_t zero = 0;
    result = write(AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
    // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
    result = write(AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;

    // Turn off data ready interrupt through INT_ENABLE_1
    result = intEnableRawDataReady(false); if (result > worstResult) worstResult = result;

    // Reset FIFO through FIFO_RST
    result = resetFIFO(); if (result > worstResult) worstResult = result;

    // Set gyro sample rate divider with GYRO_SMPLRT_DIV
    // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
    ICM_20948_smplrt_t mySmplrt;
    //mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
    //mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
    mySmplrt.g = 4; // 225Hz
    mySmplrt.a = 4; // 225Hz
    // mySmplrt.g = 8; // 112Hz
    // mySmplrt.a = 8; // 112Hz
    result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;

    // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
    result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

    // Now load the DMP firmware
    result = loadDMPFirmware(); if (result > worstResult) worstResult = result;

    // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
    result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

    // Set the Hardware Fix Disable register to 0x48
    result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
    uint8_t fix = 0x48;
    result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

    // Set the Single FIFO Priority Select register to 0xE4
    result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
    uint8_t fifoPrio = 0xE4;
    result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

    // Configure Accel scaling to DMP
    // The DMP scales accel raw data internally to align 1g as 2^25
    // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
    const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
    result = writeDMPmems(ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
    // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
    const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
    result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

    // Configure Compass mount matrix and scale to DMP
    // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
    // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
    // Each compass axis will be converted as below:
    // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
    // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
    // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
    // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
    // 2^30 / 6.66666 = 161061273 = 0x9999999
    const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
    const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

    // Configure the B2S Mounting Matrix
    const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

    // Configure the DMP Gyro Scaling Factor
    // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
    //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
    //            10=102.2727Hz sample rate, ... etc.
    // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
    //result = setGyroSF(19, 3); if (result > worstResult) worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)
    result = setGyroSF(4, 3); if (result > worstResult) worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)


    // Configure the Gyro full scale
    // 2000dps : 2^28
    // 1000dps : 2^27
    //  500dps : 2^26
    //  250dps : 2^25
    const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
    result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

    // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
    const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
    //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
    //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
    result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

    // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
    const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
    //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
    //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
    result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

    // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
    const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
    //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
    //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
    result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

    // Configure the Accel Cal Rate
    const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

    // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
    // Let's set the Compass Time Buffer to 69 (Hz).
    const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
    result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;

    // Enable DMP interrupt
    // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
    //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

#endif

  return worstResult;
}
#endif // OVERRIDEDMPSETUP
