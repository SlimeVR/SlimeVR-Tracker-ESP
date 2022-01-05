/*

This is a C-compatible interface to the features presented by the ICM 20948 9-axis device
The imementation of the interface is flexible

*/

#ifndef _ICM_20948_C_H_
#define _ICM_20948_C_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ICM_20948_REGISTERS.h"
#include "ICM_20948_ENUMERATIONS.h" // This is to give users access to usable value definiitons
#include "AK09916_ENUMERATIONS.h"
#include "ICM_20948_DMP.h"

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

extern int memcmp(const void *, const void *, size_t); // Avoid compiler warnings

// Define if the DMP will be supported
// Note: you must have 14290/14301 Bytes of program memory available to store the DMP firmware!
#define ICM_20948_USE_DMP // Uncomment this line to enable DMP support. You can of course use ICM_20948_USE_DMP as a compiler flag too

// There are two versions of the InvenSense DMP firmware for the ICM20948 - with slightly different sizes
#define DMP_CODE_SIZE 14301 /* eMD-SmartMotion-ICM20948-1.1.0-MP */
//#define DMP_CODE_SIZE 14290 /* ICM20948_eMD_nucleo_1.0 */


#define ICM_20948_I2C_ADDR_AD0 0x68 // Or 0x69 when AD0 is high
#define ICM_20948_I2C_ADDR_AD1 0x69 //
#define ICM_20948_WHOAMI 0xEA

#define MAG_AK09916_I2C_ADDR 0x0C
#define MAG_AK09916_WHO_AM_I 0x4809
#define MAG_REG_WHO_AM_I 0x00

/** @brief Max size that can be read across I2C or SPI data lines */
#define INV_MAX_SERIAL_READ 16
/** @brief Max size that can be written across I2C or SPI data lines */
#define INV_MAX_SERIAL_WRITE 16

  typedef enum
  {
    ICM_20948_Stat_Ok = 0x00, // The only return code that means all is well
    ICM_20948_Stat_Err,       // A general error
    ICM_20948_Stat_NotImpl,   // Returned by virtual functions that are not implemented
    ICM_20948_Stat_ParamErr,
    ICM_20948_Stat_WrongID,
    ICM_20948_Stat_InvalSensor, // Tried to apply a function to a sensor that does not support it (e.g. DLPF to the temperature sensor)
    ICM_20948_Stat_NoData,
    ICM_20948_Stat_SensorNotSupported,
    ICM_20948_Stat_DMPNotSupported,    // DMP not supported (no #define ICM_20948_USE_DMP)
    ICM_20948_Stat_DMPVerifyFail,      // DMP was written but did not verify correctly
    ICM_20948_Stat_FIFONoDataAvail,    // FIFO contains no data
    ICM_20948_Stat_FIFOIncompleteData, // FIFO contained incomplete data
    ICM_20948_Stat_FIFOMoreDataAvail,  // FIFO contains more data
    ICM_20948_Stat_UnrecognisedDMPHeader,
    ICM_20948_Stat_UnrecognisedDMPHeader2,
    ICM_20948_Stat_InvalDMPRegister, // Invalid DMP Register

    ICM_20948_Stat_NUM,
    ICM_20948_Stat_Unknown,
  } ICM_20948_Status_e;

  typedef enum
  {
    ICM_20948_Internal_Acc = (1 << 0),
    ICM_20948_Internal_Gyr = (1 << 1),
    ICM_20948_Internal_Mag = (1 << 2),
    ICM_20948_Internal_Tmp = (1 << 3),
    ICM_20948_Internal_Mst = (1 << 4), // I2C Master Ineternal
  } ICM_20948_InternalSensorID_bm;     // A bitmask of internal sensor IDs

  typedef union
  {
    int16_t i16bit[3];
    uint8_t u8bit[6];
  } ICM_20948_axis3bit16_t;

  typedef union
  {
    int16_t i16bit;
    uint8_t u8bit[2];
  } ICM_20948_axis1bit16_t;

  typedef struct
  {
    uint8_t a : 2;
    uint8_t g : 2;
    uint8_t reserved_0 : 4;
  } ICM_20948_fss_t; // Holds full-scale settings to be able to extract measurements with units

  typedef struct
  {
    uint8_t a;
    uint8_t g;
  } ICM_20948_dlpcfg_t; // Holds digital low pass filter settings. Members are type ICM_20948_ACCEL_CONFIG_DLPCFG_e

  typedef struct
  {
    uint16_t a;
    uint8_t g;
  } ICM_20948_smplrt_t;

  typedef struct
  {
    uint8_t I2C_MST_INT_EN : 1;
    uint8_t DMP_INT1_EN : 1;
    uint8_t PLL_RDY_EN : 1;
    uint8_t WOM_INT_EN : 1;
    uint8_t REG_WOF_EN : 1;
    uint8_t RAW_DATA_0_RDY_EN : 1;
    uint8_t FIFO_OVERFLOW_EN_4 : 1;
    uint8_t FIFO_OVERFLOW_EN_3 : 1;
    uint8_t FIFO_OVERFLOW_EN_2 : 1;
    uint8_t FIFO_OVERFLOW_EN_1 : 1;
    uint8_t FIFO_OVERFLOW_EN_0 : 1;
    uint8_t FIFO_WM_EN_4 : 1;
    uint8_t FIFO_WM_EN_3 : 1;
    uint8_t FIFO_WM_EN_2 : 1;
    uint8_t FIFO_WM_EN_1 : 1;
    uint8_t FIFO_WM_EN_0 : 1;
  } ICM_20948_INT_enable_t;

  typedef union
  {
    ICM_20948_axis3bit16_t raw;
    struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } axes;
  } ICM_20948_axis3named_t;

  typedef struct
  {
    ICM_20948_axis3named_t acc;
    ICM_20948_axis3named_t gyr;
    ICM_20948_axis3named_t mag;
    union
    {
      ICM_20948_axis1bit16_t raw;
      int16_t val;
    } tmp;
    ICM_20948_fss_t fss; // Full-scale range settings for this measurement
    uint8_t magStat1;
    uint8_t magStat2;
  } ICM_20948_AGMT_t;

  typedef struct
  {
    ICM_20948_Status_e (*write)(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
    ICM_20948_Status_e (*read)(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
    // void				(*delay)(uint32_t ms);
    void *user;
  } ICM_20948_Serif_t;                      // This is the vtable of serial interface functions
  extern const ICM_20948_Serif_t NullSerif; // Here is a default for initialization (NULL)

  typedef struct
  {
    const ICM_20948_Serif_t *_serif; // Pointer to the assigned Serif (Serial Interface) vtable
    bool _dmp_firmware_available;    // Indicates if the DMP firmware has been included. It
    bool _firmware_loaded;           // Indicates if DMP has been loaded
    uint8_t _last_bank;              // Keep track of which bank was selected last - to avoid unnecessary writes
    uint8_t _last_mems_bank;         // Keep track of which bank was selected last - to avoid unnecessary writes
    int32_t _gyroSF;                 // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
    int8_t _gyroSFpll;
    uint32_t _enabled_Android_0;      // Keep track of which Android sensors are enabled: 0-31
    uint32_t _enabled_Android_1;      // Keep track of which Android sensors are enabled: 32-
    uint32_t _enabled_Android_intr_0; // Keep track of which Android sensor interrupts are enabled: 0-31
    uint32_t _enabled_Android_intr_1; // Keep track of which Android sensor interrupts are enabled: 32-
    uint16_t _dataOutCtl1;            // Diagnostics: record the setting of DATA_OUT_CTL1
    uint16_t _dataOutCtl2;            // Diagnostics: record the setting of DATA_OUT_CTL2
    uint16_t _dataRdyStatus;          // Diagnostics: record the setting of DATA_RDY_STATUS
    uint16_t _motionEventCtl;         // Diagnostics: record the setting of MOTION_EVENT_CTL
    uint16_t _dataIntrCtl;            // Diagnostics: record the setting of DATA_INTR_CTL
  } ICM_20948_Device_t;               // Definition of device struct type

  ICM_20948_Status_e ICM_20948_init_struct(ICM_20948_Device_t *pdev); // Initialize ICM_20948_Device_t

  // ICM_20948_Status_e ICM_20948_Startup( ICM_20948_Device_t* pdev ); // For the time being this performs a standardized startup routine

  ICM_20948_Status_e ICM_20948_link_serif(ICM_20948_Device_t *pdev, const ICM_20948_Serif_t *s); // Links a SERIF structure to the device

  // use the device's serif to perform a read or write
  ICM_20948_Status_e ICM_20948_execute_r(ICM_20948_Device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len); // Executes a R or W witht he serif vt as long as the pointers are not null
  ICM_20948_Status_e ICM_20948_execute_w(ICM_20948_Device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len);

  // Single-shot I2C on Master IF
  ICM_20948_Status_e ICM_20948_i2c_controller_periph4_txn(ICM_20948_Device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);
  ICM_20948_Status_e ICM_20948_i2c_master_single_w(ICM_20948_Device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data);
  ICM_20948_Status_e ICM_20948_i2c_master_single_r(ICM_20948_Device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data);

  // Device Level
  ICM_20948_Status_e ICM_20948_set_bank(ICM_20948_Device_t *pdev, uint8_t bank);                                 // Sets the bank
  ICM_20948_Status_e ICM_20948_sw_reset(ICM_20948_Device_t *pdev);                                               // Performs a SW reset
  ICM_20948_Status_e ICM_20948_sleep(ICM_20948_Device_t *pdev, bool on);                                         // Set sleep mode for the chip
  ICM_20948_Status_e ICM_20948_low_power(ICM_20948_Device_t *pdev, bool on);                                     // Set low power mode for the chip
  ICM_20948_Status_e ICM_20948_set_clock_source(ICM_20948_Device_t *pdev, ICM_20948_PWR_MGMT_1_CLKSEL_e source); // Choose clock source
  ICM_20948_Status_e ICM_20948_get_who_am_i(ICM_20948_Device_t *pdev, uint8_t *whoami);                          // Return whoami in out prarmeter
  ICM_20948_Status_e ICM_20948_check_id(ICM_20948_Device_t *pdev);                                               // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI
  ICM_20948_Status_e ICM_20948_data_ready(ICM_20948_Device_t *pdev);                                             // Returns 'Ok' if data is ready

  // Interrupt Configuration
  ICM_20948_Status_e ICM_20948_int_pin_cfg(ICM_20948_Device_t *pdev, ICM_20948_INT_PIN_CFG_t *write, ICM_20948_INT_PIN_CFG_t *read); // Set the INT pin configuration
  ICM_20948_Status_e ICM_20948_int_enable(ICM_20948_Device_t *pdev, ICM_20948_INT_enable_t *write, ICM_20948_INT_enable_t *read);    // Write and or read the interrupt enable information. If non-null the write operation occurs before the read, so as to verify that the write was successful

  // WoM Threshold Level Configuration
  ICM_20948_Status_e ICM_20948_wom_threshold(ICM_20948_Device_t *pdev, ICM_20948_ACCEL_WOM_THR_t *write, ICM_20948_ACCEL_WOM_THR_t *read); // Write and or read the Wake on Motion threshold. If non-null the write operation occurs before the read, so as to verify that the write was successful

  // Internal Sensor Options
  ICM_20948_Status_e ICM_20948_set_sample_mode(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
  ICM_20948_Status_e ICM_20948_set_full_scale(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss);
  ICM_20948_Status_e ICM_20948_set_dlpf_cfg(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_dlpcfg_t cfg);
  ICM_20948_Status_e ICM_20948_enable_dlpf(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, bool enable);
  ICM_20948_Status_e ICM_20948_set_sample_rate(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_smplrt_t smplrt);

  // Interface Things
  ICM_20948_Status_e ICM_20948_i2c_master_passthrough(ICM_20948_Device_t *pdev, bool passthrough);
  ICM_20948_Status_e ICM_20948_i2c_master_enable(ICM_20948_Device_t *pdev, bool enable);
  ICM_20948_Status_e ICM_20948_i2c_master_reset(ICM_20948_Device_t *pdev);
  ICM_20948_Status_e ICM_20948_i2c_controller_configure_peripheral(ICM_20948_Device_t *pdev, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut);

  // Higher Level
  ICM_20948_Status_e ICM_20948_get_agmt(ICM_20948_Device_t *pdev, ICM_20948_AGMT_t *p);

  // FIFO

  ICM_20948_Status_e ICM_20948_enable_FIFO(ICM_20948_Device_t *pdev, bool enable);
  ICM_20948_Status_e ICM_20948_reset_FIFO(ICM_20948_Device_t *pdev);
  ICM_20948_Status_e ICM_20948_set_FIFO_mode(ICM_20948_Device_t *pdev, bool snapshot);
  ICM_20948_Status_e ICM_20948_get_FIFO_count(ICM_20948_Device_t *pdev, uint16_t *count);
  ICM_20948_Status_e ICM_20948_read_FIFO(ICM_20948_Device_t *pdev, uint8_t *data, uint8_t len);

  // DMP

  ICM_20948_Status_e ICM_20948_enable_DMP(ICM_20948_Device_t *pdev, bool enable);
  ICM_20948_Status_e ICM_20948_reset_DMP(ICM_20948_Device_t *pdev);
  ICM_20948_Status_e ICM_20948_firmware_load(ICM_20948_Device_t *pdev);
  ICM_20948_Status_e ICM_20948_set_dmp_start_address(ICM_20948_Device_t *pdev, unsigned short address);

  /** @brief Loads the DMP firmware from SRAM
	* @param[in] data  pointer where the image
	* @param[in] size  size if the image
	* @param[in] load_addr  address to loading the image
	* @return 0 in case of success, -1 for any error
	*/
  ICM_20948_Status_e inv_icm20948_firmware_load(ICM_20948_Device_t *pdev, const unsigned char *data, unsigned short size, unsigned short load_addr);
  /**
	*  @brief       Write data to a register in DMP memory
	*  @param[in]   DMP memory address
	*  @param[in]   number of byte to be written
	*  @param[out]  output data from the register
	*  @return     0 if successful.
	*/
  ICM_20948_Status_e inv_icm20948_write_mems(ICM_20948_Device_t *pdev, unsigned short reg, unsigned int length, const unsigned char *data);
  /**
	*  @brief      Read data from a register in DMP memory
	*  @param[in]  DMP memory address
	*  @param[in]  number of byte to be read
	*  @param[in]  input data from the register
	*  @return     0 if successful.
	*/
  ICM_20948_Status_e inv_icm20948_read_mems(ICM_20948_Device_t *pdev, unsigned short reg, unsigned int length, unsigned char *data);

  ICM_20948_Status_e inv_icm20948_set_dmp_sensor_period(ICM_20948_Device_t *pdev, enum DMP_ODR_Registers odr_reg, uint16_t interval);
  ICM_20948_Status_e inv_icm20948_enable_dmp_sensor(ICM_20948_Device_t *pdev, enum inv_icm20948_sensor sensor, int state);     // State is actually boolean
  ICM_20948_Status_e inv_icm20948_enable_dmp_sensor_int(ICM_20948_Device_t *pdev, enum inv_icm20948_sensor sensor, int state); // State is actually boolean
  uint8_t sensor_type_2_android_sensor(enum inv_icm20948_sensor sensor);
  enum inv_icm20948_sensor inv_icm20948_sensor_android_2_sensor_type(int sensor);

  ICM_20948_Status_e inv_icm20948_read_dmp_data(ICM_20948_Device_t *pdev, icm_20948_DMP_data_t *data);
  ICM_20948_Status_e inv_icm20948_set_gyro_sf(ICM_20948_Device_t *pdev, unsigned char div, int gyro_level);


  // ToDo:

  /*
	Want to access magnetometer throught the I2C master interface...

  // If using the I2C master to read from the magnetometer
  // Enable the I2C master to talk to the magnetometer through the ICM 20948
  myICM.i2cMasterEnable( true );
  SERIAL_PORT.print(F("Enabling the I2C master returned ")); SERIAL_PORT.println(myICM.statusString());
  myICM.i2cControllerConfigurePeripheral ( 0, MAG_AK09916_I2C_ADDR, REG_ST1, 9, true, true, false, false, false );
  SERIAL_PORT.print(F("Configuring the magnetometer peripheral returned ")); SERIAL_PORT.println(myICM.statusString());

  // Operate the I2C master in duty-cycled mode
  myICM.setSampleMode( (ICM_20948_Internal_Mst | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled ); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ICM_20948_C_H_ */
