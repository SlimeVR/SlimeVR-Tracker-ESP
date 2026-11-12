/******************************************************************************
BQ27441_Definitions.h
BQ27441 LiPo Fuel Gauge Definitions
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

BQ27441 hardware constants, register addresses, and bit positions.

Hardware Resources:
- Arduino Development Board
- SparkFun Battery Babysitter

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/
#define BQ72441_I2C_ADDRESS 0x55 // Default I2C address of the BQ27441-G1A

///////////////////////
// General Constants //
///////////////////////
#define BQ27441_UNSEAL_KEY	0x8000 // Secret code to unseal the BQ27441-G1A
#define BQ27441_DEVICE_ID	0x0421 // Default device ID

///////////////////////
// Standard Commands //
///////////////////////
// The fuel gauge uses a series of 2-byte standard commands to enable system 
// reading and writing of battery information. Each command has an associated
// sequential command-code pair.
#define BQ27441_COMMAND_CONTROL			0x00 // Control()
#define BQ27441_COMMAND_TEMP			0x02 // Temperature()
#define BQ27441_COMMAND_VOLTAGE			0x04 // Voltage()
#define BQ27441_COMMAND_FLAGS			0x06 // Flags()
#define BQ27441_COMMAND_NOM_CAPACITY	0x08 // NominalAvailableCapacity()
#define BQ27441_COMMAND_AVAIL_CAPACITY	0x0A // FullAvailableCapacity()
#define BQ27441_COMMAND_REM_CAPACITY	0x0C // RemainingCapacity()
#define BQ27441_COMMAND_FULL_CAPACITY	0x0E // FullChargeCapacity()
#define BQ27441_COMMAND_AVG_CURRENT		0x10 // AverageCurrent()
#define BQ27441_COMMAND_STDBY_CURRENT	0x12 // StandbyCurrent()
#define BQ27441_COMMAND_MAX_CURRENT		0x14 // MaxLoadCurrent()
#define BQ27441_COMMAND_AVG_POWER		0x18 // AveragePower()
#define BQ27441_COMMAND_SOC				0x1C // StateOfCharge()
#define BQ27441_COMMAND_INT_TEMP		0x1E // InternalTemperature()
#define BQ27441_COMMAND_SOH				0x20 // StateOfHealth()
#define BQ27441_COMMAND_REM_CAP_UNFL	0x28 // RemainingCapacityUnfiltered()
#define BQ27441_COMMAND_REM_CAP_FIL		0x2A // RemainingCapacityFiltered()
#define BQ27441_COMMAND_FULL_CAP_UNFL	0x2C // FullChargeCapacityUnfiltered()
#define BQ27441_COMMAND_FULL_CAP_FIL	0x2E // FullChargeCapacityFiltered()
#define BQ27441_COMMAND_SOC_UNFL		0x30 // StateOfChargeUnfiltered()

//////////////////////////
// Control Sub-commands //
//////////////////////////
// Issuing a Control() command requires a subsequent 2-byte subcommand. These
// additional bytes specify the particular control function desired. The 
// Control() command allows the system to control specific features of the fuel
// gauge during normal operation and additional features when the device is in 
// different access modes.
#define BQ27441_CONTROL_STATUS			0x00
#define BQ27441_CONTROL_DEVICE_TYPE		0x01
#define BQ27441_CONTROL_FW_VERSION		0x02
#define BQ27441_CONTROL_DM_CODE			0x04
#define BQ27441_CONTROL_PREV_MACWRITE	0x07
#define BQ27441_CONTROL_CHEM_ID			0x08
#define BQ27441_CONTROL_BAT_INSERT		0x0C
#define BQ27441_CONTROL_BAT_REMOVE		0x0D
#define BQ27441_CONTROL_SET_HIBERNATE	0x11
#define BQ27441_CONTROL_CLEAR_HIBERNATE	0x12
#define BQ27441_CONTROL_SET_CFGUPDATE	0x13
#define BQ27441_CONTROL_SHUTDOWN_ENABLE	0x1B
#define BQ27441_CONTROL_SHUTDOWN		0x1C
#define BQ27441_CONTROL_SEALED			0x20
#define BQ27441_CONTROL_PULSE_SOC_INT	0x23
#define BQ27441_CONTROL_RESET			0x41
#define BQ27441_CONTROL_SOFT_RESET		0x42
#define BQ27441_CONTROL_EXIT_CFGUPDATE	0x43
#define BQ27441_CONTROL_EXIT_RESIM		0x44

///////////////////////////////////////////
// Control Status Word - Bit Definitions //
///////////////////////////////////////////
// Bit positions for the 16-bit data of CONTROL_STATUS.
// CONTROL_STATUS instructs the fuel gauge to return status information to 
// Control() addresses 0x00 and 0x01. The read-only status word contains status
// bits that are set or cleared either automatically as conditions warrant or
// through using specified subcommands.
#define BQ27441_STATUS_SHUTDOWNEN	(1<<15)
#define BQ27441_STATUS_WDRESET		(1<<14)
#define BQ27441_STATUS_SS			(1<<13)
#define BQ27441_STATUS_CALMODE		(1<<12)
#define BQ27441_STATUS_CCA			(1<<11)
#define BQ27441_STATUS_BCA			(1<<10)
#define BQ27441_STATUS_QMAX_UP		(1<<9)
#define BQ27441_STATUS_RES_UP		(1<<8)
#define BQ27441_STATUS_INITCOMP		(1<<7)
#define BQ27441_STATUS_HIBERNATE	(1<<6)
#define BQ27441_STATUS_SLEEP		(1<<4)
#define BQ27441_STATUS_LDMD			(1<<3)
#define BQ27441_STATUS_RUP_DIS		(1<<2)
#define BQ27441_STATUS_VOK			(1<<1)

////////////////////////////////////
// Flag Command - Bit Definitions //
////////////////////////////////////
// Bit positions for the 16-bit data of Flags()
// This read-word function returns the contents of the fuel gauging status
// register, depicting the current operating status.
#define BQ27441_FLAG_OT			(1<<15)
#define BQ27441_FLAG_UT			(1<<14)
#define BQ27441_FLAG_FC			(1<<9)
#define BQ27441_FLAG_CHG		(1<<8)
#define BQ27441_FLAG_OCVTAKEN	(1<<7)
#define BQ27441_FLAG_ITPOR		(1<<5)
#define BQ27441_FLAG_CFGUPMODE	(1<<4)
#define BQ27441_FLAG_BAT_DET	(1<<3)
#define BQ27441_FLAG_SOC1		(1<<2)
#define BQ27441_FLAG_SOCF		(1<<1)
#define BQ27441_FLAG_DSG		(1<<0)

////////////////////////////
// Extended Data Commands //
////////////////////////////
// Extended data commands offer additional functionality beyond the standard
// set of commands. They are used in the same manner; however, unlike standard
// commands, extended commands are not limited to 2-byte words.
#define BQ27441_EXTENDED_OPCONFIG	0x3A // OpConfig()
#define BQ27441_EXTENDED_CAPACITY	0x3C // DesignCapacity()
#define BQ27441_EXTENDED_DATACLASS	0x3E // DataClass()
#define BQ27441_EXTENDED_DATABLOCK	0x3F // DataBlock()
#define BQ27441_EXTENDED_BLOCKDATA	0x40 // BlockData()
#define BQ27441_EXTENDED_CHECKSUM	0x60 // BlockDataCheckSum()
#define BQ27441_EXTENDED_CONTROL	0x61 // BlockDataControl()

////////////////////////////////////////
// Configuration Class, Subclass ID's //
////////////////////////////////////////
// To access a subclass of the extended data, set the DataClass() function
// with one of these values.
// Configuration Classes
#define BQ27441_ID_SAFETY			2   // Safety
#define BQ27441_ID_CHG_TERMINATION	36  // Charge Termination
#define BQ27441_ID_CONFIG_DATA		48  // Data
#define BQ27441_ID_DISCHARGE		49  // Discharge
#define BQ27441_ID_REGISTERS		64  // Registers
#define BQ27441_ID_POWER			68  // Power
// Gas Gauging Classes
#define BQ27441_ID_IT_CFG			80  // IT Cfg
#define BQ27441_ID_CURRENT_THRESH	81  // Current Thresholds
#define BQ27441_ID_STATE			82  // State
// Ra Tables Classes
#define BQ27441_ID_R_A_RAM			89  // R_a RAM
// Calibration Classes
#define BQ27441_ID_CALIB_DATA		104 // Data
#define BQ27441_ID_CC_CAL			105 // CC Cal
#define BQ27441_ID_CURRENT			107 // Current
// Security Classes
#define BQ27441_ID_CODES			112 // Codes

/////////////////////////////////////////
// OpConfig Register - Bit Definitions //
/////////////////////////////////////////
// Bit positions of the OpConfig Register
#define BQ27441_OPCONFIG_BIE      (1<<13)
#define BQ27441_OPCONFIG_BI_PU_EN (1<<12)
#define BQ27441_OPCONFIG_GPIOPOL  (1<<11)
#define BQ27441_OPCONFIG_SLEEP    (1<<5)
#define BQ27441_OPCONFIG_RMFCC    (1<<4)
#define BQ27441_OPCONFIG_BATLOWEN (1<<2)
#define BQ27441_OPCONFIG_TEMPS    (1<<0)