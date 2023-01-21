#define QMC_DEVADDR    0x0D
#define QMC_RA_DATA    0x00
#define QMC_RA_CONTROL 0x09
#define QMC_RA_RESET   0x0B

#define QMC_CFG_MODE_STANDBY    0b00
#define QMC_CFG_MODE_CONTINUOUS 0b01
#define QMC_CFG_ODR_10HZ        0b00 << 2
#define QMC_CFG_ODR_50HZ        0b01 << 2
#define QMC_CFG_ODR_100HZ       0b10 << 2
#define QMC_CFG_ODR_200HZ       0b11 << 2
#define QMC_CFG_RNG_2G          0b00 << 4
#define QMC_CFG_RNG_8G          0b01 << 4
#define QMC_CFG_OSR_512         0b00 << 6
#define QMC_CFG_OSR_256         0b01 << 6
#define QMC_CFG_OSR_128         0b10 << 6
#define QMC_CFG_OSR_64          0b11 << 6
