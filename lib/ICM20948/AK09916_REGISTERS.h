#ifndef _AK09916_REGISTERS_H_
#define _AK09916_REGISTERS_H_

#include <stdint.h>

typedef enum
{
  AK09916_REG_WIA1 = 0x00,
  AK09916_REG_WIA2,
  AK09916_REG_RSV1,
  AK09916_REG_RSV2, // Reserved register. We start reading here when using the DMP. Secret sauce...
  // discontinuity - containing another nine reserved registers? Secret sauce...
  AK09916_REG_ST1 = 0x10,
  AK09916_REG_HXL,
  AK09916_REG_HXH,
  AK09916_REG_HYL,
  AK09916_REG_HYH,
  AK09916_REG_HZL,
  AK09916_REG_HZH,
  // discontinuity
  AK09916_REG_ST2 = 0x18,
  // discontinuity
  AK09916_REG_CNTL2 = 0x31,
  AK09916_REG_CNTL3,
} AK09916_Reg_Addr_e;

typedef struct
{
  uint8_t WIA1;
} AK09916_WIA1_Reg_t;

typedef struct
{
  uint8_t WIA2;
} AK09916_WIA2_Reg_t;

typedef struct
{
  uint8_t DRDY : 1;
  uint8_t DOR : 1;
  uint8_t reserved_0 : 6;
} AK09916_ST1_Reg_t;

// typedef struct{

// }AK09916_HXL_Reg_t;

// typedef struct{

// }AK09916_HXH_Reg_t;
// typedef struct{

// }AK09916_HYL_Reg_t;
// typedef struct{

// }AK09916_HYH_Reg_t;
// typedef struct{

// }AK09916_HZL_Reg_t;
// typedef struct{

// }AK09916_HZH_Reg_t;

typedef struct
{
  uint8_t reserved_0 : 3;
  uint8_t HOFL : 1;
  uint8_t reserved_1 : 4;
} AK09916_ST2_Reg_t;

typedef struct
{
  uint8_t MODE : 5;
  uint8_t reserved_0 : 3;
} AK09916_CNTL2_Reg_t;

typedef struct
{
  uint8_t SRST : 1;
  uint8_t reserved_0 : 7;
} AK09916_CNTL3_Reg_t;

#endif // _AK09916_REGISTERS_H_
