/**
 * copyright (C), 2020, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file efuse.c
 * @date 2022/07/15
 * @brief none
 * @version v1.0
 */

#ifndef __DRV_EFUSE_V1_H__
#define __DRV_EFUSE_V1_H__

#include "bsp_sbr.h"
#include "sbr_api.h"

#define EFUSE_BASEADDR                   (0xBE101000)
#define EFUSE_DEVICE_MAGIC               (0xACCE5500)
#define EFUSE_DEVICE_NUM_ELD             (19)
#define EFUSE_IDLE_FLAGS                 (0)
#define EFUSE_DEVICE_SIZE_PER_ELD        (16)

// #define  EFUSE_START_CMD               (EFUSE_BASEADDR + 0x00)
// #define  EFUSE_CMD_INFO                (EFUSE_BASEADDR + 0x04)
// #define  EFUSE_PROG_CONFIG             (EFUSE_BASEADDR + 0x08)
// #define  EFUSE_RD_ELD_SEL              (EFUSE_BASEADDR + 0x0c)
// #define  EFUSE_STATUS0_REG             (EFUSE_BASEADDR + 0x10)
// #define  EFUSE_STATUS1_REG             (EFUSE_BASEADDR + 0x14)
// #define  EFUSE_PROG_DATA0              (EFUSE_BASEADDR + 0x20)
// #define  EFUSE_PROG_DATA1              (EFUSE_BASEADDR + 0x24)
// #define  EFUSE_PROG_DATA2              (EFUSE_BASEADDR + 0x28)
// #define  EFUSE_PROG_DATA3              (EFUSE_BASEADDR + 0x2C)
// #define  EFUSE_PROG_DATA4              (EFUSE_BASEADDR + 0x30)
// #define  EFUSE_PROG_DATA5              (EFUSE_BASEADDR + 0x34)
// #define  EFUSE_PROG_DATA6              (EFUSE_BASEADDR + 0x38)
// #define  EFUSE_PROG_DATA7              (EFUSE_BASEADDR + 0x3C)
// #define  EFUSE_PROG_DATA8              (EFUSE_BASEADDR + 0x40)
// #define  EFUSE_PROG_DATA9              (EFUSE_BASEADDR + 0x44)
// #define  EFUSE_PROG_DATA10             (EFUSE_BASEADDR + 0x48)
// #define  EFUSE_PROG_DATA11             (EFUSE_BASEADDR + 0x4C)
// #define  EFUSE_PROG_DATA12             (EFUSE_BASEADDR + 0x50)
// #define  EFUSE_PROG_DATA13             (EFUSE_BASEADDR + 0x54)
// #define  EFUSE_PROG_DATA14             (EFUSE_BASEADDR + 0x58)
// #define  EFUSE_PROG_DATA15             (EFUSE_BASEADDR + 0x5C)
// #define  EFUSE_READELD_DATA0           (EFUSE_BASEADDR + 0x60)
// #define  EFUSE_READELD_DATA1           (EFUSE_BASEADDR + 0x64)
// #define  EFUSE_READELD_DATA2           (EFUSE_BASEADDR + 0x68)
// #define  EFUSE_READELD_DATA3           (EFUSE_BASEADDR + 0x6C)
// #define  EFUSE_READELD_DATA4           (EFUSE_BASEADDR + 0x70)
// #define  EFUSE_READELD_DATA5           (EFUSE_BASEADDR + 0x74)
// #define  EFUSE_READELD_DATA6           (EFUSE_BASEADDR + 0x78)
// #define  EFUSE_READELD_DATA7           (EFUSE_BASEADDR + 0x7C)
// #define  EFUSE_READELD_DATA8           (EFUSE_BASEADDR + 0x80)
// #define  EFUSE_READELD_DATA9           (EFUSE_BASEADDR + 0x84)
// #define  EFUSE_READELD_DATA10          (EFUSE_BASEADDR + 0x88)
// #define  EFUSE_READELD_DATA11          (EFUSE_BASEADDR + 0x8C)
// #define  EFUSE_READELD_DATA12          (EFUSE_BASEADDR + 0x90)
// #define  EFUSE_READELD_DATA13          (EFUSE_BASEADDR + 0x94)
// #define  EFUSE_READELD_DATA14          (EFUSE_BASEADDR + 0x98)
// #define  EFUSE_READELD_DATA15          (EFUSE_BASEADDR + 0x9C)
// #define  EFUSE_LOCKACCESS              (EFUSE_BASEADDR + 0xF00)

#define EFUSE_DEV_MAX_NUM 1

typedef struct {
    volatile U32 startCmd;          //< 0x00
    volatile U32 cmdInfo;           //< 0x04
    volatile U32 procCfg;           //< 0x08
    volatile U32 rdEldSel;          //< 0x0c
    volatile U32 status0;           //< 0x10
    volatile U32 status1;           //< 0x14
    volatile U32 reserved1[2];      //< 0x18 0x1c
    volatile U32 progData[16];      //< 0x20 ~ 0x5c
    volatile U32 readEldData[16];   //< 0x60 ~ 0x9C
    volatile U32 reserved2[0x398];  //< 0xa0 ~ 0xefc
    volatile U32 lockAccess;        //< 0xF00
}efuseReg_s;

#define  EFUSE_STATE_OFFSET              (0)
#define  EFUSE_FINISH_OFFSET             (1)
#define  EFUSE_ADDR_OVER_OFFSET          (2)
#define  EFUSE_STATE_MASK                (1)
#define  EFUSE_DONE_OFFSET               (1)
#define  EFUSE_ERRCODE_OFFSET            (4)

#define  EFUSE_NUM_16                    (16)
#define  EFUSE_REG_4                     (4)

#define  EFUSE_OPCODE_MASK               (3)

typedef enum __ErrEventType {
    ERR_CODE_CMD_DONE       = 0,
    ERR_CODE_ELD_INVALID    = (1 << 0),
    ERR_CODE_OPCODE_INVALID = (1 << 1),
    ERR_CODE_BUSY           = (1 << 2),
    ERR_CODE_TIMEOUT        = (1 << 3),
    ERR_CODE_OFFSET_INVALID = (1 << 4),
    ERR_CODE_WR_FAIL        = 0xF,
    ERR_CODE_RESERVED,
} ErrEventType_e;

typedef struct {
    SbrEfuseCfg_s sbrCfg;
} efuseDrvData_s;

#endif

