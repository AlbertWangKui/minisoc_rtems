/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 * @file    drv_efuse.h
 * @author  cursor@starsmicrosystem.com
 * @date    2025/11/27
 * @brief   EFUSE driver internal header
 */

#ifndef __DRV_EFUSE_H__
#define __DRV_EFUSE_H__

#include "common_defines.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

#define EFUSE_MAGIC              0xACCE5500  ///< 解锁魔数
#define EFUSE_IDLE_TIMEOUT       20000       ///< 空闲等待超时循环次数
#define EFUSE_LOCK_TIMEOUT_MS    1000        ///< 设备锁超时(ms)
#define EFUSE_MAX_ELD_DATA       80          ///< 最大ELD数据长度(U32)
#define EFUSE_MAXELD_COUNT       28          ///< ELD总数
#define EFUSE_SM2_KEY_LEN        16          ///< SM2密钥长度(U32)
#define EFUSE_SM4_KEY_LEN        4           ///< SM4密钥长度(U32)

///< 寄存器字段偏移和掩码
#define EFUSE_STATE_OFFSET       0
#define EFUSE_STATE_MASK         0x1
#define EFUSE_ERRCODE_OFFSET     4
#define EFUSE_ERRCODE_MASK       0xF
#define EFUSE_ELDSEL_OFFSET      8
#define EFUSE_ELDSEL_MASK        0x3F
#define EFUSE_RDELDSEL_OFFSET    0
#define EFUSE_RDELDSEL_MASK      0x3F

///< 操作码定义
typedef enum {
    EFUSE_OP_SENSE  = 0,  ///< 读取操作
    EFUSE_OP_PROG   = 1,  ///< 编程操作
    EFUSE_OP_VERIFY = 2,  ///< 校验操作
    EFUSE_OP_CLEAR  = 3,  ///< 清除操作
} EfuseOpcode_e;

///< 错误码定义
typedef enum {
    EFUSE_ERR_DONE           = 0,         ///< 命令完成
    EFUSE_ERR_ELD_INVALID    = (1 << 0),  ///< ELD号无效
    EFUSE_ERR_OPCODE_INVALID = (1 << 1),  ///< 操作码无效
    EFUSE_ERR_BUSY           = (1 << 2),  ///< 设备忙
    EFUSE_ERR_TIMEOUT        = (1 << 3),  ///< 超时
    EFUSE_ERR_OFFSET_INVALID = (1 << 4),  ///< 偏移无效
    EFUSE_ERR_WR_FAIL        = 0xF,       ///< 写失败
} EfuseErrCode_e;

///< 寄存器结构体
typedef volatile struct {
    U32 startCmd;          ///< 0x00 启动命令
    U32 cmdInfo;           ///< 0x04 命令信息(opcode+eldSel)
    U32 procCfg;           ///< 0x08 处理配置
    U32 rdEldCtrl;         ///< 0x0C 读ELD控制
    U32 status0;           ///< 0x10 状态0(state+errcode)
    U32 status1;           ///< 0x14 状态1
    U32 eldWtCtrl;         ///< 0x18 ELD写控制
    U32 eldRdCtrl;         ///< 0x1C ELD读控制
    U32 reserved1[36];     ///< 0x20~0xAC 保留
    U32 opModeCtrl;        ///< 0xB0 操作模式控制
    U32 reserved2[83];     ///< 0xB4~0x1FC 保留
    U32 progData[80];      ///< 0x200~0x33C 编程数据
    U32 reserved3[48];     ///< 0x340~0x3FC 保留
    U32 readData[80];      ///< 0x400~0x53C 读取数据
    U32 reserved4[624];    ///< 0x540~0xEFC 保留
    U32 lockAccess;        ///< 0xF00 锁定访问
} EfuseReg_s;

///< 驱动数据结构体
typedef struct {
    SbrEfuseCfg_s sbrCfg;  ///< SBR配置
} EfuseDrvData_s;

#endif /* __DRV_EFUSE_H__ */

