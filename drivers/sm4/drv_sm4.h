/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 * @file    drv_sm4.h
 * @author  sangwei@starsmicrosystem.com
 * @date    2025/11/28
 * @brief   SM4 cryptographic driver internal definitions
 */

#ifndef __DRV_SM4_H__
#define __DRV_SM4_H__

#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

#define SM4_LOCK_TIMEOUT_MS     1000            ///< 设备锁超时时间
#define SM4_POLL_TIMEOUT_MS     10000           ///< 轮询超时时间
#define SM4_STATE_IDLE          0               ///< SM4空闲状态
#define SM4_KEYS_SIZE           4               ///< 密钥数组大小
#define SM4_BLOCK_SIZE          16              ///< SM4块大小(字节)
#define SM4_MIN_DATA_SIZE       16              ///< 最小数据长度
#define SM4_MAX_DATA_SIZE       (16 * 1024 * 1024) ///< 最大数据长度(16MB)
#define SM4_IRQ_MASK            0x03            ///< 中断掩码

typedef union {
    struct {
        U32 start       : 1;    ///< bit 0: 启动
        U32 kxpstartEn  : 1;    ///< bit 1: 密钥扩展启动使能
        U32 chgcntrEn   : 1;    ///< bit 2: 计数器更新使能
        U32 invalid     : 1;    ///< bit 3: 软件模式标志
        U32 sm4KeySel   : 1;    ///< bit 4: 密钥来源选择
        U32 sm4CntSel   : 1;    ///< bit 5: 计数器来源选择
        U32             : 26;
    } bit;
    U32 dword;
} Sm4CtrlReg_s;

typedef union {
    struct {
        U32 busy    : 1;        ///< bit 0: 忙标志
        U32 keySrc  : 1;        ///< bit 1: 密钥来源
        U32         : 30;
    } bit;
    U32 dword;
} Sm4StsReg_s;

typedef union {
    struct {
        U32 doneEn      : 1;    ///< bit 0: 完成中断使能
        U32 axiErrEn    : 1;    ///< bit 1: AXI错误中断使能
        U32             : 30;
    } bit;
    U32 dword;
} Sm4IrqEnReg_s;

typedef union {
    struct {
        U32 done    : 1;        ///< bit 0: 完成中断状态
        U32 axiErr  : 1;        ///< bit 1: AXI错误中断状态
        U32         : 30;
    } bit;
    U32 dword;
} Sm4IrqStsReg_s;

typedef union {
    struct {
        U32 doneClr     : 1;    ///< bit 0: 完成中断清除
        U32 axiErrClr   : 1;    ///< bit 1: AXI错误中断清除
        U32             : 30;
    } bit;
    U32 dword;
} Sm4IrqClrReg_s;

typedef union {
    struct {
        U32 softRst : 1;        ///< bit 0: 软复位
        U32         : 31;
    } bit;
    U32 dword;
} Sm4RstReg_s;

typedef volatile struct {
    Sm4CtrlReg_s ctrl;          ///< 0x00: 控制寄存器
    Sm4StsReg_s sts;            ///< 0x04: 状态寄存器
    Sm4IrqEnReg_s irqEn;        ///< 0x08: 中断使能
    Sm4IrqStsReg_s irqSts;      ///< 0x0C: 中断状态
    Sm4IrqClrReg_s irqClr;      ///< 0x10: 中断清除
    U32 rdStartAddr;            ///< 0x14: 读起始地址
    U32 wrStartAddr;            ///< 0x18: 写起始地址
    U32 dataSize;               ///< 0x1C: 数据长度
    U32 key[4];                 ///< 0x20: 密钥
    U32 cntr[4];                ///< 0x30: 计数器
    U32 cntrLen[4];             ///< 0x40: 计数器长度
    U32 din[4];                 ///< 0x50: 数据输入
    U32 dout[4];                ///< 0x60: 数据输出
    Sm4RstReg_s softReset;      ///< 0x70: 软复位寄存器
} Sm4Reg_s;

typedef struct {
    SbrSm4Cfg_s sbrCfg;         ///< SBR配置
    void (*callback)(void *);   ///< 回调函数
    void *callbackArg;          ///< 回调参数
} Sm4DrvData_s;

#endif /* __DRV_SM4_H__ */

