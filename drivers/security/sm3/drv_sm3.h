/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sm3.h
 * @author baibch@starsmicrosystem.com
 * @date 2025/09/30
 * @brief SM3 hash algorithm driver header
 */

#ifndef __DRV_SM3_H__
#define __DRV_SM3_H__

#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_sbr.h"

typedef volatile union __Sm3CtrlReg {
    struct {
        U32 logicEn : 1;
        U32 pauseEn : 1;
        U32 startEn : 1;
        U32 : 29;
    } fields;
    U32 dword;
} Sm3CtrlReg;

typedef volatile union __Sm3StatusReg {
    struct {
        U32 state : 1;
        U32 finish : 1;
        U32 addrOver : 1;
        U32 : 29;
    } fields;
    U32 dword;
} Sm3StatusReg;

typedef volatile union __Sm3StartReg {
    struct {
        U32 start : 1;
        U32 : 31;
    } fields;
    U32 dword;
} Sm3StartReg;

typedef volatile union __Sm3AxiStartReg {
    struct {
        U32 start : 1;
        U32 : 31;
    } fields;
    U32 dword;
} Sm3AxiStartReg;

typedef volatile union __Sm3StateClrReg {
    struct {
        U32 stateClr : 1;
        U32 irqClr : 1;
        U32 : 30;
    } fields;
    U32 dword;
} Sm3StateClrReg;

typedef volatile union __Sm3SoftResetReg {
    struct {
        U32 reset : 1;
        U32 : 31;
    } fields;
    U32 dword;
} Sm3SoftResetReg;

typedef volatile union __Sm3IrqEnableReg {
    struct {
        U32 enable : 1;
        U32 : 31;
    } fields;
    U32 dword;
} Sm3IrqEnableReg;

typedef volatile union __Sm3SoftIrqReg {
    struct {
        U32 softIrq : 1; /* bit[0]: write 1 to inject DONE irq */
        U32 : 31;
    } fields;
    U32 dword;
} Sm3SoftIrqReg;

typedef volatile struct __Sm3Reg_s {
    U32 dataIn[16];           /* 0x00 */
    U32 dataOut[8];           /* 0x40 */
    U32 grpCounts;            /* 0x60 */
    U32 addrStart;            /* 0x64 */
    U32 addrEnd;              /* 0x68 */
    Sm3CtrlReg ctrl;          /* 0x6C */
    Sm3StatusReg status;      /* 0x70 */
    U32 chgiv;                /* 0x74 */
    Sm3SoftResetReg reset;    /* 0x78 */
    Sm3StartReg start;        /* 0x7C */
    Sm3AxiStartReg axiStart;  /* 0x80 */
    Sm3StateClrReg stateClr;  /* 0x84 */
    Sm3IrqEnableReg irqEnable;/* 0x88 */
    Sm3SoftIrqReg swInject;   /* 0x8C */
} Sm3Reg_s;

/* SM3 Driver Data Structure */
typedef struct {
    SbrSm3Cfg_s sbrCfg;         /* SBR configuration */
    void (*callback)(void *arg); /* Callback function */
    void *callbackArg;          /* Callback argument */
} Sm3DrvData_s;

#endif /* __DRV_SM3_H__ */
