/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 * 
 * @file drv_sm2.h
 * @author pengjb@starsmicrosystem.com
 * @date 2025/09/30
 * @brief SM2 cryptographic driver implementation
 */

#ifndef __DRV_SM2_H__
#define __DRV_SM2_H__

#define SM2_STATE_IDLE 0x00
#define SM2_STATE_BUSY 0x01
#define SM2_IRQ_MASK   0x1FF
// #define CONFIG_DRV_SM2_TEST 1
#define SM2_PKEY_SOURCE_SOFTWARE  (0)
#define SM2_PKEY_SOURCE_EFUSE     (1)

#define SM2_SIGN_BUF_SIZE_DWORDS  16
#define SM2_HASH_SIZE_DWORDS       8
#define SM2_PKEY_SIZE_DWORDS      16
#define SM2_LOCK_TIMEOUT_MS       1000
typedef volatile union _FuncSelReg {
    struct {
        U32 func : 3;
        U32 pubKeys : 1;
        U32 priKeys : 1;
        U32 : 27;
    } fields;
    U32 dword;
} FuncSelReg;

typedef volatile union _Sm2StateReg {
    struct {
        U32 state : 2;
        U32 result : 1;
        U32 : 29;
    } fields;
    U32 dword;
} Sm2StateReg;

typedef volatile union _Sm2StartReg {
    struct {
        U32 start : 1;
        U32 : 31;
    } fields;
    U32 dword;
} Sm2StartReg;

typedef volatile union _Sm2ResetReg {
    struct {
        U32 reset : 1;
        U32 : 31;
    } fields;
    U32 dword;
} Sm2ResetReg;

typedef volatile union _IrqStatusReg {
    struct {
        U32 abnSign0 : 1;
        U32 abnSign1 : 1;
        U32 abnSign2 : 1;
        U32 abnSign3 : 1;
        U32 abnSign4 : 1;
        U32 abnSign5 : 1;
        U32 abnSign6 : 1;
        U32 abnSign7 : 1;
        U32 done : 1;
        U32 : 23;
    } fields;
    U32 dword;
} IrqStatusReg;

typedef volatile union _IrqEnableReg {
    struct {
        U32 abnSign0 : 1;
        U32 abnSign1 : 1;
        U32 abnSign2 : 1;
        U32 abnSign3 : 1;
        U32 abnSign4 : 1;
        U32 abnSign5 : 1;
        U32 abnSign6 : 1;
        U32 abnSign7 : 1;
        U32 done : 1;
        U32 : 23;
    } fields;
    U32 dword;
} IrqEnableReg;

typedef volatile union _IrqInjectReg {
    struct {
        U32 abnSign0 : 1;
        U32 abnSign1 : 1;
        U32 abnSign2 : 1;
        U32 abnSign3 : 1;
        U32 abnSign4 : 1;
        U32 abnSign5 : 1;
        U32 abnSign6 : 1;
        U32 abnSign7 : 1;
        U32 done : 1;
        U32 : 23;
    } fields;
    U32 dword;
} IrqInjectReg;

typedef volatile union _FinishStatusReg {
    struct {
        U32 finish : 1;
        U32 : 31;
    } fields;
    U32 dword;
} FinishStatusReg;

typedef volatile struct Sm2Reg {
    U32 dataIn[8]; ///< 0x00
    U32 pkeyX[8]; ///< 0x20
    U32 pkeyY[8]; ///< 0x40
    U32 signR[8]; ///< 0x60
    U32 signS[8]; ///< 0x80
    FuncSelReg funcSel; ///< 0xA0
    Sm2StateReg state; ///< 0xA4
    Sm2StartReg softStart; ///< 0xA8
    Sm2ResetReg softReset; ///< 0xAC
    IrqStatusReg irqStatus; ///< 0xB0
    IrqEnableReg irqEnable; ///< 0xB4
    IrqInjectReg irqInject; ///< 0xB8
    FinishStatusReg finishStatus; ///< 0xBC
    U32 signROut[8]; ///< 0xC0
    U32 signSOut[8]; ///< 0xE0
} Sm2Reg_s;

///< SM2 Driver Data Structure
typedef struct {
    SbrSm2Cfg_s sbrCfg;         ///< SBR configuration
    void (*callback)(void *arg); ///< Callback function
    void *callbackArg;          ///< Callback argument
} Sm2DrvData_s;

#endif