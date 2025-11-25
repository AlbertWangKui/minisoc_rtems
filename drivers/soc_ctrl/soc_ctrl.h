/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file soc_ctrl.h
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2024/12/27
 * @brief  soc_ctrl.soc_csrs register
 *
 * @info: SVN//minisoc_junior.soc_ctrl.soc_csrs_reglist.xlsx
 * @version
 *  Hardware version:
 *      v0.0 初版
 *      v0.1 1. 添加了LAR
 *           2. 调整了时钟分频系数配置的寄存器和软件操作方式
 *      v0.2 增加了 build cfg 和版本寄存器
 *      v0.3 增加了 debug monitor 选择控制寄存器
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2024/12/27   yangzhl3        the first version
 *
 *
 */
#ifndef __SOC_CTRL_H__
#define __SOC_CTRL_H__

#include "common_defines.h"

///< soc control instance
#define SOC_CTRL            ((SocCtrlReg_s *)SYS_SOC_CTRL_BASE_ADRS)

typedef enum BspSocCtrlPeriModule {
    BSP_SOC_CTRL_PERI_MODULE_QFLASH = 0,
    BSP_SOC_CTRL_PERI_MODULE_SM2,
    BSP_SOC_CTRL_PERI_MODULE_SM3,
    BSP_SOC_CTRL_PERI_MODULE_SM4,
    BSP_SOC_CTRL_PERI_MODULE_TRNG,
    BSP_SOC_CTRL_PERI_MODULE_GDMA,
    BSP_SOC_CTRL_PERI_MODULE_SOC,
    BSP_SOC_CTRL_PERI_MODULE_MAX,
} BspSocCtrlPeriModule_e;

typedef volatile union ClockEnableReg {
    struct {
        U32 : 4; ///< bit3~bit0
        U32 gdma : 1; ///< bit4
        U32 : 1; ///< bit5
        U32 dmacSysApb : 1; ///< bit6
        U32 : 1; ///< bit7
        U32 cpuSysCore : 1; ///< bit8
        U32 cpuSysAxi : 1; ///< bit9
        U32 cpuSysAtb : 1; ///< bit10
        U32 cpuSysTrace : 1; ///< bit11
        U32 cpuSysApb : 1; ///< bit12
        U32 : 3; ///< bit15~bit13
        U32 secSysAxi : 1; ///< bit16
        U32 secSysApb : 1; ///< bit17
        U32 : 6; ///< bit23 ~ bit18
        U32 ocmSysQspi : 1; ///< bit24
        U32 ocmSysOcp : 1; ///< bit25
        U32 ocmSysApb : 1; ///< bit26
        U32 : 1; ///< bit27
        U32 periSysApb : 1; ///< bit28
        U32 : 3; ///< bit31~bit29
    } fields;
    U32 dword;
} ClockEnableReg_u;

typedef volatile union MinisocSoftRstReg {
    struct {
        U32 : 1; ///< bit0
        U32 func : 1; ///< bit1
        U32 dbg : 1; ///< bit2
        U32 : 1; ///< bit3
        U32 gdma : 1; ///< bit4
        U32 : 3; ///< bit7~bit5
        U32 cpuSysCore : 1; ///< bit8
        U32 cpuSysDbg : 1; ///< bit9
        U32 cpuSysGic : 1; ///< bit10
        U32 cpuSysAxi : 1; ///< bit11
        U32 : 4; ///< bit15~bit12
        U32 secSysSm2 : 1; ///< bit16
        U32 secSysSm3 : 1; ///< bit17
        U32 secSysSm4 : 1; ///< bit18
        U32 secSysTrng : 1; ///< bit19
        U32 secSysAxi : 1; ///< bit20
        U32 secSysApb : 1; ///< bit21
        U32 : 2; ///< bit23~bit22
        U32 ocmSysQspi : 1; ///< bit24
        U32 ocmSysOcp : 1; ///< bit25
        U32 ocmSysApb : 1; ///< bit26
        U32 : 1; ///< bit27
        U32 periSysApb : 1; ///< bit28
        U32 : 3; ///< bit31~bit29
    } fields;
    U32 dword;
} MinisocSoftRstReg_u;

typedef volatile union SocSoftResetReg {
    struct {
        U32 fullSystem : 1; ///< bit0
        U32 : 31; ///< bit31~bit1
    } fields;
    U32 dword;
} SocSoftResetReg_u;

typedef volatile union BootStatusReg {
    struct {
        U32 : 2; ///< [1:0]: reseved
        U32 inGuomiMode : 1; ///< [2]
        U32 sramInitDisable : 1; ///< [3]
        U32 inSecureMode : 1; ///< [4]
        U32 bypassEfuse : 1; ///< [5]
        U32 bypassBisr : 1; ///< [6]
        U32 deviceSelect : 2; ///< [8:7], 11: sram, 10: flash, 00: rom, 01: rom
        U32 releaseEfuse : 1; ///< [9]
        U32 releaseBisr : 1; ///< [10]
        U32 releasePreMinisoc : 1; ///< [11]
        U32 releaseInMinisoc : 1; ///< [12]
        U32 releasePostMinisoc : 1; ///< [13]
        U32 releaseCpuCore : 1; ///< [14]
        U32 : 1; ///< [15]
        U32 efuseDone : 1; ///< [16]
        U32 bisrDone : 1; ///< [17]
        U32 preMinisocDone : 1; ///< [18]
        U32 inMinisocDone : 1; ///< [19]
        U32 postMinisocDone : 1; ///< [20]
        U32 cpuCoreDone : 1; ///< [21]
        U32 : 2; ///< [23:22]
        U32 errorEfuse0 : 1; ///< [24]
        U32 errorEfuse1 : 1; ///< [25]
        U32 errorBisr : 1; ///< [26]
        U32 errorPreMinisoc : 1; ///< [27]
        U32 errorInMinisoc : 1; ///< [28]
        U32 errorPostMinisoc : 1; ///< [29]
        U32 errorCpuCore : 1; ///< [30]
        U32 errorAny : 1; ///< [31]
    } fields;
    U32 dword;
} BootStatusReg_u;

/**
 * @brief
 * @info: SVN//minisoc_junior.soc_ctrl.soc_csrs_reglist.xlsx
 * @version
 *      v0.0 初版
 *      v0.1 1. 添加了LAR
 *           2. 调整了时钟分频系数配置的寄存器和软件操作方式
 *      v0.2 增加了 build cfg 和版本寄存器
 *      v0.3 增加了 debug monitor 选择控制寄存器
 * @details none
 */
typedef volatile struct SocCtrlReg {
    U32                bootCtrl0;         ///< 0x0
    U32                bootCtrl1;         ///< 0x4
    U32                bootloaderEntry;   ///< 0x8
    BootStatusReg_u         bootStatus;        ///< 0xc
    U32                resetCause;        ///< 0x10
    U32                resved14[3];
    U32                clkDivCfg0;       ///< 0x20
    U32                clkDivCfg1;       ///< 0x24
    U32                clkDivUpdate;     ///< 0x28
    ClockEnableReg_u        clkDivEnable;     ///< 0x2c
    U32                clkDivStatus0;    ///< 0x30
    U32                clkDivStatus1;    ///< 0x34
    U32                resved38[2];
    MinisocSoftRstReg_u    minisocSoftRst;   ///< 0x40
    SocSoftResetReg_u      socSoftReset;     ///< 0x44
    U32                resved48[2];
    U32                buildOption0;      ///< 0x50
    U32                buildOption1;      ///< 0x54
    U32                minisocVersion;    ///< 0x58
    U32                resved5c[41];
    U32                debugMonitorSelect;///< 0x100
    U32                resved104[191];
    U32                guomiKey[64];      ///< 0x400 ~ 0x4fc
    U32                resved500[64];
    U32                sm2Pubkey[16];     ///< 0x600 ~ 0x63c
    U32                resved640[112];

    U32                niuSuspendcouneEnable;    ///< 0x800
    U32                niuTimeoutEnable;         ///< 0x804
    U32                niuTimeoutStatus;         ///< 0x808
    U32                resved80c[1];
    U32                niuNoPendingTransStatus;///< 0x810
    U32                niuNoPendingReadStatus; ///< 0x814
    U32                niuNoPendingWriteStatus;///< 0x818
    U32                resved81c[1];
    U32                powerDomainCtrl0;         ///< 0x820
    U32                powerDomainCtrl1;         ///< 0x824
    U32                axiMasterTimeoutCount;   ///< 0x828
    U32                resved82c[437];
    U32                lockAccess;        ///< 0xF00
    U32                resvedF04[7];
    U32                swScratch[8];      ///< 0xF20
} SocCtrlReg_s;

/**
 * @brief clk enable or disable for modules in soc ctrl.
 * @param [in] module, module type.
 * @param [in] id, sub module id.
 * @param [in] status, true is enable, false is disable.
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 */
S32 socCtrlPeriClkEnableConfig(BspSocCtrlPeriModule_e module, Bool status);

/**
 * @brief reset hold modules in soc ctrl.
 * @param [in] module, module type.
 * @param [in] id, sub module id.
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 */
S32 socCtrlPeriResetHold(BspSocCtrlPeriModule_e module);

/**
 * @brief reset release modules in soc ctrl.
 * @param [in] module, module type.
 * @param [in] id, sub module id.
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 */
S32 socCtrlPeriResetRelease(BspSocCtrlPeriModule_e module);

/**
 * @brief reset modules in soc ctrl.
 * @param [in] module, module type.
 * @param [in] id, sub module id.
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 */
S32 socCtrlPeriReset(BspSocCtrlPeriModule_e module);

#ifdef SOC_SELFTEST_ENABLE
#include "log_msg.h"
#include <rtems/score/percpu.h>

#define GET_REG_STRUCT_BIT_POS(regVar, fieldName, bitPos) \
    do { \
        typeof(regVar) regValue; \
        U32 temp; \
        regValue.dword = 0; \
        regValue.fields.fieldName = 1; \
        temp = regValue.dword; \
        bitPos = 0; \
        while (temp && !(temp & 1)) { \
            bitPos++; \
            temp >>= 1; \
        } \
    } while(0);

static inline void testCrgWriteRegValPrint(U32 regAddr, U32 value)
{
    if (_Thread_Get_executing()) {
        LOGE("testCRG: [0x%08x] = 0x%08x\r\n", regAddr, value);
    }
}

static inline void testCrgSetRegBitPrint(U32 regAddr, U32 bit)
{
    if (_Thread_Get_executing()) {
        LOGE("testCRG: [0x%08x] |= (1 << %d))\r\n", regAddr, bit);
    }
}

static inline void testCrgClearRegBitPrint(U32 regAddr, U32 bit)
{
    if (_Thread_Get_executing()) {
        LOGE("testCRG: [0x%08x] &= ~(1 << %d))\r\n", regAddr, bit);
    }
}
#endif

#endif ///< __SOC_CTRL_H__
