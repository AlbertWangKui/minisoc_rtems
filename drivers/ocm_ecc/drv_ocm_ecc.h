/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_ocm_ecc.h
 * @author zhangxin3@starsmicrosystem.com
 * @date 2025/09/22
 * @brief ecc drver header
 */

#ifndef _DRV_OCM_ECC_H
#define _DRV_OCM_ECC_H

#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "bsp_sbr.h"
#include "sbr_api.h"
#include "udelay.h"
#include "log_msg.h"
#include "osp_interrupt.h"
#include "drv_ocm_ecc_api.h"
#include "common_defines.h"

#define ECC_SEC1_IRQ_MASK (0x1 << 0)
#define ECC_DED1_IRQ_MASK (0x1 << 1)
#define ECC_SEC2_IRQ_MASK (0x1 << 2)
#define ECC_DED2_IRQ_MASK (0x1 << 3)
#define ECC_SEC3_IRQ_MASK (0x1 << 4)
#define ECC_DED3_IRQ_MASK (0x1 << 5)
#define ECC_SEC4_IRQ_MASK (0x1 << 6)
#define ECC_DED4_IRQ_MASK (0x1 << 7)
#define ECC_SEC_ALL_IRQ_MASK (ECC_SEC1_IRQ_MASK | ECC_SEC2_IRQ_MASK | ECC_SEC3_IRQ_MASK | ECC_SEC4_IRQ_MASK)
#define ECC_DED_ALL_IRQ_MASK (ECC_DED1_IRQ_MASK | ECC_DED2_IRQ_MASK | ECC_DED3_IRQ_MASK | ECC_DED4_IRQ_MASK)

typedef union ocmCtrl {
    struct {
        U32 : 2; /* bit0~1 */
        U32 arbitrationType: 1; /* bit2 */
        U32 highPriority : 1; /* bit3 */
        U32 : 28; /* bit4~31 */
    } fields;
    U32 dword;
} ocmCtrl_u;

typedef union intEn {
    struct {
        U32 secIrqEnable : 1; /* bit0 */
        U32 dedIrqEnable : 1; /* bit1 */
        U32 ocp1WrOutIrqEnable : 1; /* bit2 */
        U32 ocp0WrOutIrqEnable : 1; /* bit3 */
        U32 : 28; /* bit4~31 */
    } fields;
    U32 dword;
} intEn_u;

typedef union intSt {
    struct {
        U32 sec1IrqSt : 1; /* bit0 */
        U32 ded1IrqSt : 1; /* bit1 */
        U32 sec2IrqSt : 1; /* bit2 */
        U32 ded2IrqSt : 1; /* bit3 */
        U32 sec3IrqSt : 1; /* bit4 */
        U32 ded3IrqSt : 1; /* bit5 */
        U32 sec4IrqSt : 1; /* bit6 */
        U32 ded4IrqSt : 1; /* bit7 */
        U32 ocp1WrOutIrqSt : 1; /* bit8 */
        U32 ocp0WrOutIrqSt : 1; /* bit9 */
        U32 : 22; /* bit10~31 */
    } fields;
    U32 dword;
} intSt_u;

typedef union intClr {
    struct {
        U32 secClr : 1; /* bit0 */
        U32 dedClr : 1; /* bit1 */
        U32 secCntClr : 1; /* bit2 */
        U32 dedCntClr : 1; /* bit3 */
        U32 ocp1WrOutClr : 1; /* bit4 */
        U32 ocp0WrOutClr : 1; /* bit5 */
        U32 : 26; /* bit6~31 */
    } fields;
    U32 dword;
} intClr_u;

typedef union eccCnt {
    struct {
        U32 secCnt : 4; /* bit0~bit3 */
        U32 dedCnt : 4; /* bit4~bit7 */
        U32 : 24; /* bit8~31 */
    } fields;
    U32 dword;
} eccCnt_u;

typedef union sec1Addr {
    struct {
        U32 sec1Addr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec1Addr_u;

typedef union sec1Dal {
    struct {
        U32 sec1Dal : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec1Dal_u;

typedef union sec1Dah {
    struct {
        U32 sec1Dah : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec1Dah_u;

typedef union ded1Addr {
    struct {
        U32 ded1Addr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded1Addr_u;

typedef union ded1Dal {
    struct {
        U32 ded1Dal : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded1Dal_u;

typedef union ded1Dah {
    struct {
        U32 ded1Dah : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded1Dah_u;

typedef union sec2Addr {
    struct {
        U32 sec2Addr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec2Addr_u;

typedef union sec2Dal {
    struct {
        U32 sec2Dal : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec2Dal_u;

typedef union sec2Dah {
    struct {
        U32 sec2Dah : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec2Dah_u;

typedef union ded2Addr {
    struct {
        U32 ded2Addr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded2Addr_u;

typedef union ded2Dal {
    struct {
        U32 ded2Dal : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded2Dal_u;

typedef union ded2Dah {
    struct {
        U32 ded2Dah : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded2Dah_u;

typedef union sec3Addr {
    struct {
        U32 sec3Addr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec3Addr_u;

typedef union sec3Dal {
    struct {
        U32 sec3Dal : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec3Dal_u;

typedef union sec3Dah {
    struct {
        U32 sec3Dah : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec3Dah_u;

typedef union ded3Addr {
    struct {
        U32 ded3Addr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded3Addr_u;

typedef union ded3Dal {
    struct {
        U32 ded3Dal : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded3Dal_u;

typedef union ded3Dah {
    struct {
        U32 ded3Dah : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded3Dah_u;

typedef union sec4Addr {
    struct {
        U32 sec4Addr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec4Addr_u;

typedef union sec4Dal {
    struct {
        U32 sec4Dal : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec4Dal_u;

typedef union sec4Dah {
    struct {
        U32 sec4Dah : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sec4Dah_u;

typedef union ded4Addr {
    struct {
        U32 ded4Addr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded4Addr_u;

typedef union ded4Dal {
    struct {
        U32 ded4Dal : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded4Dal_u;

typedef union ded4Dah {
    struct {
        U32 ded4Dah : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ded4Dah_u;

typedef union sramStartAddr {
    struct {
        U32 sramStartAddr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sramStartAddr_u;

typedef union sramEndAddr {
    struct {
        U32 sramEndAddr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} sramEndAddr_u;

typedef union ocp0WrOutAddr {
    struct {
        U32 ocp0WrOutAddr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ocp0WrOutAddr_u;

typedef union ocp1WrOutAddr {
    struct {
        U32 ocp1WrOutAddr : 32; /* bit0~31 */
    } fields;
    U32 dword;
} ocp1WrOutAddr_u;

typedef union softEccErr {
    struct {
        U32 softSecErr : 1; /* bit0 */
        U32 softDedErr : 1; /* bit1 */
        U32 softEccIpInj : 1; /* bit2 */
        U32 softEccOpSecInj : 1; /* bit3 */
        U32 softEccOpDedInj : 1; /* bit4 */
        U32 : 27; /* bit5~bit31 */
    } fields;
    U32 dword;
} softEccErr_u;

/* sram ecc寄存器列表 */
typedef volatile struct ocmEccReg {
    ocmCtrl_u ocmCtrl; // 0x00
    intEn_u intEn; // 0x04
    intSt_u intSt; // 0x08
    intClr_u intClr; // 0x0c
    eccCnt_u eccCnt; // 0x10
    sec1Addr_u sec1Addr; // 0x14
    sec1Dal_u sec1Dal; // 0x18
    sec1Dah_u sec1Dah; // 0x1c
    ded1Addr_u ded1Addr; // 0x20
    ded1Dal_u ded1Dal; // 0x24
    ded1Dah_u ded1Dah; // 0x28
    U32 reserved1; // 0x2c
    sec2Addr_u sec2Addr; // 0x30
    sec2Dal_u sec2Dal; // 0x34
    sec2Dah_u sec2Dah; // 0x38
    ded2Addr_u ded2Addr; // 0x3c
    ded2Dal_u ded2Dal; // 0x40
    ded2Dah_u ded2Dah; // 0x44
    U32 reserved2; // 0x48
    sec3Addr_u sec3Addr; // 0x4c
    sec3Dal_u sec3Dal; // 0x50
    sec3Dah_u sec3Dah; // 0x54
    ded3Addr_u ded3Addr; // 0x58
    ded3Dal_u ded3Dal; // 0x5c
    ded3Dah_u ded3Dah; // 0x60
    U32 reserved3; // 0x64
    sec4Addr_u sec4Addr; // 0x68
    sec4Dal_u sec4Dal; // 0x6c
    sec4Dah_u sec4Dah; // 0x70
    ded4Addr_u ded4Addr; // 0x74
    ded4Dal_u ded4Dal; // 0x78
    ded4Dah_u ded4Dah; // 0x7c
    U32 reserved4; // 0x80
    sramStartAddr_u sramStartAddr; // 0x84
    sramEndAddr_u sramEndAddr; // 0x88
    ocp0WrOutAddr_u ocp0WrOutAddr; // 0x8c
    ocp1WrOutAddr_u ocp1WrOutAddr; // 0x90
    softEccErr_u softEccErr; // 0x94
} ocmEccReg_s;

typedef struct OcmEccDrvPrivateData {
    SbrOcmEccCfg_s   sbrCfg; ///< ecc设备配置
    pOcmEccCallback  callback;  ///< 中断回调
    ocmEccReg_s *eccReg;
    U32 lastSecAddr;
    U64 lastSecData;
    U32 lastDedAddr;
    U64 lastDedData;
} ocmEccDrvPrivateData_s;
#endif
