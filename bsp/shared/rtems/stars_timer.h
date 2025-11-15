/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file stars_timer.h
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/06/24
 * @brief RTEMS定时器驱动头文件
 */

#ifndef _STARS_TIMER_H
#define _STARS_TIMER_H

#include "common_defines.h"

typedef union divisor {
    struct {
        U32 divisor : 16; /* bit0~15 */
        U32 reserved : 16; /* bit16~31 */
    } fields;
    U32 dword;
} Divisor_u;

typedef union irqSta {
    struct {
        U32 irqStat : 1; /* bit0 */
        U32 cntFlag : 1; /* bit1 */
        U32 reserved : 30; /* bit2~31 */
    } fields;
    U32 dword;
} IrqStat_u;

typedef union irqEn {
    struct {
        U32 irqEn : 1; /* bit0 */
        U32 reserved : 31; /* bit1~31 */
    } fields;
    U32 dword;
} IrqEn_u;

typedef union tmrEn {
    struct {
        U32 en : 1; /* bit0 */
        U32 oneShot : 1; /* bit1 */
        U32 reserved : 30; /* bit2~31 */
    } fields;
    U32 dword;
} TmrEn_u;

/* Timer寄存器列表 */
typedef volatile struct TmrReg {
    Divisor_u divisor; /* 0x00 */
    U32 setlVal; /* 0x04 */
    U32 sethVal; /* 0x08 */
    U32 rdlVal; /* 0x0c */
    U32 rdhVal; /* 0x10 */
    IrqStat_u irqStat; /* 0x14 */
    IrqEn_u irqEn; /* 0x18 */
    TmrEn_u tmrEn; /* 0x1c */
} TmrReg_s;

#endif
