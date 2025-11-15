/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_timer.h
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/08/09
 * @brief timer driver
 */

#ifndef __DRV_TIMER_H__
#define __DRV_TIMER_H__

#include <stdio.h>
#include <bsp/arm-gic-irq.h>
#include <bsp/irq-generic.h>
#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_sbr.h"
#include "sbr_api.h"
#include "osp_interrupt.h"

typedef union TimerDivisor {
    struct {
        U32 divisor : 16; /* bit0~15 */
        U32 reserved : 16; /* bit16~31 */
    } fields;
    U32 dword;
} TimerDivisor;

typedef union TimerIrqSta {
    struct {
        U32 irqSta : 1; /* bit0 */
        U32 cntFlag : 1; /* bit1 */
        U32 reserved : 30; /* bit2~31 */
    } fields;
    U32 dword;
} TimerIrqSta;

typedef union TimerIrqEn {
    struct {
        U32 irqEn : 1; /* bit0 */
        U32 reserved : 31; /* bit1~31 */
    } fields;
    U32 dword;
} TimerIrqEn;

typedef union TimerEn {
    struct {
        U32 en : 1; /* bit0 */
        U32 timerOneshot : 1; /* bit1 */
        U32 reserved : 30; /* bit2~31 */
    } fields;
    U32 dword;
} TimerEn;

typedef union TimerCntDir {
    struct {
        U32 cntDir : 1; /* bit0 */
        U32 reserved : 31; /* bit1~31 */
    } fields;
    U32 dword;
} TimerCntDir;

/* Timer寄存器列表 */
typedef volatile struct TimerReg {
    volatile TimerDivisor divisor; /* 0x00 */
    volatile U32 setlVal; /* 0x04 */
    volatile U32 sethVal; /* 0x08 */
    volatile U32 rdlVal; /* 0x0c */
    volatile U32 rdhVal; /* 0x10 */
    volatile TimerIrqSta irqSta; /* 0x14 */
    volatile TimerIrqEn irqEn; /* 0x18 */
    volatile TimerEn timerEnable; /* 0x1c */
    volatile TimerCntDir cntDir; /* 0x1c */
} TimerReg;

typedef struct TimerDrvData {
    SbrTimerCfg_s sbrCfg;
    void (*callback)(void);
} TimerDrvData_s;

#endif /* __DRV_TIMER_H__ */
