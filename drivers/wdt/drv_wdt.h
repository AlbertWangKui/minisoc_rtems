/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_wdt.h
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025/10/22
 * @brief  watchdog driver header file for minisoc
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/10/22   yangkl         the first version
 *
 */

#ifndef __DRV_WDT_H__
#define __DRV_WDT_H__

#include "common_defines.h"
#include "bsp_sbr.h"

#define WATCHDOG_INTR_ENABLE_MASK     (0x3)         ///< Watchdog interrupt enable mask
#define WATCHDOG_LOCK_TIMEOUT_MS      (1000)        ///< Watchdog lock register timeout 1000ms
#define WATCHDOG_UNLOCK_VALUE         (0x1acce551)  ///< Write this value to unlock the register
#define WATCHDOG_LOCK_VALUE           (0x1)         ///< Write any value other than 0x1acce551 to lock the register
#define WATCHDOG_MAX_TIME_MS          (60000)     ///< Maximum watchdog timeout 60000ms
#define WATCHDOG_LOADDATA_REG_MAX_VAL (0xffffffff)  ///< Maximum value of watchdog load data register
#define WATCHDOG_CALC_FEED_CNT(feedTimeMs, clkHz, div)   (((U64)(feedTimeMs) * (clkHz) / 1000) /(div))  ///< Calculate feed count, feedTimeMs unit is ms
#define WATCHDOG_RST_ENABLE_BIT       (1)
#define WATCHDOG_INT_ENABLE_BIT       (0)

typedef struct WdtReg {
    volatile U32 loadData;     ///< 0x000 Load value register
    volatile U32 curCount;     ///< 0x004 Current counter register
    volatile U32 intrEn;       ///< 0x008 Interrupt enable register
    volatile U32 intrClear;    ///< 0x00C Interrupt clear register
    volatile U32 intrRawStat;  ///< 0x010 Raw interrupt status register
    volatile U32 intrStat;     ///< 0x014 Interrupt status register
    volatile U32 div;          ///< 0x018 Prescaler register
    volatile U32 rsvd0[761];   ///< 0x01C~0xBFC Reserved register
    volatile U32 lock;         ///< 0xC00 Lock register
    volatile U32 rsvd1[191];   ///< 0xC04~0xEFC Reserved register
    volatile U32 itcr;         ///< 0xF00 Test interrupt reset enable register
    volatile U32 topa;         ///< 0xF04 Test interrupt/reset register
} WdtReg_s;

typedef struct WdtDrvData {
    SbrWdtCfg_s   sbrCfg;         ///< Watchdog device configuration
    pWdtCallback  callback;       ///< Interrupt callback function pointer
    U32           feedCnt;        ///< Feed dog count
    U32           clkHz;          ///< wdt Clock frequency in Hz
    U32           workMode;       ///< wdt current Work mode
    bool          triggered;      ///< wdt triggered flag
} WdtDrvData_s;
#endif ///< __DRV_WDT_H__

