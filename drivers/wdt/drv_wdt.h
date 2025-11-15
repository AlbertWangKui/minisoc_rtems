/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_wdt.h
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/06/05
 * @brief
 */

#ifndef __DRV_WATCHDOG_H__
#define __DRV_WATCHDOG_H__

#include "common_defines.h"
#include "bsp_sbr.h"

typedef struct wdtReg {
    volatile U32 loadData;
    volatile U32 curCount;
    volatile U32 intrEn;
    volatile U32 intrClear;
    volatile U32 intrRawStat;
    volatile U32 intrStat;
    volatile U32 div;
    volatile U32 lock;
    volatile U32 itcr;
    volatile U32 topa;
} WdtReg_s;

///< 看门狗全局变量结构体
typedef struct wdtDrvPrivateData
{
    SbrWdtCfg_s   sbrCfg; ///< 看门狗设备配置
    pWdtCallback  callback;  ///< 中断回调
    U32           notFeedCount; ///< 检测到的连续多少次没有喂狗
    U32           notFeedMax;   ///< 连续没喂狗的次数到达该值时触发看门狗
    U32           rebootTimeMs; ///< 看门狗触发后等待重启的时间，单位ms
    Bool          triggered; ///< 是否触发过
} WdtDrvPrivateData_s;


#endif ///< __WDT_H__

