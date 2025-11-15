/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_wdt.c
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/06/05
 * @brief 看门狗驱动实现
 */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <bsp/arm-gic-irq.h>
#include <bsp/irq-generic.h>
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "bsp_sbr.h"
#include "sbr_api.h"
#include "udelay.h"
#include "log_msg.h"
#include "osp_interrupt.h"
#include "drv_wdt_api.h"
#include "drv_wdt.h"


///< 分辨率为0.1ms
#define WATCHDOG_MILLISECOND_TO_VAL (10)

#define WATCHDOG_UNLOCK_VALUE       (0x1acce551) ///< 写入该值解锁
#define WATCHDOG_LOCK_VALUE         (0x1)   ///< 写入非0x1acce551的值则锁寄存器;
#define WATCHDOG_LONGEST_TIME_MS    (3600000) ///< 看门狗最长时间

__attribute__((unused)) static void wdtISR(void *arg);
static S32 wdtDevCfgGet(DevList_e devId, SbrWdtCfg_s *pWdtDevCfg);

static inline S32 wdtCalcDiv(U32 *div)
{
    U32 clk;
    S32 ret = EXIT_SUCCESS;

    if (div == NULL) {
        return -EINVAL;
    }

    if (peripsClockFreqGet(DEVICE_WDT0,&clk) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    *div = (clk / (1000 * 10) - 1);

exit:
    return ret;
}

static S32 wdtDevCfgGet(DevList_e devId, SbrWdtCfg_s *pWdtDevCfg)
{
    S32 ret = EXIT_SUCCESS;
    U32 readSize;

    if (pWdtDevCfg == NULL) {
        ret = -EIO;
        goto exit;
    }

    /* 从SBR读取WDT配置 */
    readSize = devSbrRead(devId, pWdtDevCfg, 0, sizeof(SbrWdtCfg_s));
    if (readSize != sizeof(SbrWdtCfg_s)) {
        LOGE("wdt: failed to read WDT config from SBR, readSize=%u, expected=%u\r\n",
             readSize, sizeof(SbrWdtCfg_s));
        ret = -EIO;
        goto exit;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("wdt: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, reserved:0x%08x\r\n",
         pWdtDevCfg->regAddr, pWdtDevCfg->irqNo, pWdtDevCfg->irqPrio, pWdtDevCfg->reserved);
#endif

    /* 验证配置参数 */
    if ((pWdtDevCfg->workMode >= WDT_MODE_NR) ||
        (pWdtDevCfg->feedTime == 0) ||
        (pWdtDevCfg->resetTime == 0) ||
        (pWdtDevCfg->checkTime == 0)) {
        LOGE("wdt: invalid config from SBR, workMode:%u, feedTime:%u, resetTime:%u, checkTime:%u\r\n",
            pWdtDevCfg->workMode, pWdtDevCfg->feedTime,
            pWdtDevCfg->resetTime, pWdtDevCfg->checkTime);
        ret = -EIO;
        goto exit;
    }

    if (pWdtDevCfg->feedTime > WATCHDOG_LONGEST_TIME_MS ||
        pWdtDevCfg->resetTime > WATCHDOG_LONGEST_TIME_MS ||
        pWdtDevCfg->checkTime > WATCHDOG_LONGEST_TIME_MS) {
        LOGE("wdt: time config too long from SBR, feedTime:%u, resetTime:%u, checkTime:%u\r\n",
            pWdtDevCfg->feedTime, pWdtDevCfg->resetTime, pWdtDevCfg->checkTime);
        ret = -EIO;
        goto exit;
    }

    return EXIT_SUCCESS;

exit:
    return ret;
}

__attribute__((unused)) static void wdtISR(void *arg)
{
    WdtDrvPrivateData_s *pWdtDrvData = (WdtDrvPrivateData_s*)arg;
    WdtReg_s *wdtReg = pWdtDrvData->sbrCfg.regAddr;

    if(pWdtDrvData->sbrCfg.workMode == WDT_RESET && pWdtDrvData->triggered == true) {
        return;
    }

    pWdtDrvData->notFeedCount++; ///< 增加连续未喂狗计数

    switch(pWdtDrvData->sbrCfg.workMode) {
        case WDT_RESET:
            if (pWdtDrvData->notFeedCount >= pWdtDrvData->notFeedMax) {
                pWdtDrvData->triggered = true; ///< 设置触发标记
                if(pWdtDrvData->callback != NULL) {
                    pWdtDrvData->callback(); ///< 调用中断回调函数
                }
                wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
                wdtReg->loadData = pWdtDrvData->rebootTimeMs * (WATCHDOG_MILLISECOND_TO_VAL); ///< 设置重启时间
                wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器
            } else {
                wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
                wdtReg->intrClear = 1;         ///< 清除中断
                wdtReg->itcr = 0;              ///< 写0清除软件中断和复位
                wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器
            }
            break;

        case WDT_INT:
                wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
                wdtReg->intrClear = 1;         ///< 清除中断
                wdtReg->itcr = 0;              ///< 写0清除软件中断和复位
                wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器

            if (pWdtDrvData->notFeedCount >= pWdtDrvData->notFeedMax) {
                if (pWdtDrvData->callback != NULL) {
                    pWdtDrvData->callback(); ///< 调用中断回调函数
                }
                pWdtDrvData->notFeedCount = 0; ///< 重置连续未喂狗计数
            }
            break;

        default:
            break;
    }
    return;
}

S32 wdtInit(DevList_e devId, pWdtCallback callback)
{
    S32 ret = EXIT_SUCCESS;
    WdtDrvPrivateData_s *pWdtDrvData = NULL;
    WdtReg_s *wdtReg = NULL;
    U32 div = 1;

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        return -EINVAL;
    }

    ///< 检查设备是否存在，驱动及配置文件是否匹配
    if (isDrvInit(devId) == true) {
        ret = -EBUSY;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    peripsClockEnable(devId);
    ///< reset the hardware
    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if (wdtCalcDiv(&div) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    ///< 申请驱动私有数据内存并获取设备配置
    pWdtDrvData = (WdtDrvPrivateData_s*)calloc(1, sizeof(WdtDrvPrivateData_s));
    if (pWdtDrvData == NULL) {
        ret = -EIO;
        goto unlock;
    }

    if (wdtDevCfgGet(devId, &pWdtDrvData->sbrCfg) != EXIT_SUCCESS) { ///< 获取设备配置
        ret = -EIO;
        goto freeMem;
    }

    ///< 中断模式需要回调函数
    if(callback == NULL) {
        LOGE("wdt: interrupt mode need isrCallback! \r\n");
        ret = -EINVAL;
        goto freeMem;
    }

    pWdtDrvData->callback = callback; ///< 设置中断回调函数
    ospInterruptHandlerInstall(pWdtDrvData->sbrCfg.irqNo, "watch dog",
        OSP_INTERRUPT_UNIQUE, (void (*)(void*))wdtISR, pWdtDrvData);

    if (arm_gic_irq_set_priority(pWdtDrvData->sbrCfg.irqNo, pWdtDrvData->sbrCfg.irqPrio) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freeMem;
    }

    pWdtDrvData->triggered = false; ///< 初始化触发标记
    pWdtDrvData->notFeedCount = 0; ///< 初始化连续未喂狗计数
    pWdtDrvData->rebootTimeMs = pWdtDrvData->sbrCfg.resetTime; ///< 设置重启时间为配置的重启时间
    pWdtDrvData->notFeedMax = (pWdtDrvData->sbrCfg.feedTime + pWdtDrvData->sbrCfg.checkTime - 1) / pWdtDrvData->sbrCfg.checkTime; ///< 计算连续未喂狗计数最大值

    wdtReg = pWdtDrvData->sbrCfg.regAddr;
    wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
    wdtReg->loadData = pWdtDrvData->sbrCfg.checkTime * (WATCHDOG_MILLISECOND_TO_VAL);
    wdtReg->div = div; ///< 设置分频寄存器
    wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器

    ///< 安装设备驱动
    if (drvInstall(devId, (void*)pWdtDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    ///< start the watchdog
    bsp_interrupt_vector_enable(pWdtDrvData->sbrCfg.irqNo);
    ret = EXIT_SUCCESS;
    goto unlock;

freeMem:
    free(pWdtDrvData);

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 wdtFeed(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *wdtReg = NULL;
    WdtDrvPrivateData_s *pWdtDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        return -EINVAL;
    }

    ret = getDevDriver(devId,(void **)&pWdtDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pWdtDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    wdtReg = pWdtDrvData->sbrCfg.regAddr;
    wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
    wdtReg->loadData = 0; ///< 喂狗，清除计数
    wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器
out:
    return ret;
}

S32 wdtStart(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    U32 wdtMode = 0;
    WdtReg_s *wdtReg = NULL;
    WdtDrvPrivateData_s *pWdtDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        return -EINVAL;
    }

    ret = getDevDriver(devId,(void **)&pWdtDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pWdtDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    if (pWdtDrvData->sbrCfg.workMode == WDT_RESET) {
        ///< 重启模式
        SET_BIT(wdtMode, 1);   ///< set bit[1], 使能自动重启
        SET_BIT(wdtMode, 0);   ///< set bit[0], 使能看门狗计数和中断
    } else if(pWdtDrvData->sbrCfg.workMode == WDT_INT) {
        ///< 中断模式
        CLR_BIT(wdtMode, 1);   ///< clear bit[1], 关闭自动重启
        SET_BIT(wdtMode, 0);   ///< set bit[0], 使能看门狗计数和中断
    } else {
        ret = -EIO;
        goto out;
    }

    wdtReg = pWdtDrvData->sbrCfg.regAddr;
    wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
    wdtReg->intrEn &= ~0x3; ///< 清除看门狗计数和中断使能
    wdtReg->intrEn |= wdtMode; ///< 使能看门狗计数和中断
    wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器

out:
    return ret;
}

S32 wdtStop(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *wdtReg = NULL;
    WdtDrvPrivateData_s *pWdtDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        return -EINVAL;
    }

    ret = getDevDriver(devId,(void **)&pWdtDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pWdtDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    wdtReg = pWdtDrvData->sbrCfg.regAddr;
    wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
    wdtReg->intrEn = 0;            ///< 禁止复位和中断使能
    wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器
out:
    return ret;
}

S32 wdtIrqClr(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *wdtReg = NULL;
    WdtDrvPrivateData_s *pWdtDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        return -EINVAL;
    }

    ret = getDevDriver(devId,(void **)&pWdtDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pWdtDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    wdtReg = pWdtDrvData->sbrCfg.regAddr;
    wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
    wdtReg->intrClear = 1;         ///< 清除中断
    wdtReg->itcr = 0;              ///< 写0清除软件中断和复位
    wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器
out:
    return ret;
}

S32 wdtDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtDrvPrivateData_s *pWdtDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        return -EINVAL;
    }

    ret = getDevDriver(devId,(void **)&pWdtDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pWdtDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    if (pWdtDrvData->callback != NULL) {
        ospInterruptHandlerRemove(pWdtDrvData->sbrCfg.irqNo, (void (*)(void*))wdtISR, pWdtDrvData);
        ospInterruptVectorUninit(pWdtDrvData->sbrCfg.irqNo);
    }

    peripsReset(devId); ///< 复位外设
    drvUninstall(devId); ///< 卸载设备
out:
    return ret;
}

S32 wdtSetTimeout(DevList_e devId, U32 timeOutMs)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *wdtReg = NULL;
    WdtDrvPrivateData_s *pWdtDrvData = NULL;

    if (timeOutMs > WATCHDOG_LONGEST_TIME_MS) {
        ret = -EINVAL;
        goto out;
    }

    ret = getDevDriver(devId,(void **)&pWdtDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    if (pWdtDrvData == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    wdtReg = pWdtDrvData->sbrCfg.regAddr;

    wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
    wdtReg->loadData = timeOutMs * (WATCHDOG_MILLISECOND_TO_VAL);
    wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器

    pWdtDrvData->sbrCfg.checkTime = timeOutMs;
out:
    return ret;
}

S32 wdtSetMode(DevList_e devId, WdtMode_e mode)
{
    S32 ret = EXIT_SUCCESS;
    U32 wdtMode = 0;
    U32 intrEn = 0;
    WdtReg_s *wdtReg = NULL;
    WdtDrvPrivateData_s *pWdtDrvData = NULL;

    if (mode != WDT_RESET && mode != WDT_INT) {
        LOGE("%s: invalid wdt mode %d!\r\n", __func__, mode);
        ret = -EINVAL;
        goto out;
    }

    ret = getDevDriver(devId, (void **)&pWdtDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    if (pWdtDrvData == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    wdtReg = pWdtDrvData->sbrCfg.regAddr;
    if (wdtReg == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }
#if 0
    if (mode == WDT_RESET && pWdtDrvData->callback == NULL) {
        LOGE("wdt: reset mode need isrCallback! \r\n");
        ret = -EXIT_FAILURE;
        goto out;
    }
    else {
        ospInterruptHandlerInstall(pWdtDrvData->sbrCfg.irqNo, "watch dog",
                                   OSP_INTERRUPT_UNIQUE, (void (*)(void *))wdtISR, pWdtDrvData);

        if (0 != ospInterruptSetPriority(pWdtDrvData->sbrCfg.irqNo, pWdtDrvData->sbrCfg.irqPrio)) {
            ret = -EXIT_FAILURE;
            goto out;
        }
        ospInterruptVectorEnable(pWdtDrvData->sbrCfg.irqNo);
    }
#endif
    if (mode == WDT_RESET) {
        ///< 重启模式
        SET_BIT(wdtMode, 1);   ///< set bit[1], 使能自动重启
//      SET_BIT(wdtMode, 0);   ///< set bit[0], 使能看门狗计数和中断
    }
    else /* if (mode == WDT_INT) */ {
        ///< 中断模式
        CLR_BIT(wdtMode, 1);   ///< clear bit[1], 关闭自动重启
//      SET_BIT(wdtMode, 0);   ///< set bit[0], 使能看门狗计数和中断
    }

    intrEn = wdtReg->intrEn;
    CLR_BIT(intrEn, 1);
    intrEn |= wdtMode;
    wdtReg->lock = WATCHDOG_UNLOCK_VALUE; ///< 解锁寄存器
    wdtReg->intrEn = intrEn; ///< 使能看门狗计数和中断
    wdtReg->lock = WATCHDOG_LOCK_VALUE; ///< 锁寄存器

    pWdtDrvData->sbrCfg.workMode = mode;

out:
    return ret;
}
