/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_wdt.c
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025/10/22
 * @brief  watchdog driver for minisoc
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/10/22   yangkl         the first version
 *
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

static S32 wdtDevCfgGet(DevList_e devId, SbrWdtCfg_s *pWdtSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (pWdtSbrCfg == NULL) {
        ret = -EIO;
        goto exit;
    }

    if (devSbrRead(devId, pWdtSbrCfg, 0, sizeof(SbrWdtCfg_s)) != sizeof(SbrWdtCfg_s)) {
        LOGE("wdt: failed to read WDT config from SBR\r\n");
        ret = -EIO;
        goto exit;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("wdt: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, workMode:%u, feedTime:%u, div:%u\r\n",
         pWdtSbrCfg->regAddr, pWdtSbrCfg->irqNo, pWdtSbrCfg->irqPrio, pWdtSbrCfg->workMode, 
         pWdtSbrCfg->feedTime, pWdtSbrCfg->div);    
#endif

    /* Validate configuration parameters */
    if ((pWdtSbrCfg->workMode >= WDT_MODE_NR) ||
        (pWdtSbrCfg->feedTime == 0) || (pWdtSbrCfg->div == 0)) {
        LOGE("wdt: invalid config from SBR, workMode:%u, feedTime:%u, div:%u\r\n",
            pWdtSbrCfg->workMode, pWdtSbrCfg->feedTime, pWdtSbrCfg->div);
        ret = -EIO;
        goto exit;
    }

    if (pWdtSbrCfg->feedTime > WATCHDOG_MAX_TIME_MS) {
        LOGE("wdt: time config too long from SBR, feedTime:%u\r\n",
            pWdtSbrCfg->feedTime);
        ret = -EIO;
        goto exit;
    }

exit:
    return ret;
}

static void wdtIrqHandler(void *pWdtDrvData)
{
    WdtDrvData_s *pDrvData = (WdtDrvData_s*)pWdtDrvData;
    WdtReg_s *pReg = NULL;

    if (pDrvData == NULL) {
        return;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        return;
    }

    if(pDrvData->workMode == WDT_MODE_RESET && pDrvData->triggered == true) {
        return;
    }

    switch(pDrvData->workMode) {
            case WDT_MODE_RESET:
                if (pDrvData->callback != NULL) {
                    pDrvData->callback();           ///< Call interrupt callback function
                }
                pDrvData->triggered = true;         ///< Set trigger flag
                pReg->lock = WATCHDOG_UNLOCK_VALUE; ///< Unlock register
                pReg->intrClear = 1;                ///< Clear interrupt
                pReg->loadData = pDrvData->feedCnt; ///< Set restart time
                pReg->lock = WATCHDOG_LOCK_VALUE;   ///< Lock register
                break;
            case WDT_MODE_INT:
                pReg->lock = WATCHDOG_UNLOCK_VALUE; ///< Unlock register
                pReg->intrClear = 1;                ///< Clear interrupt
                pReg->loadData = pDrvData->feedCnt; ///< Set restart time
                pReg->lock = WATCHDOG_LOCK_VALUE;   ///< Lock register
                if (pDrvData->callback != NULL) {
                    pDrvData->callback();           ///< Call interrupt callback function
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
    WdtDrvData_s *pDrvData = NULL;
    WdtReg_s *pReg = NULL;
    bool irqInstalled = false;
    U64 feedCnt = 0;
    U32 clk = 0;

    if (devLockByDriver(devId, WATCHDOG_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGW("%s: device lock failed, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (isDrvInit(devId) == true) {
        LOGW("%s: device already initialized, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        LOGW("%s: driver not match, devId=%d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to enable peripheral clock, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to reset peripheral, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsClockFreqGet(devId, &clk) != EXIT_SUCCESS) {
        LOGE("%s: failed to get peripheral clock frequency, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (clk == 0) {
        LOGE("%s: Invalid clock frequency %u Hz, devId=%d\r\n", __func__, clk, devId);
        ret = -EIO;
        goto unlock;
    }

    pDrvData = (WdtDrvData_s *)calloc(1, sizeof(WdtDrvData_s));
    if (pDrvData == NULL) {
        LOGE("%s: failed to allocate driver data, devId=%d\r\n", __func__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    if (wdtDevCfgGet(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s: failed to get device configuration, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto freeMem;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        ret = -EIO;
        LOGE("wdt: %s: register address is NULL, devId=%d\r\n", __func__, devId);
        goto freeMem;
    }

    if(callback != NULL) {
        pReg->intrEn = 0;        ///< Disable reset and interrupt enable
        ret = ospInterruptHandlerInstall(pDrvData->sbrCfg.irqNo, "wdt",
                                        OSP_INTERRUPT_UNIQUE,
                                        (OspInterruptHandler)wdtIrqHandler,
                                        pDrvData);
        if (ret == OSP_RESOURCE_IN_USE) {
            LOGW("%s: IRQ handler already installed\r\n", __func__);
            irqInstalled = true;
            ret = EXIT_SUCCESS; ///< Correctly handle OSP_RESOURCE_IN_USE error
        } else if (ret == OSP_SUCCESSFUL) {
            ospInterruptSetPriority(pDrvData->sbrCfg.irqNo, pDrvData->sbrCfg.irqPrio);
            ospInterruptVectorEnable(pDrvData->sbrCfg.irqNo);
            irqInstalled = true;
            LOGI("%s: IRQ handler installed successfully\r\n", __func__);
        } else {
            LOGE("%s: failed to install IRQ handler, ret=%d\r\n", __func__, ret);
            ret = -EIO;
            goto freeMem;
        }

        pDrvData->callback = callback;
    }

    feedCnt = WATCHDOG_CALC_FEED_CNT(pDrvData->sbrCfg.feedTime, clk, pDrvData->sbrCfg.div);
    if (feedCnt > WATCHDOG_LOADDATA_REG_MAX_VAL) {
        LOGE("%s: calculated feed count %llu exceeds maximum %u\r\n", 
             __func__, feedCnt, WATCHDOG_LOADDATA_REG_MAX_VAL);
        ret = -EINVAL;
        goto removeIrq;
    }

    /* Initialize driver private data */
    pDrvData->feedCnt = (U32)feedCnt;
    pDrvData->clkHz = clk; ///< Record clock frequency
    pDrvData->workMode = pDrvData->sbrCfg.workMode; ///< Record current work mode
    pDrvData->triggered = false; ///< Initialize trigger flag

    if (pDrvData->sbrCfg.regAddr == NULL) {
        ret = -EIO;
        LOGE("wdt: %s: register address is NULL, devId=%d\r\n", __func__, devId);
        goto removeIrq;
    }

    /* Configure WDT hardware */
    pReg->lock = WATCHDOG_UNLOCK_VALUE; ///< Unlock register
    pReg->loadData = pDrvData->feedCnt; ///< Set load data register
    pReg->div = (pDrvData->sbrCfg.div - 1); ///< Set divider register
    pReg->lock = WATCHDOG_LOCK_VALUE;   ///< Lock register

    if (drvInstall(devId, (void *)pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: failed to install driver, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto removeIrq;
    }

    devUnlockByDriver(devId);
    return EXIT_SUCCESS;

removeIrq:
    if (irqInstalled) {
        ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo,
                                   (OspInterruptHandler)wdtIrqHandler,
                                   pDrvData);
        ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
    }

freeMem:
    free(pDrvData);
    pDrvData = NULL;

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 wdtDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtDrvData_s *pDrvData = NULL;
    WdtReg_s *pReg = NULL;

    if (devLockByDriver(devId, WATCHDOG_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: failed to acquire device lock, devId=%d!\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        LOGE("%s: driver ID mismatch, devId=%d!\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (isDrvInit(devId) == false) {
        LOGW("%s: device already deinitialized, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    if (getDevDriver(devId, (void **)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: failed to get driver data, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        ret = -EIO;
        LOGE("wdt: %s: driver data is NULL, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s: register address is NULL, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        ret = -EIO;
        LOGE("wdt: %s: register address is NULL, devId=%d\r\n", __func__, devId);
        goto unlock;
    }

    pReg->lock = WATCHDOG_UNLOCK_VALUE;
    pReg->intrEn = 0;  ///< Disable WDT
    pReg->lock = WATCHDOG_LOCK_VALUE;

    if (pDrvData->callback != NULL) {
        ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
        ret = ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo,
                                        (OspInterruptHandler)wdtIrqHandler,
                                        pDrvData);
        if (ret != OSP_SUCCESSFUL) {
            LOGE("%s: failed to remove IRQ handler, ret=%d\r\n", __func__, ret);
        }
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to reset peripheral, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
    }

    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to uninstall driver, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
    }

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 wdtFeed(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *pReg = NULL;
    WdtDrvData_s *pDrvData = NULL;

    if (devLockByDriver(devId, WATCHDOG_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        ret = -EBUSY;
        LOGE("%s: failed to acquire device lock, devId=%d!\r\n", __func__, devId);
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        ret = -EINVAL;
        LOGE("%s: driver ID mismatch, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    if (isDrvInit(devId) == false) {
        LOGW("%s: device not initialized, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    ret = getDevDriver(devId,(void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to get driver data, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        ret = -EIO;
        LOGE("%s: driver data is NULL, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s: register address is NULL, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE;          ///< Unlock register
    pReg->loadData = pDrvData->feedCnt;          ///< Feed watchdog, clear counter
    pReg->lock = WATCHDOG_LOCK_VALUE;            ///< Lock register

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 wdtStart(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    U32 workMode = 0;
    WdtReg_s *pReg = NULL;
    WdtDrvData_s *pDrvData = NULL;

    if (devLockByDriver(devId, WATCHDOG_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: failed to acquire device lock, devId=%d!\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        LOGE("%s: driver ID mismatch, devId=%d!\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (isDrvInit(devId) == false) {
        LOGW("%s: device not initialized, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    ret = getDevDriver(devId,(void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        LOGE("%s: failed to get driver data, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    if (pDrvData == NULL) {
        ret = -EIO;
        LOGE("%s: driver data is NULL, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        ret = -EIO;
        LOGE("%s: register address is NULL, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    if (pDrvData->workMode == WDT_MODE_RESET) {
        /* Reset mode */
        SET_BIT(workMode, WATCHDOG_RST_ENABLE_BIT);   ///< set bit[1], enable RST output
        SET_BIT(workMode, WATCHDOG_INT_ENABLE_BIT);   ///< set bit[0], enable interrupt output
    } else if(pDrvData->workMode == WDT_MODE_INT) {
        /* Interrupt mode */
        CLR_BIT(workMode, WATCHDOG_RST_ENABLE_BIT);   ///< clear bit[1], disable RST output
        SET_BIT(workMode, WATCHDOG_INT_ENABLE_BIT);   ///< set bit[0], enable interrupt output
    } else {
        ret = -EIO;
        LOGE("%s: invalid work mode, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE; ///< Unlock register
    pReg->intrEn &= ~WATCHDOG_INTR_ENABLE_MASK;  ///< Clear watchdog count and interrupt enable
    pReg->intrEn |= workMode;            ///< Enable watchdog count and interrupt
    pReg->lock = WATCHDOG_LOCK_VALUE;   ///< Lock register

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 wdtStop(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *pReg = NULL;
    WdtDrvData_s *pDrvData = NULL;

    if (devLockByDriver(devId, WATCHDOG_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: failed to acquire device lock, devId=%d!\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        LOGE("%s: driver ID mismatch, devId=%d!\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (isDrvInit(devId) == false) {
        LOGW("%s: device not initialized, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    ret = getDevDriver(devId,(void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        LOGE("%s: failed to get driver data, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    if (pDrvData == NULL) {
        ret = -EIO;
        LOGE("%s: driver data is NULL, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        ret = -EIO;
        LOGE("%s: register address is NULL, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE; ///< Unlock register
    pReg->intrEn = 0;                   ///< Disable reset and interrupt enable
    pReg->lock = WATCHDOG_LOCK_VALUE;   ///< Lock register

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 wdtIrqClr(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *pReg = NULL;
    WdtDrvData_s *pDrvData = NULL;

    if (devLockByDriver(devId, WATCHDOG_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: failed to acquire device lock, devId=%d!\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        LOGE("%s: driver ID mismatch, devId=%d!\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (isDrvInit(devId) == false) {
        LOGW("%s: device not initialized, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    ret = getDevDriver(devId,(void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to get driver data, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        ret = -EIO;
        LOGE("%s: driver data is NULL, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        ret = -EIO;
        LOGE("%s: register address is NULL, devId=%d!\r\n", __func__, devId);
        goto unlock;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE; ///< Unlock register
    pReg->intrClear = 1;                ///< Clear interrupt
    pReg->lock = WATCHDOG_LOCK_VALUE;   ///< Lock register

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 wdtSetTimeout(DevList_e devId, U32 timeOutsMs)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *pReg = NULL;
    WdtDrvData_s *pDrvData = NULL;
    U64 feedCnt = 0;

    if ((timeOutsMs > WATCHDOG_MAX_TIME_MS) || (timeOutsMs == 0)) {
        LOGE("%s: invalid timeout value %u, must be between 0 and %u ms, devId=%d!\r\n",
             __func__, timeOutsMs, WATCHDOG_MAX_TIME_MS, devId);
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, WATCHDOG_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: failed to acquire device lock, devId=%d!\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }
    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        LOGE("%s: driver ID mismatch, devId=%d!\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (isDrvInit(devId) == false) {
        LOGW("%s: device not initialized, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    ret = getDevDriver(devId,(void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to get driver data, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: driver data is NULL, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s: register address is NULL, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Check if divisor is zero */
    if (pDrvData->sbrCfg.div == 0) {
        LOGE("%s: division by zero, div=%u\r\n", __func__, pDrvData->sbrCfg.div);
        ret = -EINVAL;
        goto unlock;
    }

    /* Check if clock frequency is valid */
    if (pDrvData->clkHz == 0) {
        LOGE("%s: Invalid clock frequency %u Hz\r\n", __func__, pDrvData->clkHz);
        ret = -EINVAL;
        goto unlock;
    }

    feedCnt = WATCHDOG_CALC_FEED_CNT(timeOutsMs, pDrvData->clkHz, pDrvData->sbrCfg.div);
    if (feedCnt > WATCHDOG_LOADDATA_REG_MAX_VAL) {
        LOGE("%s: calculated feed count %llu exceeds maximum %u\r\n", 
             __func__, feedCnt, WATCHDOG_LOADDATA_REG_MAX_VAL);
        ret = -EINVAL;
        goto unlock;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE;  ///< Unlock register
    pReg->loadData = (U32)feedCnt;
    pReg->lock = WATCHDOG_LOCK_VALUE;    ///< Lock register

    /* Update driver data after successful hardware update */
    pDrvData->feedCnt = (U32)feedCnt;    

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 wdtSetMode(DevList_e devId, WorkMode_e mode)
{
    S32 ret = EXIT_SUCCESS;
    WdtDrvData_s *pDrvData = NULL;

    if (mode != WDT_MODE_RESET && mode != WDT_MODE_INT) {
        LOGE("%s: invalid wdt mode %d!\r\n", __func__, mode);
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, WATCHDOG_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: failed to acquire device lock, devId=%d!\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        LOGE("%s: driver not match, devId=%d!\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (isDrvInit(devId) == false) {
        LOGW("%s: device not initialized, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    ret = getDevDriver(devId, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to get driver data, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: driver data is NULL, devId=%d!\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pDrvData->workMode = mode; ///< Only update software WDT work mode, register configuration is done in wdtStart();

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}
