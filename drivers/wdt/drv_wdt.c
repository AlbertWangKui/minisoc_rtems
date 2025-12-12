/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_wdt.c
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025/10/22
 * @brief watchdog driver for minisoc
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

/**
 * @brief Get watchdog device configuration from SBR
 * @param[in] devId Device ID
 * @param[out] pWdtSbrCfg Pointer to store watchdog SBR configuration
 * @return EXIT_SUCCESS on success
 * @return -EIO if SBR read fails or configuration is invalid
 */
static S32 wdtDevCfgGet(DevList_e devId, SbrWdtCfg_s *pWdtSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (pWdtSbrCfg == NULL) {
        ret = -EIO;
        goto exit;
    }

    if (devSbrRead(devId, pWdtSbrCfg, 0, sizeof(SbrWdtCfg_s)) != sizeof(SbrWdtCfg_s)) {
        LOGE("%s-%d: failed to read WDT config from SBR, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto exit;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("%s-%d: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, workMode:%u, feedTime:%u, div:%u",
         __func__, __LINE__, pWdtSbrCfg->regAddr, pWdtSbrCfg->irqNo, pWdtSbrCfg->irqPrio,
         pWdtSbrCfg->workMode, pWdtSbrCfg->feedTime, pWdtSbrCfg->div);
#endif

    if ((pWdtSbrCfg->workMode >= WDT_MODE_NR) ||
        (pWdtSbrCfg->feedTime == 0) || (pWdtSbrCfg->div == 0)) {
        LOGE("%s-%d: invalid config from SBR, workMode:%u, feedTime:%u, div:%u, devId=%d",
            __func__, __LINE__, pWdtSbrCfg->workMode, pWdtSbrCfg->feedTime,
            pWdtSbrCfg->div, devId);
        ret = -EIO;
        goto exit;
    }

    if (pWdtSbrCfg->feedTime > WATCHDOG_MAX_TIME_MS) {
        LOGE("%s-%d: time config too long from SBR, feedTime:%u, devId=%d",
            __func__, __LINE__, pWdtSbrCfg->feedTime, devId);
        ret = -EIO;
        goto exit;
    }

exit:
    return ret;
}

/**
 * @brief Watchdog interrupt handler
 * @param[in] pWdtDrvData Pointer to watchdog driver data
 */
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
                pDrvData->callback();
            }
            pDrvData->triggered = true;
            pReg->lock = WATCHDOG_UNLOCK_VALUE;
            pReg->intrClear = 1;
            pReg->loadData = pDrvData->feedCnt;
            pReg->lock = WATCHDOG_LOCK_VALUE;
            break;
        case WDT_MODE_INT:
            pReg->lock = WATCHDOG_UNLOCK_VALUE;
            pReg->intrClear = 1;
            pReg->loadData = pDrvData->feedCnt;
            pReg->lock = WATCHDOG_LOCK_VALUE;
            if (pDrvData->callback != NULL) {
                pDrvData->callback();
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
        LOGW("%s-%d: device lock failed, devId=%d", __func__, __LINE__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (isDrvInit(devId) == true) {
        LOGW("%s-%d: device already initialized, devId=%d", __func__, __LINE__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_WDT)) {
        LOGW("%s-%d: driver not match, devId=%d", __func__, __LINE__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to enable peripheral clock, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to reset peripheral, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsClockFreqGet(devId, &clk) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to get peripheral clock frequency, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (clk == 0) {
        LOGE("%s-%d: Invalid clock frequency %u Hz, devId=%d", __func__, __LINE__, clk, devId);
        ret = -EIO;
        goto unlock;
    }

    pDrvData = (WdtDrvData_s *)calloc(1, sizeof(WdtDrvData_s));
    if (pDrvData == NULL) {
        LOGE("%s-%d: failed to allocate driver data, devId=%d", __func__, __LINE__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    if (wdtDevCfgGet(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to get device configuration, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto freeMem;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        ret = -EIO;
        LOGE("%s-%d: register address is NULL, devId=%d", __func__, __LINE__, devId);
        goto freeMem;
    }

    if(callback != NULL) {
        pReg->intrEn = 0;
        ret = ospInterruptHandlerInstall(pDrvData->sbrCfg.irqNo, "wdt",
                                        OSP_INTERRUPT_UNIQUE,
                                        (OspInterruptHandler)wdtIrqHandler,
                                        pDrvData);
        if (ret == OSP_RESOURCE_IN_USE) {
            LOGW("%s-%d: IRQ handler already installed", __func__, __LINE__);
            irqInstalled = true;
            ret = EXIT_SUCCESS;
        } else if (ret == OSP_SUCCESSFUL) {
            ospInterruptSetPriority(pDrvData->sbrCfg.irqNo, pDrvData->sbrCfg.irqPrio);
            ospInterruptVectorEnable(pDrvData->sbrCfg.irqNo);
            irqInstalled = true;
            LOGI("%s-%d: IRQ handler installed successfully", __func__, __LINE__);
        } else {
            LOGE("%s-%d: failed to install IRQ handler, ret=%d", __func__, __LINE__, ret);
            ret = -EIO;
            goto freeMem;
        }

        pDrvData->callback = callback;
    }

    feedCnt = WATCHDOG_CALC_FEED_CNT(pDrvData->sbrCfg.feedTime, clk, pDrvData->sbrCfg.div);
    if (feedCnt > WATCHDOG_LOADDATA_REG_MAX_VAL) {
        LOGE("%s-%d: calculated feed count %llu exceeds maximum %u",
             __func__, __LINE__, feedCnt, WATCHDOG_LOADDATA_REG_MAX_VAL);
        ret = -EINVAL;
        goto removeIrq;
    }

    pDrvData->feedCnt = (U32)feedCnt;
    pDrvData->clkHz = clk;
    pDrvData->workMode = pDrvData->sbrCfg.workMode;
    pDrvData->triggered = false;

    if (pDrvData->sbrCfg.regAddr == NULL) {
        ret = -EIO;
        LOGE("%s-%d: register address is NULL, devId=%d", __func__, __LINE__, devId);
        goto removeIrq;
    }

    pReg->lock = WATCHDOG_UNLOCK_VALUE;
    pReg->loadData = pDrvData->feedCnt;
    pReg->div = (pDrvData->sbrCfg.div - 1);
    pReg->lock = WATCHDOG_LOCK_VALUE;

    if (drvInstall(devId, (void *)pDrvData) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to install driver, devId=%d", __func__, __LINE__, devId);
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

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_WDT, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (isDrvInit(devId) == false) {
        LOGW("%s-%d: device already deinitialized, devId=%d", __func__, __LINE__, devId);
        ret = -EBUSY;
        goto cleanup;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s-%d: register address is NULL, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        ret = -EIO;
        LOGE("%s-%d: register address is NULL, devId=%d", __func__, __LINE__, devId);
        goto cleanup;
    }

    pReg->lock = WATCHDOG_UNLOCK_VALUE;
    pReg->intrEn = 0;
    pReg->lock = WATCHDOG_LOCK_VALUE;

    if (pDrvData->callback != NULL) {
        ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
        ret = ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo,
                                        (OspInterruptHandler)wdtIrqHandler,
                                        pDrvData);
        if (ret != OSP_SUCCESSFUL) {
            LOGE("%s-%d: failed to remove IRQ handler, ret=%d", __func__, __LINE__, ret);
        }
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to reset peripheral, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
    }

    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to uninstall driver, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
    }

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 wdtFeed(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *pReg = NULL;
    WdtDrvData_s *pDrvData = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_WDT, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s-%d: register address is NULL, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE;
    pReg->loadData = pDrvData->feedCnt;
    pReg->lock = WATCHDOG_LOCK_VALUE;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 wdtStart(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    U32 workMode = 0;
    WdtReg_s *pReg = NULL;
    WdtDrvData_s *pDrvData = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_WDT, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s-%d: register address is NULL, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    if (pDrvData->workMode == WDT_MODE_RESET) {
        SET_BIT(workMode, WATCHDOG_RST_ENABLE_BIT);
        SET_BIT(workMode, WATCHDOG_INT_ENABLE_BIT);
    } else if(pDrvData->workMode == WDT_MODE_INT) {
        CLR_BIT(workMode, WATCHDOG_RST_ENABLE_BIT);
        SET_BIT(workMode, WATCHDOG_INT_ENABLE_BIT);
    } else {
        ret = -EIO;
        LOGE("%s-%d: invalid work mode, devId=%d", __func__, __LINE__, devId);
        goto cleanup;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE;
    pReg->intrEn &= ~WATCHDOG_INTR_ENABLE_MASK;
    pReg->intrEn |= workMode;
    pReg->lock = WATCHDOG_LOCK_VALUE;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 wdtStop(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *pReg = NULL;
    WdtDrvData_s *pDrvData = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_WDT, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s-%d: register address is NULL, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE;
    pReg->intrEn = 0;
    pReg->lock = WATCHDOG_LOCK_VALUE;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 wdtIrqClr(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    WdtReg_s *pReg = NULL;
    WdtDrvData_s *pDrvData = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_WDT, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s-%d: register address is NULL, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE;
    pReg->intrClear = 1;
    pReg->lock = WATCHDOG_LOCK_VALUE;

cleanup:
    funcRunEndHelper(devId);

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
        LOGE("%s-%d: invalid timeout value %u, must be between 0 and %u ms, devId=%d",
             __func__, __LINE__, timeOutsMs, WATCHDOG_MAX_TIME_MS, devId);
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_WDT, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s-%d: register address is NULL, devId=%d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    if (pDrvData->sbrCfg.div == 0) {
        LOGE("%s-%d: division by zero, div=%u", __func__, __LINE__, pDrvData->sbrCfg.div);
        ret = -EINVAL;
        goto cleanup;
    }

    if (pDrvData->clkHz == 0) {
        LOGE("%s-%d: Invalid clock frequency %u Hz", __func__, __LINE__, pDrvData->clkHz);
        ret = -EINVAL;
        goto cleanup;
    }

    feedCnt = WATCHDOG_CALC_FEED_CNT(timeOutsMs, pDrvData->clkHz, pDrvData->sbrCfg.div);
    if (feedCnt > WATCHDOG_LOADDATA_REG_MAX_VAL) {
        LOGE("%s-%d: calculated feed count %llu exceeds maximum %u",
             __func__, __LINE__, feedCnt, WATCHDOG_LOADDATA_REG_MAX_VAL);
        ret = -EINVAL;
        goto cleanup;
    }

    pReg = (WdtReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->lock = WATCHDOG_UNLOCK_VALUE;
    pReg->loadData = (U32)feedCnt;
    pReg->lock = WATCHDOG_LOCK_VALUE;

    pDrvData->feedCnt = (U32)feedCnt;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 wdtSetMode(DevList_e devId, WorkMode_e mode)
{
    S32 ret = EXIT_SUCCESS;
    WdtDrvData_s *pDrvData = NULL;

    if (mode != WDT_MODE_RESET && mode != WDT_MODE_INT) {
        LOGE("%s-%d: invalid wdt mode %d", __func__, __LINE__, mode);
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_WDT, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pDrvData->workMode = mode;

    funcRunEndHelper(devId);
    return EXIT_SUCCESS;

exit:
    return ret;
}
