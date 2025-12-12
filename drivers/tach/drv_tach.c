/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 * @file    drv_tach.c
 * @author  yangkl@starsmicrosystem.com
 * @date    2025/01/01
 * @brief   Tach driver implementation
 */

#include <stdio.h>
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "log_msg.h"
#include "drv_tach_api.h"
#include "drv_tach.h"
#include "osp_interrupt.h"
#include "bsp_drv_id.h"
#include "udelay.h"

static S32 tachDevCfgGet(DevList_e devId, SbrTachCfg_s *tachSbrCfg);
static S32 tachSetCapCfg(TachDrvData_s *pDrvData, U32 clk);
static void tachIrqHandler(TachDrvData_s *pDrvData);

/**
 * @brief   获取设备配置
 * @param[in]  devId  设备ID
 * @param[out] tachSbrCfg  SBR配置结构体指针
 * @return  EXIT_SUCCESS成功 / -EINVAL参数错误 / -EIO硬件错误
 */
static S32 tachDevCfgGet(DevList_e devId, SbrTachCfg_s *tachSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (tachSbrCfg == NULL) {
        LOGE("%s-%d: tachSbrCfg is NULL", __func__, __LINE__);
        ret = -EINVAL;
        goto exit;
    }

    if (devSbrRead(devId, tachSbrCfg, 0, sizeof(SbrTachCfg_s)) != sizeof(SbrTachCfg_s)) {
        LOGE("%s-%d: devSbrRead failed, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto exit;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("%s-%d: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u", __func__, __LINE__,
         tachSbrCfg->regAddr, tachSbrCfg->irqNo, tachSbrCfg->irqPrio);
#endif

    if (tachSbrCfg->irqNo == 0 || tachSbrCfg->irqPrio == 0 || tachSbrCfg->regAddr == NULL ||
        tachSbrCfg->polarity >= TACH_POLARITY_MAX) {
        ret = -EINVAL;
        goto exit;
    }

exit:
    return ret;
}

/**
 * @brief   设置捕获配置
 * @param[in]  pDrvData  驱动数据结构体指针
 * @param[in]  clk  时钟频率
 * @return  EXIT_SUCCESS成功 / -EINVAL参数错误
 */
static S32 tachSetCapCfg(TachDrvData_s *pDrvData, U32 clk)
{
    S32 ret = EXIT_SUCCESS;
    TachReg_s *pReg = NULL;
    U64 capTimeWidth = 0;

    if (pDrvData == NULL) {
        LOGE("%s-%d: invalid parameter, pDrvData: %p", __func__, __LINE__, pDrvData);
        ret = -EINVAL;
        goto exit;
    }

    if (clk > TACH_MAX_TIMEOUT_REG_VALUE / TACH_DEFAULT_MEASURE_TIME_S) {
        LOGE("%s-%d: clock frequency causes overflow, clk: %u", __func__, __LINE__, clk);
        ret = -EINVAL;
        goto exit;
    }

    capTimeWidth = (U64)clk * TACH_DEFAULT_MEASURE_TIME_S;
    if (capTimeWidth > TACH_MAX_TIMEOUT_REG_VALUE) {
        LOGE("%s-%d: measure time reg value is greater than max: %u", __func__, __LINE__, TACH_MAX_TIMEOUT_REG_VALUE);
        ret = -EINVAL;
        goto exit;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->timeCtrl = (U32)capTimeWidth;
    pReg->ctrl.bit.filterSel = TACH_NO_FILTER;

exit:
    return ret;
}

/**
 * @brief   中断处理函数
 * @param[in]  pDrvData  驱动数据结构体指针
 */
static void tachIrqHandler(TachDrvData_s *pDrvData)
{
    TachReg_s *pReg = NULL;

    if (pDrvData == NULL) {
        LOGE("%s-%d: pDrvData is NULL", __func__, __LINE__);
        return;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: pReg is NULL", __func__, __LINE__);
        return;
    }

    pReg->intRaw.bit.tachIrqRaw = 1;

    if (pDrvData->callback != NULL) {
        pDrvData->callback(pDrvData->arg);
    }
}

S32 tachCallbackRegister(DevList_e devId, tachIrqCallBack callback, void *arg)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;
    U32 intEn = 0;

    if (callback == NULL) {
        LOGE("%s-%d: callback is NULL", __func__, __LINE__);
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TACH, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (pDrvData->callback != NULL) {
        LOGE("%s-%d: callback already registered, devId: %d", __func__, __LINE__, devId);
        ret = -EINVAL;
        goto cleanup;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EINVAL;
        goto cleanup;
    }

    intEn = pReg->intEn.bit.tachIrqEn;
    pReg->intEn.bit.tachIrqEn = 0;

    pDrvData->callback = callback;
    pDrvData->arg = arg;

    pReg->intEn.bit.tachIrqEn = intEn;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 tachCallbackUnRegister(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;
    U32 intEn = 0;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TACH, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EINVAL;
        goto cleanup;
    }

    intEn = pReg->intEn.bit.tachIrqEn;
    pReg->intEn.bit.tachIrqEn = 0;

    pDrvData->callback = NULL;
    pDrvData->arg = NULL;

    pReg->intEn.bit.tachIrqEn = intEn;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 tachSetTrigger(DevList_e devId, TachPolarity_e polarity)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TACH, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    pReg->ctrl.bit.polaritySel = (TACH_POSITIVE_EDGE == polarity) ? 0 : 1;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 tachIrqEnable(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TACH, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    pReg->intEn.bit.tachIrqEn = 1;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 tachIrqDisable(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TACH, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    pReg->intEn.bit.tachIrqEn = 0;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 tachInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;
    bool irqInstalled = false;
    U32 clk = 0;

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s-%d: device lock failed, devId: %d", __func__, __LINE__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (isDrvInit(devId)) {
        LOGE("%s-%d: device already initialized, devId: %d", __func__, __LINE__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s-%d: device not match, devId: %d", __func__, __LINE__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to reset peripheral, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to enable peripheral clock, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsClockFreqGet(devId, &clk) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to get peripheral clock frequency, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto unlock;
    }

    pDrvData = (TachDrvData_s *)calloc(1, sizeof(TachDrvData_s));
    if (pDrvData == NULL) {
        LOGE("%s-%d: failed to allocate memory for driver data, devId: %d", __func__, __LINE__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    if (tachDevCfgGet(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to get device configuration, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto freeMem;
    }

    pDrvData->clk = clk;
    pDrvData->busy = 0;

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto freeMem;
    }

    pReg->intEn.bit.tachIrqEn = 0;
    pReg->ctrl.bit.polaritySel = pDrvData->sbrCfg.polarity & 0x01;

    ret = ospInterruptHandlerInstall(pDrvData->sbrCfg.irqNo, "tach",
                                     OSP_INTERRUPT_UNIQUE,
                                     (OspInterruptHandler)tachIrqHandler,
                                     pDrvData);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s-%d: IRQ handler already installed", __func__, __LINE__);
        irqInstalled = true;
        ret = EXIT_SUCCESS;
    } else if (ret == OSP_SUCCESSFUL) {
        ospInterruptSetPriority(pDrvData->sbrCfg.irqNo, pDrvData->sbrCfg.irqPrio);
        ospInterruptVectorEnable(pDrvData->sbrCfg.irqNo);
        irqInstalled = true;
        LOGW("%s-%d: IRQ handler installed successfully", __func__, __LINE__);
    } else {
        LOGE("%s-%d: failed to install IRQ handler, ret: %d", __func__, __LINE__, ret);
        ret = -EIO;
        goto freeMem;
    }

    ret = tachSetCapCfg(pDrvData, clk);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to set cap cfg, ret: %d", __func__, __LINE__, ret);
        ret = -EIO;
        goto removeIrq;
    }

    if (drvInstall(devId, pDrvData) != EXIT_SUCCESS) {
        LOGE("%s-%d: driver install failed, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto removeIrq;
    }

    ret = EXIT_SUCCESS;
    goto unlock;

removeIrq:
    if (irqInstalled) {
        ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
        ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo,
                                  (OspInterruptHandler)tachIrqHandler,
                                  pDrvData);
    }

freeMem:
    free(pDrvData);
    pDrvData = NULL;

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 tachDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TACH, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (isDrvInit(devId) == false) {
        LOGE("%s-%d: device not initialized, devId: %d", __func__, __LINE__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg->ctrl.bit.capEn = 0;
    pReg->intEn.bit.tachIrqEn = 0;

    if (pDrvData->sbrCfg.irqNo) {
        ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
        ret = ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo,
                                        (OspInterruptHandler)tachIrqHandler,
                                        pDrvData);
        if (ret != OSP_SUCCESSFUL) {
            LOGE("%s-%d: failed to remove IRQ handler, ret: %d", __func__, __LINE__, ret);
        }
    }

unlock:
    funcRunEndHelper(devId);

    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to uninstall driver, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto exit;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to reset peripheral, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto exit;
    }

    if (peripsClockDisable(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to disable peripheral clock, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto exit;
    }

exit:
    return ret;
}

S32 tachEnable(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TACH, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    if (pReg->ctrl.bit.capEn == 1) {
        LOGE("%s-%d: tach already enabled, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    pReg->ctrl.bit.capEn = 1;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 tachDisable(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TACH, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    if (pReg->ctrl.bit.capEn == 0) {
        LOGW("%s-%d: device already disabled (idempotent operation), devId: %d", __func__, __LINE__, devId);
        pDrvData->busy = 0;
        ret = EXIT_SUCCESS;
        goto cleanup;
    }

    pReg->ctrl.bit.capEn = 0;
    pDrvData->busy = 0;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 tachGetFreq(DevList_e devId, U32 timeoutSec, U32 *freqBuf)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;
    U32 waitCount = 0;
    U32 maxWaitCount = 0;
    U32 tachCnt = 0;

    if (freqBuf == NULL) {
        LOGE("%s-%d: freqBuf is NULL", __func__, __LINE__);
        ret = -EINVAL;
        goto exit;
    }

    if ((timeoutSec == 0) || (timeoutSec > TACH_MAX_TIMEOUT_VALUE_S)) {
        LOGE("%s-%d: timeoutSec (%u) must be between 1s and %us", __func__, __LINE__, timeoutSec, TACH_MAX_TIMEOUT_VALUE_S);
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TACH, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s-%d: invalid reg, devId: %d", __func__, __LINE__, devId);
        ret = -EIO;
        goto cleanup;
    }

    if (pReg->ctrl.bit.capEn == 0) {
        LOGE("%s-%d: device not enabled, devId: %d", __func__, __LINE__, devId);
        pDrvData->busy = 0;
        ret = -EINVAL;
        goto cleanup;
    }

    if (pDrvData->busy == 1) {
        LOGE("%s-%d: device is busy, devId: %d", __func__, __LINE__, devId);
        ret = -EBUSY;
        goto cleanup;
    }

    pDrvData->busy = 1;
    maxWaitCount = ((timeoutSec * 1000) / TACH_POLL_INTERVAL_MS);
    while (waitCount < maxWaitCount) {
        if (pReg->state.bit.capOver) {
            break;
        }
        waitCount++;
        msleep(TACH_POLL_INTERVAL_MS);
    }

    if (waitCount >= maxWaitCount) {
        LOGE("%s-%d: frequency measurement timeout, devId: %d, timeout: %us", __func__, __LINE__, devId, timeoutSec);
        pDrvData->busy = 0;
        ret = -ETIMEDOUT;
        goto cleanup;
    }

    tachCnt = pReg->state.bit.tachCnt;
    *freqBuf = (tachCnt / TACH_DEFAULT_MEASURE_TIME_S);

    pReg->ctrl.bit.capEn = 0;
    pDrvData->busy = 0;

cleanup:
    funcRunEndHelper(devId);

exit:
    return ret;
}
