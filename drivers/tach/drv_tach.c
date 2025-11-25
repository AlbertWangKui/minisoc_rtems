/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_tach.c
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025
 * @brief  tach driver implementation
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2024         tach_driver     Created initial version
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

static S32 tachDevCfgGet(DevList_e devId, SbrTachCfg_s *tachSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (tachSbrCfg == NULL) {
        LOGE("%s: tachSbrCfg is NULL\r\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (devSbrRead(devId, tachSbrCfg, 0, sizeof(SbrTachCfg_s)) != sizeof(SbrTachCfg_s)) {
        LOGE("%s: devSbrRead failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto exit;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("%s: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u\r\n", __func__,
         tachSbrCfg->regAddr, tachSbrCfg->irqNo, tachSbrCfg->irqPrio);
#endif

    if (tachSbrCfg->irqNo == 0 || tachSbrCfg->irqPrio == 0 || tachSbrCfg->regAddr == NULL || \
        tachSbrCfg->polarity >= TACH_POLARITY_MAX) {
        ret = -EINVAL;
        goto exit;
    }

exit:
    return ret;
}

static S32 tachSetCapCfg(TachDrvData_s *pDrvData, U32 clk)
{
    S32 ret = EXIT_SUCCESS;
    TachReg_s *pReg = NULL;
    U64 capTimeWidth = 0;

    if (pDrvData == NULL) {
        LOGE("%s: invalid parameter, pDrvData: %p\r\n", __func__, pDrvData);
        ret = -EINVAL;
        goto exit;
    }

    /* Check for integer overflow before calculation - security improvement */
    if (clk > TACH_MAX_TIMEOUT_REG_VALUE / TACH_DEFAULT_MEASURE_TIME_S) {
        LOGE("%s: clock frequency causes overflow, clk: %u\r\n", __func__, clk);
        ret = -EINVAL;
        goto exit;
    }

    capTimeWidth = (U64)clk * TACH_DEFAULT_MEASURE_TIME_S;
    if (capTimeWidth > TACH_MAX_TIMEOUT_REG_VALUE) {
        LOGE("%s: measure time reg value is greater than max: %u\r\n", __func__, TACH_MAX_TIMEOUT_REG_VALUE);
        ret = -EINVAL;
        goto exit;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    pReg->timeCtrl = (U32)capTimeWidth;
    pReg->ctrl.bit.filterSel = TACH_NO_FILTER;

exit:
    return ret;
}

static void tachIrqHandler(TachDrvData_s *pDrvData)
{
    TachReg_s *pReg = NULL;

    if (pDrvData == NULL) {
        LOGE("%s: pDrvData = NULL\r\n", __func__);
        return;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: pReg is NULL\r\n", __func__);
        return;
    }

    /* Clear interrupt status */
    pReg->intRaw.bit.tachIrqRaw = 1;

    if (pDrvData->callback != NULL) {
        pDrvData->callback(pDrvData->arg);
    }

    return;
}

S32 tachCallbackRegister(DevList_e devId, tachIrqCallBack callback, void *arg)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;
    U32 intEn = 0;

    /* Validate callback function pointer - critical security check */
    if (callback == NULL) {
        LOGE("%s: callback is NULL\r\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if(isDrvInit(devId) == false) {
        LOGE("%s: device not initialized, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: getDevDriver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: failed to get driver data, devId: %d\r\n", __func__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    /* Check if callback function has already been registered */
    if (pDrvData->callback != NULL) {
        LOGE("%s: callback already registered, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    intEn = pReg->intEn.bit.tachIrqEn; ///< get interrupt enable status
    pReg->intEn.bit.tachIrqEn = 0; ///< disable interrupt

    pDrvData->callback = callback;
    pDrvData->arg = arg;

    pReg->intEn.bit.tachIrqEn = intEn; ///< restore interrupt enable status

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 tachCallbackUnRegister(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;
    U32 intEn = 0;

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if(isDrvInit(devId) == false) {
        LOGE("%s: device not initialized, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: getDevDriver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: failed to get driver data, devId: %d\r\n", __func__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    intEn = pReg->intEn.bit.tachIrqEn;   ///< get interrupt enable status
    pReg->intEn.bit.tachIrqEn = 0; ///< disable interrupt

    /* Clear callback and argument */
    pDrvData->callback = NULL;
    pDrvData->arg = NULL;

    pReg->intEn.bit.tachIrqEn = intEn; ///< restore interrupt enable status

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 tachSetTrigger(DevList_e devId, TachPolarity_e polarity)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if(isDrvInit(devId) == false) {
        LOGE("%s: device not initialized, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: invalid parameter, pDrvData: %p\r\n", __func__, pDrvData);
        ret = -EINVAL;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg->ctrl.bit.polaritySel = (TACH_POSITIVE_EDGE == polarity) ? 0 : 1;

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 tachIrqEnable(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if(isDrvInit(devId) == false) {
        LOGE("%s: device not initialized, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: invalid parameter, pDrvData: %p\r\n", __func__, pDrvData);
        ret = -EINVAL;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg->intEn.bit.tachIrqEn = 1;

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 tachIrqDisable(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if(isDrvInit(devId) == false) {
        LOGE("%s: device not initialized, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: invalid parameter, pDrvData: %p\r\n", __func__, pDrvData);
        ret = -EINVAL;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg->intEn.bit.tachIrqEn = 0;

unlock:
    devUnlockByDriver(devId);

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
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (isDrvInit(devId)) {
        LOGE("%s: device already initialized, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to reset peripheral, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to enable peripheral clock, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsClockFreqGet(devId, &clk) != EXIT_SUCCESS) {
        LOGE("%s: failed to get peripheral clock frequency, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pDrvData = (TachDrvData_s*)calloc(1, sizeof(TachDrvData_s));
    if (pDrvData == NULL) {
        LOGE("%s: failed to allocate memory for driver data, devId: %d\r\n", __func__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    if (tachDevCfgGet(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s: failed to get device configuration, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto freeMem;
    }

    /* Initialize tach driver data */
    pDrvData->clk = clk;
    pDrvData->busy = 0;

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto freeMem;
    }

    pReg->intEn.bit.tachIrqEn = 0;  ///< Disable interrupt enable
    pReg->ctrl.bit.polaritySel = pDrvData->sbrCfg.polarity & 0x01;

    ret = ospInterruptHandlerInstall(pDrvData->sbrCfg.irqNo, "tach",
                                    OSP_INTERRUPT_UNIQUE,
                                    (OspInterruptHandler)tachIrqHandler,
                                    pDrvData);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s: IRQ handler already installed\r\n", __func__);
        irqInstalled = true;
        ret = EXIT_SUCCESS; ///< Correctly handle OSP_RESOURCE_IN_USE error
    } else if (ret == OSP_SUCCESSFUL) {
        ospInterruptSetPriority(pDrvData->sbrCfg.irqNo, pDrvData->sbrCfg.irqPrio);
        ospInterruptVectorEnable(pDrvData->sbrCfg.irqNo);
        irqInstalled = true;
        LOGW("%s: IRQ handler installed successfully\r\n", __func__);
    } else {
        LOGE("%s: failed to install IRQ handler, ret=%d\r\n", __func__, ret);
        ret = -EIO;
        goto freeMem;
    }

    ret = tachSetCapCfg(pDrvData, clk);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to set cap cfg, ret=%d\r\n", __func__, ret);
        ret = -EIO;
        goto removeIrq;
    }

    if (drvInstall(devId, pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: driver install failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto removeIrq;  ///< Clean up installed IRQ handler
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

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if(isDrvInit(devId) == false) {
        LOGE("%s: device not initialized, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: driver data is NULL, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
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
            LOGE("%s: failed to remove IRQ handler, ret=%d\r\n", __func__, ret);
        }
    }

    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to uninstall driver, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to reset peripheral, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Disable peripheral clock to save power */
    if (peripsClockDisable(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to disable peripheral clock, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 tachEnable(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if(isDrvInit(devId) == false) {
        LOGE("%s: device not initialized, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: failed to get driver data, devId: %d\r\n", __func__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pReg->ctrl.bit.capEn == 1) {
        LOGE("%s: tach already enabled, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg->ctrl.bit.capEn = 1;    ///< Start capture

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 tachDisable(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TachDrvData_s *pDrvData = NULL;
    TachReg_s *pReg = NULL;

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if(isDrvInit(devId) == false) {
        LOGE("%s: device not initialized, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: failed to get driver data, devId: %d\r\n", __func__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Check device state and perform idempotent disable operation */
    if (pReg->ctrl.bit.capEn == 0) {
        LOGW("%s: device already disabled (idempotent operation), devId: %d\r\n", __func__, devId);
        pDrvData->busy = 0;
        ret = EXIT_SUCCESS;
        goto unlock;
    }

    /* Stop capture */
    pReg->ctrl.bit.capEn = 0;
    pDrvData->busy = 0;

unlock:
    devUnlockByDriver(devId);

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
        LOGE("%s: freqBuf is NULL\r\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if ((timeoutSec == 0) || (timeoutSec > TACH_MAX_TIMEOUT_VALUE_S)) {
        LOGE("%s: timeoutSec (%us) must be between 1s and %us\r\n",
             __func__, timeoutSec, TACH_MAX_TIMEOUT_VALUE_S);
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, TACH_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: device lock failed, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (isDrvInit(devId) == false) {
        LOGE("%s: device not initialized, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TACH)) {
        LOGE("%s: device not match, devId: %d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: failed to get driver data, devId: %d\r\n", __func__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    pReg = (TachReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid reg, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Check if device is enabled */
    if (pReg->ctrl.bit.capEn == 0) {
        LOGE("%s: device not enabled, devId: %d\r\n", __func__, devId);
        pDrvData->busy = 0;  /* Reset busy flag on error */
        ret = -EINVAL;
        goto unlock;
    }

    /* Prevent calling tachGetFreq again before the first measurement is complete */
    if (pDrvData->busy == 1) {
        LOGE("%s: device is busy, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
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
        LOGE("%s: frequency measurement timeout, devId: %d, timeout: %us\r\n",
             __func__, devId, timeoutSec);
        pDrvData->busy = 0;  /* Reset busy flag */
        ret = -ETIMEDOUT;
        goto unlock;
    }

    tachCnt = pReg->state.bit.tachCnt;
    *freqBuf = (tachCnt / TACH_DEFAULT_MEASURE_TIME_S);

    /* Clear capEn bit after measurement is complete, need to set capEn bit again before next measurement */
    pReg->ctrl.bit.capEn = 0;
    pDrvData->busy = 0;

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}
