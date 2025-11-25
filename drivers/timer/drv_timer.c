/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_timer.c
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/08/09
 * @brief timer driver
 */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "bsp_sbr.h"
#include "sbr_api.h"
#include "osp_interrupt.h"
#include "drv_timer.h"
#include "log_msg.h"

/**
 * @brief Get timer device configuration from SBR
 * @details Read timer configuration from System Board Register
 * @param [in] devId Timer device ID
 * @param [out] cfg Pointer to store timer configuration
 * @return EXIT_SUCCESS on success
 * @return -EIO if cfg is NULL or failed to read SBR
 */
static S32 timerDevCfgGet(DevList_e devId, SbrTimerCfg_s *cfg)
{
    S32 ret = EXIT_SUCCESS;

    if (cfg == NULL) {
        ret = -EIO;
        goto out;
    }

    if (devSbrRead(devId, cfg, 0, sizeof(SbrTimerCfg_s)) != sizeof(SbrTimerCfg_s)) {
        LOGE("timer%u: failed to read timer config from SBR\r\n", devId);
        ret = -EIO;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGE("timer: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, intervalMs:%u, reserved:0x%08x\r\n",
         cfg->regAddr, cfg->irqNo, cfg->irqPrio, cfg->intervalMs, cfg->reserved);
#endif

    if (cfg->regAddr == NULL || cfg->irqNo == 0) {
        ret = -EIO;
        goto out;
    }

out:
    return ret;
}

/**
 * @brief Calculate timer ticks from milliseconds
 * @details Calculate the number of timer ticks required for the specified milliseconds
 * @param [in] devId Timer device ID
 * @param [in] mSec Milliseconds to convert
 * @param [out] ticks Pointer to store calculated ticks
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if ticks is NULL or calculation fails
 * @return -EIO if failed to get driver data or clock frequency
 */
static S32 timerCalcTicks(DevList_e devId, U32 mSec, U64 *ticks)
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;
    TimerReg *reg = NULL;
    U32 clkHz = 0;
    U64 needed = 0;

    if (ticks == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (drv == NULL || drv->sbrCfg.regAddr == NULL) {
        ret = -EIO;
        goto exit;
    }

    reg = (TimerReg *)drv->sbrCfg.regAddr;

    if (peripsClockFreqGet(devId, &clkHz) != EXIT_SUCCESS || clkHz == 0) {
        ret = -EINVAL;
        goto exit;
    }

    clkHz /= (reg->divisor.fields.divisor + 1);
    needed = (U64)mSec * (U64)clkHz;
    needed /= TIMER_MS_TO_SEC_DIVISOR;
    if (needed == 0) {
        ret = -EINVAL;
        goto exit;
    }

    *ticks = needed;

exit:
    return ret;
}

/**
 * @brief Timer interrupt service routine
 * @details Handle timer interrupt, clear interrupt status and call callback function
 * @param [in] arg Timer driver data pointer
 */
static void timerIsr(void *arg)
{
    TimerDrvData_s *data = (TimerDrvData_s *)arg;
    if (data == NULL || data->sbrCfg.regAddr == NULL) {
        return;
    }

    TimerReg *reg = (TimerReg *)data->sbrCfg.regAddr;

    if (reg->irqSta.fields.irqSta) {
        reg->irqSta.fields.irqSta = TIMER_IRQ_CLEAR;  ///< 写1清除中断
        if (data->callback != NULL) {
            data->callback();
        }
    }
}

/**
 * @brief Set timer parameters
 * @details Configure timer with specified milliseconds and timeout callback
 * @param [in] devId Timer device ID
 * @param [in] mSec Milliseconds for timer interval
 * @param [in] timeout Callback function for timeout, NULL to disable interrupt
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if failed to calculate ticks
 * @return -EIO if failed to get driver data
 */
static S32 timerParamSet(DevList_e devId, U32 mSec, void (*timeout)(void))
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;
    TimerReg *reg = NULL;
    U64 ticks = 0;

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        LOGE("timer%u: Failed to get driver data\r\n", devId);
        ret = -EIO;
        goto exit;
    }

    if (drv == NULL || drv->sbrCfg.regAddr == NULL) {
        LOGE("timer%u: Driver data or reg addr is NULL\r\n", devId);
        ret = -EIO;
        goto exit;
    }

    if (timerCalcTicks(devId, mSec, &ticks) != EXIT_SUCCESS) {
        LOGE("timer%u: Failed to calculate ticks for %u ms\r\n", devId , mSec);
        ret = -EINVAL;
        goto exit;
    }

    reg = (TimerReg *)drv->sbrCfg.regAddr;
    reg->setlVal = (U32)ticks;
    reg->sethVal = (U32)(ticks >> 32);
    drv->callback = timeout;
    reg->irqEn.fields.irqEn = (timeout != NULL) ? TIMER_IRQ_ENABLE : TIMER_IRQ_DISABLE;

exit:
    return ret;
}

S32 timerInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        LOGE("[func: %s]timer%u is system timer\r\n", __func__, devId);
        ret = -EINVAL;
        goto exit;
    }

    if (isDrvInit(devId)) {
        LOGE("[func: %s]timer%u already initialized\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TIMER)) {
        LOGE("[func: %s]timer%u driver not match\r\n", __func__, devId);
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, TIMER_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("[func: %s]timer%u device lock failed\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        LOGE("[func: %s]timer%u failed to enable peripheral clock\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("[func: %s]timer%u failed to reset peripheral\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    drv = (TimerDrvData_s *)calloc(1, sizeof(TimerDrvData_s));
    if (drv == NULL) {
        LOGE("[func: %s]timer%u failed to allocate driver data\r\n", __func__, devId);
        ret = -ENOMEM;
        goto unlock;
    }

    if (timerDevCfgGet(devId, &drv->sbrCfg) != EXIT_SUCCESS) {
        LOGE("[func: %s]timer%u failed to get device configuration\r\n", __func__, devId);
        ret = -EIO;
        goto free_mem;
    }

    if (timerParamSet(devId, drv->sbrCfg.intervalMs, NULL) != EXIT_SUCCESS) {
        LOGE("[func: %s]timer%u failed to config param\r\n", __func__, devId);
        ret = -EINVAL;
        goto free_mem;
    }

    ospInterruptHandlerInstall(drv->sbrCfg.irqNo, "timer", OSP_INTERRUPT_UNIQUE, timerIsr, drv);
    if (arm_gic_irq_set_priority(drv->sbrCfg.irqNo, drv->sbrCfg.irqPrio) != 0) {
        ret = -EIO;
        goto remove_isr;
    }

    if (drvInstall(devId, (void *)drv) != EXIT_SUCCESS) {
        LOGE("[func: %s]timer%u failed to install driver\r\n", __func__, devId);
        ret = -EIO;
        goto disable_vector;
    }

    ospInterruptVectorEnable(drv->sbrCfg.irqNo);

    ret = EXIT_SUCCESS;
    goto unlock;

disable_vector:
    ospInterruptVectorDisable(drv->sbrCfg.irqNo);
remove_isr:
    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, timerIsr, drv);
free_mem:
    free(drv);
unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 timerDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TIMER, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    peripsReset(devId);
    peripsClockDisable(devId);

    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, timerIsr, drv);
    ospInterruptVectorUninit(drv->sbrCfg.irqNo);

    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("[func: %s]timer%u failed to uninstall driver\r\n", __func__, devId);
        ret = -EIO;
    }

    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 timerSetCfg(DevList_e devId, U32 mSec, void (*timeout)(void))
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TIMER, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (timerParamSet(devId, mSec, timeout) != EXIT_SUCCESS) {
        LOGE("[func: %s]timer%u failed to config param\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

unlock:
    funcRunEndHelper(devId);
exit:
    return ret;
}

S32 timerStart(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;
    TimerReg *reg = NULL;

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        LOGE("[func: %s]timer%u is system timer\r\n", __func__, devId);
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TIMER, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    reg = (TimerReg *)drv->sbrCfg.regAddr;
    reg->timerEnable.fields.en = TIMER_ENABLE;

    funcRunEndHelper(devId);
exit:
    return ret;
}

S32 timerStop(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;
    TimerReg *reg = NULL;

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_TIMER, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    reg = (TimerReg *)drv->sbrCfg.regAddr;
    reg->timerEnable.fields.en = TIMER_DISABLE;

    funcRunEndHelper(devId);
exit:
    return ret;
}