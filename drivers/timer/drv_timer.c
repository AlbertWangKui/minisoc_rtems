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

static S32 timerDevCfgGet(DevList_e devId, SbrTimerCfg_s *cfg)
{
    S32 ret = EXIT_SUCCESS;

    if (cfg == NULL) {
        ret = -EIO;
        goto out;
    }

    if (devSbrRead(devId, cfg, 0, sizeof(SbrTimerCfg_s)) != sizeof(SbrTimerCfg_s)) {
        ret = -EIO;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("timer: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, intervalMs:%u, reserved:0x%08x\r\n",
         cfg->regAddr, cfg->irqNo, cfg->irqPrio, cfg->intervalMs, cfg->reserved);
#endif

    if (cfg->regAddr == NULL || cfg->irqNo == 0) {
        ret = -EIO;
        goto out;
    }

out:
    return ret;
}

static S32 timerCalcTicks(DevList_e devId, TimerReg *reg, U32 mSec, U64 *ticks)
{
    U32 clkHz = 0;
    U64 needed;
    if (ticks == NULL || reg == NULL) {
        return -EINVAL;
    }
    if (peripsClockFreqGet(devId, &clkHz) != EXIT_SUCCESS || clkHz == 0) {
        return -EINVAL;
    }
    clkHz /= (reg->divisor.fields.divisor + 1);
    needed = (U64)mSec * (U64)clkHz;
    needed /= 1000ULL;
    if (needed == 0) {
        return -EINVAL;
    }
    *ticks = needed;
    return EXIT_SUCCESS;
}

static void timerIsr(void *arg)
{
    TimerDrvData_s *data = (TimerDrvData_s *)arg;
    if (data == NULL || data->sbrCfg.regAddr == NULL) {
        return;
    }

    TimerReg *reg = (TimerReg *)data->sbrCfg.regAddr;

    if (reg->irqSta.fields.irqSta) {
        reg->irqSta.fields.irqSta = 1; /* 写1清除中断 */
        if (data->callback != NULL) {
            data->callback();
        }
    }
}

S32 timerInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;

    if (isDrvInit(devId)) {
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_TIMER)) {
        return -EINVAL;
    }

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }
    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    drv = (TimerDrvData_s *)calloc(1, sizeof(TimerDrvData_s));
    if (drv == NULL) {
        ret = -ENOMEM;
        goto unlock;
    }

    if (timerDevCfgGet(devId, &drv->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto free_mem;
    }

    drv->callback = NULL;

    ospInterruptHandlerInstall(drv->sbrCfg.irqNo, "timer", OSP_INTERRUPT_UNIQUE, timerIsr, drv);
    if (arm_gic_irq_set_priority(drv->sbrCfg.irqNo, drv->sbrCfg.irqPrio) != 0) {
        ret = -EIO;
        goto remove_isr;
    }
    bsp_interrupt_vector_enable(drv->sbrCfg.irqNo);

    if (drvInstall(devId, (void *)drv) != EXIT_SUCCESS) {
        ret = -EIO;
        goto disable_vector;
    }

    ret = EXIT_SUCCESS;
    goto unlock;

disable_vector:
    bsp_interrupt_vector_disable(drv->sbrCfg.irqNo);
remove_isr:
    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, timerIsr, drv);
free_mem:
    free(drv);
    goto unlock;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 timerDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_TIMER)) {
        return -EINVAL;
    }

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (isDrvInit(devId) == false) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }
    if (drv == NULL) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, timerIsr, drv);
    ospInterruptVectorUninit(drv->sbrCfg.irqNo);
    peripsReset(devId);
    drvUninstall(devId);

    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 timerSetCfg(DevList_e devId, U32 mSec, void (*timeout)(void))
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;
    U64 ticks = 0;

    if (!isDrvMatch(devId, DRV_ID_STARS_TIMER)) {
        return -EINVAL;
    }

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }
    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }
    if (drv == NULL) {
        ret = -EIO;
        goto unlock;
    }

    if (timerCalcTicks(devId, (TimerReg *)drv->sbrCfg.regAddr, mSec, &ticks) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }
    ((TimerReg *)drv->sbrCfg.regAddr)->setlVal = (U32)ticks;
    ((TimerReg *)drv->sbrCfg.regAddr)->sethVal = (U32)(ticks >> 32);
    drv->callback = timeout;
    ((TimerReg *)drv->sbrCfg.regAddr)->irqEn.fields.irqEn = (timeout != NULL) ? 1 : 0;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 timerStart(DevList_e devId)
{

    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_TIMER)) {
        return -EINVAL;
    }

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if (drv == NULL) {
        ret = -EIO;
        goto unlock;
    }

    ((TimerReg *)drv->sbrCfg.regAddr)->timerEnable.fields.en = 1;

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 timerStop(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TimerDrvData_s *drv = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_TIMER)) {
        return -EINVAL;
    }

    if (devId == SYS_TICK_TIMER_DEV || devId == SYS_HR_TIMER_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if (drv == NULL) {
        ret = -EIO;
        goto unlock;
    }

    ((TimerReg *)drv->sbrCfg.regAddr)->timerEnable.fields.en = 0;

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}