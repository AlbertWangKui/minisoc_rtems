/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sm3.c
 * @author baibch
 * @date 2025/09/30
 * @brief SM3 hash algorithm driver
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
#include "osp_interrupt.h"
#include "drv_sm3.h"
#include "drv_sm3_api.h"
#include "log_msg.h"
#include "bsp_drv_id.h"
#include "udelay.h"

static S32 sm3DevCfgGet(DevList_e devId, SbrSm3Cfg_s *cfg)
{
    S32 ret = EXIT_SUCCESS;

    if (cfg == NULL) {
        ret = -EINVAL;
        goto out;
    }

    if (devSbrRead(devId, cfg, 0, sizeof(SbrSm3Cfg_s)) != sizeof(SbrSm3Cfg_s)) {
        ret = -EINVAL;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("sm3: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u\r\n",
         cfg->regAddr, cfg->irqNo, cfg->irqPrio);
#endif

    if (cfg->regAddr == NULL) {
        ret = -EINVAL;
        goto out;
    }

out:
    return ret;
}

static void sm3Isr(void *arg)
{
    Sm3DrvData_s *data = (Sm3DrvData_s *)arg;
    if (data == NULL || data->sbrCfg.regAddr == NULL) {
        return;
    }

    LOGD("sm3 irq...\n");
    Sm3Reg_s *reg = (Sm3Reg_s *)data->sbrCfg.regAddr;
    reg->irqEnable.fields.enable = 0;
    if (reg->status.fields.finish) {
        reg->stateClr.fields.irqClr = 1;
        reg->stateClr.fields.stateClr = 1;

        if (data->callback != NULL) {
            data->callback(data->callbackArg);
        }
    }
}

static void sm3InnerReset(Sm3DrvData_s *sm3Drv)
{
    Sm3Reg_s *reg = NULL;
    if (sm3Drv == NULL) {
        return;
    }
    if (sm3Drv->sbrCfg.regAddr == NULL) {
        return;
    }

    reg = (Sm3Reg_s *)sm3Drv->sbrCfg.regAddr;
    reg->reset.fields.reset = 1;
    asm("NOP\n\t");
    asm("NOP\n\t");
    asm("NOP\n\t");
}

static S32 sm3IdlePoll(Sm3DrvData_s *sm3Drv, U32 timeoutMs)
{
    S32 ret = EXIT_SUCCESS;
    U32 idleFlag = 0;
    Sm3Reg_s *reg = NULL;
    U32 elapsedTime = 0;

    if (sm3Drv == NULL) {
        return -EINVAL;
    }
    if (sm3Drv->sbrCfg.regAddr == NULL) {
        return -EINVAL;
    }

    reg = (Sm3Reg_s *)sm3Drv->sbrCfg.regAddr;

    // Poll with timeout in milliseconds
    while (elapsedTime < timeoutMs) {
        idleFlag = reg->status.fields.state;
        if (idleFlag == SM3_STATE_IDLE) {
            break;
        }
        msleep(1); // 1ms delay
        elapsedTime += 1;
    }

    if (idleFlag != SM3_STATE_IDLE) {
        ret = -EIO;
    }

    return ret;
}

static S32 sm3FinishPoll(Sm3DrvData_s *sm3Drv, U32 timeoutMs)
{
    S32 ret = EXIT_SUCCESS;
    U32 finishFlag = 0;
    Sm3Reg_s *reg = NULL;
    U32 elapsedTime = 0;

    if (sm3Drv == NULL) {
        return -EINVAL;
    }
    if (sm3Drv->sbrCfg.regAddr == NULL) {
        return -EINVAL;
    }

    reg = (Sm3Reg_s *)sm3Drv->sbrCfg.regAddr;

    // Poll with timeout in milliseconds
    while (elapsedTime < timeoutMs) {
        finishFlag = reg->status.fields.finish;
        if (finishFlag == SM3_FINISH_FLAGS) {
            break;
        }
        msleep(1);
        elapsedTime += 1;
    }

    if (finishFlag != SM3_FINISH_FLAGS) {
        ret = -EIO;
    }
    
    return ret;
}

S32 sm3Init(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    Sm3Reg_s *reg = NULL;
    Sm3DrvData_s *drv = NULL;

    if (isDrvInit(devId)) {
        ret = -EBUSY; // 16
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL; // 22
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }
#if 0
    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        ret = -EIO; // 5
        goto unlock;
    }
#endif
    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    drv = (Sm3DrvData_s *)calloc(1, sizeof(Sm3DrvData_s));
    if (drv == NULL) {
        ret = -ENOMEM; // 12
        goto unlock;
    }

    if (sm3DevCfgGet(devId, &drv->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto free_mem;
    }

    drv->callback = NULL;
    drv->callbackArg = NULL;

    ret = ospInterruptHandlerInstall(drv->sbrCfg.irqNo, "sm3", OSP_INTERRUPT_UNIQUE, sm3Isr, drv);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s: irq handler already installed\r\n", __func__);
    } else if (ret != OSP_SUCCESSFUL) {
        LOGE("%s: irq handler install failed\r\n", __func__);
        ret = -EIO;
        goto free_mem;
    }

    if (arm_gic_irq_set_priority(drv->sbrCfg.irqNo, drv->sbrCfg.irqPrio) != 0) {
        ret = -EIO;
        goto remove_isr;
    }
    
    bsp_interrupt_vector_enable(drv->sbrCfg.irqNo);
    if (drvInstall(devId, (void *)drv) != EXIT_SUCCESS) {
        ret = -EIO;
        goto disable_vector;
    }

    sm3InnerReset(drv);
    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    reg->ctrl.fields.logicEn = SM3_MODE_HW; /* default HW mode */
    reg->irqEnable.fields.enable = 0; /* Disable interrupt */

    ret = EXIT_SUCCESS;
    goto unlock;

disable_vector:
    bsp_interrupt_vector_disable(drv->sbrCfg.irqNo);
remove_isr:
    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, sm3Isr, drv);
free_mem:
    free(drv);
    goto unlock;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3HashCalc(DevList_e devId, U32 *dataIn, U32 len, U32 *hashBuf)
{
    S32 ret = EXIT_SUCCESS;
    Sm3Reg_s *reg = NULL;
    Sm3DrvData_s *drv = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (dataIn == NULL || hashBuf == NULL) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
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

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    if (reg->ctrl.fields.logicEn == 1) {
        // HW calc
        ret = sm3HwModeCalc(devId, dataIn, len, 20000);
        if (ret != EXIT_SUCCESS) {
            goto unlock;
        } else {
            ret = sm3HwResultGet(devId, hashBuf, 20000);
            if (ret != EXIT_SUCCESS) {
                goto unlock;
            }
        }
    } else if (reg->ctrl.fields.logicEn == 0) {
        // SW calc
        ret = sm3SoftModeLoopCalc(devId, dataIn, len, 10000);
        if (ret != EXIT_SUCCESS) {
            goto unlock;
        } else {
            ret = sm3SoftResultGet(devId, hashBuf, 10000);
            if (ret != EXIT_SUCCESS) {
                goto unlock;
            }
        }
    }

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

Sm3State_e sm3StateGet(DevList_e devId)
{
    Sm3DrvData_s *drv = NULL;
    Sm3Reg_s *reg = NULL;
    Sm3State_e state = SM3_STATE_ERROR;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return SM3_STATE_ERROR;
    }

    if (isDrvInit(devId) == false) {
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        goto exit;
    }
    
    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        goto unlock;
    }
    
    if (drv == NULL) {
        goto unlock;
    }

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    
    // Check if the module is busy
    if (reg->status.fields.state) {
        state = SM3_STATE_BUSY;
    }
    // Check if calculation is complete
    else if (reg->status.fields.finish) {
        state = SM3_STATE_COMPLETE;
    }
    // Otherwise, it's idle
    else {
        state = SM3_STATE_IDLE;
    }

unlock:
    devUnlockByDriver(devId);
exit:
    return state;
}

S32 sm3ModeSelect(DevList_e devId, Sm3Mode_e mode)
{
    S32 ret = EXIT_SUCCESS;
    Sm3DrvData_s *drv = NULL;
    Sm3Reg_s *reg = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (mode < SM3_MODE_SW || mode > SM3_MODE_HW) {
        LOGE("%s: arg error %d\r\n", __func__, mode);
        ret = -EINVAL;
        goto exit;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
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

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    reg->ctrl.fields.logicEn = (SM3_MODE_SW == mode) ? 0 : 1;

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3SoftResultGet(DevList_e devId, U32 *hashBuf, U32 timeout)
{
    S32 ret = EXIT_SUCCESS;
    Sm3DrvData_s *drv = NULL;
    Sm3Reg_s *reg = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (hashBuf == NULL) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
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

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    reg->stateClr.fields.irqClr = 1;
    reg->stateClr.fields.stateClr = 1;
    for (int i = 0; i < 8; i++) {
        hashBuf[i] = reg->dataOut[i];
    }

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3HwResultGet(DevList_e devId, U32 *hashBuf, U32 timeout)
{
    S32 ret = EXIT_SUCCESS;
    Sm3DrvData_s *drv = NULL;
    Sm3Reg_s *reg = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (hashBuf == NULL) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
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

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    if (sm3FinishPoll(drv, timeout) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }
    reg->stateClr.fields.irqClr = 1;
    reg->stateClr.fields.stateClr = 1;
    for (int i = 0; i < 8; i++) {
        hashBuf[i] = reg->dataOut[i];
    }

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3GroupSet(DevList_e devId, U32 grpNum)
{
    S32 ret = EXIT_SUCCESS;
    Sm3DrvData_s *drv = NULL;
    Sm3Reg_s *reg = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
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

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    reg->grpCounts = grpNum;

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3ChgivSet(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    Sm3DrvData_s *drv = NULL;
    Sm3Reg_s *reg = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
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

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    reg->chgiv = 1;

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3HwModeCalc(DevList_e devId, U32 *dataIn, U32 len, U32 timeout)
{
    S32 ret = EXIT_SUCCESS;
    Sm3Reg_s *reg = NULL;
    Sm3DrvData_s *drv = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
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

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    sm3InnerReset(drv);
    if (sm3IdlePoll(drv, timeout) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }
    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }
    reg->ctrl.fields.logicEn = 1;
    reg->grpCounts = len * 8 / SM3_NUM_BITS_PER_GRP;
    reg->addrStart = (U32)dataIn;
    reg->addrEnd = (U32)dataIn + len - 4;
    reg->ctrl.fields.startEn = 1;
    reg->axiStart.fields.start = 1;

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3SoftModeLoopCalc(DevList_e devId, U32 *dataIn, U32 len, U32 timeout)
{
    S32 ret = EXIT_SUCCESS;
    Sm3Reg_s *reg = NULL;
    Sm3DrvData_s *drv = NULL;
    U32 grpCount;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
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

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    grpCount = len * 8 / SM3_NUM_BITS_PER_GRP;

    sm3InnerReset(drv);
    if (sm3IdlePoll(drv, timeout) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }
    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }
    reg->ctrl.fields.logicEn = 0;
    reg->chgiv = 1;
    do {
        dwMemCpy(reg->dataIn, (void *)dataIn, 16);
        dataIn += 16;
        reg->start.fields.start = 1;
        if (reg->status.fields.state != SM3_STATE_IDLE) {
            reg->stateClr.fields.stateClr = 1;
        }
        if (sm3IdlePoll(drv, timeout) != EXIT_SUCCESS) {
            ret = -EXIT_FAILURE;
            goto unlock;
        }
        grpCount--;
    } while (grpCount > 0);

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3DeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    Sm3DrvData_s *drv = NULL;
    Sm3Reg_s *reg = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (!isDrvInit(devId)) {
        ret = -EIO;
        goto exit;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
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

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    
    // Disable interrupts
    reg->irqEnable.fields.enable = 0;
    
    // Reset the SM3 module
    sm3InnerReset(drv);
    
    // Disable interrupt vector
    bsp_interrupt_vector_disable(drv->sbrCfg.irqNo);
    
    // Remove interrupt handler
    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, sm3Isr, drv);
    
    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    // Uninstall the driver
    if (drvUninstall(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    ret = EXIT_SUCCESS;
unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}
