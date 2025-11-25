/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sm3.c
 * @author baibch@starsmicrosystem.com
 * @date 2025/11/08
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

/**
 * @brief 获取SM3设备配置信息
 *
 * 从SBR中读取SM3设备的配置信息，
 * 包括寄存器基地址、中断号和中断优先级等信息。
 *
 * @param[in] devId 设备ID
 * @param[out] cfg 指向SbrSm3Cfg_s结构体的指针，用于存储读取到的配置信息
 * @return S32 返回执行结果
 *         - EXIT_SUCCESS: 成功获取配置信息
 *         - -EINVAL: 参数错误或寄存器地址为空
 */
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
    LOGE("sm3: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u\r\n",
         cfg->regAddr, cfg->irqNo, cfg->irqPrio);
#endif

    if (cfg->regAddr == NULL) {
        ret = -EINVAL;
        goto out;
    }

out:
    return ret;
}

/**
 * @brief SM3中断服务程序
 *
 * 处理SM3硬件完成计算后的中断事件，清除中断标志，
 * 并执行注册的回调函数（如果存在）。
 *
 * @note 回调函数在ISR上下文中执行，必须遵循以下要求：
 *       1. 不得执行任何阻塞操作
 *       2. 应尽快执行完毕
 *       3. 不得使用任何睡眠或延迟函数
 *       4. 应避免复杂计算
 *       5. 如需复杂处理，应将其推迟到任务/线程中执行
 *
 * @param[in] arg 指向Sm3DrvData_s结构体的指针，包含设备数据和回调信息
 * @return void 无返回值
 */
static void sm3Isr(void *arg)
{
    Sm3DrvData_s *data = (Sm3DrvData_s *)arg;
    if (data == NULL || data->sbrCfg.regAddr == NULL) {
        return;
    }

    Sm3Reg_s *reg = (Sm3Reg_s *)data->sbrCfg.regAddr;

    if (reg->status.fields.finish) {
        reg->stateClr.fields.irqClr = 1;
        reg->stateClr.fields.stateClr = 1;

        if (data->callback != NULL) {
            data->callback(data->callbackArg);
        }
    }
}

/**
 * @brief 模块内部复位
 *
 * 执行SM3硬件模块的软件复位操作，通过设置复位寄存器位来
 * 复位整个SM3计算单元，并执行几个NOP指令以确保复位完成。
 *
 * @param[in] sm3Drv 指向Sm3DrvData_s结构体的指针，包含设备寄存器地址
 * @return void 无返回值
 */
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

/**
 * @brief 等待SM3模块进入IDLE状态
 *
 * 轮询检查SM3模块的状态寄存器，等待模块进入空闲状态(IDLE)。
 * 如果在指定超时时间内仍未进入空闲状态，则返回错误。
 *
 * @param[in] sm3Drv 指向Sm3DrvData_s结构体的指针，包含设备寄存器地址
 * @param[in] timeoutMs 超时时间(毫秒)
 * @return S32 返回执行结果
 *         - EXIT_SUCCESS: 模块成功进入空闲状态
 *         - -EINVAL: 参数错误
 *         - -EIO: 超时未能进入空闲状态
 */
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

    ///< Poll with timeout in milliseconds
    while (elapsedTime < timeoutMs) {
        idleFlag = reg->status.fields.state;
        if (idleFlag == SM3_STATE_IDLE) {
            break;
        }
        msleep(1);
        elapsedTime += 1;
    }

    if (idleFlag != SM3_STATE_IDLE) {
        ret = -EIO;
    }

    return ret;
}

/**
 * @brief 等待SM3计算完成
 *
 * 轮询检查SM3模块的完成标志位，等待计算完成。
 * 如果在指定超时时间内计算仍未完成，则返回错误。
 *
 * @param[in] sm3Drv 指向Sm3DrvData_s结构体的指针，包含设备寄存器地址
 * @param[in] timeoutMs 超时时间(毫秒)
 * @return S32 返回执行结果
 *         - EXIT_SUCCESS: 计算成功完成
 *         - -EINVAL: 参数错误
 *         - -EIO: 超时未能完成计算
 */
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

    ///< Poll with timeout in milliseconds
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
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, SM3_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    drv = (Sm3DrvData_s *)calloc(1, sizeof(Sm3DrvData_s));
    if (drv == NULL) {
        ret = -ENOMEM;
        goto unlock;
    }

    if (sm3DevCfgGet(devId, &drv->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto free_mem;
    }

    drv->callback = NULL;
    drv->callbackArg = NULL;

    ret = ospInterruptHandlerInstall(drv->sbrCfg.irqNo, "sm3", OSP_INTERRUPT_UNIQUE, sm3Isr, drv);
    if (ret != OSP_SUCCESSFUL) {
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

    if (len == 0 || (len % SM3_DATA_STD_SIZE) != 0) {
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
    if (reg->ctrl.fields.logicEn == SM3_MODE_HW) {
        ///< HW calc
        ret = sm3HwModeCalcStart(devId, dataIn, len, SM3_HW_CALC_TIMEOUT_MS);
        if (ret != EXIT_SUCCESS) {
            goto unlock;
        }
        ret = sm3HwResultGet(devId, hashBuf, SM3_HW_CALC_TIMEOUT_MS);
        if (ret != EXIT_SUCCESS) {
            goto unlock;
        }
    } else if (reg->ctrl.fields.logicEn == SM3_MODE_SW) {
        ///< SW calc
        ret = sm3SoftModeLoopCalcStart(devId, dataIn, len, SM3_SW_CALC_TIMEOUT_MS);
        if (ret != EXIT_SUCCESS) {
            goto unlock;
        }
        ret = sm3SoftResultGet(devId, hashBuf, SM3_SW_CALC_TIMEOUT_MS);
        if (ret != EXIT_SUCCESS) {
            goto unlock;
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
    U32 busyFlag = 0;
    U32 finishFlag = 0;

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
    if (drv->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address\r\n", __func__);
        goto unlock;
    }

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    busyFlag = reg->status.fields.state;
    finishFlag = reg->status.fields.finish;

    ///< Check if the module is busy
    if (busyFlag == SM3_BUSY_FLAGS) {
        state = SM3_STATE_BUSY;
    }
    ///< Check if calculation is complete
    else if (finishFlag == SM3_FINISH_FLAGS) {
        state = SM3_STATE_COMPLETE;
    }
    ///< Otherwise, it's idle
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
    if (drv->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address\r\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    if (reg->status.fields.state != SM3_STATE_IDLE) {
        LOGE("%s: Module is not idle\r\n", __func__);
        ret = -EIO;
        goto unlock;
    }
    reg->ctrl.fields.logicEn = mode;

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

    if(sm3IdlePoll(drv, timeout) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    reg->stateClr.fields.irqClr = 1;
    reg->stateClr.fields.stateClr = 1;
    for (int i = 0; i < SM3_RESULT_DWORDS; i++) {
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
    for (int i = 0; i < SM3_RESULT_DWORDS; i++) {
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

    if (grpNum == 0) {
        LOGE("%s: arg error %d\r\n", __func__, grpNum);
        return -EINVAL;
    }

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
    if (drv->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address\r\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    if (reg->status.fields.state != SM3_STATE_IDLE) {
        LOGE("%s: Module is not idle\r\n", __func__);
        ret = -EIO;
        goto unlock;
    }
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
    if (drv->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address\r\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    if (reg->status.fields.state != SM3_STATE_IDLE) {
        LOGE("%s: Module is not idle\r\n", __func__);
        ret = -EIO;
        goto unlock;
    }
    reg->chgiv = 1;

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3HwModeCalcStart(DevList_e devId, U32 *dataIn, U32 len, U32 timeout)
{
    S32 ret = EXIT_SUCCESS;
    Sm3Reg_s *reg = NULL;
    Sm3DrvData_s *drv = NULL;
    uintptr_t startAddr = 0;
    uintptr_t endAddr = 0;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (dataIn == NULL) {
        return -EINVAL;
    }

    if (len == 0 || (len % SM3_DATA_STD_SIZE) != 0) {
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
    if (drv->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address\r\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    if (!IS_ALIGN((uintptr_t)dataIn, 4)) {
        LOGE("%s: dataIn must be 4-byte aligned\r\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    startAddr = (uintptr_t)dataIn;
    endAddr = (uintptr_t)((U8*)dataIn + len - 4);
    if (endAddr > 0xFFFFFFFFUL) {
        LOGE("%s: address out of range for 32-bit hardware\r\n", __func__);
        ret = -EINVAL;
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
    reg->ctrl.fields.logicEn = SM3_MODE_HW;
    reg->grpCounts = len * BYTE_BITS / SM3_NUM_BITS_PER_GRP;
    reg->addrStart = (U32)startAddr;
    reg->addrEnd = (U32)endAddr;
    reg->ctrl.fields.startEn = 1;
    reg->axiStart.fields.start = 1;

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 sm3SoftModeLoopCalcStart(DevList_e devId, U32 *dataIn, U32 len, U32 timeout)
{
    S32 ret = EXIT_SUCCESS;
    Sm3Reg_s *reg = NULL;
    Sm3DrvData_s *drv = NULL;
    U32 grpCount;

    if (!isDrvMatch(devId, DRV_ID_STARS_SM3)) {
        return -EINVAL;
    }

    if (dataIn == NULL) {
        return -EINVAL;
    }

    if (len == 0 || (len % SM3_DATA_STD_SIZE) != 0) {
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
    if (drv->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address\r\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    reg = (Sm3Reg_s *)drv->sbrCfg.regAddr;
    grpCount = len * BYTE_BITS / SM3_NUM_BITS_PER_GRP;

    sm3InnerReset(drv);
    if (sm3IdlePoll(drv, timeout) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }
    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }
    reg->ctrl.fields.logicEn = SM3_MODE_SW;
    reg->chgiv = 1;
    do {
        dwMemCpy(reg->dataIn, (void *)dataIn, SM3_DATA_STD_SIZE / 4);
        dataIn += (SM3_DATA_STD_SIZE / 4);
        reg->start.fields.start = 1;
        if (reg->status.fields.state != SM3_STATE_IDLE) {
            reg->stateClr.fields.stateClr = 1;
        }
        if (sm3IdlePoll(drv, timeout) != EXIT_SUCCESS) {
            ret = -EIO;
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

    ///< Reset the SM3 module
    sm3InnerReset(drv);

    ///< Disable interrupts
    reg->irqEnable.fields.enable = 0;

    ///< Disable interrupt vector
    bsp_interrupt_vector_disable(drv->sbrCfg.irqNo);

    ///< Remove interrupt handler
    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, sm3Isr, drv);

    ///< External reset for recovery. (may fail, but continue anyway)
    peripsReset(devId);

    ///< Uninstall the driver
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
