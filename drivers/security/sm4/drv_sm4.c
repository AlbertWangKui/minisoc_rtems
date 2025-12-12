/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 * @file    drv_sm4.c
 * @author  sangwei@starsmicrosystem.com
 * @date    2025/11/28
 * @brief   SM4 cryptographic driver implementation
 */

#include <stdio.h>
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "log_msg.h"
#include "drv_sm4_api.h"
#include "drv_sm4.h"
#include "osp_interrupt.h"
#include "bsp_drv_id.h"

#define IRQ_DONE_BIT    (1 << 0)
#define IRQ_ERR_BIT     (1 << 1)
#define IRQ_ALL_BITS    (IRQ_DONE_BIT | IRQ_ERR_BIT)

/**
 * @brief 软复位SM4模块
 * @details 重置SM4硬件模块的所有状态和寄存器
 * @param [in] reg SM4寄存器基地址
 * @return void
 * @note 调用后需要等待硬件进入IDLE状态
 */
static void sm4Reset(Sm4Reg_s *reg)
{
    if (reg == NULL) {
        return;
    }
    reg->softReset.bit.softRst = 1;
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
}

/**
 * @brief 等待SM4空闲
 * @details 轮询检查SM4模块是否进入IDLE状态
 * @param [in] reg SM4寄存器基地址
 * @return S32 成功返回EXIT_SUCCESS，超时返回-ETIMEDOUT
 * @note 调用前需要确保SM4模块已启动
 */
static S32 sm4WaitIdle(Sm4Reg_s *reg)
{
    U32 timeout = SM4_POLL_TIMEOUT_MS;

    while (reg->sts.bit.busy != SM4_STATE_IDLE && timeout > 0) {
        timeout--;
    }

    return (timeout > 0) ? EXIT_SUCCESS : -ETIMEDOUT;
}

/**
 * @brief 等待SM4完成
 * @details 轮询检查SM4模块是否完成当前事务
 * @param [in] reg SM4寄存器基地址
 * @return S32 成功返回EXIT_SUCCESS，超时返回-ETIMEDOUT
 * @note 调用前需要确保SM4模块已启动
 */
static S32 sm4WaitDone(Sm4Reg_s *reg)
{
    U32 timeout = SM4_POLL_TIMEOUT_MS;

    while ((reg->irqSts.dword & IRQ_DONE_BIT) == 0 && timeout > 0) {
        timeout--;
    }

    return (timeout > 0) ? EXIT_SUCCESS : -ETIMEDOUT;
}

/**
 * @brief 设置密钥和计数器参数
 * @details 根据事务描述符设置SM4模块的密钥和计数器参数
 * @param [in] reg SM4寄存器基地址
 * @param [in] desc 事务描述符指针
 * @return void
 * @note 调用前需要确保SM4模块已启动
 */
static void sm4SetParams(Sm4Reg_s *reg, sm4TransactionDesc_s *desc)
{
    for (U32 i = 0; i < SM4_KEYS_SIZE; i++) {
        reg->key[i] = desc->keys[i];
        reg->cntr[i] = desc->cnt[i];
        reg->cntrLen[i] = desc->cntLen[i];
    }
}

/**
 * @brief 验证事务描述符
 * @details 检查事务描述符是否包含有效参数
 * @param [in] desc 事务描述符指针
 * @return S32 成功返回EXIT_SUCCESS，失败返回错误码
 * @note 调用前需要确保SM4模块已启动
 */
static S32 sm4ValidateDesc(sm4TransactionDesc_s *desc)
{
    if (desc == NULL) {
        return -EINVAL;
    }
    if (desc->srcAddr == 0 || desc->destAddr == 0) {
        LOGE("%s-%d: null address", __func__, __LINE__);
        return -EINVAL;
    }
    if (desc->len == 0 || desc->len > SM4_MAX_DATA_SIZE) {
        LOGE("%s-%d: invalid len=%u", __func__, __LINE__, desc->len);
        return -EINVAL;
    }
    if (desc->len % SM4_BLOCK_SIZE != 0) {
        LOGE("%s-%d: len=%u not aligned", __func__, __LINE__, desc->len);
        return -EINVAL;
    }
    return EXIT_SUCCESS;
}

/**
 * @brief 中断处理函数
 * @details 处理SM4模块的中断事件，包括完成和错误中断
 * @param [in] arg 驱动数据指针，包含SM4寄存器基地址和回调函数
 * @return void
 * @note 调用前需要确保SM4模块已启动
 */
static void sm4IrqHandler(void *arg)
{
    Sm4DrvData_s *drv = (Sm4DrvData_s *)arg;

    if (drv == NULL || drv->sbrCfg.regAddr == NULL) {
        return;
    }

    Sm4Reg_s *reg = (Sm4Reg_s *)drv->sbrCfg.regAddr;
    if (reg->irqSts.dword != 0) {
        LOGD("%s-%d: irqSts=0x%x", __func__, __LINE__, reg->irqSts.dword);
        reg->irqClr.dword = SM4_IRQ_MASK;
    }

    if (drv->callback != NULL) {
        drv->callback(drv->callbackArg);
    }
}

/**
 * @brief 硬件DMA模式加密
 * @details 使用SM4模块的硬件DMA模式进行加密操作
 * @param [in] reg SM4寄存器基地址
 * @param [in] desc 事务描述符指针
 * @param [in] keySel 密钥选择，0为内部密钥，1为外部密钥
 * @return S32 成功返回EXIT_SUCCESS，失败返回错误码
 * @note 调用前需要确保SM4模块已启动
 */
static S32 sm4EncryptHw(Sm4Reg_s *reg, sm4TransactionDesc_s *desc, U32 keySel)
{
    S32 ret = EXIT_SUCCESS;

    reg->ctrl.bit.invalid = 0;
    reg->ctrl.bit.sm4KeySel = keySel;
    reg->ctrl.bit.sm4CntSel = 0;
    reg->ctrl.bit.kxpstartEn = 1;
    reg->ctrl.bit.chgcntrEn = 1;
    reg->dataSize = desc->len;
    reg->rdStartAddr = desc->srcAddr;
    reg->wrStartAddr = desc->destAddr;

    reg->ctrl.bit.start = 1;

    ret = sm4WaitDone(reg);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s-%d: wait done timeout", __func__, __LINE__);
        goto exit;
    }

    reg->irqClr.dword = IRQ_ALL_BITS;

exit:
    return ret;
}

/**
 * @brief 软件模式加密(逐块)
 * @details 使用SM4模块的软件模式进行加密操作，逐块处理数据
 * @param [in] reg SM4寄存器基地址
 * @param [in] desc 事务描述符指针
 * @return S32 成功返回EXIT_SUCCESS，失败返回错误码
 * @note 调用前需要确保SM4模块已启动
 */
static S32 sm4EncryptSw(Sm4Reg_s *reg, sm4TransactionDesc_s *desc)
{
    S32 ret = EXIT_SUCCESS;
    U32 groups = desc->len / SM4_BLOCK_SIZE;
    U32 srcAddr = desc->srcAddr;
    U32 destAddr = desc->destAddr;

    reg->ctrl.bit.invalid = 1;
    reg->ctrl.bit.sm4KeySel = 0;
    reg->ctrl.bit.sm4CntSel = 0;
    reg->ctrl.bit.kxpstartEn = 1;
    reg->ctrl.bit.chgcntrEn = 1;

    for (U32 i = 0; i < groups; i++) {
        volatile U32 *src = (volatile U32 *)srcAddr;
        volatile U32 *dest = (volatile U32 *)destAddr;

        for (U32 j = 0; j < SM4_KEYS_SIZE; j++) {
            reg->din[j] = src[j];
        }

        reg->ctrl.bit.start = 1;

        ret = sm4WaitDone(reg);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s-%d: block %u timeout", __func__, __LINE__, i);
            reg->irqClr.dword = IRQ_ALL_BITS;
            goto exit;
        }

        for (U32 j = 0; j < SM4_KEYS_SIZE; j++) {
            dest[j] = reg->dout[j];
        }

        srcAddr += SM4_BLOCK_SIZE;
        destAddr += SM4_BLOCK_SIZE;

        if (i == 0) {
            reg->ctrl.bit.kxpstartEn = 0;
            reg->ctrl.bit.chgcntrEn = 0;
        }
    }

exit:
    return ret;
}

#ifndef CONFIG_DRV_SM4_TEST
/**
 * @brief 获取设备配置
 * @details 从SBR中读取SM4模块的配置参数
 * @param [in] devId 设备ID
 * @param [out] cfg 配置参数指针
 * @return S32 成功返回EXIT_SUCCESS，失败返回错误码
 * @note 调用前需要确保SM4模块已启动
 */
static S32 sm4GetDevCfg(DevList_e devId, SbrSm4Cfg_s *cfg)
{
    if (cfg == NULL) {
        return -EINVAL;
    }

    if (devSbrRead(devId, cfg, 0, sizeof(SbrSm4Cfg_s)) != sizeof(SbrSm4Cfg_s)) {
        return -EIO;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("%s-%d: regAddr=%p irqNo=%u irqPrio=%u",
         __func__, __LINE__, cfg->regAddr, cfg->irqNo, cfg->irqPrio);
#endif

    if (cfg->regAddr == NULL || cfg->irqNo == 0 || cfg->irqPrio == 0) {
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}
#endif

/** 
 * @brief 初始化SM4驱动
 * @details 从SBR中读取SM4模块的配置参数，并初始化驱动数据结构
 * @param [in] devId 设备ID     
 * @return S32 成功返回EXIT_SUCCESS，失败返回错误码
 * @note 调用前需要确保SM4模块已启动
 */
S32 sm4Init(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    Sm4DrvData_s *drv = NULL;

    if (devLockByDriver(devId, SM4_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s-%d: lock failed devId=%d", __func__, __LINE__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (isDrvInit(devId)) {
        LOGE("%s-%d: already init devId=%d", __func__, __LINE__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_SM4)) {
        LOGE("%s-%d: drv not match devId=%d", __func__, __LINE__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    ret = peripsClockEnable(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO) {
        LOGE("%s-%d: clk enable failed ret=%d", __func__, __LINE__, ret);
        ret = -EIO;
        goto unlock;
    }

    ret = peripsReset(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO) {
        LOGE("%s-%d: reset failed ret=%d", __func__, __LINE__, ret);
        ret = -EIO;
        goto unlock;
    }

    drv = (Sm4DrvData_s *)calloc(1, sizeof(Sm4DrvData_s));
    if (drv == NULL) {
        LOGE("%s-%d: alloc failed", __func__, __LINE__);
        ret = -ENOMEM;
        goto unlock;
    }

#ifndef CONFIG_DRV_SM4_TEST
    ret = sm4GetDevCfg(devId, &drv->sbrCfg);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s-%d: get sbr failed", __func__, __LINE__);
        ret = -EIO;
        goto freeMem;
    }
#else
    drv->sbrCfg.regAddr = (void *)0xBE220000;
    drv->sbrCfg.irqNo = 52;
    drv->sbrCfg.irqPrio = 127;
#endif

    ret = ospInterruptHandlerInstall(drv->sbrCfg.irqNo, "sm4", OSP_INTERRUPT_UNIQUE,
                                     (OspInterruptHandler)sm4IrqHandler, drv);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s-%d: irq already installed", __func__, __LINE__);
    } else if (ret == OSP_SUCCESSFUL) {
        ret = ospInterruptSetPriority(drv->sbrCfg.irqNo, (OspInterruptPriority_e)drv->sbrCfg.irqPrio);
        if (ret != OSP_SUCCESSFUL) {
            LOGE("%s-%d: set irq prio failed ret=%d", __func__, __LINE__, ret);
            ret = -EIO;
            goto removeIrq;
        }
    } else {
        LOGE("%s-%d: install irq failed ret=%d", __func__, __LINE__, ret);
        ret = -EIO;
        goto freeMem;
    }

    ospInterruptVectorEnable(drv->sbrCfg.irqNo);

    ret = drvInstall(devId, drv);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s-%d: drv install failed", __func__, __LINE__);
        ret = -EIO;
        goto disableIrq;
    }

    ret = EXIT_SUCCESS;
    goto unlock;

disableIrq:
    ospInterruptVectorDisable(drv->sbrCfg.irqNo);
removeIrq:
    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, sm4IrqHandler, drv);
freeMem:
    free(drv);
unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}
/**
 * @brief 反初始化SM4驱动
 * @details 关闭SM4模块的中断、重置模块并释放驱动数据结构
 * @param [in] devId 设备ID
 * @return S32 成功返回EXIT_SUCCESS，失败返回错误码
 * @note 调用前需要确保SM4模块已启动
 */
S32 sm4DeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    Sm4DrvData_s *drv = NULL;
    Sm4Reg_s *reg = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_SM4, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (drv->sbrCfg.regAddr == NULL) {
        LOGE("%s-%d: invalid reg addr", __func__, __LINE__);
        ret = -EINVAL;
        goto unlock;
    }

    reg = (Sm4Reg_s *)drv->sbrCfg.regAddr;

    reg->irqEn.dword = 0;
    sm4Reset(reg);

    ospInterruptVectorDisable(drv->sbrCfg.irqNo);
    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, (OspInterruptHandler)sm4IrqHandler, drv);

    ret = drvUninstall(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s-%d: uninstall failed ret=%d", __func__, __LINE__, ret);
        ret = -EIO;
    }

unlock:
    funcRunEndHelper(devId);
exit:
    return ret;
}
/**
 * @brief 加密数据
 * @details 从SBR中读取SM4模块的配置参数，根据指定模式进行加密操作
 * @param [in] devId 设备ID
 * @param [out] desc 事务描述符指针
 * @param [in] mode 加密模式    
 * @return S32 成功返回EXIT_SUCCESS，失败返回错误码
 * @note 调用前需要确保SM4模块已启动
 */
S32 sm4EncryptData(DevList_e devId, sm4TransactionDesc_s *desc, sm4Mode_e mode)
{
    S32 ret = EXIT_SUCCESS;
    Sm4DrvData_s *drv = NULL;
    Sm4Reg_s *reg = NULL;

    ret = sm4ValidateDesc(desc);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (mode >= SM4_MODE_INVALID) {
        LOGE("%s-%d: invalid mode=%d", __func__, __LINE__, mode);
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_SM4, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (drv->sbrCfg.regAddr == NULL) {
        LOGE("%s-%d: invalid reg addr", __func__, __LINE__);
        ret = -EINVAL;
        goto unlock;
    }

    reg = (Sm4Reg_s *)drv->sbrCfg.regAddr;

    sm4Reset(reg);
    ret = sm4WaitIdle(reg);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s-%d: wait idle timeout", __func__, __LINE__);
        goto unlock;
    }

    reg->irqEn.dword = 0;

    sm4SetParams(reg, desc);

    switch (mode) {
    case SM4_MODE_HW:
        ret = sm4EncryptHw(reg, desc, 0);
        break;
    case SM4_MODE_SW:
        ret = sm4EncryptSw(reg, desc);
        break;
    case SM4_MODE_KEY_HW_AND_CNT_SW:
        ret = sm4EncryptHw(reg, desc, 1);
        break;
    default:
        ret = sm4EncryptHw(reg, desc, 0);
        break;
    }

unlock:
    funcRunEndHelper(devId);
exit:
    return ret;
}

