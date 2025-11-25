/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sm2.c
 * @author pengjb@starsmicrosystem.com
 * @date 2025/09/30
 * @brief SM2 cryptographic driver implementation
 */
#include <stdio.h>
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "log_msg.h"
#include "osp_interrupt.h"
#include "udelay.h"
#include "bsp_sbr.h"
#include "sbr_api.h"
#include "drv_sm2_api.h"
#include "drv_sm2.h"

#ifndef CONFIG_DRV_SM2_TEST
/**
 * @brief 获取SM2设备配置
 * @details 从SBR中读取SM2设备配置，包括寄存器地址、中断号和中断优先级
 * @param [in] devId 设备ID
 * @param [out] cfg 指向SbrSm2Cfg_s结构体的指针，用于存储配置信息
 * @return EXIT_SUCCESS表示成功获取配置，-EINVAL表示参数无效，-EIO表示读取失败
 */
static S32 sm2DevCfgGet(DevList_e devId, SbrSm2Cfg_s *cfg)
{
    if (cfg == NULL) {
        return -EINVAL;
    }

    if (devSbrRead(devId, cfg, 0, sizeof(SbrSm2Cfg_s)) != sizeof(SbrSm2Cfg_s)) {
        return -EIO;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("sm2: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u\r\n",
         cfg->regAddr, cfg->irqNo, cfg->irqPrio);
#endif

    if ((cfg->regAddr == NULL) || (cfg->irqNo == 0) || (cfg->irqPrio == 0)) {
        return -EINVAL;
    }
    return EXIT_SUCCESS;
}
#endif
/**
 * @brief 软复位SM2硬件模块
 * @details 通过写复位寄存器触发硬件复位，清除内部状态机
 * @param [in] sm2Drv 驱动数据结构
 * @return void
 * @note 调用后需要等待硬件进入IDLE状态
 */
static void sm2InnerReset(Sm2DrvData_s *sm2Drv)
{
    Sm2Reg_s *reg = NULL;
    if (sm2Drv == NULL) {
        return;
    }
    if (sm2Drv->sbrCfg.regAddr == NULL) {
        return;
    }

    reg = (Sm2Reg_s *)sm2Drv->sbrCfg.regAddr;
    reg->softReset.fields.reset = 1;
    asm("NOP\n\t");
    asm("NOP\n\t");
    asm("NOP\n\t");
}

/**
 * @brief 禁用SM2中断
 * @details 清除中断使能寄存器，禁用所有SM2中断
 * @param [in] sm2Drv 驱动数据结构
 * @return EXIT_SUCCESS表示操作成功，-EINVAL表示参数无效
 */
static S32 sm2InterruptDisable(Sm2DrvData_s *sm2Drv)
{
    Sm2Reg_s *reg = NULL;
    if (sm2Drv == NULL) {
        return -EINVAL;
    }
    if (sm2Drv->sbrCfg.regAddr == NULL) {
        return -EINVAL;
    }
    reg = (Sm2Reg_s *)sm2Drv->sbrCfg.regAddr;
    reg->irqEnable.dword = 0;

    return EXIT_SUCCESS;
}

/**
 * @brief 使能SM2中断
 * @details 设置中断使能寄存器，使能所有SM2中断
 * @param [in] sm2Drv 驱动数据结构
 * @return EXIT_SUCCESS表示操作成功，-EINVAL表示参数无效
 */
static S32 sm2InterruptEnable(Sm2DrvData_s *sm2Drv)
{
    Sm2Reg_s *reg = NULL;
    if (sm2Drv == NULL) {
        return -EINVAL;
    }
    if (sm2Drv->sbrCfg.regAddr == NULL) {
        return -EINVAL;
    }
    reg = (Sm2Reg_s *)sm2Drv->sbrCfg.regAddr;
    reg->irqEnable.dword = SM2_IRQ_MASK;

    return EXIT_SUCCESS;
}

/**
 * @brief 轮询SM2操作结果
 * @details 检查状态寄存器中的结果位，判断操作是否完成
 * @param [in] sm2Drv 驱动数据结构
 * @return EXIT_SUCCESS表示操作成功，-EINVAL表示参数无效，-EIO表示操作失败
 */
static S32 sm2PollResult(Sm2DrvData_s *sm2Drv)
{
    Sm2Reg_s *reg = NULL;

    if (sm2Drv == NULL) {
        return -EINVAL;
    }
    if (sm2Drv->sbrCfg.regAddr == NULL) {
        return -EINVAL;
    }

    reg = (Sm2Reg_s *)sm2Drv->sbrCfg.regAddr;
    return (reg->state.fields.result == 0 ? EXIT_SUCCESS : -EIO);
}

/**
 * @brief SM2中断处理函数
 * @details 处理SM2硬件触发的中断，检查中断状态并调用回调函数
 * @param [in] arg 指向SM2驱动数据结构的指针
 * @return void
 */
static void sm2IrqHandler(void *arg)
{
    Sm2DrvData_s *data = (Sm2DrvData_s *)arg;
    if (data == NULL || data->sbrCfg.regAddr == NULL) {
        return;
    }

    Sm2Reg_s *reg = (Sm2Reg_s *)data->sbrCfg.regAddr;
    if (reg->irqStatus.dword != 0) {
        LOGD("SM2 IRQ STATUS 0x%x\r\n", reg->irqStatus.dword);
        LOGD("SM2 Result 0x%x\r\n", reg->state.fields.result);
        reg->irqStatus.dword = SM2_IRQ_MASK;
    }

    if (data->callback != NULL) {
        data->callback(data->callbackArg);
    }
}

/**
 * @brief 轮询SM2硬件空闲状态
 * @param [in] sm2Drv 驱动数据结构
 * @return EXIT_SUCCESS表示硬件空闲，-EINVAL表示参数无效，-EIO表示硬件繁忙
 */
static S32 sm2PollIdle(Sm2DrvData_s *sm2Drv)
{
    Sm2Reg_s *reg = NULL;
    if (sm2Drv == NULL) {
        return -EINVAL;
    }
    if (sm2Drv->sbrCfg.regAddr == NULL) {
        return -EINVAL;
    }
    reg = (Sm2Reg_s *)sm2Drv->sbrCfg.regAddr;
    return(reg->state.fields.state == SM2_STATE_IDLE ? EXIT_SUCCESS : -EIO);
}

/**
 * @brief 轮询SM2操作完成标志
 * @param [in] sm2Drv 驱动数据结构
 * @return EXIT_SUCCESS表示操作完成，-EINVAL表示参数无效，-EIO表示操作未完成
 */
static S32 sm2PollDone(Sm2DrvData_s *sm2Drv)
{
    Sm2Reg_s *reg = NULL;
    if (sm2Drv == NULL) {
        return -EINVAL;
    }
    if (sm2Drv->sbrCfg.regAddr == NULL) {
        return -EINVAL;
    }
    reg = (Sm2Reg_s *)sm2Drv->sbrCfg.regAddr;
    return(reg->finishStatus.fields.finish == 1 ? EXIT_SUCCESS : -EIO);
}

/**
 * @brief SM2验签公共准备函数
 * @param [in] pDrvData 驱动数据
 * @param [in] signBuf 签名缓冲区
 * @param [in] pkey 公钥（NULL表示eFuse）
 * @param [in] dataIn SM3哈希值
 * @return EXIT_SUCCESS表示准备成功，-EIO表示硬件忙或操作失败
 */
static S32 sm2VerifyPrepare(Sm2DrvData_s *pDrvData, U32 *signBuf, U32 *pkey, U32 *dataIn)
{
    Sm2Reg_s *reg = NULL;

    ///< 参数有效性检查
    if (pDrvData == NULL || signBuf == NULL || dataIn == NULL) {
        LOGE("%s: Invalid parameters\r\n", __func__);
        return -EINVAL;
    }
    ///< 软复位
    sm2InnerReset(pDrvData);

    ///< 等待空闲
    S32 ret = sm2PollIdle(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sm2PollIdle failed, ret = %d\r\n", __func__, ret);
        return -EIO;
    }

    ///< 检查地址有效性
    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address\r\n", __func__);
        return -EINVAL;
    }
    ///< 配置寄存器
    reg = (Sm2Reg_s *)pDrvData->sbrCfg.regAddr;
    dwMemCpy((volatile U32 *)&reg->signR[0], signBuf, SM2_SIGN_BUF_SIZE_DWORDS);
    dwMemCpy((volatile U32 *)&reg->dataIn[0], dataIn, SM2_HASH_SIZE_DWORDS);

    ///< 设置密钥来源
    if (pkey != NULL) {
        dwMemCpy((volatile U32 *)&reg->pkeyX[0], pkey, SM2_PKEY_SIZE_DWORDS);
        reg->funcSel.fields.pubKeys = SM2_PKEY_SOURCE_SOFTWARE;
    } else {
        reg->funcSel.fields.pubKeys = SM2_PKEY_SOURCE_EFUSE;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief SM2驱动初始化
 * @param [in] devId 控制器编号
 * @return EXIT_SUCCESS表示初始化成功，-EBUSY表示获取锁失败，-EINVAL表示参数无效，-EIO表示硬件操作失败，-ENOMEM表示内存分配失败
 */
S32 sm2Init(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    Sm2DrvData_s *pDrvData = NULL;

    if (devLockByDriver(devId, SM2_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: Failed to lock device, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if(isDrvInit(devId) == true) {
        ret = -EBUSY;
        goto unlock;
    }

    if(!isDrvMatch(devId, DRV_ID_STARS_SM2)) {
        ret = -EINVAL;
        goto unlock;
    }

    /* 时钟使能 tianhe sm2 无此时钟开关 */
    ret = peripsClockEnable(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO)  {
        LOGE("%s: clock enable failed\r\n", __func__);
        ret = -EIO;
        goto unlock;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    pDrvData = (Sm2DrvData_s*)calloc(1, sizeof(Sm2DrvData_s));
    if(pDrvData == NULL) {
        LOGE("%s: SM2 alloc memory failed\r\n", __func__);
        ret = -ENOMEM;
        goto unlock;
    }

#ifndef CONFIG_DRV_SM2_TEST
    if (sm2DevCfgGet(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s: get sbr failed\r\n", __func__);
        ret = -EIO;
        goto free_mem;
    }
#else
    pDrvData->sbrCfg.regAddr = (void*)0xBE200000;
    pDrvData->sbrCfg.irqNo = 50;
    pDrvData->sbrCfg.irqPrio = 127;
#endif

    ret = ospInterruptHandlerInstall(pDrvData->sbrCfg.irqNo, "sm2", OSP_INTERRUPT_UNIQUE,
        (OspInterruptHandler)sm2IrqHandler, pDrvData);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s: irq handler already installed\r\n", __func__);
    } else if (ret == OSP_SUCCESSFUL) {
        ret = ospInterruptSetPriority(pDrvData->sbrCfg.irqNo, (OspInterruptPriority_e)pDrvData->sbrCfg.irqPrio);
        if(ret != OSP_SUCCESSFUL) {
            LOGE("%s: irq priority set failed\r\n", __func__);
            ret = -EIO;
            goto remove_isr;
        }
    } else {
        LOGE("%s: irq handler install failed\r\n", __func__);
        ret = -EIO;
        goto free_mem;
    }

    ospInterruptVectorEnable(pDrvData->sbrCfg.irqNo);
    if(drvInstall(devId, pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: drv install failed\r\n", __func__);
        ret = -EIO;
        goto disable_vector;
    }

    ret = EXIT_SUCCESS;
    goto unlock;

disable_vector:
    ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
remove_isr:
    ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo, sm2IrqHandler, pDrvData);
free_mem:
    free(pDrvData);
unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief 中断方式实现SM2验签
 * @param [in] devId 控制器编号
 * @param [in] signBuf 签名数据，共64bytes, 前32bytes是sign_r，后32bytes是sign_s
 * @param [in] pkey 公钥数据，共64bytes, 前32bytes是keys_x，后32bytes是keys_y；如果keys来源于eFuse，参数直接写成NULL
 * @param [in] dataIn 存储sm3计算的hash值, hash值是256bit
 * @return EXIT_SUCCESS表示验签请求发送成功，-EINVAL表示参数无效，-EBUSY表示获取锁失败，-EIO表示硬件操作失败
 */
S32 sm2DoVerify(DevList_e devId, U32 *signBuf, U32 *pkey, U32 *dataIn)
{
    S32 ret = EXIT_SUCCESS;
    Sm2DrvData_s *pDrvData = NULL;
    Sm2Reg_s *reg = NULL;


    if (devLockByDriver(devId, SM2_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: Failed to lock device, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_SM2)) {
        ret = -EINVAL;
        goto unlock;
    }

    if (signBuf == NULL || dataIn == NULL) {
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void **)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: Failed to get driver data, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: Driver data is NULL, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    ret = sm2VerifyPrepare(pDrvData, signBuf, pkey, dataIn);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sm2VerifyPrepare failed, ret = %d\r\n", __func__, ret);
        goto unlock;
    }

    ///< 使能中断
    ret = sm2InterruptEnable(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sm2InterruptEnable failed, ret = %d\r\n", __func__, ret);
        goto unlock;
    }

    ///< 检查地址有效性
    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address, devId=%d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    reg = (Sm2Reg_s *)pDrvData->sbrCfg.regAddr;
    reg->softStart.fields.start = 1;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief 查询方式实现SM2验签
 * @param [in] devId 控制器编号
 * @param [in] dataIn 存储sm3计算的hash值, hash值是256bit
 * @param [in] signBuf 签名数据，共64bytes, 前32bytes是sign_r，后32bytes是sign_s
 * @param [in] pkey 公钥数据，共64bytes, 前32bytes是keys_x，后32bytes是keys_y；如果keys来源于eFuse，参数直接写成NULL
 * @param [in] timeout 轮询超时时间，单位为1毫秒
 * @return EXIT_SUCCESS表示验签成功，-EINVAL表示参数无效，-EBUSY表示获取锁失败，-EIO表示硬件操作失败，-ETIMEDOUT表示操作超时
 */
S32 sm2DoVerifyByPolling(DevList_e devId, U32 *dataIn, U32 *signBuf, U32 *pkey, U32 timeout)
{
    S32 ret = EXIT_SUCCESS;
    Sm2DrvData_s *pDrvData = NULL;
    Sm2Reg_s *reg = NULL;
    U32 waitCount = 0;

    if (devLockByDriver(devId, SM2_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: Failed to lock device, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_SM2)) {
        ret = -EINVAL;
        goto unlock;
    }

    if (signBuf == NULL || dataIn == NULL) {
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void **)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: Failed to get driver data, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: Driver data is NULL, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    ret = sm2VerifyPrepare(pDrvData, signBuf, pkey, dataIn);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sm2VerifyPrepare failed, ret = %d\r\n", __func__, ret);
        goto unlock;
    }

    ///< 禁用中断
    ret = sm2InterruptDisable(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sm2InterruptDisable failed, ret = %d\r\n", __func__, ret);
        ret = -EIO;
        goto unlock;
    }

    ///< 检查地址有效性
    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address, devId=%d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    reg = (Sm2Reg_s *)pDrvData->sbrCfg.regAddr;
    reg->softStart.fields.start = 1;

    while (waitCount <= timeout) {
        ret = sm2PollDone(pDrvData);
        if (ret == EXIT_SUCCESS) {
            break;
        }
        mdelay(1);
        waitCount++;
    }

    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sm2PollDone failed, ret = %d\r\n", __func__, ret);
        ret = -ETIMEDOUT;
        goto unlock;
    }

    ret = sm2PollResult(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sm2PollResult failed, ret = %d\r\n", __func__, ret);
        ret = -EIO;
        goto unlock;
    }
unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief 注册SM2中断回调函数
 * @param [in] devId 控制器编号
 * @param [in] callback 中断回调函数接口
 * @param [in] callbackArg 中断回调函数参数
 * @return EXIT_SUCCESS表示注册成功，-EINVAL表示参数无效，-EBUSY表示获取锁失败，-EIO表示硬件操作失败
 */
S32 sm2CallbackRegister(DevList_e devId, sm2IrqCallBack callback, void *callbackArg)
{
    S32 ret = EXIT_SUCCESS;
    Sm2DrvData_s *pDrvData = NULL;

    if (devLockByDriver(devId, SM2_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: Failed to lock device, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    ///< 检查驱动匹配
    if (!isDrvMatch(devId, DRV_ID_STARS_SM2)) {
        ret = -EINVAL;
        goto unlock;
    }

    if (callback == NULL) {
        LOGE("%s: callback is NULL\r\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void **)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: Failed to get driver data, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: Driver data is NULL, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pDrvData->callback = callback;
    pDrvData->callbackArg = callbackArg;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief SM2驱动去初始化
 * @param [in] devId 控制器编号
 * @return EXIT_SUCCESS表示去初始化成功，-EINVAL表示参数无效，-EIO表示硬件操作失败或设备未初始化，-EBUSY表示获取锁失败
 */
S32 sm2Deinit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    Sm2DrvData_s *pDrvData = NULL;
    Sm2Reg_s *reg = NULL;

    if (devLockByDriver(devId, SM2_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: Failed to lock device, devId=%d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_SM2)) {
        ret = -EINVAL;
        goto unlock;
    }

    if (!isDrvInit(devId)) {
        LOGE("%s: Device not initialized, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (getDevDriver(devId, (void **)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: Failed to get driver data, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData == NULL) {
        LOGE("%s: Driver data is NULL, devId=%d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    ///< 检查地址有效性
    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address, devId=%d\r\n", __func__, devId);
        ret = -EINVAL;
        goto unlock;
    }

    reg = (Sm2Reg_s *)pDrvData->sbrCfg.regAddr;

    ///< 禁用中断
    reg->irqEnable.dword = 0;

    ///< 重置SM2模块
    sm2InnerReset(pDrvData);

    ///< 禁用中断向量
    ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);

    ///< 移除中断处理函数
    ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo, sm2IrqHandler, pDrvData);

    ///< 卸载驱动
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