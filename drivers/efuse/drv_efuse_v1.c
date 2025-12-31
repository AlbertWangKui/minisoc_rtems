/**
 * copyright (C), 2020, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file efuse.c
 * @author zhangmx3
 * @date 2024/01/09
 * @brief none
 * @version v1.0
 */
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_api.h"
#include "osp_interrupt.h"
#include "log_msg.h"
#include "drv_efuse_v1.h"
#include "drv_efuse_api.h"

static inline S32 efuseGetEldLen(EfuseEldNum_e eldNum)
{
    S32 efuse_eld_len[17] = {
        5, 4, 4, 4, 4, 4, 16, 16, 16, 16,
        16, 16, 16, 4, 4, 4, 4
    };
    return efuse_eld_len[eldNum];
}

 /**
  * @brief   等待efuse状态变成IDLE状态
  * @param   regs, efuse registers base address
  * @return  EXIT_SUCCESS  成功
  *          -EXIT_FAILURE 失败
  * @warning 写寄存器值
  * @note    新生成函数
  * @note    增加超时机制，防止程序阻塞在此处
  */
 static S32 efuseIdleWait(efuseReg_s *regs)
 {
    S32 idle_flags;
    S32 ret = EXIT_SUCCESS;
    U32 timeout = 20000;

    if (regs == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    do {
        idle_flags = (regs->status0 >> EFUSE_STATE_OFFSET);
        idle_flags &= EFUSE_STATE_MASK;
    } while ((EFUSE_IDLE_FLAGS != idle_flags) && (timeout--));

    if (EFUSE_IDLE_FLAGS != idle_flags) {
        ret = -ERR_CODE_TIMEOUT;
    }

out:
    return ret;
 }

 /*
  * @brief   efuse解锁
  * @param   regs, efuse registers base address
  * @return  EXIT_SUCCESS  成功
  *          -EXIT_FAILURE 失败
  * @warning 写寄存器值
  * @note    新生成函数
  */
 static S32 efuseUnlock(efuseReg_s *regs)
 {
    S32 ret = EXIT_SUCCESS;

    if (regs == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }
    regs->lockAccess = EFUSE_DEVICE_MAGIC;

out:
    return ret;
 }

 /**
  * @brief   efuse上锁
  * @param   regs, efuse registers base address
  * @return  EXIT_SUCCESS  成功
  *          -EXIT_FAILURE 失败
  * @warning 写寄存器值
  * @note    新生成函数
  */
 static  __attribute__((unused)) S32 efuseLock(efuseReg_s *regs)
 {
    S32 ret = EXIT_SUCCESS;

    if (regs == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }
    regs->lockAccess = 0;

out:
    return ret;
 }

  /**
  * @brief   获取efuse的错误原因
  * @param   regs, efuse registers base address
  * @param   errCode, output parameters, errcode pointer
  * @return  EXIT_SUCCESS  成功
  *          -EXIT_FAILURE 失败
  * @warning 写寄存器值
  * @note    新生成函数
  */
 static S32 efuseErrCodeGet(efuseReg_s *regs,ErrEventType_e *errCode)
 {
    S32 ret = EXIT_SUCCESS;

    if (regs == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }
    *errCode = (ErrEventType_e)((regs->status0 & 0x000000F0) >> EFUSE_ERRCODE_OFFSET);

out:
    return ret;
 }

/**
 * @brief    efuse eld valid set
 * @param   regs, efuse registers base address
 * @param    eldNum eld号
 * @param    len 数据长度
 * @param    pSrc 写入的数据
 * @return   EXIT_SUCCESS 成功
 *           EXIT_FAILURE 失败
 */
static S32 efuseSetWrEldValid(efuseReg_s *regs,EfuseEldNum_e eldNum)
{
    U32 regValue;
    S32 ret = EXIT_SUCCESS;

    if (regs == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    regValue = regs->cmdInfo;
    regValue &= ~(0x00003f00);
    regValue |= (eldNum << 8);
    regs->cmdInfo = regValue;

    ret = EXIT_SUCCESS;
out:
    return ret;
}

/**
  * @brief    efuse设置opcode为写。
  * @param   regs, efuse registers base address
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
  * @note    新生成函数
 */
static S32 efuseSetOpcodeWrite(efuseReg_s *regs)
{
    U32 tmp;
    S32 ret = EXIT_SUCCESS;

    if (regs == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    tmp = regs->cmdInfo;
    tmp  = tmp & (~(EFUSE_OPCODE_MASK));
    tmp = tmp | 1;
    regs->cmdInfo = tmp;

out:
    return ret;
}

/**
 * @brief    efuse开始工作
  * @param   regs, efuse registers base address
 * @return   EXIT_SUCCESS 成功
 *           EXIT_FAILURE 失败
 * @note    新生成函数
 */
static S32 efuseStart(efuseReg_s *regs)
{
    S32 ret = EXIT_SUCCESS;

    if (regs == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    regs->startCmd = 1;
out:
    return ret;
}

/**
 * @brief    efuse设置读eld号
 * @param    regs, efuse registers base address
 * @param    eldNum eld号
 * @param    index 偏移
 * @param    len 数据长度
 * @param    pSrc 写入的数据
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 */
static S32 efuseSetRdEldValid(efuseReg_s *regs,EfuseEldNum_e eldNum)
{
    S32 ret = EXIT_SUCCESS;
    U32 regValue;

   if (regs == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    regValue = regs->rdEldSel;
    regValue &= ~(0x0000003f);
    regValue |= (eldNum << 0);
    regs->rdEldSel = regValue;

out:

    return ret;
}

/**
 * @brief    devId: efuse device ID
 * @param    eldNum eld号
 * @param    offset 偏移
 * @return   0 ELD未烧录过
 *           1 ELD烧录过
 *           -EXIT_FAILURE 失败
 */
static S32 isEfuseEldProgamed(DevList_e devId, EfuseEldNum_e eldNum,U32 offset,U32 keyLen)
{
    U32 i;
    U32 key[16];
    S32 ret = 0;

    if (efuseEldRead(devId,eldNum,offset,16,key) != 0) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    for (i = 0; i < keyLen; i++) {
        if (key[i] != 0) {
            ret = 1;
            goto exit;
        }
    }

exit:
    return ret;
}

static S32 efuseDevCfgGet(DevList_e devId, SbrEfuseCfg_s *pEfuseCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (pEfuseCfg == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    if (devSbrRead(devId, pEfuseCfg, 0, sizeof(SbrEfuseCfg_s)) != sizeof(SbrEfuseCfg_s)) {
        ret = -EXIT_FAILURE;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("efuse: SBR dump - regAddr:0x%08x\r\n",
         pEfuseCfg->regAddr);
#endif

out:
    return ret;
}


/**
 * @brief    初始化软件资源和复位硬件
 * @param    devId: efuse device ID
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 */
S32 efuseInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    Device_s *efuseDevice = NULL;
    efuseDrvData_s *pEfuseDrvData = NULL;

    if (isDrvInit(devId)) {
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1_EFUSE)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    efuseDevice = getDevice(devId);
    if (efuseDevice == NULL) {
        ret = -EIO;
        goto exit;
    }

    ///< 申请驱动私有数据内存并获取设备配置
    pEfuseDrvData = (efuseDrvData_s*)calloc(1, sizeof(efuseDrvData_s));
    if (pEfuseDrvData == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    if (efuseDevCfgGet(devId, &pEfuseDrvData->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    ret = peripsReset(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO) {
        ret = -EIO;
        goto exit;
    }

    // efuseDevice->driver = (void*)pEfuseDrvData;
    if (drvInstall(devId,(void*)pEfuseDrvData) != 0) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = EXIT_SUCCESS;

exit:
    if (ret != EXIT_SUCCESS && pEfuseDrvData != NULL) {
        free(pEfuseDrvData);
    }
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief    反初始化
 * @param    devId: efuse device ID
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 */
S32 efuseDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    efuseDrvData_s *pEfuseDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_V1_EFUSE)) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == false) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (getDevDriver(devId,(void **)&pEfuseDrvData) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (pEfuseDrvData == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = peripsReset(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (drvUninstall(devId) != 0) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief    efuse写操作
 * @param    eldNum eld号
 * @param    index 偏移
 * @param    len 数据长度
 * @param    pSrc 写入的数据
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 */
S32 efuseEldProgram(DevList_e devId, EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pSrc)
{
    S32 ret = EXIT_SUCCESS;
    U32 setLast;
    U32 efuseEldLen;
    ErrEventType_e err_code;
    efuseDrvData_s *pEfuseDrvData = NULL;
    efuseReg_s *regs = NULL;

    if (pSrc == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (getDevDriver(devId,(void **)&pEfuseDrvData) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (pEfuseDrvData == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    regs = (efuseReg_s *)(pEfuseDrvData->sbrCfg.regAddr);

    if (efuseUnlock(regs) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = efuseIdleWait(regs);
    if(EXIT_SUCCESS != ret){
        goto exit;
    }
    efuseEldLen = efuseGetEldLen(eldNum);
    setLast = offset + len;
    if (setLast > efuseEldLen) {
        ret = -ERR_CODE_OFFSET_INVALID;
        goto exit;
    }

    ///< 读取eld当前数据并保存到EFUSE_PROG_DATA，不会实际写入到efuse
    ///< 更新EFUSE_PROG_DATA中的eld数据，不会实际写入到efuse
    if (efuseSetWrEldValid(regs,eldNum) != EXIT_SUCCESS) {
        ret = -ERR_CODE_ELD_INVALID;
        goto exit;
    }

    for (U32 cnt = offset; cnt < setLast; cnt++) {
        regs->progData[cnt] = *pSrc++;
    }

    ///< 设置opcode
    efuseSetOpcodeWrite(regs);

    ///< start，开始写入到efuse
    efuseStart(regs);

    ///< 等待idle
    ret = efuseIdleWait(regs);
    if(EXIT_SUCCESS != ret){
        goto exit;
    }

    ///< 读取状态位
    if (EXIT_SUCCESS != efuseErrCodeGet(regs,&err_code)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (ERR_CODE_CMD_DONE != err_code) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

exit:
    efuseLock(regs);
    devUnlockByDriver(devId);
    return ret;
}

 /**
 * @brief    efuse读操作
 * @param    eldNum eld号
 * @param    offset 偏移
 * @param    len 数据长度 有多少个32bit的数据。
 * @param    pDest 读到的数据buf
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 * @note     原流程，eld2的bit127被置1后，不只eld写会失败，eld读也会失败，
 *           与硬件核对操作流程后删除了不需要opcode、start等操作（PR2517）
 */
S32 efuseEldRead(DevList_e devId,EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pDest)
{
    S32 ret = EXIT_SUCCESS;
    U32 getLast;
    U32 efuseEldLen;
    efuseReg_s *regs = NULL;
    efuseDrvData_s *pEfuseDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_V1_EFUSE)) {
        return -EINVAL;
    }

    if (pDest == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if(getDevDriver(devId,(void **)&pEfuseDrvData) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto unlock;
    }

    if (pEfuseDrvData == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    regs = (efuseReg_s *)(pEfuseDrvData->sbrCfg.regAddr);

    if (efuseUnlock(regs) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto unlock;
    }

    ret = efuseIdleWait(regs);
    if(EXIT_SUCCESS != ret){
        goto unlock;
    }
    efuseEldLen = efuseGetEldLen(eldNum);
    getLast = offset + len;
    if (getLast>efuseEldLen) {
        ret = -ERR_CODE_OFFSET_INVALID;
        goto unlock;
    }

    getLast = (getLast > efuseEldLen) ? efuseEldLen : getLast;

    efuseSetRdEldValid(regs,eldNum);
    for (U32 cnt = offset; cnt < getLast; cnt++) {
        *(pDest++) = regs->readEldData[cnt];
    }

    if (efuseLock(regs) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto unlock;
    }

unlock:
    if (devUnlockByDriver(devId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

 /**
 * @brief    sm2 key 烧录
 * @param    devId efuse id
 * @param    buf 待写入缓冲区
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 */
S32 efuseWriteSm2Key(DevList_e devId, U8 *buf)
{
    S32 ret = EXIT_SUCCESS;

    if (!isDrvMatch(devId, DRV_ID_STARS_V1_EFUSE)) {
        return -EINVAL;
    }

    if (isEfuseEldProgamed(devId,EFUSE_ELD_8,0,16) == 0) {
        ret = efuseEldProgram(devId,EFUSE_ELD_8,0,16,(U32*)buf);
        goto exit;
    }

    if (isEfuseEldProgamed(devId,EFUSE_ELD_11,0,16) == 0) {
        ret = efuseEldProgram(devId,EFUSE_ELD_11,0,16,(U32*)buf);
        goto exit;
    }

    if (isEfuseEldProgamed(devId,EFUSE_ELD_10,0,16) == 0) {
        ret = efuseEldProgram(devId,EFUSE_ELD_10,0,16,(U32*)buf);
        goto exit;
    }

    if (isEfuseEldProgamed(devId,EFUSE_ELD_9,0,16) == 0) {
        ret = efuseEldProgram(devId,EFUSE_ELD_9,0,16,(U32*)buf);
        goto exit;
    }

    ret = -EXIT_FAILURE;
exit:
    return ret;
}

 /**
 * @brief    sm4 key 烧录
 * @param    devId efuse id
 * @param    buf 待写入缓冲区
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 */
S32 efuseWriteSm4Key(DevList_e devId, U8 *buf)
{
    S32 ret = EXIT_SUCCESS;

    if (!isDrvMatch(devId, DRV_ID_STARS_V1_EFUSE)) {
        return -EINVAL;
    }

    if (isEfuseEldProgamed(devId,EFUSE_ELD_12,0,4) == 0) {
        ret = efuseEldProgram(devId,EFUSE_ELD_12,0,4,(U32*)buf);
        goto exit;
    }

    if (isEfuseEldProgamed(devId,EFUSE_ELD_12,4,4) == 0) {
        ret = efuseEldProgram(devId,EFUSE_ELD_12,4,4,(U32*)buf);
        goto exit;
    }

    if (isEfuseEldProgamed(devId,EFUSE_ELD_12,8,4) == 0) {
        ret = efuseEldProgram(devId,EFUSE_ELD_12,8,4,(U32*)buf);
        goto exit;
    }

    ret = -EXIT_FAILURE;
exit:
    return ret;
}
