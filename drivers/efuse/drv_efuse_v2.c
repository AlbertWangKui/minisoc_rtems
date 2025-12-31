/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 * @file    drv_efuse.c
 * @author  cursor@starsmicrosystem.com
 * @date    2025/11/27
 * @brief   EFUSE driver - eFuse read/write, permission control, clear and verify
 */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_api.h"
#include "log_msg.h"
#include "drv_efuse_v2.h"
#include "drv_efuse_api.h"

///< 每个ELD的有效长度(U32)
static const U8 g_eldLen[EFUSE_MAXELD_COUNT] = {
    3, 6, 3, 4, 4, 4, 16, 16, 18,
    18, 18, 18, 6, 6, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 10, 10, 4, 2, 128
};

///< 获取ELD有效长度
static inline S32 getEldLen(EfuseEldNum_e eldNum)
{
    return (eldNum < EFUSE_MAXELD_COUNT) ? (S32)g_eldLen[eldNum] : -EINVAL;
}

///< 等待EFUSE空闲
static S32 waitIdle(EfuseReg_s *reg)
{
    U32 timeout = EFUSE_IDLE_TIMEOUT;
    while (((reg->status0 >> EFUSE_STATE_OFFSET) & EFUSE_STATE_MASK) != 0) {
        if (--timeout == 0) {
            return -ETIMEDOUT;
        }
    }
    return EXIT_SUCCESS;
}

///< 执行命令并等待完成
static S32 execCmd(EfuseReg_s *reg, EfuseOpcode_e op, EfuseEldNum_e eldNum)
{
    U32 errCode = 0;

    ///< 设置opcode和eldSel
    reg->cmdInfo = (reg->cmdInfo & ~0x3F00) | ((U32)eldNum << EFUSE_ELDSEL_OFFSET);
    reg->cmdInfo = (reg->cmdInfo & ~0x3) | (U32)op;

    ///< 启动命令
    reg->startCmd = 1;

    ///< 等待完成
    if (waitIdle(reg) != EXIT_SUCCESS) {
        return -ETIMEDOUT;
    }

    ///< 检查错误码
    errCode = (reg->status0 >> EFUSE_ERRCODE_OFFSET) & EFUSE_ERRCODE_MASK;
    if (errCode != EFUSE_ERR_DONE) {
        LOGE("%s-%d: errCode=%u", __func__, __LINE__, errCode);
        return -EIO;
    }
    return EXIT_SUCCESS;
}

///< 获取寄存器指针
static S32 getRegs(EfuseDrvData_s *drv, EfuseReg_s **reg)
{
    if (drv->sbrCfg.regAddr == 0) {
        return -EIO;
    }
    *reg = (EfuseReg_s *)drv->sbrCfg.regAddr;
    return EXIT_SUCCESS;
}

///< 校验ELD参数
static S32 checkEldParams(EfuseEldNum_e eldNum, U32 offset, U32 len)
{
    S32 maxLen = 0;

    if (eldNum < EFUSE_ELD_1 || eldNum >= EFUSE_ELD_26) {
        LOGE("%s-%d: invalid eldNum=%d", __func__, __LINE__, eldNum);
        return -EINVAL;
    }

    maxLen = getEldLen(eldNum);
    if (maxLen < 0) {
        return -EINVAL;
    }

    if (len == 0 || offset > (U32)maxLen || len > (U32)maxLen ||
        (offset + len) > (U32)maxLen || (offset + len) > EFUSE_MAX_ELD_DATA) {
        return -EINVAL;
    }
    return EXIT_SUCCESS;
}

///< 读取SBR配置
static S32 readSbrCfg(DevList_e devId, SbrEfuseCfg_s *cfg)
{
    if (devSbrRead(devId, cfg, 0, sizeof(SbrEfuseCfg_s)) != sizeof(SbrEfuseCfg_s)) {
        return -EIO;
    }
#ifdef CONFIG_DUMP_SBR
    LOGI("%s-%d: regAddr=0x%08x", __func__, __LINE__, cfg->regAddr);
#endif
    return EXIT_SUCCESS;
}

S32 efuseInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    EfuseDrvData_s *drv = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_V2_EFUSE)) {
        return -EINVAL;
    }

    if (isDrvInit(devId)) {
        return -EBUSY;
    }

    if (devLockByDriver(devId, EFUSE_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    drv = (EfuseDrvData_s *)calloc(1, sizeof(EfuseDrvData_s));
    if (drv == NULL) {
        LOGE("%s-%d: alloc failed", __func__, __LINE__);
        ret = -ENOMEM;
        goto unlock;
    }

    if (readSbrCfg(devId, &drv->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s-%d: sbr read failed", __func__, __LINE__);
        ret = -EIO;
        goto freeMem;
    }

    if (drvInstall(devId, drv) != EXIT_SUCCESS) {
        LOGE("%s-%d: install failed", __func__, __LINE__);
        ret = -EIO;
        goto freeMem;
    }

    goto unlock;

freeMem:
    free(drv);
unlock:
    devUnlockByDriver(devId);
    return ret;
}

S32 efuseDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    EfuseDrvData_s *drv = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_V2_EFUSE, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ret = drvUninstall(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s-%d: uninstall failed", __func__, __LINE__);
    }

    funcRunEndHelper(devId);
    return ret;
}

S32 efuseEldProgram(DevList_e devId, EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pSrc)
{
    S32 ret = EXIT_SUCCESS;
    EfuseDrvData_s *drv = NULL;
    EfuseReg_s *reg = NULL;
    U32 i = 0;

    if (pSrc == NULL) {
        return -EINVAL;
    }

    ret = checkEldParams(eldNum, offset, len);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_V2_EFUSE, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ret = getRegs(drv, &reg);
    if (ret != EXIT_SUCCESS) {
        goto unlock;
    }

    reg->lockAccess = EFUSE_MAGIC;

    ret = waitIdle(reg);
    if (ret != EXIT_SUCCESS) {
        goto lock;
    }

    ///< 写入数据
    for (i = 0; i < len; i++) {
        reg->progData[offset + i] = pSrc[i];
    }

    ///< 执行编程命令
    ret = execCmd(reg, EFUSE_OP_PROG, eldNum);

lock:
    reg->lockAccess = 0;
unlock:
    funcRunEndHelper(devId);
    return ret;
}

S32 efuseEldRead(DevList_e devId, EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pDest)
{
    S32 ret = EXIT_SUCCESS;
    EfuseDrvData_s *drv = NULL;
    EfuseReg_s *reg = NULL;
    U32 i = 0;

    if (pDest == NULL) {
        return -EINVAL;
    }

    ret = checkEldParams(eldNum, offset, len);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_V2_EFUSE, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ret = getRegs(drv, &reg);
    if (ret != EXIT_SUCCESS) {
        goto unlock;
    }

    reg->lockAccess = EFUSE_MAGIC;

    ret = waitIdle(reg);
    if (ret != EXIT_SUCCESS) {
        goto lock;
    }

    ///< 设置读ELD选择
    reg->rdEldCtrl = (reg->rdEldCtrl & ~EFUSE_RDELDSEL_MASK) | (U32)eldNum;

    ///< 读取数据
    for (i = 0; i < len; i++) {
        pDest[i] = reg->readData[offset + i];
    }

lock:
    reg->lockAccess = 0;
unlock:
    funcRunEndHelper(devId);
    return ret;
}

S32 efuseEldSenseRead(DevList_e devId, EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pDest)
{
    S32 ret = EXIT_SUCCESS;
    EfuseDrvData_s *drv = NULL;
    EfuseReg_s *reg = NULL;
    U32 i = 0;

    if (pDest == NULL) {
        return -EINVAL;
    }

    ret = checkEldParams(eldNum, offset, len);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_V2_EFUSE, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ret = getRegs(drv, &reg);
    if (ret != EXIT_SUCCESS) {
        goto unlock;
    }

    reg->lockAccess = EFUSE_MAGIC;

    ret = waitIdle(reg);
    if (ret != EXIT_SUCCESS) {
        goto lock;
    }

    ///< 执行SENSE命令
    ret = execCmd(reg, EFUSE_OP_SENSE, eldNum);
    if (ret != EXIT_SUCCESS) {
        goto lock;
    }

    ///< 设置读ELD选择
    reg->rdEldCtrl = (reg->rdEldCtrl & ~EFUSE_RDELDSEL_MASK) | (U32)eldNum;

    ///< 读取数据
    for (i = 0; i < len; i++) {
        pDest[i] = reg->readData[offset + i];
    }

lock:
    reg->lockAccess = 0;
unlock:
    funcRunEndHelper(devId);
    return ret;
}

S32 efuseEldClear(DevList_e devId, EfuseEldNum_e eldNum)
{
    S32 ret = EXIT_SUCCESS;
    EfuseDrvData_s *drv = NULL;
    EfuseReg_s *reg = NULL;

    if (eldNum < EFUSE_ELD_1 || eldNum >= EFUSE_ELD_26) {
        LOGE("%s-%d: invalid eldNum=%d", __func__, __LINE__, eldNum);
        return -EINVAL;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_V2_EFUSE, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ret = getRegs(drv, &reg);
    if (ret != EXIT_SUCCESS) {
        goto unlock;
    }

    reg->lockAccess = EFUSE_MAGIC;

    ret = waitIdle(reg);
    if (ret != EXIT_SUCCESS) {
        goto lock;
    }

    ret = execCmd(reg, EFUSE_OP_CLEAR, eldNum);

lock:
    reg->lockAccess = 0;
unlock:
    funcRunEndHelper(devId);
    return ret;
}

S32 efuseEldVerify(DevList_e devId, EfuseEldNum_e eldNum)
{
    S32 ret = EXIT_SUCCESS;
    EfuseDrvData_s *drv = NULL;
    EfuseReg_s *reg = NULL;

    if (eldNum < EFUSE_ELD_1 || eldNum >= EFUSE_ELD_26) {
        LOGE("%s-%d: invalid eldNum=%d", __func__, __LINE__, eldNum);
        return -EINVAL;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_V2_EFUSE, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ret = getRegs(drv, &reg);
    if (ret != EXIT_SUCCESS) {
        goto unlock;
    }

    reg->lockAccess = EFUSE_MAGIC;

    ret = waitIdle(reg);
    if (ret != EXIT_SUCCESS) {
        goto lock;
    }

    ret = execCmd(reg, EFUSE_OP_VERIFY, eldNum);

lock:
    reg->lockAccess = 0;
unlock:
    funcRunEndHelper(devId);
    return ret;
}

///< 检查ELD是否已烧录
static Bool isEldProgramed(DevList_e devId, EfuseEldNum_e eldNum, U32 offset, U32 keyLen)
{
    U32 key[16] = {0};
    S32 ret = 0;
    U32 i = 0;

    if (keyLen == 0 || keyLen > 16) {
        return 0;  ///< 参数无效，返回未烧录
    }

    ret = efuseEldRead(devId, eldNum, offset, keyLen, key);
    if (ret != EXIT_SUCCESS) {
        return 0;  ///< 读取失败，返回未烧录
    }

    for (i = 0; i < keyLen; i++) {
        if (key[i] != 0) {
            return 1;  ///< 已烧录
        }
    }
    return 0;  ///< 未烧录
}

S32 efuseWriteSm2Key(DevList_e devId, U8 *buf)
{
    S32 ret = 0;
    EfuseDrvData_s *drv = NULL;
    static const EfuseEldNum_e sm2Elds[] = {EFUSE_ELD_8, EFUSE_ELD_11, EFUSE_ELD_10, EFUSE_ELD_9};
    U32 i = 0;

    if (buf == NULL) {
        return -EINVAL;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_V2_EFUSE, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    for (i = 0; i < sizeof(sm2Elds) / sizeof(sm2Elds[0]); i++) {
        if (!isEldProgramed(devId, sm2Elds[i], 0, EFUSE_SM2_KEY_LEN)) {
            ret = efuseEldProgram(devId, sm2Elds[i], 0, EFUSE_SM2_KEY_LEN, (U32 *)buf);
            goto unlock;
        }
    }

    ret = -EIO;  ///< 所有ELD已被占用

unlock:
    funcRunEndHelper(devId);
    return ret;
}

S32 efuseWriteSm4Key(DevList_e devId, U8 *buf)
{
    S32 ret = 0;
    EfuseDrvData_s *drv = NULL;
    static const EfuseEldNum_e sm4Elds[] = {EFUSE_ELD_12, EFUSE_ELD_13};
    U32 i = 0;

    if (buf == NULL) {
        return -EINVAL;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_V2_EFUSE, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    for (i = 0; i < sizeof(sm4Elds) / sizeof(sm4Elds[0]); i++) {
        if (!isEldProgramed(devId, sm4Elds[i], 0, EFUSE_SM4_KEY_LEN)) {
            ret = efuseEldProgram(devId, sm4Elds[i], 0, EFUSE_SM4_KEY_LEN, (U32 *)buf);
            goto unlock;
        }
    }

    ret = -EIO;  ///< 所有ELD已被占用

unlock:
    funcRunEndHelper(devId);
    return ret;
}

