/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_trng_th.c
 * @author dingwei (dingwei@starsmicrosystem.com)
 * @date 2025/11/26
 * @brief TRNG driver implementation
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common_defines.h"
#include "bsp_api.h"
#include "udelay.h"
#include "log_msg.h"
#include "drv_trng_th.h"
#include "drv_trng_api.h"
#include "osp_interrupt.h"

#define DEFAULT_SEED_EMPTY_LEVEL       (0x14U)
#define DEFAULT_RAND_EMPTY_LEVEL       (0x10U)
#define DEFAULT_RAND_THRESHOLD         (512U)

/**
 * @brief Wait for TRNG to become idle
 * @param [in] reg TRNG register base
 * @param [in] timeoutMs Timeout in milliseconds
 * @return EXIT_SUCCESS on success, -EIO on timeout
 */
static S32 waitIdle(TrngReg_s *reg, U32 timeoutMs)
{
    while (timeoutMs > 0) {
        if (reg->stat.b.busy == 0) {
            return EXIT_SUCCESS;
        }
        udelay(TRNG_POLL_DELAY_US);
        timeoutMs--;
    }
    return -EIO;
}

/**
 * @brief Wait for random data available
 * @param [in] reg TRNG register base
 * @param [in] timeoutMs Timeout in milliseconds
 * @return EXIT_SUCCESS on success, -EIO on timeout
 */
static S32 waitRandReady(TrngReg_s *reg, U32 timeoutMs)
{
    while (timeoutMs > 0) {
        if (reg->stat.b.randEmpty == 0) {
            return EXIT_SUCCESS;
        }
        udelay(TRNG_POLL_DELAY_US);
        timeoutMs--;
    }
    return -EIO;
}

/**
 * @brief Execute zeroize and generate noise
 * @param [in] reg TRNG register base
 * @return EXIT_SUCCESS on success, -EIO on timeout
 */
static S32 zeroize(TrngReg_s *reg)
{
    reg->ctrl.w = CMD_ZEROIZE;
    if (waitIdle(reg, TRNG_WAIT_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: timeout\n", __func__);
        return -EIO;
    }

    reg->ctrl.w = CMD_GEN_NOISE; ///< CMD_GEN_NOISE 之后硬件一直处于busy状态

    return EXIT_SUCCESS;
}

/**
 * @brief TRNG interrupt handler
 * @param [in] arg Driver data pointer
 */
static void trngIsr(void *arg)
{
    TrngDrvData_s *drv = (TrngDrvData_s *)arg;
    TrngReg_s *reg = NULL;
    U32 istat = 0;

    if (drv == NULL || drv->sbrCfg.regAddr == NULL) {
        return;
    }

    reg = (TrngReg_s *)drv->sbrCfg.regAddr;
    istat = reg->istat.w;
    if (istat == 0) {
        return;
    }

    reg->istat.w = istat;  ///< Clear interrupt status
}

/**
 * @brief Configure TRNG hardware
 * @param [in] devId Device ID
 * @param [in] drv Driver data
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 hwConfig(DevList_e devId, TrngDrvData_s *drv)
{
    TrngReg_s *reg = (TrngReg_s *)drv->sbrCfg.regAddr;
    S32 ret = EXIT_SUCCESS;

    ret = peripsReset(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: reset failed\n", __func__);
        return -EIO;
    }

    if (waitIdle(reg, TRNG_WAIT_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: wait idle timeout\n", __func__);
        return -EIO;
    }

    reg->trngMux.b.rdseedPriHigh = 0;
    reg->trngMux.b.rdSeedSelect = 0;

    reg->smode.b.secureEn = 0;
    reg->smode.b.nonce = 0;
    reg->smode.b.bgEnable = TRNG_ENTROPY_ALL;
    reg->smode.b.xorAll = 1;

    reg->seedAlmostEmptyLevel = DEFAULT_SEED_EMPTY_LEVEL;
    reg->randAlmostEmptyLevel = DEFAULT_RAND_EMPTY_LEVEL;
    reg->randCountThreshold = DEFAULT_RAND_THRESHOLD;

    if (waitIdle(reg, TRNG_WAIT_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: wait idle timeout\n", __func__);
        return -EIO;
    }

    reg->mode.b.secAlg = 1;  ///< SM4 algorithm
    reg->mode.b.katVec = 0;
    reg->mode.b.katSel = 0;

    reg->ctrl.w = CMD_RUN_KAT;
    if (waitIdle(reg, TRNG_WAIT_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: KAT timeout\n", __func__);
        return -EIO;
    }

    ret = zeroize(reg);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    return EXIT_SUCCESS;
}

S32 trngInit(DevList_e devId)
{
    TrngDrvData_s *drv = NULL;
    TrngReg_s *reg = NULL;
    S32 ret = EXIT_SUCCESS;

    if (!isDrvMatch(devId, DRV_ID_TH_TRNG)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, TRNG_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (isDrvInit(devId)) {
        ret = -EBUSY;
        goto unlock;
    }

    drv = (TrngDrvData_s *)calloc(1, sizeof(TrngDrvData_s));
    if (drv == NULL) {
        LOGE("%s: alloc failed\n", __func__);
        ret = -ENOMEM;
        goto unlock;
    }

    if (devSbrRead(devId, &drv->sbrCfg, 0, sizeof(SbrTrngCfg_s)) != sizeof(SbrTrngCfg_s) ||
        drv->sbrCfg.regAddr == NULL) {
        LOGE("%s: read SBR failed\n", __func__);
        ret = -EIO;
        goto cleanup;
    }

    reg = (TrngReg_s *)drv->sbrCfg.regAddr;

    ret = ospInterruptHandlerInstall(drv->sbrCfg.irqNo, "trng",
                                         OSP_INTERRUPT_UNIQUE,
                                         (OspInterruptHandler)trngIsr, drv);
    if (ret != OSP_SUCCESSFUL) {
        LOGE("%s: IRQ install failed\n", __func__);
        ret = -EIO;
        goto cleanup;
    }

    ospInterruptVectorEnable(drv->sbrCfg.irqNo);

    ret = hwConfig(devId, drv);
    if (ret != EXIT_SUCCESS) {
        goto cleanup;
    }

    if (drvInstall(devId, drv) != EXIT_SUCCESS) {
        LOGE("%s: driver install failed\n", __func__);
        ret = -EIO;
        goto cleanup;
    }

    reg->ie.w |= TRNG_IE_ALL;

    ret = EXIT_SUCCESS;
    goto unlock;

cleanup:
    if (drv != NULL && drv->sbrCfg.irqNo != 0) {
        ospInterruptVectorDisable(drv->sbrCfg.irqNo);
        ospInterruptHandlerRemove(drv->sbrCfg.irqNo, (OspInterruptHandler)trngIsr, drv);
    }
    if (drv != NULL) {
        free(drv);
    }

unlock:
    devUnlockByDriver(devId);
    return ret;
}

S32 trngDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    TrngDrvData_s *drv = NULL;
    TrngReg_s *reg = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_TH_TRNG, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    reg = (TrngReg_s *)drv->sbrCfg.regAddr;
    reg->ie.w &= ~TRNG_IE_ALL;

    if (drv->sbrCfg.irqNo != 0) {
        ospInterruptVectorDisable(drv->sbrCfg.irqNo);
        ospInterruptHandlerRemove(drv->sbrCfg.irqNo, (OspInterruptHandler)trngIsr, drv);
    }

    peripsReset(devId);
    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("%s: uninstall failed\n", __func__);
        ret = -EIO;
    }

    funcRunEndHelper(devId);
    return ret;
}

S32 trngRandGen(DevList_e devId, U32 *randomOut, U32 len)
{
    S32 ret = EXIT_SUCCESS;
    TrngDrvData_s *drv = NULL;
    TrngReg_s *reg = NULL;

    if (randomOut == NULL || len == 0) {
        return -EINVAL;
    }

    if (len > TRNG_MAX_RANDOM_WORDS) {
        LOGE("%s: request too large %u (max %u)\n", __func__, len, TRNG_MAX_RANDOM_WORDS);
        return -EINVAL;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_TH_TRNG, (void **)&drv);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    reg = (TrngReg_s *)drv->sbrCfg.regAddr;
    if (reg == NULL) {
        LOGE("%s: reg addr NULL\n", __func__);
        ret = -EIO;
        goto exit;
    }

    while (len > 0) {
        if (waitRandReady(reg, TRNG_WAIT_TIMEOUT_MS) != EXIT_SUCCESS) {
            LOGD("%s: rand empty timeout\n", __func__);
            ret = -EIO;
            goto exit;
        }

        *randomOut++ = reg->rdrandValue;
        len--;
    }

exit:
    funcRunEndHelper(devId);
    return ret;
}
