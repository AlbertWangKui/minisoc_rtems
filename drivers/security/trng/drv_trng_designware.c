/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_trng.c
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/05/18
 * @brief
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/05/18   yangzhl3        porting from rtems_minisoc
 * 2025/06/05   yangzhl3        add JSON parse
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common_defines.h"
#include "bsp_api.h"
#include "udelay.h"
#include "log_msg.h"
#include "drv_trng.h"

#define TRNG_WAIT_FOR_IDLE_TIMEOUT      (100)

static int32_t trngWaitIdle(TrngReg_s *baseaddr)
{
    int32_t ret = EXIT_SUCCESS;
    uint32_t timeout = TRNG_WAIT_FOR_IDLE_TIMEOUT;
    TrngReg_s *reg = baseaddr;

    while (reg->stat.fields.busy) {
        if (!timeout) {
            ret = -EXIT_FAILURE;
            break;
        }
        timeout--;
        udelay(1000);
    }

    return ret;
}

static int32_t trngWaitDone(TrngReg_s *baseaddr)
{
    int32_t ret = EXIT_SUCCESS;
    uint32_t timeout = TRNG_WAIT_FOR_IDLE_TIMEOUT;
    TrngReg_s *reg = baseaddr;

    while (!(reg->istat.fields.done)) {
        if (!timeout) {
            ret = -EXIT_FAILURE;
            break;
        }
        timeout--;
        udelay(1000);
    }

    return ret;
}

static S32 trngDevCfgGet(DevList_e devId, SbrTrngCfg_s *trngSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (trngSbrCfg == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    if (devSbrRead(devId, trngSbrCfg, 0, sizeof(SbrTrngCfg_s)) != sizeof(SbrTrngCfg_s)) {
        ret = -EXIT_FAILURE;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGE("trng: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, reserved:0x%08x\r\n",
         trngSbrCfg->regAddr, trngSbrCfg->irqNo, trngSbrCfg->irqPrio, trngSbrCfg->reserved);
#endif

    if (trngSbrCfg->regAddr == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

out:
    return ret;
}

S32 trngInit(DevList_e trngId)
{
    S32 ret = EXIT_SUCCESS;
    TrngDrvData_s *pTrngDrvData = NULL;
    TrngReg_s *reg = NULL;

    if (isDrvInit(trngId)) {
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(trngId, DRV_ID_DW_TRNG)) {
        return -EINVAL;
    }

    if (devLockByDriver(trngId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    pTrngDrvData = (TrngDrvData_s*)calloc(1, sizeof(TrngDrvData_s));
    if (pTrngDrvData == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    if (trngDevCfgGet(trngId, &pTrngDrvData->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freemem;
    }

    peripsReset(trngId);

    reg = (TrngReg_s *)pTrngDrvData->sbrCfg.regAddr;
    /* step1: wait for idle */
    ret = trngWaitIdle(reg);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto freemem;
    }
    /* step2.2: generate noise seed */
    reg->ctrl.fields.cmd = CMD_TYPE_GEN_NOISE;
    if (trngWaitDone(reg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freemem;
    }
    reg->istat.fields.done = 1;

    /* step3.2: create state at first time */
    reg->ctrl.fields.cmd = CMD_TYPE_CREATE_STATE;
    if (trngWaitDone(reg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freemem;
    }
    reg->istat.fields.done = 1;

    /* step4: install driver */
    if (drvInstall(trngId, (void*)pTrngDrvData) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto freemem;
    }

    ret = EXIT_SUCCESS;

freemem:
    if (ret != EXIT_SUCCESS && pTrngDrvData != NULL) {
        free(pTrngDrvData);
    }
exit:
    devUnlockByDriver(trngId);
    return ret;
}

S32 trngGetRandom(DevList_e trngId, U32 *randomOut, U32 lenDw)
{
    S32 ret = EXIT_SUCCESS;
    U32 copy_len = 0;
    TrngDrvData_s *pTrngDrvData = NULL;
    TrngReg_s *reg = NULL;

    if (randomOut == NULL || lenDw == 0) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(trngId, DRV_ID_DW_TRNG)) {
        return -EINVAL;
    }

    if (getDevDriver(trngId,(void **)&pTrngDrvData) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (pTrngDrvData == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    reg = (TrngReg_s *)(pTrngDrvData->sbrCfg.regAddr);

    /* lock */
    if (devLockByDriver(trngId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    while (lenDw) {
        reg->ctrl.fields.cmd = CMD_TYPE_GEN_RANDOM;
        if (trngWaitDone(reg) != EXIT_SUCCESS) {
            ret = -EIO;
            goto unlock;
        }

        reg->istat.fields.done = 1; /* clear done */
        LOGD("trng generate random: %#x %#x %#x %#x\r\n",
            reg->rand[0], reg->rand[1], reg->rand[2], reg->rand[3]);

        copy_len = lenDw > 4 ? 4 : lenDw;
        dwMemCpy((volatile U32 *)randomOut, (volatile U32 *)&reg->rand, copy_len);
        randomOut += copy_len;
        lenDw -= copy_len;
    }

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(trngId);

exit:
    return ret;
}
