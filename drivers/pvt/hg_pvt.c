/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file hg_pvt.c
 * @author taohb@starsmicrosystem.com
 * @date 2024.08.27
 * @brief hg pvt driver.
 */

#include "drv_hg_pvt.h"
#include "drv_hg_pvt_api.h"
#include "udelay.h"
#include "log_msg.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "osp_interrupt.h"
#include "sbr_api.h"
#include "bsp_drv_id.h"

/* base addr is PVT_TMON_THRESHOLD_IRQ_ADDR_CHANNEL_X */
#define DTS_CHANNEL_TCON_THRESHOLD_OFFSET (0x514)

static Bool gPvtInitdone = false;

static S32 pvtDevCfgGet(DevList_e devId, SbrPvtChCfg_s *pvtSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (pvtSbrCfg == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    if (devSbrRead(devId, pvtSbrCfg, 0, sizeof(SbrPvtChCfg_s)) != sizeof(SbrPvtChCfg_s)) {
        ret = -EXIT_FAILURE;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("pvt: SBR dump - regAddr:%p, irqAddr:%p, tmonThresholdIrqAddr:%p, irqNo:%u, irqPrio:%u\r\n",
         pvtSbrCfg->regAddr, pvtSbrCfg->irqAddr, pvtSbrCfg->tmonThresholdIrqAddr,
         pvtSbrCfg->irqNo, pvtSbrCfg->irqPrio);
    LOGI("pvt: SBR dump - tmon:%u, dts:%u, dtsTempPoints:%u, dtsVolPoints:%u, dtsCfgLatency:%u\r\n",
         pvtSbrCfg->tmon, pvtSbrCfg->dts, pvtSbrCfg->dtsTempPoints, pvtSbrCfg->dtsVolPoints, pvtSbrCfg->dtsCfgLatency);
    LOGI("pvt: SBR dump - tmonTempTh:%u, dtsTempTh:%u\r\n",
         pvtSbrCfg->tmonTempTh, pvtSbrCfg->dtsTempTh);
#endif

    if (pvtSbrCfg->regAddr == NULL || pvtSbrCfg->irqAddr == NULL ||
        pvtSbrCfg->tmonThresholdIrqAddr == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

out:
    return ret;
}

static U32 tmonTempValueRegOffset[] = {
    0xb34,
    0xb38,
    0xb6c,
    0xb70,
    0xbc8,
};

static void inline dtsDummyRead(PvtDrvData_s *pPvtDrvData)
{
    udelay(500);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x16c);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x170);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x12c);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x130);
}

static S32 pvtTotalChannelGet(U32 *pTotalChannels)
{
    S32 ret = EXIT_SUCCESS;
    pvtCfg_s pvtCfg;

    if (pTotalChannels == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (devSbrRead(DEVICE_PVT, &pvtCfg, 0, sizeof(pvtCfg_s)) != sizeof(pvtCfg_s)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    *pTotalChannels = pvtCfg.totalCh;
    return EXIT_SUCCESS;

exit:
    return ret;
}

/**
 * @brief get pvt channel number.
 * @return pvt channel number.
 */
U32 pvtGetChannelNum(void)
{
    S32 ret = EXIT_SUCCESS;
    U32 totalChannels;

    ret = pvtTotalChannelGet(&totalChannels);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }
    return totalChannels;
}

/**
 * @brief get tmon temperature testPoint number for the specail channel.
 * @param [in] channel channel id of pvt.
 * @return temperature testPoint number for the specail channel.
 */
U32 tmonGetTempTestpointNum(U32 channel)
{
    U32 testPoints = 0;
    S32 ret = EXIT_SUCCESS;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGD("[pvt][%d] get drv data fail\r\n", __LINE__, channel);
        goto exit;
    }

    if (!pPvtDrvData || !pPvtDrvData->tmonInfo.channelExist) {
        LOGD("[pvt][%d] tmon not in this channel\r\n", __LINE__, channel);
        goto exit;
    }

    testPoints = pPvtDrvData->tmonInfo.testPoints;
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return testPoints;
}

/**
 * @brief get dts temperature testPoint number for the specail channel.
 * @param [in] channel channel id of pvt.
 * @return temperature testPoint number for the specail channel.
 */
U32 dtsGetTempTestpointNum(U32 channel)
{
    U32 testPoints = 0;
    S32 ret = EXIT_SUCCESS;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGD("[pvt][%d] get drv data fail\r\n", __LINE__, channel);
        goto exit;
    }

    if (!pPvtDrvData->dtsInfo.channelExist) {
        LOGD("[pvt][%d] dts not in this channel\r\n", __LINE__, channel);
        goto exit;
    }
    testPoints = pPvtDrvData->dtsInfo.tempTestPoints;
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return testPoints;
}

static S32 dtsTemperatureParamConfig(U32 channel)
{
    S32 ret = EXIT_SUCCESS;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        ret = -EXIT_FAILURE;
        LOGE("%s(): channel[%d] has no drvdata\r\n", __func__, channel);
        goto exit;
    }

    if (pPvtDrvData->dtsInfo.channelExist) {
        /* 8.config latency */
        reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0b0, pPvtDrvData->dtsInfo.configLatency);
        /* 9.thempack count window */
        reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0xa24, 0x01003301);
        reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0xa2c, 0x0a07a120);
        /* 10.release resetn */
        reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x50c, 0x01);
        /* 11.tcon_cntl enable and select mode */
        reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x510, 0x21);
        reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x514, 0x3f);
    } else {
        LOGE("%s(): channel %d has no dtsinfo\r\n", __func__, channel);
        goto exit;
    }
exit:
    return ret;
}

/**
 * @brief get dts voltage testPoint number for the specail channel.
 * @param [in] channel channel id of pvt.
 * @return voltage testPoint number for the specail channel.
 */
U32 dtsGetVoltTestpointNum(U32 channel)
{
    U32 testPoints = 0;
    S32 ret = EXIT_SUCCESS;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGD("[pvt][%d] get drv data fail\r\n", __LINE__, channel);
        goto exit;
    }

    if (!pPvtDrvData->dtsInfo.channelExist) {
        LOGD("[pvt][%d] dts not in this channel\r\n", __LINE__, channel);
        goto exit;
    }

    testPoints = pPvtDrvData->dtsInfo.volTestPoints;
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return testPoints;
}

/**
 * @brief Interface for get current processor temperature by tmon.
 * @param [in] channel channel id to get temperature.
 * @param [in] testPoint test point of current channel.
 * @param [out] temperature：Pointer to storage processor temperature value.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 tmonGetTemperature(U32 channel, U32 testPoint, float *temperature)
{
    S32 ret = EXIT_SUCCESS;
    TmonTempValueReg_u *tempReg;
    DevList_e devId = DEVICE_PVT_CH0 + channel;

    /* Check driver match */
    if (!isDrvMatch(devId, DRV_ID_HG_PVT)) {
        return -EINVAL;
    }
    U32 testPointMax = tmonGetTempTestpointNum(channel);
    PvtDrvData_s *pPvtDrvData = NULL;

    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGD("[pvt][%d] get drv data fail\r\n", __LINE__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (!pPvtDrvData->tmonInfo.channelExist) {
        LOGD("[pvt][%d] tmon not in this channel\r\n", __LINE__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (!temperature) {
        LOGE("temperature pointer null value\r\n");
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (testPoint >= testPointMax) {
        LOGE("%s(): testPoint %d should be [0, %d]\r\n", __func__, testPoint,
            testPointMax - 1);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    tempReg =
        (TmonTempValueReg_u *)((U32)pPvtDrvData->sbrCfg.regAddr + tmonTempValueRegOffset[testPoint]);
    if (!tempReg->fields.valid) {
        LOGE("%s(): tmon testPoint %d temp value not ready\r\n", __func__,
            testPoint);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    *temperature = 0.125f * tempReg->fields.realTempValue - 49;
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
}

/**
 * @brief Interface for get current processor temperature by dts.
 * @param [in] channel channel id to get temperature.
 * @param [in] testPoint test point of current channel.
 * @param [out] temperature：Pointer to storage processor temperature value.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsGetTemperature(U32 channel, U32 testPoint, float *temperature)
{
    S32 ret = EXIT_SUCCESS;
    S32 tempReg;
    U32 testPointMax = dtsGetTempTestpointNum(channel);
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGE("%s(): channel %d get drv data fail\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (!pPvtDrvData->dtsInfo.channelExist) {
        LOGE("%s(): channel %d invalid\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (!temperature) {
        LOGE("%s(): temperature null pointer\r\n", __func__);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    if (testPoint >= testPointMax) {
        LOGE("%s(): testPoint %d should be [0, %d]\r\n", __func__, testPoint,
            testPointMax - 1);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    udelay(3000);
    tempReg = reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x620 + testPoint * 4);
    *temperature = tempReg * 1.0f / 65536;
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
}

/**
 * @brief get voltage origin value from dts. This is a test function.
 * @param [in] channel channel id of pvt.
 * @param [out] valueArray array, to storage volatage value, which size is testPoints.
 * @param [in] testPoints testPoint number for valueArray which should be less than
 *      current channel's max volatage testPoints.
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 */
S32 dtsVoltRegValueRead(U32 channel, U32 *valueArray,
    U32 testPoints)
{
    S32 ret = EXIT_SUCCESS;
    U32 i;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGE("%s(): channel %d get drv data fail\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (!pPvtDrvData->dtsInfo.channelExist) {
        LOGE("%s(): channel %d invalid\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (!valueArray) {
        LOGE("pvt valuearray null pointer\r\n");
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (testPoints > pPvtDrvData->dtsInfo.volTestPoints) {
        LOGE("%s(): testPoints %d exceed %d\r\n", __func__, testPoints,
            pPvtDrvData->dtsInfo.volTestPoints);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    /* 8.config latency and cntl first line */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0b0, pPvtDrvData->dtsInfo.configLatency);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0b8, 0x08);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0c0, 0x09);
    /* 9.first line first avfs read */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3030);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac);
    dtsDummyRead(pPvtDrvData);
    /* 10.first line first avfs write psm enable clkdiv and avfs read psm parameter */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ec, 0x01000300);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0f0, 0x00);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3020);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3030);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac);
    dtsDummyRead(pPvtDrvData);
    /* 11.first line first avfs write psm enable clkdiv and avfs read psm parameter */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ec, 0x01000200);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0f0, 0x00);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3020);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3030);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac);
    dtsDummyRead(pPvtDrvData);
    /* 12.first line first avfs write psm enable clkdiv wdr_override and avfs read psm parameter */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ec, 0x01000200);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0f0, 0x02000000);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3020);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3030);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac);
    dtsDummyRead(pPvtDrvData);

    /*
     * 13.first line first avfs write psm enable clkdiv wdr_override maskskipphase
     * and avfs read psm parameter
     */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ec, 0x01001201);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0f0, 0x03000000);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3020);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3030);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac);
    dtsDummyRead(pPvtDrvData);
    /*
     * 14.first line first avfs write psm rst psm enable clkdiv wdr_override maskskipphase
     * and avfs read psm parameter
     */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ec, 0x01001200);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0f0, 0x03000000);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x3020);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac, 0x30b0);
    reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x0ac);
    udelay(500);
    for (i = 0; i < testPoints; i++) {
        switch (i) {
        case 0:
            valueArray[i] = reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x16c);
            break;
        case 1:
            valueArray[i] = reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x170);
            break;
        case 2:
            valueArray[i] = reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x12c);
            break;
        case 3:
            valueArray[i] = reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x130);
            break;
        default:
            break;
        }
    }

exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
}

static void hgPvtIsr(void *arg)
{
    PvtDrvData_s *pPvtDrvData = arg;
    U32 channel = pPvtDrvData->channel;
    HgPvtChSoftIrqReg_u *pPvtRegs = (HgPvtChSoftIrqReg_u *)pPvtDrvData->sbrCfg.irqAddr;
    ChSoftIrqReg_u intStatus = { 0 };
    Bool dtsTempAlarm = false;
    U32 tempReg;
    DtsThresholdReg_u *pThresholdReg;

    if(pPvtDrvData == NULL) {
        LOGE("%s(): pvtDrvData null pointer\r\n", __func__);
        return;
    }

    intStatus.tmon.dword = pPvtRegs->irqStatusReg.tmon.dword;

    if (pPvtDrvData->tmonInfo.channelExist) {
        if (pPvtRegs->irqStatusReg.tmon.fields.dts) {
            dtsTempAlarm = true;
        }
    } else if (pPvtDrvData->dtsInfo.channelExist) {
        if (pPvtRegs->irqStatusReg.dts.fields.dts) {
            dtsTempAlarm = true;
        }
    }
    if (dtsTempAlarm) {
        tempReg = reg32Read((U32)pPvtDrvData->sbrCfg.regAddr + 0x620);
        tempReg = tempReg; // fix warning
        if (pPvtDrvData->dtsTempAlarmIsrCb) {
            pPvtDrvData->dtsTempAlarmIsrCb(channel);
        } else {
            if (pPvtDrvData->dtsInfo.channelExist) {
                pThresholdReg = (DtsThresholdReg_u *)((U32)pPvtDrvData->sbrCfg.regAddr + DTS_CHANNEL_TCON_THRESHOLD_OFFSET);
                pThresholdReg->fields.thremTripLimit = DTS_TEMPERATURE_THRESHOLD_MAX;
            } else {
                // LOGD("%s(): channel %d has no dtsinfo\r\n", __func__, channel);
            }
        }
    }
    /* clear all */
    pPvtRegs->irqClearReg.tmon.dword = intStatus.tmon.dword;
    return;
}

S32 dtsTempThresholdSet(U32 channel, S16 tempThreshold)
{
    S32 ret = EXIT_SUCCESS;
    DtsThresholdReg_u *thresholdReg;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGE("%s(): channel %d has no drvdata\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (pPvtDrvData->dtsInfo.channelExist) {
        thresholdReg = (DtsThresholdReg_u *)((U32)pPvtDrvData->sbrCfg.regAddr + DTS_CHANNEL_TCON_THRESHOLD_OFFSET);
        thresholdReg->fields.thremTripLimit = tempThreshold;
    } else {
        LOGD("%s(): channel %d has no dtsinfo\r\n", __func__, channel);
        goto exit;
    }
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
}

/**
 * @brief register dts temperature alarm callback which is in isr.
 * @param [in] channel channel id.
 * @param [in] dtsTempratureAlarmIsrCallback isr callback.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsTemAlarmCallbackRegister(U32 channel,
    dtsTempAlarmIsrCb isrCb)
{
    S32 ret = EXIT_SUCCESS;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGE("%s(): channel %d invalid\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (!pPvtDrvData->dtsInfo.channelExist) {
        LOGE("%s(): channel %d has no dtsinfo\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    pPvtDrvData->dtsTempAlarmIsrCb = isrCb;
exit:
    return ret;
}
/**
 * @brief set dts temperature threshold.
 * @param [in] channel channel id.
 * @param [in] tempThreshold unit is Centigrade.
 * @param [in] dtsTempratureAlarmIsrCallback isr callback.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsTempAlarmThresholdSet(U32 channel, int16_t tempThreshold,
    dtsTempAlarmIsrCb isrCb)
{
    S32 ret = EXIT_SUCCESS;
    float temperature;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }

    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGE("%s(): channel %d invalid\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (EXIT_SUCCESS != dtsTemAlarmCallbackRegister(channel, isrCb)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    dtsGetTemperature(channel, 0, &temperature);
    if (EXIT_SUCCESS != dtsTempThresholdSet(channel, tempThreshold)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
    dtsGetTemperature(channel, 0, &temperature);
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
}

/**
 * @brief enable pvt channel interrupt.
 * @param [in] channel channel id.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 pvtInterruptEnable(U32 channel)
{
    S32 ret = EXIT_SUCCESS;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId = DEVICE_PVT_CH0 + channel;

    /* Check driver match */
    if (!isDrvMatch(devId, DRV_ID_HG_PVT)) {
        return -EINVAL;
    }
    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGE("%s(): channel %d invalid\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    if (!pPvtDrvData->sbrCfg.irqNo || !pPvtDrvData->sbrCfg.irqPrio) {
        /* interrupt info is empty */
        goto exit;
    }
    ospInterruptVectorEnable(pPvtDrvData->sbrCfg.irqNo);
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
}

/**
 * @brief disable pvt channel interrupt.
 * @param [in] channel channel id.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 pvtInterruptDisable(U32 channel)
{
    S32 ret = EXIT_SUCCESS;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId = DEVICE_PVT_CH0 + channel;

    /* Check driver match */
    if (!isDrvMatch(devId, DRV_ID_HG_PVT)) {
        return -EINVAL;
    }

    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGE("%s(): channel %d invalid\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    if (!pPvtDrvData->sbrCfg.irqNo || !pPvtDrvData->sbrCfg.irqPrio) {
        /* interrupt info is empty */
        goto exit;
    }
    ospInterruptVectorDisable(pPvtDrvData->sbrCfg.irqNo); /* disable interrupt by default */
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
}

/**
 * @brief import temperature calibration parameter of dts.
 * @param [in] channel channel id to get temperature.
 * @param [in] caliParam dts temperature's calibration paramter, it's usally saved in efuse by ate.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsTempCaliParamImport(U32 channel,
    DtsChTempCaliParam_s *caliParam)
{
    S32 ret = EXIT_SUCCESS;
    U32 i, testPoints;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    if (EXIT_SUCCESS != devLockByDriver(devId, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGE("%s(): channel %d invalid\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    if (!caliParam) {
        LOGE("%s(): caliParam NULL pointer\r\n", __func__);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    if (!pPvtDrvData->dtsInfo.channelExist) {
        LOGI("[pvt][%d] no dts in channel[%d]\r\n", __LINE__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    testPoints = dtsGetVoltTestpointNum(channel);
    if (testPoints > caliParam->testPointNum) {
        testPoints = caliParam->testPointNum;
    }
    /* Step 1: config tcon_param_A,tcon_param_B,tcon_param_C,tcon_param_D for channel */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x518, 0x4628);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x51c, 0x3316);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x520, 0x1ccc);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x524, 0x31f);
    /* Step 2: config tcon_param_M and tcon_param_N for channel */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x528, caliParam->paramM);
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x52c, caliParam->paramN);
    /* Step 3: config tcon_param_X for channel */
    reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x530, 0xa333);
    /* Step 4: config tcon_param_K for testPoints */
    for (i = 0; i < testPoints; i++) {
        reg32Write((U32)pPvtDrvData->sbrCfg.regAddr + 0x534 + i * 4, caliParam->paramK[i]);
    }
    /* Step 5: reconfig dts temperature */
    if (EXIT_SUCCESS != dtsTemperatureParamConfig(channel)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(devId)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
}



S32 tmonThresholdSet(U32 channel, U8 thresholdRaw)
{
    S32 ret = EXIT_SUCCESS;
    ChSoftIrqReg_u *thresholdEnReg;
    TmonThresholdReg_u *thresholdReg, value;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    ret = getDevDriver(devId, (void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if(pPvtDrvData == NULL) {
        LOGE("%s(): channel %d invalid\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (pPvtDrvData->tmonInfo.channelExist) {
        thresholdEnReg = (ChSoftIrqReg_u *)pPvtDrvData->sbrCfg.tmonThresholdIrqAddr;
        thresholdReg = (TmonThresholdReg_u *)((U32)pPvtDrvData->sbrCfg.tmonThresholdIrqAddr + 0x08);
        thresholdEnReg->dts.fields.tmonThresholdEn = 0x2; /* enable threshold modify */
        value.dword = 0; /* other bit must be clear to 0 */
        value.fields.tmonThreshold = thresholdRaw;
        thresholdReg->dword = value.dword;
        thresholdEnReg->dts.fields.tmonThresholdEn = 0x00; /* disable threshold modify */
    } else {
        LOGD("%s(): channel %d has no tmoninfo\r\n", __func__, channel);
        goto exit;
    }
exit:
    return ret;
}

static S32 pvtInterruptInit(U32 channel)
{
    S32 ret = EXIT_SUCCESS;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;

    devId = DEVICE_PVT_CH0 + channel;
    ret = getDevDriver(devId,(void **)&pPvtDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("[pvt][%d] get driver failed\r\n", __LINE__);
        goto exit;
    }

    if (!pPvtDrvData) {
        LOGE("%s(): channel %d invalid\r\n", __func__, channel);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (pPvtDrvData->sbrCfg.irqNo && pPvtDrvData->sbrCfg.irqPrio) {
        ospInterruptHandlerInstall(pPvtDrvData->sbrCfg.irqNo, "hg pvt",
            OSP_INTERRUPT_UNIQUE, (void (*)(void*))hgPvtIsr, pPvtDrvData);
        ospInterruptSetPriority(pPvtDrvData->sbrCfg.irqNo, pPvtDrvData->sbrCfg.irqPrio);
        ospInterruptVectorDisable(pPvtDrvData->sbrCfg.irqNo); /* disable interrupt by default */
    } else {
        LOGE("%s(): channel %d has no interrupt\r\n", __func__, channel);
        goto exit;
    }

exit:
    return ret;
}

/**
 * @brief hg pvt module init.
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 */
S32 pvtChannelInit(U32 channel)
{
    S32 ret = EXIT_SUCCESS;
    Device_s *pDevice = NULL;
    DevList_e devId = DEVICE_PVT_CH0 + channel;
    PvtDrvData_s *pPvtDrvData = NULL;

    if (isDrvInit(devId)) {
        LOGE("%s(): has already implement driver\r\n", __func__);
        ret = -EBUSY;
        goto exit;
    }

    /* Check driver match */
    if (!isDrvMatch(devId, DRV_ID_HG_PVT)) {
        return -EINVAL;
    }

    pDevice = getDevice(devId);
    if (pDevice == NULL) {
        LOGE("%s(): get device fail\r\n", __func__);
        ret = -EXIT_FAILURE;
        goto exit0;
    }

    ///< 申请驱动私有数据内存并获取设备配置
    pPvtDrvData = (PvtDrvData_s*)calloc(1, sizeof(PvtDrvData_s));
    if (pPvtDrvData == NULL) {
        LOGE("%s() alloc pPvtDrvData fail\r\n", __func__);
        ret = -EXIT_FAILURE;
        goto exit0;
    }

    if (pvtDevCfgGet(devId, &pPvtDrvData->sbrCfg) != EXIT_SUCCESS) { ///< 获取设备配置
        LOGE("%s() pvtDevCfgGet fail\r\n", __func__);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ///< 从SBR配置映射到驱动数据
    pPvtDrvData->channel = channel;

    ///< 配置TMON信息
    if (pPvtDrvData->sbrCfg.tmon) {
        pPvtDrvData->tmonInfo.channelExist = true;
        pPvtDrvData->tmonInfo.testPoints = pPvtDrvData->sbrCfg.dtsTempPoints;
        pPvtDrvData->tmonInfo.defalutTempTh = pPvtDrvData->sbrCfg.tmonTempTh;
    } else {
        pPvtDrvData->tmonInfo.channelExist = false;
    }

    ///< 配置DTS信息
    if (pPvtDrvData->sbrCfg.dts) {
        pPvtDrvData->dtsInfo.channelExist = true;
        pPvtDrvData->dtsInfo.tempTestPoints = pPvtDrvData->sbrCfg.dtsTempPoints;
        pPvtDrvData->dtsInfo.volTestPoints = pPvtDrvData->sbrCfg.dtsVolPoints;
        pPvtDrvData->dtsInfo.configLatency = pPvtDrvData->sbrCfg.dtsCfgLatency;
        pPvtDrvData->dtsInfo.defaultTempTh = pPvtDrvData->sbrCfg.dtsTempTh;
    } else {
        pPvtDrvData->dtsInfo.channelExist = false;
    }

    ///< 安装设备驱动
    if (drvInstall(devId, (void*)pPvtDrvData) != EXIT_SUCCESS) {
        LOGE("%s() drvInstall fail\r\n", __func__);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dtsTemperatureParamConfig(channel) != EXIT_SUCCESS) {
        LOGE("channel[%d] dtsTemperatureParamConfig fail\r\n", channel);
        goto exit;
    }

    if (tmonThresholdSet(channel, pPvtDrvData->tmonInfo.defalutTempTh) != EXIT_SUCCESS) {
        LOGE("channel[%d] tmonThresholdSet fail\r\n", channel);
        goto exit;
    }

    if (dtsTempThresholdSet(channel, pPvtDrvData->dtsInfo.defaultTempTh) != EXIT_SUCCESS) {
        LOGE("channel[%d] dtsTempThresholdSet fail\r\n", channel);
        goto exit;
    }

    if (pvtInterruptInit(channel) != EXIT_SUCCESS) {
        LOGE("channel[%d] pvtInterruptInit fail\r\n", channel);
        goto exit;
    }

exit0:
    return ret;
exit:
    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("channel %d drv uninstalll failed\r\n", channel);
    }
    return ret;
}

/**
 * @brief pvt init.
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 */
S32 pvtInit(void)
{
    S32 ret = EXIT_SUCCESS;
    U32 channel, totalChannels;

    if (EXIT_SUCCESS != devLockByDriver(DEVICE_PVT, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }
    if (gPvtInitdone == true) {
        LOGI("pvt has init already\r\n");
        goto exit;
    }

    ret = pvtTotalChannelGet(&totalChannels);
    if (ret != EXIT_SUCCESS) {
        LOGE("pvt get channel nums fail\r\n");
        goto exit;
    }
    LOGD("[%s][%d] totalchannels:%d\r\n", __func__, __LINE__,totalChannels);

    for (channel = 0; channel < totalChannels; channel++) {
        ret = pvtChannelInit(channel);
        if (ret != EXIT_SUCCESS) {
            LOGE("pvtChannel[%d] init fail\r\n", channel);
            goto exit;
        }
    }
    gPvtInitdone = true;
    if (EXIT_SUCCESS != devUnlockByDriver(DEVICE_PVT)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
exit:
    if (EXIT_SUCCESS != devUnlockByDriver(DEVICE_PVT)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return -EXIT_FAILURE;
}

/**
 * @brief pvt deinitialization
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 */
S32 pvtDeInit(void)
{
    S32 ret = EXIT_SUCCESS;
    PvtDrvData_s *pPvtDrvData = NULL;
    DevList_e devId;
    U32 totalChannels;

    if (EXIT_SUCCESS != devLockByDriver(DEVICE_PVT, 1000)) {
        LOGE("pvt get lock failed, %s, %d\r\n", __func__, __LINE__);
        goto exit;
    }

    if (gPvtInitdone == false) {
        LOGI("pvt has deinitialization already\r\n");
        goto exit;
    }

    ret = pvtTotalChannelGet(&totalChannels);
    if (ret != EXIT_SUCCESS) {
        LOGE("pvt get channel nums fail\r\n");
        goto exit;
    }
    LOGD("[%s][%d] totalchannels:%d\r\n", __func__, __LINE__,totalChannels);

    for (U8 i = 0; i < totalChannels; i++) {
        devId = DEVICE_PVT_CH0 + i;
        ret = getDevDriver(devId,(void **)&pPvtDrvData);
        if (ret != EXIT_SUCCESS) {
            ret = -EXIT_FAILURE;
            goto exit;
        }
        if (pPvtDrvData != NULL) {
            if (pPvtDrvData->sbrCfg.irqNo != 0) {
                ospInterruptHandlerRemove(pPvtDrvData->sbrCfg.irqNo, (void (*)(void*))hgPvtIsr, pPvtDrvData);
                ospInterruptVectorUninit(pPvtDrvData->sbrCfg.irqNo);
            }
            if (drvUninstall(devId) != EXIT_SUCCESS) {
                LOGE("%s() drvUnistall fail\r\n", __func__);
                ret = -EXIT_FAILURE;
                goto exit;
            }
        }
    }
    gPvtInitdone = false;
    LOGD("pvt deinit\r\n");
    if (EXIT_SUCCESS != devUnlockByDriver(DEVICE_PVT)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return ret;
exit:
    LOGE("pvt deinit fail\r\n");
    if (EXIT_SUCCESS != devUnlockByDriver(DEVICE_PVT)) {
        LOGE("pvt unlock failed, %s, %d\r\n", __func__, __LINE__);
    }
    return -EXIT_FAILURE;
}