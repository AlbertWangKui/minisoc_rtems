/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_pwm.c
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025/10/22
 * @brief  pwm driver for minisoc
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/10/22   yangkl         the first version
 *
 */

#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "bsp_drv_id.h"
#include "osp_interrupt.h"
#include "log_msg.h"
#include "drv_pwm_api.h"
#include "drv_pwm.h"
#include "sbr_api.h"

static S32 getPwmDevConfig(DevList_e devId, SbrPwmCfg_s * pwmSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (pwmSbrCfg == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (devSbrRead(devId, pwmSbrCfg, 0, sizeof(SbrPwmCfg_s)) != sizeof(SbrPwmCfg_s)) {
        ret = -EIO;
        goto exit;
    }

#if CONFIG_DUMP_SBR
    LOGI("%s: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, chNum:%u\r\n", __func__,
         pwmSbrCfg->regAddr, pwmSbrCfg->irqNo, pwmSbrCfg->irqPrio, pwmSbrCfg->chNum);
#endif

    if (pwmSbrCfg->irqNo == 0 || pwmSbrCfg->irqPrio == 0 || pwmSbrCfg->regAddr == NULL) {
        ret = -EINVAL;
        goto exit;
    }

exit:
    return ret;
}

static S32 pwmGetPrescaler(DevList_e devId, U16 *psc)
{
    S32 ret = EXIT_SUCCESS;
    U32 prescalerValue = 0;
    U32 clk = 0;

    if (psc == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (peripsClockFreqGet(devId, &clk) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    prescalerValue = (clk / PWM_CNT_FREQ);
    if (prescalerValue == 0 || prescalerValue > (PWM_MAX_PRESCALER + 1)) {
        ret = -EINVAL;
        LOGE("%s: prescalerValue %u is invalid, clk: %u\r\n", __func__, prescalerValue, clk);
        goto exit;
    }

    *psc = (U16)(prescalerValue - 1);

exit:
    return ret;
}

static S32 pwmIrqEnable(PwmDrvData_s *pDrvData, PwmIntrType_e intrType)
{
    S32 ret = EXIT_SUCCESS;
    PwmReg_s *pReg = NULL;

    /* Input parameter validation */
    if (pDrvData == NULL) {
        LOGE("%s: Invalid parameter, pDrvData is NULL\r\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    /* Validate interrupt type */
    if (intrType < PWM_CHAN_1_INT || intrType > PWM_ALL_INT) {
        LOGE("%s: Invalid interrupt type %d\r\n", __func__, intrType);
        ret = -EINVAL;
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: pReg is NULL\r\n", __func__);
        ret = -EIO;
        goto exit;
    }

    /* Configure interrupt enable bits based on type */
    switch (intrType) {
    case PWM_CHAN_1_INT: pReg->pwmIntrCtrl.chan1IntrEn = 1; break;
    case PWM_CHAN_2_INT: pReg->pwmIntrCtrl.chan2IntrEn = 1; break;
    case PWM_CHAN_3_INT: pReg->pwmIntrCtrl.chan3IntrEn = 1; break;
    case PWM_CHAN_4_INT: pReg->pwmIntrCtrl.chan4IntrEn = 1; break;
    case PWM_BRAKE_INT:  pReg->pwmIntrCtrl.brakeIntrEn = 1; break;
    case PWM_ALL_INT:
        pReg->pwmIntrCtrl.chan1IntrEn = 1;
        pReg->pwmIntrCtrl.chan2IntrEn = 1;
        pReg->pwmIntrCtrl.chan3IntrEn = 1;
        pReg->pwmIntrCtrl.chan4IntrEn = 1;
        pReg->pwmIntrCtrl.brakeIntrEn = 1;
        break;
    default:
        LOGE("%s: Invalid interrupt type %d\r\n", __func__, intrType);
        ret = -EINVAL;
        break;
    }

exit:
    return ret;
}

static S32 pwmIrqDisable(PwmDrvData_s *pDrvData, PwmIntrType_e intrType)
{
    S32 ret = EXIT_SUCCESS;
    PwmReg_s *pReg = NULL;

    /* Input parameter validation */
    if (pDrvData == NULL) {
        LOGE("%s: Invalid parameter, pDrvData is NULL\r\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: pReg is NULL\r\n", __func__);
        ret = -EIO;
        goto exit;
    }

    /* Validate interrupt type */
    if (intrType < PWM_CHAN_1_INT || intrType > PWM_ALL_INT) {
        LOGE("%s: Invalid interrupt type %d\r\n", __func__, intrType);
        ret = -EINVAL;
        goto exit;
    }

    /* Configure interrupt enable bits based on type */
    switch (intrType) {
    case PWM_CHAN_1_INT: pReg->pwmIntrCtrl.chan1IntrEn = 0; break;
    case PWM_CHAN_2_INT: pReg->pwmIntrCtrl.chan2IntrEn = 0; break;
    case PWM_CHAN_3_INT: pReg->pwmIntrCtrl.chan3IntrEn = 0; break;
    case PWM_CHAN_4_INT: pReg->pwmIntrCtrl.chan4IntrEn = 0; break;
    case PWM_BRAKE_INT:  pReg->pwmIntrCtrl.brakeIntrEn = 0; break;
    case PWM_ALL_INT:
        pReg->pwmIntrCtrl.chan1IntrEn = 0;
        pReg->pwmIntrCtrl.chan2IntrEn = 0;
        pReg->pwmIntrCtrl.chan3IntrEn = 0;
        pReg->pwmIntrCtrl.chan4IntrEn = 0;
        pReg->pwmIntrCtrl.brakeIntrEn = 0;
        break;
    default:
        LOGE("%s: Invalid interrupt type %d\r\n", __func__, intrType);
        ret = -EINVAL;
        break;
    }

exit:
    return ret;
}

static void pwmIrqHandler(PwmDrvData_s *pDrvData)
{
    U32 stat = 0;
    U32 regVal = 0;
    PwmReg_s *pReg = NULL;

    /* Input parameter validation */
    if (pDrvData == NULL) {
        LOGE("%s: Invalid parameter, pDrvData is NULL\r\n", __func__);
        return;
    }

    if (pDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s: Invalid register address\r\n", __func__);
        return;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;

    /* Read interrupt status */
    regVal = pReg->pwmIntrStat.dword;
    stat = regVal & PWM_INTR_STAT_ALL;

    if (stat == 0) {
        return;
    }

    /* Clear all interrupt status bits */
    pReg->pwmIntrCtrl.dword = (regVal & (~PWM_INTR_STAT_ALL));    ///< pwm int write 0 clear
    if (pDrvData->irqCb.cb != NULL) {
        pDrvData->irqCb.cb(pDrvData->irqCb.arg, stat);
    }
}

S32 pwmCallbackRegister(DevList_e devId, pwmIrqCallBack callback, void *arg)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;
    PwmIntrCtrl_s intStat = {0};

    if (callback == NULL) {
        LOGE("%s: Invalid parameter, callback is NULL\r\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    /* Lock device and perform validation */
    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (pDrvData == NULL) {
        LOGE("%s: invalid pDrvData, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    intStat.dword = pReg->pwmIntrCtrl.dword;

    if(pwmIrqDisable(pDrvData, PWM_ALL_INT) != EXIT_SUCCESS) {
        LOGE("%s: failed to disable interrupt: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Set callback function and argument */
    pDrvData->irqCb.cb = callback;
    pDrvData->irqCb.arg = arg;

    pReg->pwmIntrCtrl.dword = intStat.dword;

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 pwmCallbackUnRegister(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;
    PwmIntrCtrl_s intStat = {0};

    /* Lock device and perform validation */
    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    intStat.dword = pReg->pwmIntrCtrl.dword;

    if(pwmIrqDisable(pDrvData, PWM_ALL_INT) != EXIT_SUCCESS) {
        LOGE("%s: failed to disable interrupt: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Clear callback function and argument */
    pDrvData->irqCb.cb = NULL;
    pDrvData->irqCb.arg = NULL;

    pReg->pwmIntrCtrl.dword = intStat.dword;

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 pwmInterruptEnable(DevList_e devId, PwmIntrType_e intrType)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    ret = pwmIrqEnable(pDrvData, intrType);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: pwm interrupt enable fail, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 pwmInterruptDisable(DevList_e devId, PwmIntrType_e intrType)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    ret = pwmIrqDisable(pDrvData, intrType);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: pwm interrupt disable fail, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 pwmCfg(DevList_e devId, PwmOutParameter_s *cfg)
{
    S32 ret = EXIT_SUCCESS;
    U16 psc = 0;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;
    U16 dutyCompareVal = 0;
    U32 cntEn = 0;
    U32 preloadRegCnt = 0;

    /* Input parameter validation */
    if (cfg == NULL) {
        LOGE("%s: Invalid parameter, cfg is NULL\r\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (cfg->freq == 0) {
        LOGE("%s: Invalid frequency %u\r\n", __func__, cfg->freq);
        ret = -EINVAL;
        goto exit;
    }

    /* Validate channel ID */
    if (cfg->chId >= (U32)PWM_CHANNEL_MAX) {
        LOGE("%s: Invalid chId %u (max %u)\r\n", __func__, cfg->chId, PWM_CHANNEL_MAX - 1);
        ret = -EINVAL;
        goto exit;
    }

    /* Validate duty cycle */
    if (cfg->duty > PWM_DUTY_MAX) {
        LOGE("%s: Invalid duty %u (max %u)\r\n", __func__, cfg->duty, PWM_DUTY_MAX);
        ret = -EINVAL;
        goto exit;
    }

    /* Validate polarity */
    if (cfg->polarity >= PWM_POLARITY_INVALID) {
        LOGE("%s: Invalid polarity %u (valid values: %u or %u)\r\n",
            __func__, cfg->polarity, PWM_POLARITY_ACT_HIGH, PWM_POLARITY_ACT_LOW);
        ret = -EINVAL;
        goto exit;
    }

    /* Lock device and perform validation */
    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    ret = pwmGetPrescaler(devId, &psc);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to convert freq to prescaler, ret: %d\r\n", __func__, ret);
        ret = -EIO;
        goto unlock;
    }

    preloadRegCnt = PWM_GET_PRELOAD_CNT(cfg->freq);
    if ((preloadRegCnt >= PWM_MAX_PRELOAD) || (preloadRegCnt == 0)) {
        LOGE("%s: invalid preloadRegCnt: %d\r\n", __func__, preloadRegCnt);
        ret = -EINVAL;
        goto exit;
    }

    /* calculate duty compare value */
    dutyCompareVal = (U16)(PWM_GET_DUTY_CMPARE_VAL(preloadRegCnt, cfg->duty));
    if (dutyCompareVal > PWM_DUTY_COMPARE_MAX_VAL) {
        LOGE("%s: invalid dutyCompareVal: %d\r\n", __func__, dutyCompareVal);
        ret = -EINVAL;
        goto exit;
    }

    /* Configure each channel output */
    cntEn = pReg->pwmRegCtrl.countEn;
    pReg->pwmRegCtrl.countEn = 0; ///< Disable PWM main counter

    /* Set prescaler and preload values for Freq*/
    pReg->pwmPreDivClk.divClk = psc;
    pReg->pwmPreLoad.preCount = (U16)preloadRegCnt;
    pDrvData->preloadRegCnt = preloadRegCnt;

    /* Set duty compare value */
    pReg->pwmCaptureData[cfg->chId].dataCount = dutyCompareVal;

    switch (cfg->chId) {
    case PWM_CHAN_0:
        pReg->ch12InOutCtrl.pwmOutCompare.cc13Select = 0; ///< 0 for output
        pReg->ch12InOutCtrl.pwmOutCompare.oc13Ref = PWM_OUTPUTCOMPARE_MODE1;
        pReg->pwmCapCmpCtrl.cc1Polarity = (PWM_POLARITY_ACT_LOW == cfg->polarity) ? (1) : (0);
        break;
    case PWM_CHAN_1:
        pReg->ch12InOutCtrl.pwmOutCompare.cc24Select = 0; ///< 0 for output
        pReg->ch12InOutCtrl.pwmOutCompare.oc24Ref = PWM_OUTPUTCOMPARE_MODE1;
        pReg->pwmCapCmpCtrl.cc2Polarity = (PWM_POLARITY_ACT_LOW == cfg->polarity) ? (1) : (0);
        break;
    case PWM_CHAN_2:
        pReg->ch34InOutCtrl.pwmOutCompare.cc13Select = 0; ///< 0 for output
        pReg->ch34InOutCtrl.pwmOutCompare.oc13Ref = PWM_OUTPUTCOMPARE_MODE1;
        pReg->pwmCapCmpCtrl.cc3Polarity = (PWM_POLARITY_ACT_LOW == cfg->polarity) ? (1) : (0);
        break;
    case PWM_CHAN_3:
        pReg->ch34InOutCtrl.pwmOutCompare.cc24Select = 0; ///< 0 for output
        pReg->ch34InOutCtrl.pwmOutCompare.oc24Ref = PWM_OUTPUTCOMPARE_MODE1;
        pReg->pwmCapCmpCtrl.cc4Polarity = (PWM_POLARITY_ACT_LOW == cfg->polarity) ? (1) : (0);
        break;
    default:
        ret = -EINVAL;
        break;
    }

    pReg->pwmRegCtrl.countEn = cntEn; ///< Restore counter state

unlock:
    funcRunEndHelper(devId);
exit:
    return ret;
}

S32 pwmSetFreq(DevList_e devId, U32 Freq)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;
    U16 psc = 0;

    /* Input parameter validation */
    if (Freq == 0) {
        LOGE("%s: Invalid frequency %u\r\n", __func__, Freq);
        ret = -EINVAL;
        goto exit;
    }

    /* Validate frequency range - minimum 1Hz, maximum based on clock */
    if (Freq > PWM_CNT_FREQ) {
        LOGE("%s: Frequency %u exceeds maximum %u\r\n", __func__, Freq, PWM_CNT_FREQ);
        ret = -EINVAL;
        goto exit;
    }

    if (PWM_GET_PRELOAD_CNT(Freq) >= PWM_MAX_PRELOAD) {
        LOGE("%s: Frequency is too low: %d\r\n", __func__, PWM_GET_PRELOAD_CNT(Freq));
        ret = -EINVAL;
        goto exit;
    }

    /* Lock device and perform validation */
    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Convert frequency to prescaler */
    ret = pwmGetPrescaler(devId, &psc);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to convert freq to prescaler, ret: %d\r\n", __func__, ret);
        ret = -EIO;
        goto unlock;
    }

    /* Set prescaler and preload values */
    pReg->pwmPreDivClk.divClk = psc;
    pReg->pwmPreLoad.preCount = (U16)PWM_GET_PRELOAD_CNT(Freq);
    pDrvData->preloadRegCnt = (U16)PWM_GET_PRELOAD_CNT(Freq);

unlock:
    funcRunEndHelper(devId);
exit:
    return ret;
}

S32 pwmSetDuty(DevList_e devId, PwmChannel_e chId, U32 duty)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;
    U32 dutyCompareVal = 0;
    U32 preloadCnt = 0;

    /* Input parameter validation */
    if (duty > PWM_DUTY_MAX) {
        LOGE("%s: invalid duty: %d\r\n", __func__, duty);
        ret = -EINVAL;
        goto exit;
    }

    if (chId >= PWM_CHANNEL_MAX) {
        LOGE("%s: invalid channel ID: %d\r\n", __func__, chId);
        ret = -EINVAL;
        goto exit;
    }

    /* Lock device and perform validation */
    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Calculate duty compare value */
    preloadCnt = pDrvData->preloadRegCnt;
    if (preloadCnt == 0) {
        LOGE("%s: preload count is 0, frequency may not be set\r\n", __func__);
        ret = -EIO;
        goto unlock;
    }

    dutyCompareVal = (U16)(PWM_GET_DUTY_CMPARE_VAL(preloadCnt, duty));

    if (dutyCompareVal > PWM_DUTY_COMPARE_MAX_VAL) {
        LOGE("%s: invalid dutyCompareVal: %d\r\n", __func__, dutyCompareVal);
        ret = -EINVAL;
        goto unlock;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Set duty compare value */
    pReg->pwmCaptureData[chId].dataCount = dutyCompareVal;

unlock:
    funcRunEndHelper(devId);
exit:
    return ret;
}

S32 pwmSetPolarity(DevList_e devId, PwmChannel_e chId, PwmOutputPolarity_e polarity)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;

    /* Input parameter validation */
    if (polarity >= PWM_POLARITY_INVALID) {
        LOGE("%s: invalid polarity: %d\r\n", __func__, polarity);
        ret = -EINVAL;
        goto exit;
    }

    if (chId >= PWM_CHANNEL_MAX) {
        LOGE("%s: invalid channel ID: %d\r\n", __func__, chId);
        ret = -EINVAL;
        goto exit;
    }

    /* Lock device and perform validation */
    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    switch (chId) {
    case PWM_CHAN_0:
        pReg->ch12InOutCtrl.pwmOutCompare.cc13Select = 0; ///< 0 for output
        pReg->ch12InOutCtrl.pwmOutCompare.oc13Ref = PWM_OUTPUTCOMPARE_MODE1;
        pReg->pwmCapCmpCtrl.cc1Polarity = (PWM_POLARITY_ACT_LOW == polarity) ? (1) : (0);
        break;
    case PWM_CHAN_1:
        pReg->ch12InOutCtrl.pwmOutCompare.cc24Select = 0; ///< 0 for output
        pReg->ch12InOutCtrl.pwmOutCompare.oc24Ref = PWM_OUTPUTCOMPARE_MODE1;
        pReg->pwmCapCmpCtrl.cc2Polarity = (PWM_POLARITY_ACT_LOW == polarity) ? (1) : (0);
        break;
    case PWM_CHAN_2:
        pReg->ch34InOutCtrl.pwmOutCompare.cc13Select = 0; ///< 0 for output
        pReg->ch34InOutCtrl.pwmOutCompare.oc13Ref = PWM_OUTPUTCOMPARE_MODE1;
        pReg->pwmCapCmpCtrl.cc3Polarity = (PWM_POLARITY_ACT_LOW == polarity) ? (1) : (0);
        break;
    case PWM_CHAN_3:
        pReg->ch34InOutCtrl.pwmOutCompare.cc24Select = 0; ///< 0 for output
        pReg->ch34InOutCtrl.pwmOutCompare.oc24Ref = PWM_OUTPUTCOMPARE_MODE1;
        pReg->pwmCapCmpCtrl.cc4Polarity = (PWM_POLARITY_ACT_LOW == polarity) ? (1) : (0);
        break;
    default:
        ret = -EINVAL;
    }

unlock:
    funcRunEndHelper(devId);
exit:
    return ret;
}

S32 pwmStart(DevList_e devId, PwmChannel_e chId)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;

    /* Input parameter validation */
    if (chId >= PWM_CHANNEL_MAX) {
        LOGE("%s: invalid channel ID: %d\r\n", __func__, chId);
        ret = -EINVAL;
        goto exit;
    }

    /* Lock device and perform validation */
    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Enable PWM main counter */
    pReg->pwmRegCtrl.countEn = 1;

    /* Enable channel for PWM output */
    switch(chId) {
        case PWM_CHAN_0: pReg->pwmCapCmpCtrl.cc1Enable = 1; break;
        case PWM_CHAN_1: pReg->pwmCapCmpCtrl.cc2Enable = 1; break;
        case PWM_CHAN_2: pReg->pwmCapCmpCtrl.cc3Enable = 1; break;
        case PWM_CHAN_3: pReg->pwmCapCmpCtrl.cc4Enable = 1; break;
        default:
            LOGE("%s: Invalid channel ID %u\r\n", __func__, chId);
            ret = -EIO;
            break;
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 pwmStop(DevList_e devId, PwmChannel_e chId)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;

    /* Input parameter validation */
    if (chId >= PWM_CHANNEL_MAX) {
        LOGE("%s: invalid channel ID: %d\r\n", __func__, chId);
        ret = -EINVAL;
        goto exit;
    }

    /* Lock device and perform validation */
    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Disable channel compare enable */
    switch(chId) {
        case PWM_CHAN_0: pReg->pwmCapCmpCtrl.cc1Enable = 0; break;
        case PWM_CHAN_1: pReg->pwmCapCmpCtrl.cc2Enable = 0; break;
        case PWM_CHAN_2: pReg->pwmCapCmpCtrl.cc3Enable = 0; break;
        case PWM_CHAN_3: pReg->pwmCapCmpCtrl.cc4Enable = 0; break;
        default:
            LOGE("%s: Invalid channel ID %u\r\n", __func__, chId);
            ret = -EIO;
            break;
    }

    /* Disable PWM main counter */
    if (pReg->pwmCapCmpCtrl.cc1Enable == 0 && pReg->pwmCapCmpCtrl.cc2Enable == 0
        && pReg->pwmCapCmpCtrl.cc3Enable == 0 && pReg->pwmCapCmpCtrl.cc4Enable == 0) {
        pReg->pwmRegCtrl.countEn = 0;
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 pwmInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;

    /* Lock device */
    if(devLockByDriver(devId, PWM_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: failed to lock driver, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    /* Check if driver is already initialized */
    if (isDrvInit(devId) == true) {
        LOGE("%s: driver already initialized, devId: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    /* Check driver match */
    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        LOGE("%s: driver not matched, devId: %d\r\n", __func__, devId);
        ret = -ENODEV;
        goto unlock;
    }

    /* Enable peripheral clock */
    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to enable clock, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Reset peripheral */
    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to reset periphs, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Allocate driver data */
    pDrvData = calloc(1, sizeof(PwmDrvData_s));
    if (pDrvData == NULL) {
        ret = -ENOMEM;
        LOGE("%s: failed to allocate memory, devId: %d\r\n", __func__, devId);
        goto unlock;
    }

    /* Get device configuration */
    if (getPwmDevConfig(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s: failed to get sbr config, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto freeMem;
    }

    if(pwmIrqDisable(pDrvData, PWM_ALL_INT) != EXIT_SUCCESS) {
        LOGE("%s: failed to disable interrupt, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto freeMem;
    }

    /* Install interrupt handler */
    ret = ospInterruptHandlerInstall(pDrvData->sbrCfg.irqNo, "pwm", OSP_INTERRUPT_UNIQUE,
        (OspInterruptHandler)pwmIrqHandler, (void*)pDrvData);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s: irq handler already installed, devId: %d\r\n", __func__, devId);
    } else if (ret == OSP_SUCCESSFUL) {
        ospInterruptVectorEnable((pDrvData->sbrCfg.irqNo));
    } else {
        LOGE("%s: irq handler install failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto freeMem;
    }

    /* Initialize preload count to 0 */
    pDrvData->preloadRegCnt = 0;

    /* Install device driver */
    if (drvInstall(devId, pDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto removeIrq;
    }

    ret = EXIT_SUCCESS;
    goto unlock;

removeIrq:
    if (pDrvData->sbrCfg.irqNo) {
        ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
        ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo, (OspInterruptHandler)pwmIrqHandler, pDrvData);
    }

freeMem:
    if (pDrvData != NULL) {
        free(pDrvData);
        pDrvData = NULL;
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 pwmDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;

    /* Lock device and perform validation */
    ret = funcRunBeginHelper(devId, DRV_ID_STARS_PWM, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg->pwmRegCtrl.countEn = 0;   ///< Disable counter en

    /* Disable interrupt */
    if(pwmIrqDisable(pDrvData, PWM_ALL_INT) != EXIT_SUCCESS) {
        LOGE("%s: failed to disable interrupt: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if (pDrvData->sbrCfg.irqNo) {
        ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
        ret = ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo,
                                         (OspInterruptHandler)pwmIrqHandler,
                                         pDrvData);
        if (ret != OSP_SUCCESSFUL) {
            LOGE("%s: failed to remove IRQ handler, ret=%d\r\n", __func__, ret);
        }
    }

    /* Reset peripheral */
    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to reset peripheral, devId: %d\r\n", __func__, devId);
        ret = -EIO;
    }

    /* disable peripheral clock */
    if (peripsClockDisable(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to disable peripheral clock, devId: %d\r\n", __func__, devId);
        ret = -EIO;
    }

    /* Uninstall driver */
    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to uninstall driver, devId: %d\r\n", __func__, devId);
        ret = -EIO;
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

