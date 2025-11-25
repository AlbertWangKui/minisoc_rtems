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

static S32 convertPwmFreqToPrescaler(DevList_e devId, U16 *psc)
{
    S32 ret = EXIT_SUCCESS;
    U32 prescalerValue;
    U32 clk = 0;

    if (psc == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (peripsClockFreqGet(devId, &clk) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    prescalerValue = clk / PWM_CNT_FREQ;
    if (prescalerValue == 0 || prescalerValue > (PWM_MAX_PRESCALER + 1)) {
        ret = -EINVAL;
        LOGE("%s: prescalerValue %u is invalid, clk: %u\r\n", __func__, prescalerValue, clk);
        goto exit;
    }

    *psc = (U16)(prescalerValue - 1);

exit:
    return ret;
}

static S32 pwmDrvLockAndCheck(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;

    /* lock */
    if (devLockByDriver(devId, PWM_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: %u lock failure!\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        LOGE("%s: %u driver mismatch!\r\n", __func__, devId);
        ret = -ENODEV;
        goto exit;
    }

    if(isDrvInit(devId) == false) {
        ret = -EINVAL;
        LOGE("%s: %u is unintialized!\r\n", __func__, devId);
        goto exit;
    }

exit:
    return ret;
}

static S32 pwmInterruptEnable(PwmDrvData_s *pDrvData, PwmIntrType_e intrType, bool enable)
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

    /* Configure interrupt enable bits based on type */
    switch (intrType) {
    case PWM_CHAN_1_INT:
        pReg->pwmIntrCtrl.chan1IntrEn = (enable == true) ? (1) : (0);
        break;
    case PWM_CHAN_2_INT:
        pReg->pwmIntrCtrl.chan2IntrEn = (enable == true) ? (1) : (0);
        break;
    case PWM_CHAN_3_INT:
        pReg->pwmIntrCtrl.chan3IntrEn = (enable == true) ? (1) : (0);
        break;
    case PWM_CHAN_4_INT:
        pReg->pwmIntrCtrl.chan4IntrEn = (enable == true) ? (1) : (0);
        break;
    case PWM_BRAKE_INT:
        pReg->pwmIntrCtrl.brakeIntrEn = (enable == true) ? (1) : (0);
        break;
    case PWM_ALL_INT:
        pReg->pwmIntrCtrl.chan1IntrEn = (enable == true) ? (1) : (0);
        pReg->pwmIntrCtrl.chan2IntrEn = (enable == true) ? (1) : (0);
        pReg->pwmIntrCtrl.chan3IntrEn = (enable == true) ? (1) : (0);
        pReg->pwmIntrCtrl.chan4IntrEn = (enable == true) ? (1) : (0);
        pReg->pwmIntrCtrl.brakeIntrEn = (enable == true) ? (1) : (0);
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
    reg32Write(&pReg->pwmIntrStat, (~PWM_INTR_STAT_ALL) & regVal);  ///< pwm int write 0 clear
    if (pDrvData->irqCb.cb != NULL) {
        pDrvData->irqCb.cb(pDrvData->irqCb.arg, stat);
    }
}

static S32 cfgPwmOutput(PwmReg_s * pReg, PwmOutParameter_s * cfg)
{
    S32 ret = EXIT_SUCCESS;
    U32 cntEn = 0;

    /* Input parameter validation */
    if ((NULL == pReg) || (NULL == cfg)) {
        LOGE("%s: Invalid parameter, pReg or cfg is NULL\r\n", __func__);
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
            __func__, cfg->polarity, PWM_POLARITY_NORMAL, PWM_POLARITY_REVERSE);
        ret = -EINVAL;
        goto exit;
    }

    /* Configure each channel output */
    cntEn = pReg->pwmRegCtrl.countEn;
    pReg->pwmRegCtrl.countEn = 0; ///< Disable PWM main counter
    
    switch (cfg->chId) {
    case PWM_CHAN_0:
        pReg->ch12InOutCtrl.pwmOutCompare.cc13Select = 0; ///< 0 for output
        pReg->ch12InOutCtrl.pwmOutCompare.oc13Ref = PWM_OC_MODE1;
        pReg->pwmCapCmpCtrl.cc1Polarity = (PWM_POLARITY_REVERSE == cfg->polarity) ? (1) : (0);
        break;
    case PWM_CHAN_1:
        pReg->ch12InOutCtrl.pwmOutCompare.cc24Select = 0; ///< 0 for output
        pReg->ch12InOutCtrl.pwmOutCompare.oc24Ref = PWM_OC_MODE1;
        pReg->pwmCapCmpCtrl.cc2Polarity = (PWM_POLARITY_REVERSE == cfg->polarity) ? (1) : (0);
        break;
    case PWM_CHAN_2:
        pReg->ch34InOutCtrl.pwmOutCompare.cc13Select = 0; ///< 0 for output
        pReg->ch34InOutCtrl.pwmOutCompare.oc13Ref = PWM_OC_MODE1;
        pReg->pwmCapCmpCtrl.cc3Polarity = (PWM_POLARITY_REVERSE == cfg->polarity) ? (1) : (0);
        break;
    case PWM_CHAN_3:
        pReg->ch34InOutCtrl.pwmOutCompare.cc24Select = 0; ///< 0 for output
        pReg->ch34InOutCtrl.pwmOutCompare.oc24Ref = PWM_OC_MODE1;
        pReg->pwmCapCmpCtrl.cc4Polarity = (PWM_POLARITY_REVERSE == cfg->polarity) ? (1) : (0);
        break;
    default:
        pReg->pwmRegCtrl.countEn = cntEn;
        ret = -EINVAL;
        goto exit;
    }

exit:
    return ret;
}

static S32 pwmChannelEnable(PwmDrvData_s *pDrvData, U32 chId, bool enable)
{
    PwmReg_s *pReg = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Input parameter validation */
    if((pDrvData == NULL) || (chId >= PWM_CHANNEL_MAX)) {
        LOGE("%s: Invalid parameter, pDrvData is NULL or chId %u is invalid\r\n", __func__, chId);
        ret = -EINVAL;
        goto exit;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, chId: %d\r\n", __func__, chId);
        ret = -EIO;
        goto exit;
    }

    /* Configure channel enable bits */
    switch(chId) {
        case PWM_CHAN_0:
            pReg->pwmCapCmpCtrl.cc1Enable = (enable == true) ? (1) : (0);
            break;
        case PWM_CHAN_1:
            pReg->pwmCapCmpCtrl.cc2Enable = (enable == true) ? (1) : (0);
            break;
        case PWM_CHAN_2:
            pReg->pwmCapCmpCtrl.cc3Enable = (enable == true) ? (1) : (0);
            break;
        case PWM_CHAN_3:
            pReg->pwmCapCmpCtrl.cc4Enable = (enable == true) ? (1) : (0);
            break;
        default:
            LOGE("%s: Invalid channel ID %u\r\n", __func__, chId);
            ret = -EIO;
            break;
    }

exit:
    return ret;
}

S32 pwmCallbackRegister(DevList_e devId,
        PwmIntrType_e intrType, pwmIrqCallBack callback, void *arg)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;

    /* Input parameter validation */
    if (intrType < PWM_CHAN_1_INT || intrType > PWM_ALL_INT) {
        LOGE("%s: Invalid interrupt type %d\r\n", __func__, intrType);
        ret = -EINVAL;
        goto exit;
    }

    if (callback == NULL) {
        LOGE("%s: Invalid parameter, callback is NULL\r\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    /* Lock device and perform validation */
    ret = pwmDrvLockAndCheck(devId);
    if (-EBUSY == ret) {
        goto exit;
    } else if (EXIT_SUCCESS != ret) {
        goto unlock;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if(pwmInterruptEnable(pDrvData, intrType, false) != EXIT_SUCCESS) {
        LOGE("%s: failed to disable interrupt: %d\r\n", __func__, intrType);
        ret = -EIO;
        goto unlock;
    }

    /* Set callback function and argument */
    pDrvData->irqCb.cb = callback;
    pDrvData->irqCb.arg = arg;

    /* Re-enable interrupt */
    if(pwmInterruptEnable(pDrvData, intrType, true) != EXIT_SUCCESS) {
        LOGE("%s: failed to enable interrupt: %d\r\n", __func__, intrType);
        ret = -EIO;
        goto unlock;
    }

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 pwmCallbackUnRegister(DevList_e devId, PwmIntrType_e intrType)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;

    /* Lock device and perform validation */
    ret = pwmDrvLockAndCheck(devId);
    if (-EBUSY == ret) {
        goto exit;
    } else if (EXIT_SUCCESS != ret) {
        goto unlock;
    }

    /* Input parameter validation */
    if (intrType < PWM_CHAN_1_INT || intrType > PWM_ALL_INT) {
        LOGE("%s: Invalid interrupt type %d\r\n", __func__, intrType);
        ret = -EINVAL;
        goto unlock;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    if(pwmInterruptEnable(pDrvData, intrType, false) != EXIT_SUCCESS) {
        LOGE("%s: failed to disable interrupt: %d\r\n", __func__, intrType);
        ret = -EIO;
        goto unlock;
    }

    /* Clear callback function and argument */
    pDrvData->irqCb.cb = NULL;
    pDrvData->irqCb.arg = NULL;

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 pwmSetCfg(DevList_e devId, PwmOutParameter_s *cfg)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;

    /* Input parameter validation */
    if (cfg == NULL) {
        LOGE("%s: Invalid parameter, cfg is NULL\r\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    /* Lock device and perform validation */
    ret = pwmDrvLockAndCheck(devId);
    if (-EBUSY == ret) {
        goto exit;
    } else if (EXIT_SUCCESS != ret) {
        goto unlock;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Configure PWM output */
    ret = cfgPwmOutput(pReg, cfg);

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 pwmSetFreq(DevList_e devId, U32 Freq)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;
    PwmReg_s *pReg = NULL;
    U16 prescaler = 0;

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
    ret = pwmDrvLockAndCheck(devId);
    if (-EBUSY == ret) {
        goto exit;
    } else if (EXIT_SUCCESS != ret) {
        goto unlock;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Convert frequency to prescaler */
    ret = convertPwmFreqToPrescaler(devId, &prescaler);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to convert freq to prescaler, ret: %d\r\n", __func__, ret);
        ret = -EIO;
        goto unlock;
    }

    /* Set prescaler and preload values */
    pReg->pwmPreDivClk.divClk = prescaler;
    pReg->pwmPreLoad.preCount = (U16)PWM_GET_PRELOAD_CNT(Freq);
    pDrvData->preloadRegCnt = (U16)PWM_GET_PRELOAD_CNT(Freq);

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 pwmSetDuty(DevList_e devId, U32 chId, U32 duty)
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
    ret = pwmDrvLockAndCheck(devId);
    if (-EBUSY == ret) {
        goto exit;
    } else if (EXIT_SUCCESS != ret) {
        goto unlock;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
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
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 pwmStart(DevList_e devId, U32 chId)
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
    ret = pwmDrvLockAndCheck(devId);
    if (-EBUSY == ret) {
        goto exit;
    } else if (EXIT_SUCCESS != ret) {
        goto unlock;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Enable PWM main counter */
    pReg->pwmRegCtrl.countEn = 1; ///< Enable PWM main counter

    /* Enable channel for PWM output */
    ret = pwmChannelEnable(pDrvData, chId, 1);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to enable channel: %d, ret: %d\r\n", __func__, chId, ret);
        ret = -EIO;
        goto unlock;
    }

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 pwmStop(DevList_e devId, U32 chId)
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
    ret = pwmDrvLockAndCheck(devId);
    if (-EBUSY == ret) {
        goto exit;
    } else if (EXIT_SUCCESS != ret) {
        goto unlock;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    pReg = (PwmReg_s *)pDrvData->sbrCfg.regAddr;
    if (pReg == NULL) {
        LOGE("%s: invalid register, devId: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Disable channel compare enable */
    ret = pwmChannelEnable(pDrvData, chId, 0);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: failed to disable channel: %d, ret: %d\r\n", __func__, chId, ret);
        ret = -EIO;
        goto unlock;
    }

    /* Disable PWM main counter */
    if (pReg->pwmCapCmpCtrl.cc1Enable == 0 && pReg->pwmCapCmpCtrl.cc2Enable == 0
        && pReg->pwmCapCmpCtrl.cc3Enable == 0 && pReg->pwmCapCmpCtrl.cc4Enable == 0) {
        pReg->pwmRegCtrl.countEn = 0;
    }
    
unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 pwmInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;

    /* Lock device */
    if(devLockByDriver(devId, PWM_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        LOGE("%s: failed to lock driver: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto exit;
    }

    /* Check if driver is already initialized */
    if (isDrvInit(devId) == true) {
        LOGE("%s: driver already initialized: %d\r\n", __func__, devId);
        ret = -EBUSY;
        goto unlock;
    }

    /* Check driver match */
    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        LOGE("%s: driver not matched: %d\r\n", __func__, devId);
        ret = -ENODEV;
        goto unlock;
    }

    /* Enable peripheral clock */
    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to enable clock: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Reset peripheral */
    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to reset periphs: %d\r\n", __func__, devId);
        ret = -EIO;
        goto unlock;
    }

    /* Allocate driver data */
    pDrvData = calloc(1, sizeof(PwmDrvData_s));
    if (pDrvData == NULL) {
        ret = -ENOMEM;
        LOGE("%s: failed to allocate memory: %d\r\n", __func__, devId);
        goto unlock;
    }

    /* Get device configuration */
    if (getPwmDevConfig(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s: failed to get sbr config: %d\r\n", __func__, devId);
        ret = -EIO;
        goto freeMem;
    }

    /* Install interrupt handler */
    ret = ospInterruptHandlerInstall(pDrvData->sbrCfg.irqNo, "pwm", OSP_INTERRUPT_UNIQUE,
        (OspInterruptHandler)pwmIrqHandler, (void*)pDrvData);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s: irq handler already installed\r\n", __func__);
    } else if (ret == OSP_SUCCESSFUL) {
        ospInterruptVectorEnable((pDrvData->sbrCfg.irqNo));
    } else {
        LOGE("%s: irq handler install failed\r\n", __func__);
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
    devUnlockByDriver(devId);

exit:
    return ret;
}

S32 pwmDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pDrvData = NULL;

    /* Lock device and perform validation */
    ret = pwmDrvLockAndCheck(devId);
    if (-EBUSY == ret) {
        goto exit;
    } else if (EXIT_SUCCESS != ret) {
        goto unlock;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s: get device driver failed, devId: %d\r\n", __func__, devId);
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

    /* Uninstall driver */
    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("%s: failed to uninstall driver, devId: %d\r\n", __func__, devId);
        ret = -EIO;
    }

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

