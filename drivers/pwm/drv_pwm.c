/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_pwm.c
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/03/17
 * @brief
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/03/17   yangzhl3        移植自产品软件
 *
 *
 */

#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "osp_interrupt.h"
#include "log_msg.h"
#include "drv_pwm.h"
#include "drv_pwm_api.h"
#include "sbr_api.h"

static S32 pwmDevCfgGet(DevList_e devId, SbrPwmCfg_s *pwmSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (pwmSbrCfg == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    if (devSbrRead(devId, pwmSbrCfg, 0, sizeof(SbrPwmCfg_s)) != sizeof(SbrPwmCfg_s)) {
        ret = -EXIT_FAILURE;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("pwm: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, chNum:%u\r\n",
         pwmSbrCfg->regAddr, pwmSbrCfg->irqNo, pwmSbrCfg->irqPrio, pwmSbrCfg->chNum);
#endif

    if (pwmSbrCfg->irqNo == 0 || pwmSbrCfg->irqPrio == 0 || pwmSbrCfg->regAddr == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

out:
    return ret;
}

static S32 pwmConvertFreqToPsc(DevList_e devId, U32 freq, U16 *psc)
{
    S32 ret = EXIT_SUCCESS;
    U32 clk;

    if (freq == 0) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (peripsClockFreqGet(devId, &clk) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
    clk = clk/100;
    *psc = (U16)((clk / freq) - 1);

exit:
    return ret;
}

/**
 * @brief 初始化PWM模块：注册设备到系统、获取设备信息
 * @param [in] devId,PWM设备枚举ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmInit(DevList_e  devId)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pwmDrvData = NULL;

    if (isDrvInit(devId)) {
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        return -EINVAL;
    }

    if(devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    pwmDrvData = calloc(1, sizeof(PwmDrvData_s));
    if (pwmDrvData == NULL) {
        ret = -ENOMEM;
        goto unlock;
    }

    if (pwmDevCfgGet(devId, &pwmDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s: failed to get sbr\r\n", __func__);
        ret = -EIO;
        goto freemem0;
    }

    ///< 最后安装设备驱动
    if (drvInstall(devId, (void*)pwmDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freemem0;
    }

    ret = EXIT_SUCCESS;
    goto unlock;

freemem0:
    free(pwmDrvData);

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/**
 * @brief 去初始化设备：关闭PWM设备下所有通道，卸载驱动
 * @param [in] devId,PWM设备枚举ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;

    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        return -EINVAL;
    }

    if(devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    drvUninstall(devId); ///< 卸载设备

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}


/**
 * @brief 配置PWM参数
 * @param [in] devId,PWM设备枚举ID
 * @param [in] cfg,PWM参数,包括通道ID，周期、占空比、分频系数、输出极性和计数器预装载值
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmSetCfg(DevList_e devId, PwmOutParameter_s *cfg)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pwmDrvData;
    PwmReg_u regVal;
    U32 chid;

    if (cfg == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId,(void**)&pwmDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if(pwmDrvData == NULL) {
        ret = -EIO;
        goto unlock;
    }

    chid = cfg->chid;
    if (chid >= pwmDrvData->sbrCfg.chNum) {
        ret = -EINVAL;
        goto unlock;
    }

    ///< config channel to output
    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_CAPTURE_MODE_REG(chid+1));
    if ((chid == 0)||(chid == 2)) {
        regVal.pwmOutCapture.cc13Select = 0; ///< 0为输出
        regVal.pwmOutCapture.oc13Ref    = PWM_OC_MODE1;
    }
    else {
        regVal.pwmOutCapture.cc24Select = 0; ///< 0为输出
        regVal.pwmOutCapture.oc24Ref    = PWM_OC_MODE1;
    }
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_CAPTURE_MODE_REG(chid+1), regVal.all);

    ///< config prescaler
    regVal.all                 = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_PRE_DIV_CLK);
    regVal.pwmPreDivClk.divClk = cfg->prescaler;
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_PRE_DIV_CLK, regVal.all);

    ///< config polatity
    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_CAPTURE_CTRL);
    switch (chid) {
        case 0:
            regVal.pwmCaptureCtrl.cc1Polarity = (PWM_POLARITY_REVERSE == cfg->polarity) ? 1 : 0;
            break;
        case 1:
            regVal.pwmCaptureCtrl.cc2Polarity = (PWM_POLARITY_REVERSE == cfg->polarity) ? 1 : 0;
            break;
        case 2:
            regVal.pwmCaptureCtrl.cc3Polarity = (PWM_POLARITY_REVERSE == cfg->polarity) ? 1 : 0;
            break;
        case 3:
            regVal.pwmCaptureCtrl.cc4Polarity = (PWM_POLARITY_REVERSE == cfg->polarity) ? 1 : 0;
            break;
        default:
            ret = -EXIT_FAILURE;
            goto unlock;
    }

    ///< disable pwm
    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_CTRL);
    regVal.pwmRegCtrl.countEn = 0;
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_CTRL, regVal.all);
    regVal.all = cfg->preload;
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_PRE_LOAD, regVal.all);
    ///< enable pwm
    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_CTRL);
    regVal.pwmRegCtrl.countEn = 1;
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_CTRL, regVal.all);

    ///< duty
    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_DATA_REG(chid+1));
    regVal.pwmCaptureData.dataCount = cfg->duty;
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_DATA_REG(chid+1), regVal.all);

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief 设置PWM波输出频率
 * @param [in] devId,PWM设备枚举ID
 * @param [in] chid,PWM 某个设备的通道ID
 * @param [in] freq,PWM波输出频率
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmSetFreq(DevList_e devId, U8 chid,  U32 freq)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pwmDrvData;
    PwmReg_u regVal;
    U16 psc;

    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId,(void**)&pwmDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if(pwmDrvData == NULL) {
        ret = -EIO;
        goto unlock;
    }

    if (chid >= pwmDrvData->sbrCfg.chNum) {
        ret = -EINVAL;
        goto unlock;
    }

    ///< config prescaler(according to freq)
    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_PRE_DIV_CLK);
    if (pwmConvertFreqToPsc(devId, freq, &psc) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    regVal.pwmPreDivClk.divClk = psc;
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_PRE_DIV_CLK, regVal.all);

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief 设置占空比
 * @param [in] devId,PWM设备枚举ID
 * @param [in] chid,PWM 某个设备的通道ID
 * @param [in] duty,占空比（0~100），值除以100为占空比
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmSetDuty(DevList_e devId, U8 chid,  U32 duty)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pwmDrvData;
    PwmReg_u regVal;

    if (duty > 100) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId,(void**)&pwmDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if(pwmDrvData == NULL) {
        ret = -EIO;
        goto unlock;
    }

    if (chid >= pwmDrvData->sbrCfg.chNum) {
        ret = -EINVAL;
        goto unlock;
    }

    ///< config duty
    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_DATA_REG(chid+1));
    regVal.pwmCaptureData.dataCount = duty;
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_DATA_REG(chid+1), regVal.all);

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief 开启PWM
 * @param [in] devId,PWM设备枚举ID
 * @param [in] chid,PWM 某个设备的通道ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmStart(DevList_e devId, U32 chid)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pwmDrvData;
    PwmReg_u regVal;

    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId,(void**)&pwmDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if(pwmDrvData == NULL) {
        ret = -EIO;
        goto unlock;
    }

    if (chid >= pwmDrvData->sbrCfg.chNum) {
        ret = -EINVAL;
        goto unlock;
    }

    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_CAPTURE_CTRL);
    switch(chid) {
        case 0:
            regVal.pwmCaptureCtrl.cc1Enable = 1;
            break;
        case 1:
            regVal.pwmCaptureCtrl.cc2Enable = 1;
            break;
        case 2:
            regVal.pwmCaptureCtrl.cc3Enable = 1;
            break;
        case 3:
            regVal.pwmCaptureCtrl.cc4Enable = 1;
            break;
        default:
            ret = -EINVAL;
            goto unlock;
    }
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_CAPTURE_CTRL, regVal.all);

    ///< enable pwm
    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_CTRL);
    regVal.pwmRegCtrl.countEn = 1;
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_CTRL, regVal.all);

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/**
 * @brief 关闭PWM
 * @param [in] devId,PWM设备枚举ID
 * @param [in] chid,PWM 某个设备的通道ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmStop(DevList_e devId, U32 chid)
{
    S32 ret = EXIT_SUCCESS;
    PwmDrvData_s *pwmDrvData;
    PwmReg_u regVal;

    if (!isDrvMatch(devId, DRV_ID_STARS_PWM)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId,(void**)&pwmDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if(pwmDrvData == NULL) {
        ret = -EIO;
        goto unlock;
    }

    if (chid >= pwmDrvData->sbrCfg.chNum) {
        ret = -EINVAL;
        goto unlock;
    }

    regVal.all = reg32Read((U32)pwmDrvData->sbrCfg.regAddr+PWM_CTRL);
    regVal.pwmRegCtrl.countEn = 0;
    reg32Write((U32)pwmDrvData->sbrCfg.regAddr+PWM_CTRL, regVal.all);

    ret = EXIT_SUCCESS;

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}
