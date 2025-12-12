/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_gpio.c
 * @author zhaorui (zhaorui@starsmicrosystem.com)
 * @date 2025/11/7
 * @brief  gpio driver
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/10/22   zhaorui         the first version
 *
 */

#include <stdio.h>

#include "common_defines.h"
#include "bsp_config.h"
#include "bsp_api.h"
#include "bsp_drv_id.h"
#include "bsp_topcrg.h"
#include "drv_gpio_api.h"
#include "drv_gpio.h"
#include "log_msg.h"
#include "osp_interrupt.h"
#if defined(CONFIG_BSP_TIANHE)
#include "bsp_gpio_hp.h"
#endif

#define GPIO_CTRL_DEV_ID_BASE   DEVICE_GPIO0
#define PIN_TO_DEV_ID(pin)      (GPIO_CTRL_DEV_ID_BASE + (pin) / GPIO_PIN_COUNT)
#define PIN_TO_OFFSET(pin)      ((pin) % GPIO_PIN_COUNT)

/**
 * @brief 获取GPIO设备SBR配置
 * @param[in] devId 设备ID
 * @param[out] gpioSbrCfg GPIO SBR配置指针
 * @return EXIT_SUCCESS 成功
 * @return -EINVAL 参数错误或配置无效
 */
static S32 gpioDevCfgGet(DevList_e devId, SbrGpioCfg_s *gpioSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (gpioSbrCfg == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (devSbrRead(devId, gpioSbrCfg, 0, sizeof(SbrGpioCfg_s)) != sizeof(SbrGpioCfg_s)) {
        ret = -EINVAL;
        goto exit;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("%s-%d: SBR dump regAddr=%p irqNo=%u irqPrio=%u totalPinCnt=%u dir=0x%08x dataOut=0x%08x",
         __func__, __LINE__, gpioSbrCfg->regAddr, gpioSbrCfg->irqNo, gpioSbrCfg->irqPrio,
         gpioSbrCfg->totalPinCnt, gpioSbrCfg->dir, gpioSbrCfg->dataOut);
#endif

    if (gpioSbrCfg->totalPinCnt == 0 || gpioSbrCfg->irqNo == 0 || gpioSbrCfg->irqPrio == 0 || \
        gpioSbrCfg->regAddr == NULL) {
        ret = -EINVAL;
        goto exit;
    }

exit:
    return ret;
}

/**
 * @brief 获取GPIO控制寄存器指针
 * @param[in] pin GPIO引脚号
 * @param[out] pCtrlReg GPIO控制寄存器指针
 * @return EXIT_SUCCESS 成功
 * @return -EINVAL 参数错误
 * @return -EIO 获取驱动失败
 */
static S32 getCtrlReg(U32 pin, GpioReg_s **pCtrlReg)
{
    S32 ret = EXIT_SUCCESS;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);

    if (pCtrlReg == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if(getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (pin >= pDrvData->sbrCfg.totalPinCnt) {
        ret = -EINVAL;
        goto exit;
    }

    if (NULL == pDrvData->sbrCfg.regAddr) {
        ret = -EINVAL;
        goto exit;
    }

    *pCtrlReg = (volatile GpioReg_s*)pDrvData->sbrCfg.regAddr;

exit:
    return ret;
}


S32 gpioValueSet(U32 pin, GpioValue_e value)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);

    if ((value != GPIO_LOW_LEVEL && value != GPIO_HIGH_LEVEL)) {
        LOGE("%s-%d: arg error pin=%u value=%d", __func__, __LINE__, pin, value);
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    switch (value) {
        case GPIO_LOW_LEVEL:
            CLR_BIT(pCtrlReg->dataOut, pinIdx);
            break;
        case GPIO_HIGH_LEVEL:
            SET_BIT(pCtrlReg->dataOut, pinIdx);
            break;
        default:
            ret = -EINVAL;
            goto unlock;
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 gpioValueGet(U32 pin, GpioValue_e *pVal)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);

    if (pVal == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    *pVal = BIT_IS_ONE(pCtrlReg->dataIn, PIN_TO_OFFSET(pin)) ? GPIO_HIGH_LEVEL : GPIO_LOW_LEVEL;

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 gpioDirSet(U32 pin, GpioDir_e dir)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);

    if ((dir != GPIO_DIR_IN && dir != GPIO_DIR_OUT)) {
        LOGE("%s-%d: arg error pin=%u dir=%d", __func__, __LINE__, pin, dir);
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    switch (dir) {
    case GPIO_DIR_OUT:
        SET_BIT(pCtrlReg->dir, pinIdx);
        break;
    case GPIO_DIR_IN:
        CLR_BIT(pCtrlReg->dir, pinIdx);
        break;
    default:
        ret = -EINVAL;
        goto unlock;
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 gpioDirGet(U32 pin, GpioDir_e *pDir)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);

    if (pDir == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    *pDir = BIT_IS_ONE(pCtrlReg->dir, pinIdx) ? GPIO_DIR_OUT : GPIO_DIR_IN;

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 gpioInterruptEnable(U32 pin)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    SET_BIT(pCtrlReg->irqEn, pinIdx);

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 gpioInterruptDisable(U32 pin)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    CLR_BIT(pCtrlReg->irqEn, pinIdx);

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

/**
 * @brief 清除GPIO中断
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -ENODEV: 没有这个外设
 *         -EBUSY: 资源或者外设被占用
 *         -EINVAL: 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInterruptClear(U32 pin)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    SET_BIT(pCtrlReg->eoi, pinIdx);

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

/**
 * @brief 设置GPIO中断模式（内部函数，不加锁）
 * @param[in] pin GPIO引脚号
 * @param[in] type GPIO中断类型
 * @return EXIT_SUCCESS 成功
 * @return -EINVAL 参数错误
 */
static S32 __gpioInterruptModeSet(U32 pin, GpioIrqType_e type)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    U8 pinIdx = PIN_TO_OFFSET(pin);

    if (type < GPIO_INT_LOW || type > GPIO_INT_BOTHEDGE) {
        ret = -EINVAL;
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto exit;
    }

    ///< mask intrs(0 - enable intr. 1 - disable intr)
    CLR_BIT(pCtrlReg->irqMask, pinIdx);
    SET_BIT(pCtrlReg->debounce, pinIdx);
    switch (type) {
    case GPIO_INT_RISING:
        SET_BIT(pCtrlReg->irqType, pinIdx);
        SET_BIT(pCtrlReg->irqPol, pinIdx);
        CLR_BIT(pCtrlReg->intBothedge, pinIdx);
        break;
    case GPIO_INT_FALLING:
        SET_BIT(pCtrlReg->irqType, pinIdx);
        CLR_BIT(pCtrlReg->irqPol, pinIdx);
        CLR_BIT(pCtrlReg->intBothedge, pinIdx);
        break;
    case GPIO_INT_BOTHEDGE:
        SET_BIT(pCtrlReg->irqType, pinIdx);
        SET_BIT(pCtrlReg->intBothedge, pinIdx);
        break;
    case GPIO_INT_HIGH:
        CLR_BIT(pCtrlReg->irqType, pinIdx);
        SET_BIT(pCtrlReg->irqPol, pinIdx);
        CLR_BIT(pCtrlReg->intBothedge, pinIdx);
        break;
    case GPIO_INT_LOW:
        CLR_BIT(pCtrlReg->irqType, pinIdx);
        CLR_BIT(pCtrlReg->irqPol, pinIdx);
        CLR_BIT(pCtrlReg->intBothedge, pinIdx);
        break;
    default:
        ret = -EINVAL;
        goto exit;
    }

exit:
    return ret;
}

S32 gpioInterruptModeSet(U32 pin, GpioIrqType_e type)
{
    S32 ret = EXIT_SUCCESS;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    ret = __gpioInterruptModeSet(pin, type);
    if (ret != EXIT_SUCCESS) {
        goto unlock;
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 gpioInterruptModeGet(U32 pin, GpioIrqType_e *pIrqType)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);

    if (pIrqType == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    U32 type = BIT_IS_ONE(pCtrlReg->irqType, pinIdx);
    U32 polarity = BIT_IS_ONE(pCtrlReg->irqPol, pinIdx);
    U32 isBoth = BIT_IS_ONE(pCtrlReg->intBothedge, pinIdx);
    if (type) {
        if (isBoth) {
            *pIrqType = GPIO_INT_BOTHEDGE;
            goto unlock;
        }
        if (polarity) {
            *pIrqType = GPIO_INT_RISING;
        } else {
            *pIrqType = GPIO_INT_FALLING;
        }
    } else {
        if (polarity) {
            *pIrqType = GPIO_INT_HIGH;
        } else {
            *pIrqType = GPIO_INT_LOW;
        }
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 gpioInterruptMask(U32 pin)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    SET_BIT(pCtrlReg->irqMask, pinIdx);

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

/**
 * @brief 取消屏蔽GPIO中断
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -ENODEV: 没有这个外设
 *         -EBUSY: 资源或者外设被占用
 *         -EINVAL: 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInterruptUnmask(U32 pin)
{
    S32 ret = EXIT_SUCCESS;
    GpioReg_s *pCtrlReg = NULL;
    GpioDrvData_s *pDrvData = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    CLR_BIT(pCtrlReg->irqMask, pinIdx);

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

/**
 * @brief GPIO中断处理函数
 * @param[in] pDrvData GPIO驱动数据指针
 */
static void gpioIrqHandler(GpioDrvData_s *pDrvData)
{
    U32 pinIdx = 0;
    U32 stat = 0;
#if defined(CONFIG_BSP_TIANHE)
    U32 pinHpHighBitmap = 0;
    U32 pinHpLowBitmap = 0;
#endif

    if (pDrvData == NULL) {
        LOGE("%s-%d: pDrvData is NULL", __func__, __LINE__);
        return;
    }

    stat = ((volatile GpioReg_s*)pDrvData->sbrCfg.regAddr)->irqStat;

    if (stat == 0) {
        return;
    }
#if defined(CONFIG_BSP_TIANHE)
    gpioHpIntPinGet(&pinHpHighBitmap, &pinHpLowBitmap);
    if (pDrvData->devId == DEVICE_GPIO0) {
        stat = stat & (~pinHpLowBitmap);
    } else if (pDrvData->devId == DEVICE_GPIO1) {
        stat = stat & (~pinHpHighBitmap);
    }
#endif

    ///< get irqStat which bit set
    pinIdx = GPIO_MAX_BIT_INDEX_REG - __builtin_clz(stat);

    SET_BIT(((volatile GpioReg_s*)pDrvData->sbrCfg.regAddr)->eoi, pinIdx); ///< clear irq
    if (pDrvData->irqCbList[pinIdx].cb != NULL) {
        pDrvData->irqCbList[pinIdx].cb(pDrvData->irqCbList[pinIdx].arg);
    }
}

S32 gpioCallbackRegister(U32 pin,
        GpioIrqType_e type, gpioIrqCallBack callback, void *arg)
{
    S32 ret = EXIT_SUCCESS;
    GpioDrvData_s *pDrvData = NULL;
    GpioReg_s *pCtrlReg = NULL;
    DevList_e devId = PIN_TO_DEV_ID(pin);
    U8 pinIdx = PIN_TO_OFFSET(pin);
    U8 intStat = 0;

    if (type < GPIO_INT_LOW || type > GPIO_INT_BOTHEDGE) {
        ret = -EINVAL;
        goto exit;
    }

    if (callback == NULL) {
        LOGE("%s-%d: callback is NULL", __func__, __LINE__);
        ret = -EINVAL;
        goto exit;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    if (getCtrlReg(pin, &pCtrlReg) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    intStat = pCtrlReg->irqEn;
    CLR_BIT(pCtrlReg->irqEn, pinIdx); ///< 关闭中断
    pDrvData->irqCbList[pinIdx].cb = callback;
    pDrvData->irqCbList[pinIdx].arg = arg;
    __gpioInterruptModeSet(pin, type);

    if (BIT_IS_ONE(intStat, pinIdx)) {
        SET_BIT(pCtrlReg->irqEn, pinIdx); ///< 开启中断
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}

S32 gpioInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    GpioDrvData_s *pDrvData = NULL;

    ///< lock
    if (devLockByDriver(devId, GPIO_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    ///< Check driver match
    if (!isDrvMatch(devId, DRV_ID_STARS_GPIO)) {
        ret = -ENODEV;
        goto unlock;
    }

    if(isDrvInit(devId) == true) {
        ret = -EBUSY;
        goto unlock;
    }

    pDrvData = (GpioDrvData_s*)calloc(1, sizeof(GpioDrvData_s));
    if(pDrvData == NULL) {
        LOGE("%s-%d: alloc memory failed", __func__, __LINE__);
        ret = -ENOMEM;
        goto unlock;
    }

    if (gpioDevCfgGet(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s-%d: get sbr failed", __func__, __LINE__);
        ret = -EIO;
        goto freeMem;
    }

    pDrvData->devId = devId;
    ret = ospInterruptHandlerInstall(pDrvData->sbrCfg.irqNo, "gpio", OSP_INTERRUPT_UNIQUE,
        (OspInterruptHandler)gpioIrqHandler, pDrvData);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s-%d: irq handler already installed", __func__, __LINE__);
    } else if (ret == OSP_SUCCESSFUL) {
        ospInterruptVectorEnable((pDrvData->sbrCfg.irqNo));
    } else {
        LOGE("%s-%d: irq handler install failed", __func__, __LINE__);
        ret = -EXIT_FAILURE;
        goto freeMem;
    }

    ///< set dir and dataOut
    ((volatile GpioReg_s*)pDrvData->sbrCfg.regAddr)->dir = pDrvData->sbrCfg.dir;
    ((volatile GpioReg_s*)pDrvData->sbrCfg.regAddr)->dataOut = pDrvData->sbrCfg.dataOut;

    if(drvInstall(devId, pDrvData) != EXIT_SUCCESS) {
        LOGE("%s-%d: drv install failed", __func__, __LINE__);
        ret = -EIO;
        goto freeMem;
    }

    ret = peripsReset(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO)  {
        LOGE("%s-%d: reset failed", __func__, __LINE__);
        ret = -EIO;
        goto unInstall;
    }

    ///< 有些项目gpio没有时钟开关，如果没有，则peripsClockEnable返回-ENXIO
    ret = peripsClockEnable(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO)  {
        LOGE("%s-%d: clock enable failed", __func__, __LINE__);
        ret = -EIO;
        goto unInstall;
    }

    ret = EXIT_SUCCESS;
    goto unlock;

unInstall:
    if (isDrvInit(devId)) {
        ///< 如果驱动已注册，则调用drvUninstall来清理（包括释放sgpioDrvData）
        if (drvUninstall(devId) != EXIT_SUCCESS) {
            LOGE("%s-%d: drvUninstall failed during cleanup", __func__, __LINE__);
        }
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

S32 gpioDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    GpioDrvData_s *pDrvData = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_GPIO, (void **)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    ospInterruptVectorDisable((pDrvData->sbrCfg.irqNo));
    ret = ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo, (OspInterruptHandler)gpioIrqHandler,
        pDrvData);
    if (ret != OSP_SUCCESSFUL) {
        LOGE("%s-%d: irq handler remove failed ret=%d", __func__, __LINE__, ret);
        goto unlock;
    }

    ret = peripsReset(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO)  {
        LOGE("%s-%d: reset failed", __func__, __LINE__);
        ret = -EIO;
    }

    ret = peripsClockDisable(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO)  {
        LOGE("%s-%d: clock disable failed", __func__, __LINE__);
        ret = -EIO;
    }

    ret = drvUninstall(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s-%d: Devid=%u Deinit failed", __func__, __LINE__, devId);
    }

unlock:
    funcRunEndHelper(devId);

exit:
    return ret;
}
