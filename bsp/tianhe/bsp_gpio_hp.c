/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_gpio_hp.c
 * @author zhaorui@starsmicrosystem.com
 * @date 2025/11/10
 * @brief tianhe hotplug gpio driver
 */

#include "common_defines.h"
#include "log_msg.h"
#include "bsp_gpio_hp.h"

/**
 * @brief 设置GPIO热插拔中断引脚
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpIntPinSet(U32 pin)
{
    S32 ret = EXIT_SUCCESS;

    switch(pin / GPIO_GRP_MAX_PIN_NUM) {
    case 0:
        SET_BIT(GPIO_0_HP_REG, pin % GPIO_GRP_MAX_PIN_NUM);
        break;
    case 1:
        SET_BIT(GPIO_1_HP_REG, pin % GPIO_GRP_MAX_PIN_NUM);
        break;
    default:
        LOGE("%s: pin number error\r\n", __func__);
        ret = -EXIT_FAILURE;
        break;
    } 

    return ret;
}


/**
 * @brief 禁用GPIO热插拔中断引脚
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpIntPinDisable(U32 pin)
{
    S32 ret = EXIT_SUCCESS;

    switch(pin / GPIO_GRP_MAX_PIN_NUM) {
    case 0:
        CLR_BIT(GPIO_0_HP_REG, pin % GPIO_GRP_MAX_PIN_NUM);
        break;
    case 1:
        CLR_BIT(GPIO_1_HP_REG, pin % GPIO_GRP_MAX_PIN_NUM);
        break;
    default:
        LOGE("%s: pin number error\r\n", __func__);
        ret = -EXIT_FAILURE;
        break;
    } 

    return ret;
}

/**
 * @brief 获取GPIO热插拔中断引脚bitmap
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpIntPinGet(U32 *pHpPinHighBitmap, U32 *pHpPinLowBitmap)
{
    S32 ret = EXIT_SUCCESS;
    if ((pHpPinHighBitmap == NULL) || (pHpPinLowBitmap == NULL)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
#if defined(CONFIG_BSP_TIANHE)
    *pHpPinHighBitmap = GPIO_1_HP_REG;
    *pHpPinLowBitmap = GPIO_0_HP_REG;
#else
    *pHpPinHighBitmap = 0;
    *pHpPinLowBitmap = 0;
#endif

exit:
    return ret;
}

/**
 * @brief 使能GPIO热插拔控制模式
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpCtrlModeEnable(U32 pin)
{
    S32 ret = EXIT_SUCCESS;
    switch(pin / GPIO_GRP_MAX_PIN_NUM) {
    case 0:
        SET_BIT(GPIO_0_HP_INT_REG, pin % GPIO_GRP_MAX_PIN_NUM);
        break;
    case 1:
        SET_BIT(GPIO_1_HP_INT_REG, pin % GPIO_GRP_MAX_PIN_NUM);
        break;
    default:
        LOGE("%s: pin number error\r\n", __func__);
        ret = -EXIT_FAILURE;
        break;
    } 
    return ret;
}

/**
 * @brief 禁用GPIO热插拔控制模式
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpCtrlModeDisable(U32 pin)
{
    S32 ret = EXIT_SUCCESS;
    switch(pin / GPIO_GRP_MAX_PIN_NUM) {
    case 0:
        CLR_BIT(GPIO_0_HP_INT_REG, pin % GPIO_GRP_MAX_PIN_NUM);
        break;
    case 1:
        CLR_BIT(GPIO_1_HP_INT_REG, pin % GPIO_GRP_MAX_PIN_NUM);
        break;
    default:
        LOGE("%s: pin number error\r\n", __func__);
        ret = -EXIT_FAILURE;
        break;
    } 
    return ret;
}