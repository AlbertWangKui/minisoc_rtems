/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_gpio_hp.h
 * @author zhaorui@starsmicrosystem.com
 * @date 2025/11/10
 * @brief tianhe hotplug gpio driver
 */

#ifndef __BSP_GPIO_HP_H__
#define __BSP_GPIO_HP_H__

#define GPIO_0_HP_REG  *(volatile U32*)0xBE100F20
#define GPIO_1_HP_REG  *(volatile U32*)0xBE100F24
#define GPIO_0_HP_INT_REG *(volatile U32*)0xB8041008
#define GPIO_1_HP_INT_REG *(volatile U32*)0xB8041408
#define GPIO_GRP_MAX_PIN_NUM 32

/**
 * @brief 设置GPIO热插拔中断引脚
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpIntPinSet(U32 pin);

/**
 * @brief 禁用GPIO热插拔中断引脚
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpIntPinDisable(U32 pin);

/**
 * @brief 获取GPIO热插拔中断引脚bitmap
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpIntPinGet(U32 *pHpPinHighBitmap, U32 *pHpPinLowBitmap);

/**
 * @brief 使能GPIO热插拔控制模式
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpCtrlModeEnable(U32 pin);

/**
 * @brief 禁用GPIO热插拔控制模式
 * @param [in] pin,GPIO引脚号
 * @param [out] none
 * @return EXIT_SUCCESS: 0
 *         -EXIT_FAILURE: -1
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioHpCtrlModeDisable(U32 pin);

#endif