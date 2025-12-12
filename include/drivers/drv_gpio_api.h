/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_gpio_api.h
 * @author zhaorui (zhaorui@starsmicrosystem.com)
 * @date 2025/11/7
 * @brief  gpio driver api header
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/10/22   zhaorui         the first version
 *
 */

#ifndef __DRV_GPIO_API_H__
#define __DRV_GPIO_API_H__

#include "common_defines.h"
#include "bsp_device.h"

typedef enum gpioValue {
    GPIO_LOW_LEVEL = 0,
    GPIO_HIGH_LEVEL,
} GpioValue_e;

typedef enum gpioDir {
    GPIO_DIR_IN,
    GPIO_DIR_OUT,
} GpioDir_e;

typedef enum gpioIntr {
    GPIO_INT_LOW,
    GPIO_INT_HIGH,
    GPIO_INT_RISING,
    GPIO_INT_FALLING,
    GPIO_INT_BOTHEDGE,
} GpioIrqType_e;

typedef void (*gpioIrqCallBack)(void *arg);

/**
 * @brief 设置GPIO引脚电平
 * @param[in] pin GPIO引脚号
 * @param[in] value GPIO引脚电平值
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioValueSet(U32 pin, GpioValue_e value);

/**
 * @brief 获取GPIO引脚电平
 * @param[in] pin GPIO引脚号
 * @param[out] pVal GPIO引脚电平值指针
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioValueGet(U32 pin, GpioValue_e *pVal);

/**
 * @brief 设置GPIO引脚方向
 * @param[in] pin GPIO引脚号
 * @param[in] dir GPIO引脚方向
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioDirSet(U32 pin, GpioDir_e dir);

/**
 * @brief 获取GPIO引脚方向
 * @param[in] pin GPIO引脚号
 * @param[out] pDir GPIO引脚方向指针
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioDirGet(U32 pin, GpioDir_e *pDir);

/**
 * @brief 使能GPIO中断
 * @param[in] pin GPIO引脚号
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInterruptEnable(U32 pin);

/**
 * @brief 禁用GPIO中断
 * @param[in] pin GPIO引脚号
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInterruptDisable(U32 pin);

/**
 * @brief 清除GPIO中断
 * @param[in] pin GPIO引脚号
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInterruptClear(U32 pin);

/**
 * @brief 设置GPIO中断模式
 * @param[in] pin GPIO引脚号
 * @param[in] type GPIO中断模式
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInterruptModeSet(U32 pin, GpioIrqType_e type);

/**
 * @brief 获取GPIO中断模式
 * @param[in] pin GPIO引脚号
 * @param[out] pIrqType GPIO中断模式指针
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInterruptModeGet(U32 pin, GpioIrqType_e *pIrqType);

/**
 * @brief 屏蔽GPIO中断
 * @param[in] pin GPIO引脚号
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInterruptMask(U32 pin);

/**
 * @brief 取消屏蔽GPIO中断
 * @param[in] pin GPIO引脚号
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInterruptUnmask(U32 pin);

/**
 * @brief 注册GPIO中断回调函数
 * @details 驱动内部做中断路由, 并在跳转callback前清除中断, 用户注册的callback
 *          无需判断中断IsPending及调用InterruptClear
 *          注册完中断后用户需如果需要屏蔽中断调用 gpioInterruptMask
 * @param[in] pin GPIO引脚号
 * @param[in] type 中断类型
 * @param[in] callback callback函数指针
 * @param[in] arg callback函数参数指针
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EINVAL 参数错误
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioCallbackRegister(U32 pin, GpioIrqType_e type,
            gpioIrqCallBack callback, void *arg);

/**
 * @brief GPIO模块初始化
 * @param[in] devId GPIO设备ID
 *                   DEVICE_GPIO0 -> init gpio 0 ~ 31
 *                   DEVICE_GPIO1 -> init gpio 32 ~ 63
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -ENOMEM 堆内存不足
 * @return -EIO I/O 错误
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioInit(DevList_e devId);

/**
 * @brief 注销GPIO初始化
 * @param[in] devId GPIO设备ID
 *                  DEVICE_GPIO0 -> init gpio 0 ~ 31
 *                  DEVICE_GPIO1 -> init gpio 32 ~ 63
 * @return EXIT_SUCCESS 成功
 * @return -ENODEV 没有这个外设
 * @return -EBUSY 外设或者资源被占用
 * @return -EIO I/O 错误
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 gpioDeInit(DevList_e devId);

#endif
