/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_wdt_api.h
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/06/05
 * @brief 看门口api定义
 */

#ifndef __DRV_WDT_API_H__
#define __DRV_WDT_API_H__

#include "common_defines.h"
#include "bsp_device.h"

///< wdt中断模式: 复位/中断
typedef enum WdtMode
{
    WDT_RESET = 0,
    WDT_INT = 1,
    WDT_MODE_NR,
} WdtMode_e;

typedef void (*pWdtCallback)(void);

/**
 * @brief wdt模块初始化
 * @details 该函数用于初始化wdt模块
 * @param [in] devId 设备ID
 * @param [in] callback 中断回调函数
 * @return EXIT_SUCCESS or EXIT_FAILURE
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 wdtInit(DevList_e devId, pWdtCallback callback);

/**
 * @brief wdt模块反初始化
 * @details 该函数用于反初始化wdt模块
 * @param [in] devId 设备ID
 * @return EXIT_SUCCESS or EXIT_FAILURE
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 wdtDeInit(DevList_e devId);

/**
 * @brief 启动wdt
 * @details 该函数用于启动wdt用户需要在调用该函数前确保wdt模块已初始化
 * @param [in] devId 设备ID
 * @return EXIT_SUCCESS or EXIT_FAILURE
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 wdtStart(DevList_e devId);

/**
 * @brief 停止wdt
 * @details 该函数用于停止wdt,用户需要在调用该函数前确保wdt模块已初始化
 * @param [in] devId 设备ID
 * @return EXIT_SUCCESS or EXIT_FAILURE
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 wdtStop(DevList_e devId);

/**
 * @brief 喂狗
 * @details 该函数用于喂狗，防止看门狗复位,用户需要在调用该函数前确保wdt模块已初始化
 * @param [in] devId 设备ID
 * @return EXIT_SUCCESS or EXIT_FAILURE
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 wdtFeed(DevList_e devId);

/**
 * @brief 清除wdt模块中断
 * @details 该函数用于清除wdt模块中断,用户需要在调用该函数前确保wdt模块已初始化
 * @param [in] devId 设备ID
 * @return EXIT_SUCCESS or EXIT_FAILURE
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 wdtIrqClr(DevList_e devId);

/**
 * @brief 设置wdt超时时间。
 * @details 该函数用于设置wdt超时时间,用户需要在调用该函数前确保wdt模块已初始化
 * @param [in] devId 设备ID
 * @param [in] timeOutMs 超时时间
 * @return EXIT_SUCCESS or EXIT_FAILURE
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 wdtSetTimeout(DevList_e devId, U32 timeOutMs);

/**
 * @brief 设置wdt中断mode
 * @details 该函数用于设置wdt中断mode,用户需要在调用该函数前确保wdt模块已初始化
 * @param [in] devId 设备ID
 * @param [in] mode 中断mode
 * @return EXIT_SUCCESS or EXIT_FAILURE
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 wdtSetMode(DevList_e devId, WdtMode_e mode);

#endif
