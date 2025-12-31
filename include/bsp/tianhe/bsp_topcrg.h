/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_topcrg.h
 * @author shaogl@starsmicrosystem.com
 * @date 2024/06/30
 * @brief top crg for special bsp.
 *
 * @author huyl3@starsmicrosystem.com
 * @date 2025/09/26
 * @brief add getHwPlatformType & getHwPlatformVersion
 */

#ifndef __BSP_TOPCRG_H__
#define __BSP_TOPCRG_H__

#include "common_defines.h"

typedef void (*pMinisocIdleIrqCallBack)(void *arg);

/**
 * @brief  get hardware platform type.
 * @details none
 * @param [in] none
 * @return platform type name, string.
 * @warning
 */
const U8 *getHwPlatformType(void);

/**
 * @brief  get hardware platform type.
 * @details none
 * @param [out] strBuffer will contain platform version string.
 * @param [in]  strLen strBuffer size.
 * @return string length which copied to strBuffer.
 * @warning
 */
U32 getHwPlatformVersion(U8 *strBuffer, U32 strLen);

/**
 * @brief minisoc idle初始化
 * @details minisoc idle初始化
 * @param [in] none
 * @param [inout] none
 * @return OSP_RESOURCE_IN_USE 或 OSP_SUCCESSFUL，失败返回-EXIT_FAILURE
 */
S32 minisocIdleInit(void);

/**
 * @brief 注册minisoc idle中断回调
 * @details 注册minisoc idle的复位握手中断
 * @param [in] callback, 表示回调函数
 * @param [in] callbackArg, 表示回调函数的入参
 * @param [inout] none
 * @return EXIT_SUCCESS 或 -EINVAL
 */
S32 minisocIdleCallbackRegister(pMinisocIdleIrqCallBack callback, void *callbackArg);

#endif ///< __BSP_TOPCRG_H__
