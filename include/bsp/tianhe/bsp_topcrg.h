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

#endif ///< __BSP_TOPCRG_H__
