/**
 * copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_version.c
 * @author taohb@starsmicrosystem.com
 * @date 2023/11/28
 * @brief version information
 * @note None
 * @version v1.0
 */

#ifndef _BSP_VERSION_H
#define _BSP_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  * @field major 主版本号 - 不兼容的API变更
 *   * @field minor 次版本号 - 向后兼容的功能性新增
 *    * @field patch 修订号   - 向后兼容的问题修复
 *     */
#define BSP_VERSION_MAJOR 1
#define BSP_VERSION_MINOR 0
#define BSP_VERSION_PATCH 0

#ifdef __cplusplus
}
#endif

#endif /* _BSP_VERSION_H */
