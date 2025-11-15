/**
 * copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file version.c
 * @author taohb@starsmicrosystem.com
 * @date 2023/11/28
 * @brief version information
 * @note None
 * @version v1.0
 */

#ifndef _VERSION_H__
#define _VERSION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common_defines.h"

/**
 * @struct VersionInfo
 * @brief 版本信息数据结构体
 * 包含完整的语义化版本信息及构建元数据
 * @field major 主版本号 - 不兼容的API变更
 * @field minor 次版本号 - 向后兼容的功能性新增
 * @field patch 修订号   - 向后兼容的问题修复
 * @field str   完整版本字符串 (格式: "vX.Y.Z[+<metadata>]")
 */
typedef struct VersionInfo {
    U8 major;
    U8 minor;
    U8 patch;
    char str[16];    // 版本字符串
} VersionInfo_s;

/**
 * @brief 获取OS版本信息
 * 返回指向BSP版本信息结构体的常量指针
 * @return VersionInfo*  OS版本信息
 * @note 此函数永不返回NULL
 * @warning 返回的指针指向只读内存区域
 */
const VersionInfo_s* getOsVersion(void);

/**
 * @brief 获取OS版本字符串
 * 返回OS版本的字符串表示形式
 * @return char*  OS版本字符串 (格式: "vX.Y.Z")
 */
const char* getOsVersionStr(void);

/**
 * @brief 获取BSP版本信息
 * 返回指向BSP版本信息结构体的常量指针
 * @return VersionInfo*  BSP版本信息
 * @note 此函数永不返回NULL
 * @warning 返回的指针指向只读内存区域
 */
const VersionInfo_s* getBspVersion(void);

/**
 * @brief 获取BSP版本字符串
 * 返回BSP版本的字符串表示形式
 * @return char*  BSP版本字符串 (格式: "vX.Y.Z")
 */
const char* getBspVersionStr(void);

/**
 * @brief 获取驱动的版本信息
 * @return VersionInfo* 驱动版本信息指针
 * @warning 返回的指针指向只读内存区域
 */
const VersionInfo_s* getDriverVersion();

/**
 * @brief 获取驱动版本字符串
 * @return const char* 驱动版本字符串
 * @note char*  驱动版本字符串 (格式: "vX.Y.Z")
 */
const char* getDriverVersionStr();

/**
 * @brief 打印系统所有组件版本
 * 
 * 输出格式:
 *   System Versions:
 *     RTEMS: v5.1.0
 *     BSP: v2.1.3
 *     Driver: v1.0.7 (a1b2c3d)
 * @warning 此函数不是线程安全的（如果日志系统非线程安全）
 */
void socShowVersionInfo(void);

#ifdef __cplusplus
}
#endif

#endif /* _VERSION_H */
