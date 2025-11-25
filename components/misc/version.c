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

//#include "config.h"
#include "bsp_version.h"
#include "version.h"
#include "auto_version.h"
#include "log_msg.h"
#include <rtems/version.h>

// driver版本号
#define DRIVER_VERSION_MAJOR 1
#define DRIVER_VERSION_MINOR 0
#define DRIVER_VERSION_PATCH 0

// 将数字转换为字符串
#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)

// 生成BSP版本字符串, 由每个bsp定义, bsp_config.h
#define BSP_VERSION_STR "v" \
    STRINGIFY(BSP_VERSION_MAJOR) "." \
    STRINGIFY(BSP_VERSION_MINOR) "." \
    STRINGIFY(BSP_VERSION_PATCH)

// 生成driver版本字符串
#define DRIVER_VERSION_STR "v" \
    STRINGIFY(DRIVER_VERSION_MAJOR) "." \
    STRINGIFY(DRIVER_VERSION_MINOR) "." \
    STRINGIFY(DRIVER_VERSION_PATCH)

// OS版本信息, 通过docker中rtems内核函数获取
static VersionInfo_s osVersion;

// BSP版本信息
static VersionInfo_s bspVersion = {
    .major = BSP_VERSION_MAJOR,
    .minor = BSP_VERSION_MINOR,
    .patch = BSP_VERSION_PATCH,
    .str = BSP_VERSION_STR,
};

// driver版本信息
static VersionInfo_s drvVersion = {
    .major = DRIVER_VERSION_MAJOR,
    .minor = DRIVER_VERSION_MINOR,
    .patch = DRIVER_VERSION_PATCH,
    .str = DRIVER_VERSION_STR,
};

const VersionInfo_s* getOsVersion(void)
{
    osVersion.major = (U8)rtems_version_major();
    osVersion.minor = (U8)rtems_version_minor();
    osVersion.patch = (U8)rtems_version_revision();
    snprintf(osVersion.str, sizeof(osVersion.str), "v%d.%d.%d", osVersion.major,
            osVersion.minor, osVersion.patch);
    return &osVersion;
}

const char* getOsVersionStr(void)
{
    snprintf(osVersion.str, sizeof(osVersion.str), "v%d.%d.%d", rtems_version_major(),
            rtems_version_minor(), rtems_version_revision());
    return osVersion.str;
}

const VersionInfo_s* getBspVersion(void)
{
    return &bspVersion;
}

const char* getBspVersionStr(void)
{
    return bspVersion.str;
}

const VersionInfo_s* getDriverVersion()
{
    return &drvVersion;
}

const char* getDriverVersionStr()
{
    return drvVersion.str;
}

void socShowVersionInfo(void)
{
    LOGE("\r\n-------------------------------------------\r\n");
    LOGE("Soc Versions:\n");
    LOGE("RTEMS:  %s\n", getOsVersionStr());
    LOGE("BSP:    %s\n", getBspVersionStr());
    LOGE("Driver: %s (%s)\n", getDriverVersionStr(), MINISOC_GIT_VERSION);
    LOGE("----------------------------------------------\r\n");
}
