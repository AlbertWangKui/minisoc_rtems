/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_std.h
 * @author    xiezhj
 * @date      2021.04.08
 * @brief     文件说明
 * @note
 */

#ifndef __CLI_PORTING_H__
#define __CLI_PORTING_H__

#include "auto_version.h"
#include "log_msg.h"

#define CLI_BUILD_TIME MINISOC_BUILD_TIME

#define CLI_DEFAULT_PASSWORD "stars"

///< 日志打印
#define CLI_LOG_FATAL(fmt, ...)                                       \
    LOGF(fmt, ##__VA_ARGS__)

#define CLI_LOG_ERROR(fmt, ...)                                       \
    LOGE(fmt, ##__VA_ARGS__)

#define CLI_LOG_WARN(fmt, ...)                                        \
    LOGW(fmt, ##__VA_ARGS__)

#define CLI_LOG_INFO(fmt, ...)                                        \
    LOGE(fmt, ##__VA_ARGS__)

#define CLI_LOG_DEBUG(fmt, ...)                                       \
    LOGD(fmt, ##__VA_ARGS__)

#define CLI_LOG_TRACE(fmt, ...)                                       \
    LOGT(fmt, ##__VA_ARGS__)

#define CLI_LOG_DATA(fmt, ...) printf(fmt, ##__VA_ARGS__)

#endif ///< __CLI_PORTING_H__

