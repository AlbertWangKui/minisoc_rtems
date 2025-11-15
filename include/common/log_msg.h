/**
 * copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd.
 *
 * @file log_msg.h
 * @author shaogl@starsmicrosystem.com
 * @date 2023/03/17
 * @brief log_msg相关实现
 */

#ifndef __LOG_MSG_H__
#define __LOG_MSG_H__

#include "common_defines.h"

#define LOGF(format, ...) portLogOutput(LOG_LEVEL_FATAL, format, ##__VA_ARGS__)
#define LOGE(format, ...) portLogOutput(LOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#define LOGW(format, ...) portLogOutput(LOG_LEVEL_WARN, format, ##__VA_ARGS__)
#define LOGI(format, ...) portLogOutput(LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define LOGD(format, ...) portLogOutput(LOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define LOGT(format, ...) portLogOutput(LOG_LEVEL_TRACE, format, ##__VA_ARGS__)

typedef enum LogLevel {
    LOG_LEVEL_FATAL,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_TRACE,
    LOG_LEVEL_MAX,
} LogLevel_e;

/*
 * @brief 获取日志等级
 * @param [in] none
 * @return 日志等级
 * @warning 阻塞；可以重入；可用于中断上下文；可以用于线程上下文
 */
LogLevel_e logMsgLevelGet(void);

/*
 * @brief 设置日志等级
 * @param [in] level 日志等级
 * @param [out] none
 * @warning 阻塞；可以重入；可用于中断上下文；可以用于线程上下文
 */
void logMsgLevelSet(LogLevel_e level);

/*
 * @brief 日志输出接口，该接口具有weak属性，soc提供了一个默认的实现，用户需要重写该接口.
 * @param [in] level 日志等级
 * @param [in] format 日志的可变参数
 * @param [out] none
 * @warning 阻塞；可以重入；可用于中断上下文；可以用于线程上下文
 */
__attribute__((weak)) void portLogOutput(LogLevel_e level, const S8 *format, ...);

#endif ///< __LOG_MSG_H__

