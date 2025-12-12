/**
 * copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd.
 *
 * @file logmsg.c
 * @author shaogl@starsmicrosystem.com
 * @date 2023/03/17
 * @brief log_msg相关实现
 */

#include "uart/drv_uart.h"
#include <rtems/bspIo.h>
#include "log_msg.h"

#define COMMAND_INT_MAX_OUTPUT_SIZE (100)

static LogLevel_e logMsgLevel = LOG_LEVEL_INFO;

LogLevel_e logMsgLevelGet(void)
{
    return logMsgLevel;
}

void logMsgLevelSet(LogLevel_e level)
{
    if (level < LOG_LEVEL_MAX) {
        logMsgLevel = level;
    }
}

__attribute__((weak)) void portLogOutput(LogLevel_e level, const S8 *format, ...)
{
    va_list va;
    S8 printBufToQueue[COMMAND_INT_MAX_OUTPUT_SIZE] = { 0 };

    if (!format || (level >= LOG_LEVEL_MAX)) {
        return;
    }

    if (logMsgLevelGet() >= level) {
        va_start(va, format);
        vsnprintf(printBufToQueue, COMMAND_INT_MAX_OUTPUT_SIZE, format, va);
        va_end(va);

        if (rtems_interrupt_is_in_progress()) {
            printk("%s", printBufToQueue);
        } else {
            printf("%s", printBufToQueue);
        }
    }
}

