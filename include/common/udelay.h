/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file udelay.h
 * @author liugh2@starsmicrosystem.com
 * @date 2021.06.11
 * @brief 无
 * @note 无
 * @version v1.0
 */

#ifndef __UDELAY_H__
#define __UDELAY_H__

#include <stdint.h>
#include <stdio.h>

#define SYS_TICK_PER_SECOND (1000 / rtems_configuration_get_milliseconds_per_tick())

/**
 * @brief Delays execution for a specified number of microseconds.
 * @details This function creates a busy-wait loop that blocks the CPU for the given number of microseconds.
 *          It is typically used for short, precise delays where context switching is not desired.
 * @param [in] usecs The number of microseconds to delay.
 * @note The actual delay time may vary depending on system clock and implementation.
 */
void udelay(uint32_t usecs);

/**
 * @brief Delays execution for a specified number of milliseconds.
 * @details This function creates a busy-wait loop that blocks the CPU for the given number of milliseconds.
 *          It is typically used for short, precise delays where context switching is not desired.
 * @param [in] msecs The number of milliseconds to delay.
 * @note The actual delay time may vary depending on system clock and implementation.
 */
void mdelay(uint32_t msecs);

/**
 * @brief Sleeps for a specified number of milliseconds.
 * @details This function puts the current task to sleep for the specified duration, allowing other tasks to run.
 *          It is typically used for longer delays where context switching is acceptable.
 * @param [in] msecs The number of milliseconds to sleep.
 * @note The actual sleep time may vary depending on system clock and task scheduling.
 */
void msleep(uint32_t msecs);

/**
 * @brief Gets the current system tick count.
 * @details This function returns the number of ticks since the system started.
 *          It is typically used for measuring time intervals or scheduling tasks.
 * @return The current system tick count.
 */
uint32_t sysTickGet(void);

#endif // __UDELAY_H__
