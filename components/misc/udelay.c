/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file udelay.c
 * @author liugh2@starsmicrosystem.com
 * @date 2021.06.11
 * @brief none
 * @note none
 * @version v1.0
 */

#include "udelay.h"
#include "timer/drv_timer.h"
#include <string.h>
#include <rtems/counter.h>

extern volatile uint32_t Clock_driver_ticks; /* it is defined in kernel */

static void port_udelay(uint32_t usecs)
{
    uint32_t ns = 1000 * (uint32_t)usecs;

    _Assert((uint32_t)usecs <= UINT32_MAX / 1000);
    rtems_counter_delay_nanoseconds(ns);
}

/**
 * @brief timer忙等函数
 * @param [in] usecs 忙等时间,单位微秒
 * @param [out] none
 * @return none
 */
void udelay(uint32_t usecs)
{
    port_udelay(usecs);
}

/**
 * @brief timer忙等函数
 * @param [in] msecs 忙等时间,单位毫秒
 * @param [out] none
 * @return none
 */
void mdelay(uint32_t msecs)
{
    uint32_t i;

    for (i = 0; i < msecs; i++) {
        udelay(1000);
    }
}

void msleep(uint32_t msecs)
{
    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(msecs));
}

uint32_t sysTickGet(void)
{
    return Clock_driver_ticks;
}