/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    task_bootup.h
 * @author  lichenxiang
 * @date    2020.09.24
 * @brief   os 第一个任务启动实现
 * @note    NA
 */

#ifndef __TASK_BOOTUP_H__
#define __TASK_BOOTUP_H__

#include <osp_appinit.h>
#include <rtems/rtems/tasks.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIGURE_INIT
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_ENTRY_POINT ospTaskBootup
#define CONFIGURE_INIT_TASK_NAME        rtems_build_name('B', 'T', 'U', 'P')

#define CONFIGURE_TICKS_PER_TIMESLICE 2

/**
 * @brief   启动入口
 * @param   argument [in/out],  任务参数
 * @return  NA
 * @warning NA
 * @note    NA
 */
extern rtems_task ospTaskBootup(rtems_task_argument argument);

#ifdef __cplusplus
}
#endif

#endif
