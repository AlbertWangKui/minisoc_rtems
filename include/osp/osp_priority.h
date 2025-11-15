/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_priority.h
 * @author  tianye@tianye.com
 * @date    2020.09.09
 * @brief   porting rtems priority
 * @note    NA
 */

#ifndef __OSP_PRIORITY_H__
#define __OSP_PRIORITY_H__

#ifdef __cplusplus
extern "C" {
#endif

///< 任务优先级
typedef enum OspTaskPriority {
    USER_TASK_PRIORITY_INVALID = 0, ///< 无效优先级
    USER_TASK_PRIORITY_LOWEST,      ///< 用户最低优先级
    USER_TASK_PRIORITY_LOWER,       ///< 用户次低优先级
    USER_TASK_PRIORITY_LOW,         ///< 用户低优先级
    USER_TASK_PRIORITY_NORMAL,      ///< 用户正常优先级
    USER_TASK_PRIORITY_HIGH,        ///< 用户高优先级
    USER_TASK_PRIORITY_HIGHER,      ///< 用户次高优先级
    USER_TASK_PRIORITY_HIGHEST,     ///< 用户最高优先级
    USER_TASK_PRIORITY_BUTT = 255,
} OspTaskPriority_e;

#ifdef __cplusplus
}
#endif

#endif
