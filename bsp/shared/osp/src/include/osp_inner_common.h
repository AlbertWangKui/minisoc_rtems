/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_common.h
 * @author  tianye
 * @date    2020.08.29
 * @brief   libospporting模块内部公共函数头文件
 * @note    NA
 */

#ifndef _OSP_INNER_COMMON_H_
#define _OSP_INNER_COMMON_H_

#include <osp_priority.h>
#include <rtems/rtems/tasks.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct OspRtemsPriorityTable {
    OspTaskPriority_e ospPriority;
    rtems_task_priority rtemsPriority;
} OspRtemsPriorityTabl_s;

/**
 * @brief   将rtems接口的状态码转换成ps3项目的32位错误码
 * @param   statusCode ,rtems接口的状态码
 * @return  转换后的ps3项目32位错误码
 * @warning NA
 * @note    NA
 */
int ospConvertStatusCodeToErrno(rtems_status_code statusCode);

/**
 * @brief   rtems错误码和曙光os适配层错误码转换函数
 * @param   ret ,             rtems错误码
 * @return  OspStatusCode_e   转换后的os适配层错误码
 * @warning NA
 * @note    NA
 */
static inline OspStatusCode_e ospConvertReturnValue(rtems_status_code ret)
{
    return (OspStatusCode_e)ret; ///< 当前是一一映射，直接强制类型转换
}

/**
 * @brief   转换优先级
 * @param   ospTaskPriority ,       待转换优先级
 * @param   rtemsTaskPriority [out],转换后的任务优先级
 * @return  OspStatusCode_e         转换结果
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospConvertToRtemsPriority(OspTaskPriority_e ospTaskPriority, rtems_task_priority *rtemsTaskPriority);
/**
 * @brief   转换优先级(rtems to osp)
 * @param   rtemsTaskPriority ,             rtems任务优先级
 * @param   ospTaskPriority [out],          OSP任务优先级
 * @return  OspStatusCode_e                 转换结果
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospConvertToOspPriority(rtems_task_priority rtemsTaskPriority, OspTaskPriority_e *ospTaskPriority);
#ifdef __cplusplus
}
#endif
#endif
