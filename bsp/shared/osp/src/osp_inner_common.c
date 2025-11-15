/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_common.c
 * @author  tianye
 * @date    2020.08.29
 * @brief   libospporting模块内部公共函数
 * @note    NA
 */

#include <osp_status.h>
#include <osp_priority.h>
#include <osp_errno.h>
#include <rtems/rtems/status.h>
#include <rtems/rtems/tasks.h>
#include "include/osp_inner_common.h"

#define RTEMS_PRIORITY_HIGHEST 2 ///< 最高优先级定义为操作系统中能够给用户使用的最高优先级
#define RTEMS_PRIORITY_HIGHER  3 ///< 次高优先级定义为操作系统中能够给用户使用的次高优先级
#define RTEMS_PRIORITY_HIGH    64
#define RTEMS_PRIORITY_NORMAL  128
#define RTEMS_PRIORITY_LOW     192
#define RTEMS_PRIORITY_LOWER   254 ///< 次低优先级定义为操作系统中能够给用户使用的次低优先级
#define RTEMS_PRIORITY_LOWEST  255 ///< 最低优先级定义为操作系统中能够给用户使用的最低优先级

OspRtemsPriorityTabl_s gPriorityChgTable[] = {
    {USER_TASK_PRIORITY_LOWEST, RTEMS_PRIORITY_LOWEST},   {USER_TASK_PRIORITY_LOWER, RTEMS_PRIORITY_LOWER},
    {USER_TASK_PRIORITY_LOW, RTEMS_PRIORITY_LOW},         {USER_TASK_PRIORITY_NORMAL, RTEMS_PRIORITY_NORMAL},
    {USER_TASK_PRIORITY_HIGH, RTEMS_PRIORITY_HIGH},       {USER_TASK_PRIORITY_HIGHER, RTEMS_PRIORITY_HIGHER},
    {USER_TASK_PRIORITY_HIGHEST, RTEMS_PRIORITY_HIGHEST},
};

static const int gStatusCodeToErrno[RTEMS_STATUS_CODES_LAST + 1] = {[OSP_SUCCESSFUL]               = 0,
                                                                    [OSP_TASK_EXITTED]             = OSP_EIO,
                                                                    [OSP_MP_NOT_CONFIGURED]        = OSP_EIO,
                                                                    [OSP_INVALID_NAME]             = OSP_EINVAL,
                                                                    [OSP_INVALID_ID]               = OSP_EIO,
                                                                    [OSP_TOO_MANY]                 = OSP_EIO,
                                                                    [OSP_TIMEOUT]                  = OSP_ETIMEDOUT,
                                                                    [OSP_OBJECT_WAS_DELETED]       = OSP_EIO,
                                                                    [OSP_INVALID_SIZE]             = OSP_EIO,
                                                                    [OSP_INVALID_ADDRESS]          = OSP_EIO,
                                                                    [OSP_INVALID_NUMBER]           = OSP_EBADF,
                                                                    [OSP_NOT_DEFINED]              = OSP_EIO,
                                                                    [OSP_RESOURCE_IN_USE]          = OSP_EBUSY,
                                                                    [OSP_UNSATISFIED]              = OSP_ENODEV,
                                                                    [OSP_INCORRECT_STATE]          = OSP_EIO,
                                                                    [OSP_ALREADY_SUSPENDED]        = OSP_EIO,
                                                                    [OSP_ILLEGAL_ON_SELF]          = OSP_EIO,
                                                                    [OSP_ILLEGAL_ON_REMOTE_OBJECT] = OSP_EIO,
                                                                    [OSP_CALLED_FROM_ISR]          = OSP_EIO,
                                                                    [OSP_INVALID_PRIORITY]         = OSP_EIO,
                                                                    [OSP_INVALID_CLOCK]            = OSP_EINVAL,
                                                                    [OSP_INVALID_NODE]             = OSP_EINVAL,
                                                                    [OSP_NOT_CONFIGURED]           = OSP_ENOSYS,
                                                                    [OSP_NOT_OWNER_OF_RESOURCE]    = OSP_EPERM,
                                                                    [OSP_NOT_IMPLEMENTED]          = OSP_ENOSYS,
                                                                    [OSP_INTERNAL_ERROR]           = OSP_EIO,
                                                                    [OSP_NO_MEMORY]                = OSP_ENOMEM,
                                                                    [OSP_IO_ERROR]                 = OSP_EIO,
                                                                    [OSP_INTERRUPTED]              = OSP_EINTR,
                                                                    [OSP_PROXY_BLOCKING]           = OSP_EIO};

/**
 * @brief   将rtems接口的状态码转换成ps3项目的32位错误码
 * @param   statusCode ,rtems接口的状态码
 * @return  转换后的ps3项目32位错误码
 * @warning NA
 * @note    NA
 */
int ospConvertStatusCodeToErrno(rtems_status_code statusCode)
{
    int eno = OSP_EINVAL;

    if((unsigned)statusCode <= OSP_STATUS_CODES_LAST) {
        eno = gStatusCodeToErrno[statusCode];
    }

    ///< 表中查询的是系统错误码，需要使用如下函数将其映射为OSP模块的KERNEL子模块的错误码
    eno = OSP_SYSNO(Kernel, eno);

    return eno;
}


/**
 * @brief   转换优先级(osp to rtems)
 * @param   ospTaskPriority ,       待转换优先级
 * @param   rtemsTaskPriority [out],转换后的任务优先级
 * @return  OspStatusCode_e                     转换结果
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospConvertToRtemsPriority(OspTaskPriority_e ospTaskPriority, rtems_task_priority *rtemsTaskPriority)
{
    int i;
    OspStatusCode_e ospConvertRet = OSP_NOT_DEFINED;

    if(rtemsTaskPriority == NULL) {
        return OSP_INVALID_ADDRESS;
    }

    for(i = 0; i < (int)(sizeof(gPriorityChgTable) / sizeof(gPriorityChgTable[0])); i++) {
        if(ospTaskPriority != gPriorityChgTable[i].ospPriority) {
            continue;
        }
        *rtemsTaskPriority = gPriorityChgTable[i].rtemsPriority;
        ospConvertRet      = OSP_SUCCESSFUL;
        break;
    }

    if(i == sizeof(gPriorityChgTable) / sizeof(gPriorityChgTable[0])) {
        ospConvertRet = OSP_NOT_DEFINED;
    }
    return ospConvertRet;
}

/**
 * @brief   转换优先级(rtems to osp)
 * @param   rtemsTaskPriority ,             rtems任务优先级
 * @param   ospTaskPriority [out],          OSP任务优先级
 * @return  OspStatusCode_e                 转换结果
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospConvertToOspPriority(rtems_task_priority rtemsTaskPriority, OspTaskPriority_e *ospTaskPriority)
{
    int i;
    OspStatusCode_e ospConvertRet = OSP_NOT_DEFINED;

    if(ospTaskPriority == NULL) {
        return OSP_INVALID_ADDRESS;
    }
    for(i = 0; i < (int)(sizeof(gPriorityChgTable) / sizeof(gPriorityChgTable[0])); i++) {
        if(rtemsTaskPriority != gPriorityChgTable[i].rtemsPriority) {
            continue;
        }
        *ospTaskPriority = gPriorityChgTable[i].ospPriority;
        ospConvertRet    = OSP_SUCCESSFUL;
        break;
    }

    if(i == sizeof(gPriorityChgTable) / sizeof(gPriorityChgTable[0])) {
        ospConvertRet = OSP_NOT_DEFINED;
    }
    return ospConvertRet;
}
