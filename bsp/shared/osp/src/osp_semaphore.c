/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_semaphore.c
 * @author  pengqianheng
 * @date    2020.08.29
 * @brief   封装rtems操作系统信号量对外接口
 * @note    NA
 */
#include <osp_attr.h>
#include <osp_options.h>
#include <osp_semaphore.h>
#include <osp_priority.h>
#include <rtems/rtems/sem.h>
#include <rtems/rtems/status.h>
#include "include/osp_inner_common.h"

/**
 * @brief   创建信号量
 * @param   name ,            信号量名称
 * @param   count ,           初始的信号量计数值
 * @param   attributeSet ,    信号量属性选项，参考 OSP_DEFAULT_ATTRIBUTES
 * @param   priorityCeiling , 优先级天花板
 * @param   id [out],         RTEMS内核分配的信号量ID
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospSemaphoreCreate(OspName name, uint32_t count, OspAttribute attributeSet,
                                   OspTaskPriority_e priorityCeiling, OspID *id)
{
    rtems_status_code ret;
    rtems_task_priority rtemsPriority;
    OspStatusCode_e ospResult;

    ospResult = ospConvertToRtemsPriority(priorityCeiling, &rtemsPriority);
    if(ospResult != OSP_SUCCESSFUL) {
        return ospResult;
    }

    ret       = rtems_semaphore_create(name, count, attributeSet, rtemsPriority, id);
    ospResult = ospConvertReturnValue(ret);

    return ospResult;
}

/**
 * @brief   删除信号量
 * @param   id , 信号量ID
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospSemaphoreDelete(OspID id)
{
    rtems_status_code ret;
    OspStatusCode_e ospResult;

    ret       = rtems_semaphore_delete(id);
    ospResult = ospConvertReturnValue(ret);

    return ospResult;
}

/**
 * @brief   获取信号量
 * @param   id ,        信号量ID
 * @param   optionSet , 选择是否要等待，参考 osp_options.h
 * @param   timeout ,   最大等待(阻塞)时间
 * @return  执行结果
 * @warning 无
 * @note    如果选择 optionSet 为 OSP_NO_WAIT ，那么本接口会立即返回
 */
OspStatusCode_e ospSemaphoreObtain(OspID id, OspOption optionSet, OspInterval timeout)
{
    rtems_status_code ret;
    OspStatusCode_e ospResult;

    ret       = rtems_semaphore_obtain(id, optionSet, timeout);
    ospResult = ospConvertReturnValue(ret);

    return ospResult;
}

/**
 * @brief   释放信号量
 * @param   id , 信号量ID
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospSemaphoreRelease(OspID id)
{
    rtems_status_code ret;
    OspStatusCode_e ospResult;

    ret       = rtems_semaphore_release(id);
    ospResult = ospConvertReturnValue(ret);

    return ospResult;
}
