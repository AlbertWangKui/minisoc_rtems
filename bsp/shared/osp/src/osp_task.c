/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_task.c
 * @author  tianye
 * @date    2020.08.31
 * @brief   封装rtems操作系统任务管理对外接口
 * @note    NA
 */

#include <stdio.h>

#include <osp_attr.h>
#include <osp_mode.h>
#include <osp_task.h>
#include <osp_priority.h>
#include <rtems/rtems/status.h>
#include <rtems/rtems/types.h>
#include <rtems/rtems/tasks.h>
#include <rtems/rtems/support.h>
#include <rtems/score/threadimpl.h>
#include "include/osp_inner_common.h"
#include <rtems/score/percpu.h>
#include <osp_cpuopts.h>

/**
 * @brief   创建任务
 * @param   name ,                task名称
 * @param   initialTaskPriority , task优先级
 * @param   stackSize ,           task栈大小
 * @param   initialModes ,        task初始化模式
 * @param   attributeSet ,        task属性，如全局/局部等
 * @param   id [out],             task的id
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskCreate(OspName name, OspTaskPriority_e initialTaskPriority, size_t stackSize,
                              OspMode initialModes, OspAttribute attributeSet, OspID *id)
{
    rtems_status_code ret;
    rtems_task_priority rtemsPriority;
    OspStatusCode_e ospRet;

    ospRet = ospConvertToRtemsPriority(initialTaskPriority, &rtemsPriority);
    if(ospRet != OSP_SUCCESSFUL) {
        printf("ospTaskCreate failed(%d).\n", ospRet);
        goto exit;
    }

    ret    = rtems_task_create(name, rtemsPriority, stackSize, initialModes, attributeSet, id);
    ospRet = ospConvertReturnValue(ret);

exit:
    return ospRet;
}

/**
 * @brief   设置任务名字
 * @param   id ,                task的id
 * @param   name ,              task名称
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSetTaskName(OspID id, OspName name)
{
    /*
     *  Don't even think about deleting a resource from an ISR.
     */
    Thread_Control    *the_thread;
    ISR_lock_Context   lock_context;

    if ( !rtems_is_name_valid( name ) )
        return OSP_INVALID_NAME;

    _Objects_Allocator_lock();

    the_thread = _Thread_Get( id, &lock_context );

    if ( the_thread == NULL)
    {
        _Objects_Allocator_unlock();
        return OSP_INVALID_ID;
    }

    _ISR_lock_ISR_enable( &lock_context );
    the_thread->Object.name.name_u32 = (uint32_t)name;
    _Objects_Allocator_unlock();

    return OSP_SUCCESSFUL;
}

/**
 * @brief   获取任务名字
 * @param   id ,                task的id
 * @param   name ,              task名称
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospGetTaskName(OspID id, OspName *name)
{
    /*
     *  Don't even think about deleting a resource from an ISR.
     */
    Thread_Control    *the_thread;
    ISR_lock_Context   lock_context;

    _Objects_Allocator_lock();

    the_thread = _Thread_Get( id, &lock_context );

    if ( the_thread == NULL)
    {
        _Objects_Allocator_unlock();
        return OSP_INVALID_ID;
    }

    _ISR_lock_ISR_enable( &lock_context );
    *name = the_thread->Object.name.name_u32;
    _Objects_Allocator_unlock();

    return OSP_SUCCESSFUL;
}

/**
 * @brief   获取任务id
 * @param   name ,      task名称
 * @param   node ,      指定搜索的节点，如SEARCH_ALL_NODES等
 * @param   id [out],   task的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskIdent(OspName name, uint32_t node, OspID *id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_ident(name, node, id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   获取指定任务是否就绪
 * @param   id ,                task的id
 * @param   bool ,              task是否就绪
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskIsReady(OspID id, bool *isReady)
{
    Thread_Control    *the_thread;
    ISR_lock_Context   lock_context;

    _Objects_Allocator_lock();

    the_thread = _Thread_Get( id, &lock_context );

    if ( the_thread == NULL)
    {
        _Objects_Allocator_unlock();
        return OSP_INVALID_ID;
    }

    _ISR_lock_ISR_enable( &lock_context );
    *isReady = _States_Is_ready(the_thread->current_state);
    _Objects_Allocator_unlock();

    return OSP_SUCCESSFUL;
}

/**
 * @brief   调用者获取自己的任务id
 * @param   NA
 * @return  OspID 任务id
 * @warning NA
 * @note    NA
 */
OspID ospTaskSelf(void)
{
    rtems_id ret;

    ret = rtems_task_self( );
    return (OspID)ret; ///< 当前只是一一映射，直接强制类型转换
}

/**
 * @brief   开始任务
 * @param   id ,                task的id
 * @param   entryPoint [in],    任务函数指针
 * @param   argument [in/out],  任务参数
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskStart(OspID id, OspTaskEntry entryPoint, OspTaskArgument argument)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_start(id, entryPoint, argument);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   重新开始任务
 * @param   id ,                task的id
 * @param   argument [in/out],  任务参数
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskRestart(OspID id, OspTaskArgument argument)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_restart(id, argument);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   删除任务
 * @param   id ,                task的id
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskDelete(OspID id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_delete(id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   任务退出
 * @param   NA
 * @return  NA
 * @warning NA
 * @note    NA
 */
void ospTaskExit(void)
{
    rtems_task_exit( );
    return;
}

/**
 * @brief   挂起任务
 * @param   id ,                task的id
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskSuspend(OspID id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_suspend(id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   恢复任务
 * @param   id ,                task的id
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskResume(OspID id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_resume(id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   检查任务是否被挂起
 * @param   id ,                task的id
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskIsSuspended(OspID id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_is_suspended(id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   设置任务优先级
 * @param   id ,              task的id
 * @param   newPriority ,     任务的新优先级
 * @param   oldPriority [out],任务的老优先级
 * @return  OspStatusCode_e       返回值
 * @warning 注意该接口与优先级继承策略（互斥锁、优先级继承信号量、优先级天花板信号量）的冲突，
 *          使用前找os对齐流程
 * @note    1、如果降低正在执行任务的优先级或者提高其他任务的优先级，且当前执行
 *          任务支持被抢占，会引发一次调度；
 *          2、新设置优先级和任务当前优先级相同，不会有人会改变也不会有调度发生；
 *          3、如果在支持优先级继承策略的互斥锁或者信号量中调用该接口，降低当前
 *          任务优先级不会马上发生，直到任务是否所有优先级继承策略的互斥锁或者信
 *          号量之后，该任务的优先级才会生效。这种情况下，设置高优先级则会立即生效。
 */
OspStatusCode_e ospTaskSetPriority(OspID id, OspTaskPriority_e newPriority, OspTaskPriority_e *oldPriority)
{
    rtems_status_code ret;
    rtems_task_priority rtemsNewPriority;
    rtems_task_priority rtemsOldPriority;
    OspStatusCode_e ospRet;

    ospRet = ospConvertToRtemsPriority(newPriority, &rtemsNewPriority);
    if(ospRet != OSP_SUCCESSFUL) {
        printf("ospTaskSetPriority failed(%d).\n", ospRet);
        goto exit;
    }

    ret    = rtems_task_set_priority(id, rtemsNewPriority, &rtemsOldPriority);
    ospRet = ospConvertReturnValue(ret);
    if(ospRet != OSP_SUCCESSFUL) {
        printf("rtems_task_set_priority failed(%d).\n", ospRet);
        goto exit;
    }

    ospRet = ospConvertToOspPriority(rtemsOldPriority, oldPriority);

exit:
    return ospRet;
}

/**
 * @brief   获取任务优先级
 * @param   id ,              task的id
 * @param   priority [out],   任务的优先级
 * @return  OspStatusCode_e       返回值
 * @warning 注意该接口与优先级继承策略（互斥锁、优先级继承信号量、优先级天花板信号量）的冲突，
 *          使用前找os对齐流程
 * @note    1、该接口获取的是任务当前真实优先级，如果任务因为优先级继承策略等被os
 *          动态调整过优先级，则为调整过后的优先级。
 *          2、问题场景：通过该接口获取到优先级后再设置。有可能该接口获取到的是被提升后的优先级，
 *          设置后，会导致被提升后的优先级设置到任务，而非想要的任务原始优先级被设置
 */
OspStatusCode_e ospTaskGetPriority(OspID id, OspTaskPriority_e *priority)
{
    rtems_status_code ret;
    rtems_id schedulerId;
    rtems_task_priority rtemsPriority;
    OspStatusCode_e ospRet;

    ret    = rtems_task_get_scheduler(id, &schedulerId);
    ospRet = ospConvertReturnValue(ret);
    if(ospRet != OSP_SUCCESSFUL) {
        goto exit;
    }

    ret    = rtems_task_get_priority(id, schedulerId, &rtemsPriority);
    ospRet = ospConvertReturnValue(ret);
    if(ospRet != OSP_SUCCESSFUL) {
        goto exit;
    }

    ospRet = ospConvertToOspPriority(rtemsPriority, priority);

exit:
    return ospRet;
}

/**
 * @brief   设置任务模式
 * @param   modeSet ,             任务模式
 * @param   mask ,                任务模式掩码
 * @param   previousModeSet [out],任务的旧模式
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskMode(OspMode modeSet, OspMode mask, OspMode *previousModeSet)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_mode(modeSet, mask, previousModeSet);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   任务休眠指定单位的ticks
 * @param   ticks ,       休眠ticks
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskWakeAfter(OspInterval ticks)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_wake_after(ticks);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   任务切换
 * @param
 * @return
 * @warning NA
 * @note    NA
 */
void ospTaskCondResched()
{
    (void)rtems_task_wake_after(RTEMS_YIELD_PROCESSOR);
}

/**
 * @brief   任务休眠到指定时间
 * @param   timeBuffer [in], 休眠截止时间
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskWakeWhen(OspTimeOfDay_s *timeBuffer)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_wake_when((rtems_time_of_day *)timeBuffer);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

#if 0
/**
 * @brief   获取任务调度器
 * @param   [in]  taskId：      任务ID
 * @param   [out] *schedulerId  调度器编号
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskGetScheduler(OspID taskId, OspID *schedulerId)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_get_scheduler(taskId, schedulerId);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   设置任务调度策略
 * @param   [in]  taskId：          任务ID
 * @param   [in]  schedulerId       调度器编号
 * @param   [in]  priority          任务优先级
 * @return  OspStatusCode_e       返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskSetScheduler(OspID taskId, OspID schedulerId,
                                    OspTaskPriority_e priority)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_set_scheduler(taskId, schedulerId, priority);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}
#endif

/**
 * @brief   获取任务亲核性
 * @param   taskId ,      任务ID
 * @param   cpusetSize ,  sizeof(cpu_set_t)
 * @param   cpuset [out], cpu集
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskGetAffinity(OspID taskId, size_t cpusetSize, cpu_set_t *cpuset)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_get_affinity(taskId, cpusetSize, cpuset);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   设置任务亲核性
 * @param   taskId ,      任务ID
 * @param   cpusetSize ,  sizeof(cpu_set_t)
 * @param   cpuset [in],  cpu集
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskSetAffinity(OspID taskId, size_t cpusetSize, const cpu_set_t *cpuset)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_set_affinity(taskId, cpusetSize, cpuset);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   Get scheduler of a task
 * @param   taskId ,      任务ID
 * @param   schedulerId,  schedulerId
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskGetScheduler(OspID taskId, OspID *schedulerId)
{
    (void)taskId;
    (void)schedulerId;
#if 0
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_get_scheduler(taskId, schedulerId);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
#endif
    return OSP_SUCCESSFUL;
}

/**
 * @brief   Set scheduler of a task
 * @param   taskId ,      任务ID
 * @param   schedulerId,  schedulerId
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskSetScheduler(OspID taskId, OspID schedulerId, OspTaskPriority_e priority)
{
    (void)taskId;
    (void)schedulerId;
    (void)priority;
#if 0
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_task_set_scheduler(taskId, schedulerId, priority);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
#endif
    return OSP_SUCCESSFUL;
}

#if 0
/**
 * @brief   任务迭代
 * @param   [in]  visitor：     用戶函數
 * @param   [in]  *arg          sizeof(cpu_set_t)
 * @return  void
 * @warning NA
 * @note    暂时无需求不提供
 */
void ospTaskIterate(OspTaskVisitor visitor, void *arg)
{
    rtems_task_iterate(visitor, arg);
    return;
}
#endif

/**
 * @brief   Disables thread dispatching.
 * @return  OspPerCPUControl, per cpu control
 * @warning NA
 * @note    return The current processor
 */
OspPerCPUControl ospThreadDispatchDisable(void)
{
    return (OspPerCPUControl)_Thread_Dispatch_disable();
}

/**
 * @brief   Enables thread dispatching.
 * @param   cpuSelf, The current processor per cpu control
 * @return
 * @warning NA
 * @note    May perform a thread dispatch if necessary as a side-effect.
 */
void ospThreadDispatchEnable(OspPerCPUControl cpuSelf)
{
    _Thread_Dispatch_enable((Per_CPU_Control*)cpuSelf);
}

#if defined(PS3OS_COREDUMP) && !defined(PS3OS_SMP)
static bool in_task_stack(const uint32_t *p, Thread_Control *tsk)
{
    Stack_Control *sc;

    sc = &(tsk->Start.Initial_stack);

    return (uint32_t) p > (uint32_t) sc->area
        && (uint32_t) p < (uint32_t) sc->area + (uint32_t) sc->size;
}
#endif

/**
 * @brief   Get interrupted task backtrace.
 * @param   task backtrace info
 * @return
 * @warning only can be used in isr
 * @note    none
 */
OspStatusCode_e ospGetInterruptedTaskBt(OspTaskBt_s *taskBt)
{
#if defined(PS3OS_COREDUMP) && !defined(PS3OS_SMP)
    uint32_t counter;
    struct _Thread_Control *thread = _Thread_Executing;
    uint32_t *pc = (uint32_t *)(thread->temp_frame.register_pc);
    uint32_t *fp = (uint32_t *)(thread->temp_frame.register_r11);

    taskBt->name = thread->Object.name.name_u32;
    taskBt->id = thread->Object.id;

    taskBt->addr[0] = pc;

    for (counter = 1; counter < OSP_MAX_BACKTRACE_CNT; counter++) {
        if (!in_task_stack(fp, thread)) {
            break;
        }

        taskBt->addr[counter] = (uint32_t *) *(fp - 1);
        fp = (uint32_t *) *(fp - 3);
    }

    taskBt->cnt = counter;

    return OSP_SUCCESSFUL;
#else
    (void)taskBt;
    return OSP_NOT_IMPLEMENTED;
#endif
}
