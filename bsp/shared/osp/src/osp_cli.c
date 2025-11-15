/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_cli.c
 * @author  tianye
 * @date    2021.06.21
 * @brief   封装HOST端过来的os信息查询接口
 * @note    expander/switch本端的os相关命令，cli直接封装rtems的shell接口
 */

#include <string.h>
#include <osp_cli.h>
#include <rtems/config.h>
#include <rtems/score/heapinfo.h>
#include <rtems/score/protectedheap.h>
#include <rtems/score/percpu.h>
#include <rtems/irq-extension.h>
#include <rtems/counter.h>
#include <rtems/monitor.h>
#include <rtems/rtems/tasksdata.h>
#include <rtems/score/threadq.h>
#include <rtems/score/thread.h>
#include <rtems/score/threadimpl.h>
#include <rtems/score/schedulerimpl.h>
#include <rtems/score/todimpl.h>
#include <rtems/score/timestampimpl.h>
#include <rtems/score/cpu.h>
#include <rtems/score/percpu.h>
#include <rtems/score/object.h>
#include <rtems/coredump.h>
#include "include/osp_inner_common.h"
#include <bsp/irq.h>
#include <rtems/fatal.h>
#include <bsp/linker-symbols.h>
#include <bspopts.h>
#include <osp_cpuopts.h>
#include <rtems/fatal.h>
#include <bsp/linker-symbols.h>

typedef struct OspTaskInfoInner {
    uint32_t taskCnt;
    uint32_t curTaskCnt;
    Timestamp_Control uptimeFromLastRest;
    OspTaskInfo_s *taskInfo;
} OspTaskInfoInner_s;

typedef struct OspCpuCoreInfoInner {
    uint32_t cpuCoreCnt;
    uint32_t curcpuCoreCnt;
    OspCpuCoreInfo_s *cpuCoreInfo;
} OspCpuCoreInfoInner_s;

Timestamp_Control gOspCPUUsageUptimeAtLastReset = 0;
extern int malloc_info(Heap_Information_block *the_info);
extern Heap_Control _Workspace_Area;

///< 内部函数:计算系统当前任务数量
static bool ospCliInnerGetTaskCnt(rtems_tcb *tcb, void *arg)
{
  uint32_t *taskCount = arg;

  tcb = tcb;
  *taskCount = *taskCount + 1;
  return false; /*lint !e438 */
}

/**
 * @brief   获取系统当前任务数量
 * @param   taskCnt [out],  系统当前任务数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTaskCnt(uint32_t *taskCnt)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (taskCnt == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    *taskCnt = 0;
    rtems_task_iterate(ospCliInnerGetTaskCnt, taskCnt);

exit:
    return status;
}

///< 内部函数：重置任务cpu利用率统计
static bool ospCliInnerResetCpuUsage(Thread_Control *theThread, void *arg)
{
  const Scheduler_Control *scheduler;
  ISR_lock_Context         stateLockContext;
  ISR_lock_Context         schedulerLockContext;

  ///< 消除编译告警
  arg = arg;

  _Thread_State_acquire( theThread, &stateLockContext );
  scheduler = _Thread_Scheduler_get_home( theThread );
  _Scheduler_Acquire_critical( scheduler, &schedulerLockContext );
  _Timestamp_Set_to_zero( &theThread->cpu_time_used );
  _Scheduler_Release_critical( scheduler, &schedulerLockContext );
  _Thread_State_release( theThread, &stateLockContext );
  return false; /*lint !e438 */
}

/**
 * @brief   重置任务cpu利用率统计
 * @param   void
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliResetCpuUsage(void)
{
    uint32_t cpuCount;
    uint32_t cpuIndex;
    Per_CPU_Control *cpu;

    _TOD_Get_uptime( &gOspCPUUsageUptimeAtLastReset);
    cpuCount = rtems_scheduler_get_processor_maximum();
    for ( cpuIndex = 0 ; cpuIndex < cpuCount ; ++cpuIndex ) {
        cpu = _Per_CPU_Get_by_index( cpuIndex );
        cpu->cpu_usage_timestamp = gOspCPUUsageUptimeAtLastReset;
    }
    rtems_task_iterate(ospCliInnerResetCpuUsage, NULL);

    return OSP_SUCCESSFUL;
}

uint32_t ospCliGetMaxStackUse(const Thread_Control *thread)
{
#define STACK_PATTERN_AREA_SIZE 16
#define STACK_UNUSED_PATTERN    0xA5A5A5A5
#define STACK_PATTERN           {0xFEEDF00D, 0x0BAD0D06, 0xDEADF00D, 0x600D0D06}
    Stack_Control *stack = &((Thread_Control *)thread)->Start.Initial_stack;
    uint32_t *base = (uint32_t *)((uint32_t)stack->area + STACK_PATTERN_AREA_SIZE);
    uint32_t size = (uint32_t)stack->size - STACK_PATTERN_AREA_SIZE;
    uint32_t len = size / 4;
    uint32_t stack_pattern[] = STACK_PATTERN;
    uint32_t *pattern_base = (uint32_t *)stack->area;
    uint32_t i, *ebase;

    for(i = 0; i < sizeof(stack_pattern) / sizeof(uint32_t); ++i)
    {
        ///< CONFIGURE_STACK_CHECKERE_ENABLED is not defined
        if( stack_pattern[i] != *(pattern_base+i) )
        {
            return 0xFFFFFFFF;
        }
    }

    for(ebase = base + len; base < ebase; ++base)
    {
        if(STACK_UNUSED_PATTERN != *base)
            break;
    }

    return (uint32_t)ebase - (uint32_t)base;
}

static void __ospCliInnerGetTaskInfo(Thread_Control *thread,
        OspTaskInfo_s *taskInfo,
        Timestamp_Control  *uptimeFromLastRest)
{
    RTEMS_API_Control *api;
    uint32_t           ival;
    uint32_t           fval;
    Timestamp_Control totalTime;
    Timestamp_Control  uptime;
    Timestamp_Control  used;
    uint32_t           seconds;
    uint32_t           nanoseconds;

#ifdef PS3OS_SMP
    taskInfo->switchInTime = thread->switch_in_time;
    taskInfo->switchOutTime = thread->switch_out_time;
#endif

    taskInfo->nvcsw = thread->nvcsw;
    taskInfo->nivcsw = thread->nivcsw;

    api = thread->API_Extensions[THREAD_API_RTEMS];
    taskInfo->taskID = thread->Object.id;
    taskInfo->taskName = thread->Object.name.name_u32;
    taskInfo->schedulerName = _Thread_Scheduler_get_home(thread)->name;
    if (ospConvertToOspPriority(_Thread_Get_unmapped_priority(thread),
                &(taskInfo->taskPriority)) != OSP_SUCCESSFUL) {
        taskInfo->taskPriority = USER_TASK_PRIORITY_INVALID;
    }
    taskInfo->taskState = thread->current_state;
    taskInfo->events = api->Event.pending_events;
    ///< 获取任务堆栈信息
    taskInfo->stackLow =
        (uintptr_t)thread->Start.Initial_stack.area;
    taskInfo->stackHigh =
        (uintptr_t)thread->Start.Initial_stack.area
        + (uintptr_t)thread->Start.Initial_stack.size - 1;
    taskInfo->stackCurr =
        (uintptr_t) _CPU_Context_Get_SP(&thread->Registers);
    taskInfo->maxStackUse =
        ospCliGetMaxStackUse(thread);
    ///< 统计任务cpu利用率
    _Thread_Get_CPU_time_used( thread, &used );
    _TOD_Get_uptime( &uptime );
    _Timestamp_Subtract( uptimeFromLastRest,
            &uptime, &totalTime);
    _Timestamp_Divide( &used, &totalTime, &ival, &fval );
    seconds = _Timestamp_Get_seconds( &used );
    nanoseconds = _Timestamp_Get_nanoseconds( &used ) / TOD_NANOSECONDS_PER_MICROSECOND;
    taskInfo->cpuUseSec = seconds;
    taskInfo->cpuUseNanosec = nanoseconds;
    taskInfo->cpuUseIval = ival;
    taskInfo->cpuUseFval = fval;
}
///< 内部函数：获取任务信息
static bool ospCliInnerGetTaskInfo(rtems_tcb *tcb, void *arg)
{
    bool ret = false;
    uint32_t taskIndex = 0;
    Thread_Control    *curThread = (Thread_Control *)tcb;
    OspTaskInfoInner_s *taskInnerInfo = (OspTaskInfoInner_s *)arg;

    ///< 查询任务数量到的buffer空间时结束迭代
    if (taskInnerInfo->curTaskCnt == taskInnerInfo->taskCnt) {
        ret = true;
        goto exit;
    }
    taskIndex = taskInnerInfo->curTaskCnt;
    ///< 获取任务信息
    __ospCliInnerGetTaskInfo(curThread, &taskInnerInfo->taskInfo[taskIndex],
            &taskInnerInfo->uptimeFromLastRest);
    ///< 当前任务数量自加
    taskInnerInfo->curTaskCnt++;

exit:
    return ret;
}

/**
 * @brief   获取系统任务信息
 * @param   taskCnt 任务数量
 * @param   taskInfo [out], 任务信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTaskInfo(uint32_t taskCnt, OspTaskInfo_s *taskInfo)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;
    OspTaskInfoInner_s innerTaskInfo;

    if (taskCnt == 0 || taskInfo == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }

    innerTaskInfo.taskCnt = taskCnt;
    innerTaskInfo.curTaskCnt = 0;
    innerTaskInfo.taskInfo = taskInfo;
    innerTaskInfo.uptimeFromLastRest = gOspCPUUsageUptimeAtLastReset;
    rtems_task_iterate(ospCliInnerGetTaskInfo, &innerTaskInfo);
exit:
    return status;
}

/**
 * @brief   通过id获取系统任务信息
 * @param   id 任务id
 * @param   taskInfo [out], 任务信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTaskInfoByID(OspID id, OspTaskInfo_s *taskInfo)
{
    Thread_Control *thread;
    ISR_lock_Context   lock_context;

    if( !taskInfo )
        return OSP_INVALID_ADDRESS;

    _Objects_Allocator_lock();

    thread = _Thread_Get( id, &lock_context );

    if ( thread == NULL)
    {
        _Objects_Allocator_unlock();
        return OSP_INVALID_ID;
    }

    _ISR_lock_ISR_enable( &lock_context );

    __ospCliInnerGetTaskInfo(thread, taskInfo, &gOspCPUUsageUptimeAtLastReset);

    _Objects_Allocator_unlock();
    return OSP_SUCCESSFUL;
}
/**
 * @brief   获取系统当前cpu core数量
 * @param   cpuCoreCnt [out], 系统当前cpu core数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetCpuCoreCnt(uint32_t *cpuCoreCnt)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (cpuCoreCnt == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    *cpuCoreCnt = rtems_configuration_get_maximum_processors();
exit:
    return status;
}

/**
 * @brief   获取系统cpu core信息
 * @param   cpuCoreCnt cpu core数量
 * @param   cpuCoreInfo [out], cpu core信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetCpuCoreInfo(uint32_t cpuCoreCnt, OspCpuCoreInfo_s *cpuCoreInfo)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;
    uint32_t cpu_index = 0;
    const Per_CPU_Control *cpu;
    const Scheduler_Control *scheduler;

    if (cpuCoreCnt == 0 || cpuCoreInfo == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }

    for ( cpu_index = 0; cpu_index < cpuCoreCnt; ++cpu_index ) {
        cpuCoreInfo[cpu_index].cpuCoreIndex = cpu_index;
        cpu = _Per_CPU_Get_by_index( cpu_index );
        cpuCoreInfo[cpu_index].cpuCoreOnline = _Per_CPU_Is_processor_online( cpu );
        scheduler = _Scheduler_Get_by_CPU( cpu );
        if ( scheduler != NULL ) {
            cpuCoreInfo[cpu_index].schedulerName = scheduler->name;
        } else {
            cpuCoreInfo[cpu_index].schedulerName = 0;
        }
        cpuCoreInfo[cpu_index].stackLow = (uintptr_t)cpu->interrupt_stack_low;
        cpuCoreInfo[cpu_index].stackHigh = (uintptr_t)cpu->interrupt_stack_high;
        cpuCoreInfo[cpu_index].stackCurr =  (uintptr_t)cpu->interrupt_stack_high - (size_t)bsp_stack_fiq_size - (size_t)bsp_stack_abt_size - (size_t)bsp_stack_und_size - (size_t)bsp_stack_hyp_size;
    }

exit:
    return status;
}

/**
 * @brief   获取系统cpu uptime信息和cpu上idle任务运行总时长
 * @param   cnt cpu core数量
 * @param   cpuUsage [out], cpu uptime和idle任务时长信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetCpuRunningTime(uint32_t cnt, OspCpuUsage_s *cpuUsage)
{
    uint32_t index;
    uint32_t cpu_max;
    Per_CPU_Control *cpu;

    cpu_max = rtems_configuration_get_maximum_processors();
    if(cnt > cpu_max || NULL == cpuUsage)
    {
        goto arg_invalid;
    }

    for(index = 0; index < cnt; ++index)
    {
        cpu = _Per_CPU_Get_by_index( index );
        if ( _Per_CPU_Is_processor_online( cpu ) )
        {
            cpuUsage[index].cpuIndex = index;
            _TOD_Get_uptime( &(cpuUsage[index].totalTime));
            cpuUsage[index].idleTime = cpu->idle_time;
        }
        else
            goto internal_error;
    }

    return OSP_SUCCESSFUL;
arg_invalid:
    return OSP_INVALID_ID;
internal_error:
    return OSP_INTERNAL_ERROR;
}

/**
 * @brief   获取系统内存统计
 * @param   memoryInfo 内存统计信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetMemoryInfo(OspMemoryInfo_s *memoryInfo)
{
    memoryInfo->isUnified = rtems_configuration_get_unified_work_area();
    (void)malloc_info((Heap_Information_block *)&(memoryInfo->heapInfo));
    return OSP_SUCCESSFUL;
}

/**
 * @brief   获取系统workspace统计
 * @param   memoryInfo 内存统计信息
 * @return  执行结果
 * @warning 无
 * @note    如果workspace和malloc内存是一个内存堆，这该接口和ospCliGetMemoryInfo接口一致
 */
OspStatusCode_e ospCliGetWorkspaceInfo(OspMemoryInfo_s *memoryInfo)
{
    memoryInfo->isUnified = rtems_configuration_get_unified_work_area();
    (void)_Protected_heap_Get_information(&_Workspace_Area, (Heap_Information_block *)&(memoryInfo->heapInfo));
    return OSP_SUCCESSFUL;
}

///< 内部函数：获取注册到系统中的中断数量
static void ospCliInnerGetRegisteredInterCnt(void *arg, const char *info, rtems_option options,
        rtems_interrupt_handler handler, void *handlerArg)
{
    uint32_t *regInterCnt = (uint32_t *)arg;

    ///< 消除编译告警
    info = info;
    options = options;
    handler = handler;
    handlerArg = handlerArg;

    *regInterCnt = *regInterCnt + 1;
    return; /*lint !e438 */
}

/**
 * @brief   获取注册到系统中的中断数量
 * @param   regInterCnt [out], 系统当前注册的中断数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetRegisteredInterCnt(uint32_t *regInterCnt)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;
    OspVector v = 0;

    if (regInterCnt == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }

    *regInterCnt = 0;
    for (v = BSP_INTERRUPT_VECTOR_MIN; v <= BSP_INTERRUPT_VECTOR_MAX; ++v) {
        (void)rtems_interrupt_handler_iterate(v, ospCliInnerGetRegisteredInterCnt, regInterCnt);
    }
exit:
    return status;
}

typedef struct OspRegInterInner {
    uint32_t count;
    uint32_t maxCnt;
    OspVector vector;
    OspRegInterInfo_s *regInterInfo;
} OspRegInterInner_s;

static void ospCliInnerGetRegInterInfo(void *arg, const char *info, rtems_option options,
        rtems_interrupt_handler handler, void *handlerArg)
{
    OspRegInterInner_s *regInterInnerInfo = (OspRegInterInner_s *)arg;
    OspRegInterInfo_s *regInterInfo = regInterInnerInfo->regInterInfo;

    ///< 消除编译告警
    info = info;

    if ((regInterInnerInfo->count) >= (regInterInnerInfo->maxCnt)) {
        return;
    }

    regInterInfo[regInterInnerInfo->count].vector = regInterInnerInfo->vector;
    regInterInfo[regInterInnerInfo->count].options = options;
    regInterInfo[regInterInnerInfo->count].handler = handler;
    regInterInfo[regInterInnerInfo->count].handlerArg = handlerArg;
    regInterInnerInfo->count = regInterInnerInfo->count + 1;
    return; /*lint !e438 */
}

/**
 * @brief   获取系统已注册的中断信息
 * @param   regInterCnt 已经注册的数量
 * @param   regInterInfo [out], 已经注册的中断信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetRegInterInfo(uint32_t regInterCnt, OspRegInterInfo_s *regInterInfo)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;
    uint32_t v;
    OspRegInterInner_s regInterInnerInfo;

    if (regInterCnt == 0 || regInterInfo == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }

    memset(regInterInfo, 0, sizeof(OspRegInterInfo_s) * regInterCnt);
    regInterInnerInfo.count = 0;
    regInterInnerInfo.maxCnt = regInterCnt;
    regInterInnerInfo.regInterInfo = regInterInfo;
    for (v = BSP_INTERRUPT_VECTOR_MIN; v <= BSP_INTERRUPT_VECTOR_MAX; ++v) {
        if (regInterInnerInfo.count >= regInterCnt) {
            break;
        }
        regInterInnerInfo.vector = v;
        if (rtems_interrupt_handler_iterate(v, ospCliInnerGetRegInterInfo, &regInterInnerInfo) != RTEMS_SUCCESSFUL) {
            continue;
        }
    }
exit:
    return status;
}

///< 内部函数：将rtems系统资源拷贝到osp结构体中
typedef void (*ospResourceConvertFn)(void *singleResourceInfo, rtems_monitor_union_t canonicalInfo);
///< 内部结构体：资源拷贝结构体
typedef struct OspInnterResourceGetObj {
    rtems_monitor_object_type_t type;
    ospResourceConvertFn convertResourceInfo;
} OspInnterResourceGetObj_s;
///< 内部函数：拷贝消息队列信息
static void queueResourceConvert(void *singleResourceInfo, rtems_monitor_union_t canonicalInfo)
{
    singleResourceInfo = singleResourceInfo;
    ((OspMessageQueueInfo_s *)singleResourceInfo)->id = canonicalInfo.queue.id;
    ((OspMessageQueueInfo_s *)singleResourceInfo)->name = canonicalInfo.queue.name;
    ((OspMessageQueueInfo_s *)singleResourceInfo)->attribute = canonicalInfo.queue.attributes;
    ((OspMessageQueueInfo_s *)singleResourceInfo)->numberOfPendingMessages = canonicalInfo.queue.number_of_pending_messages;
    ((OspMessageQueueInfo_s *)singleResourceInfo)->maximumPendingMessages = canonicalInfo.queue.maximum_pending_messages;
    ((OspMessageQueueInfo_s *)singleResourceInfo)->maximumMessageSize = canonicalInfo.queue.maximum_message_size;
    return;
}
///< 内部函数：拷贝信号量信息
static void semaInfoResourceConvert(void *singleResourceInfo, rtems_monitor_union_t canonicalInfo)
{
    singleResourceInfo = singleResourceInfo;
    ((OspSemaphoreInfo_s *)singleResourceInfo)->id = canonicalInfo.sema.id;
    ((OspSemaphoreInfo_s *)singleResourceInfo)->name = canonicalInfo.sema.name;
    ((OspSemaphoreInfo_s *)singleResourceInfo)->attribute = canonicalInfo.sema.attribute;
    ((OspSemaphoreInfo_s *)singleResourceInfo)->maxCount = canonicalInfo.sema.max_count;
    ((OspSemaphoreInfo_s *)singleResourceInfo)->curCount = canonicalInfo.sema.cur_count;
    ((OspSemaphoreInfo_s *)singleResourceInfo)->holderId = canonicalInfo.sema.holder_id;
    return;
}
///< 内部函数：拷贝timer信息
static void timerInfoResourceConvert(void *singleResourceInfo, rtems_monitor_union_t canonicalInfo)
{
    singleResourceInfo = singleResourceInfo;
    ((OspTimerInfo_s *)singleResourceInfo)->id = canonicalInfo.timer.id;
    ((OspTimerInfo_s *)singleResourceInfo)->name = canonicalInfo.timer.name;
    ((OspTimerInfo_s *)singleResourceInfo)->routine = canonicalInfo.timer.routine;
    ((OspTimerInfo_s *)singleResourceInfo)->initTime = canonicalInfo.timer.initial;
    ((OspTimerInfo_s *)singleResourceInfo)->startTime = canonicalInfo.timer.start_time;
    ((OspTimerInfo_s *)singleResourceInfo)->stopTime = canonicalInfo.timer.stop_time;
    return;
}

///< 内部函数：拷贝region信息
static void regionInfoResourceConvert(void *singleResourceInfo, rtems_monitor_union_t canonicalInfo)
{
    singleResourceInfo = singleResourceInfo;
    ((OspRegionInfo_s *)singleResourceInfo)->id = canonicalInfo.region.id;
    ((OspRegionInfo_s *)singleResourceInfo)->name = canonicalInfo.region.name;
    ((OspRegionInfo_s *)singleResourceInfo)->attribute = canonicalInfo.region.attribute;
    ((OspRegionInfo_s *)singleResourceInfo)->startAddr = canonicalInfo.region.start_addr;
    ((OspRegionInfo_s *)singleResourceInfo)->length = canonicalInfo.region.length;
    ((OspRegionInfo_s *)singleResourceInfo)->pageSize = canonicalInfo.region.page_size;
    ((OspRegionInfo_s *)singleResourceInfo)->usedBlocks = canonicalInfo.region.used_blocks;
    return;
}

OspInnterResourceGetObj_s gOspInnterResourceGet[] = {
    {RTEMS_MONITOR_OBJECT_QUEUE, (ospResourceConvertFn)queueResourceConvert},
    {RTEMS_MONITOR_OBJECT_SEMAPHORE, (ospResourceConvertFn)semaInfoResourceConvert},
    {RTEMS_MONITOR_OBJECT_TIMERS, (ospResourceConvertFn)timerInfoResourceConvert},
    {RTEMS_MONITOR_OBJECT_REGION, (ospResourceConvertFn)regionInfoResourceConvert}
};

///< 内部函数：获取对外结构体长度
static uint32_t ospCliInnerGetResStructLen(rtems_monitor_object_type_t type)
{
    uint32_t len = 0;
    switch(type) {
        case RTEMS_MONITOR_OBJECT_QUEUE:
            len = sizeof(OspMessageQueueInfo_s);
            break;
        case RTEMS_MONITOR_OBJECT_SEMAPHORE:
            len = sizeof(OspSemaphoreInfo_s);
            break;
        case RTEMS_MONITOR_OBJECT_TIMERS:
            len = sizeof(OspTimerInfo_s);
            break;
        case RTEMS_MONITOR_OBJECT_REGION:
            len = sizeof(OspRegionInfo_s);
            break;
        default:
            len = 0;
            break;
    }
    return len;
}

///< 内部函数：获取指定类型资源的信息
static OspStatusCode_e ospCliInnerGetResourceInfo(rtems_monitor_object_type_t type, uint32_t cnt, void *resourceInfo)
{
    const rtems_monitor_object_info_t *info = NULL;
    OspStatusCode_e status = OSP_SUCCESSFUL;
    uint32_t current_node;
    rtems_id next_id;
    uint32_t resourceCnt;
    ospResourceConvertFn convertFunc;
    rtems_monitor_union_t canonical;
    int32_t i;
    uint32_t structLen = 0;

    ///< 内部函数调用者保证入参合法性
    info = rtems_monitor_object_lookup(type);
    if (info == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    for (i = 0; i < (int32_t)(sizeof(gOspInnterResourceGet)/sizeof(gOspInnterResourceGet[0])); i++) {
        if (type == gOspInnterResourceGet[i].type) {
            convertFunc = gOspInnterResourceGet[i].convertResourceInfo;
            break;
        }
    }
    if (i == sizeof(gOspInnterResourceGet)/sizeof(gOspInnterResourceGet[0])) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }

    structLen = ospCliInnerGetResStructLen(type);
    current_node = rtems_object_id_get_node(rtems_task_self());
    next_id = RTEMS_OBJECT_ID_INITIAL(OBJECTS_CLASSIC_API, info->type, current_node);
    resourceCnt = 0;
    while ((next_id = rtems_monitor_object_canonical_next(info, next_id, &canonical)) != RTEMS_OBJECT_ID_FINAL) {
        if (resourceCnt >= cnt) {
            break;
        }
        convertFunc(resourceInfo + resourceCnt * structLen, canonical);
        resourceCnt++;
    }

exit:
    return status;

}

///< 内部函数：获取系统指定类型资源当前使用的数量
static OspStatusCode_e ospCliInnerGetResourceCnt(rtems_monitor_object_type_t type, uint32_t *cnt)
{
    const rtems_monitor_object_info_t *info = NULL;
    OspStatusCode_e status = OSP_SUCCESSFUL;
    uint32_t current_node;
    rtems_id next_id;
    rtems_monitor_union_t canonical;

    ///< 内部函数调用者保证入参合法性
    info = rtems_monitor_object_lookup(type);
    if (info == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    current_node = rtems_object_id_get_node(rtems_task_self());
    next_id = RTEMS_OBJECT_ID_INITIAL(OBJECTS_CLASSIC_API, info->type, current_node);
    *cnt = 0;
    while ((next_id = rtems_monitor_object_canonical_next(info, next_id, &canonical)) != RTEMS_OBJECT_ID_FINAL) {
        (*cnt)++;
    }

exit:
    return status;
}

/**
 * @brief   获取系统中的消息队列数量
 * @param   messageQueueCnt [out], 系统当前的消息队列数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetMessageQueueCnt(uint32_t *messageQueueCnt)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (messageQueueCnt == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    status = ospCliInnerGetResourceCnt(RTEMS_MONITOR_OBJECT_QUEUE, messageQueueCnt);

exit:
    return status;
}

/**
 * @brief   获取系统中的消息队列信息
 * @param   messageQueueCnt 系统中的消息队列数量
 * @param   messageQueueInfo [out], 系统中的消息队列信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetMessageQueueInfo(uint32_t messageQueueCnt, OspMessageQueueInfo_s *messageQueueInfo)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (messageQueueCnt == 0 || messageQueueInfo == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    status = ospCliInnerGetResourceInfo(RTEMS_MONITOR_OBJECT_QUEUE, messageQueueCnt, (void *)messageQueueInfo);

exit:
    return status;
}

/**
 * @brief   获取系统中的信号量数量
 * @param   semaphoreCnt [out], 系统当前的信号量数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetSemaphoreCnt(uint32_t *semaphoreCnt)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (semaphoreCnt == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    status = ospCliInnerGetResourceCnt(RTEMS_MONITOR_OBJECT_SEMAPHORE, semaphoreCnt);

exit:
    return status;
}

/**
 * @brief   获取系统中的信号量信息
 * @param   semaphoreCnt 系统中的信号量数量
 * @param   semaphoreInfo [out], 系统中的信息量信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetSemaphoreInfo(uint32_t semaphoreCnt, OspSemaphoreInfo_s *semaphoreInfo)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (semaphoreCnt == 0 || semaphoreInfo == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    status = ospCliInnerGetResourceInfo(RTEMS_MONITOR_OBJECT_SEMAPHORE, semaphoreCnt, (void *)semaphoreInfo);

exit:
    return status;
}

/**
 * @brief   获取系统中的timer数量
 * @param   timerCnt [out], 系统当前的timer数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTimerCnt(uint32_t *timerCnt)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (timerCnt == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    status = ospCliInnerGetResourceCnt(RTEMS_MONITOR_OBJECT_TIMERS, timerCnt);

exit:
    return status;
}

/**
 * @brief   获取系统中的timer信息
 * @param   timerCnt 系统中的timer数量
 * @param   timerInfo [out], 系统中的timer信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTimerInfo(uint32_t timerCnt, OspTimerInfo_s *timerInfo)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (timerCnt == 0 || timerInfo == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    status = ospCliInnerGetResourceInfo(RTEMS_MONITOR_OBJECT_TIMERS, timerCnt, (void *)timerInfo);

exit:
    return status;
}

/**
 * @brief   获取系统中的region数量
 * @param   regionCnt [out], 系统当前的region数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetRegionCnt(uint32_t *regionCnt)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (regionCnt == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    status = ospCliInnerGetResourceCnt(RTEMS_MONITOR_OBJECT_REGION, regionCnt);

exit:
    return status;
}

/**
 * @brief   获取系统中的region信息
 * @param   regionCnt 系统中的region数量
 * @param   regionInfo [out], 系统中的region信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetRegionInfo(uint32_t regionCnt, OspRegionInfo_s *regionInfo)
{
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (regionCnt == 0 || regionInfo == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }
    status = ospCliInnerGetResourceInfo(RTEMS_MONITOR_OBJECT_REGION, regionCnt, (void *)regionInfo);

exit:
    return status;
}

/**
 * @brief   获取系统中的资源规格
 * @param   resourceSpec [out], 系统中的资源规格
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetSystemResourceSpec(OspSystemResourceSpec_s *resourceSpec)
{
    rtems_monitor_config_t canonical;
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (resourceSpec == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }

    rtems_monitor_config_canonical(&canonical, NULL);
    resourceSpec->maxTasks = canonical.maximum_tasks;
    resourceSpec->maxTimers = canonical.maximum_timers;
    resourceSpec->maxSemaphores = canonical.maximum_semaphores;
    resourceSpec->maxMessageQueues = canonical.maximum_message_queues;
    resourceSpec->maxRegions = canonical.maximum_regions;
    resourceSpec->microsecondsPerTick = canonical.microseconds_per_tick;

exit:
    return status;
}

/**
 * @brief   获取系统性能统计信息
 * @param   cpuCoreCnt cpu core数量
 * @param   profInfo [out], 每个cpu core的prof信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetProfInfo(uint32_t cpuCoreCnt, OspProfInfo_s *profInfo)
{
#ifdef OS_SUPPORT_PROFILING
    uint32_t i;
    const Per_CPU_Control *perCpu;
    const Per_CPU_Stats *stats;
    OspStatusCode_e status = OSP_SUCCESSFUL;

    if (cpuCoreCnt == 0 || profInfo == NULL) {
        status = OSP_INVALID_NUMBER;
        goto exit;
    }

    for (i = 0; i < cpuCoreCnt; ++i) {
        perCpu = _Per_CPU_Get_by_index(i);
        stats = &perCpu->Stats;
        profInfo[i].cpuCoreIndex = i;
        profInfo[i].maxThreadDispatchDisabledTime =
            rtems_counter_ticks_to_nanoseconds(stats->max_thread_dispatch_disabled_time);
        profInfo[i].maxInterruptTime =
            rtems_counter_ticks_to_nanoseconds(stats->max_interrupt_time);
    }
exit:
    return status;
#else
    (void )cpuCoreCnt;
    (void) profInfo;
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   关闭coredump功能
 * @param   true/false
 * @return  执行结果
 * @warning 无
 * @note    1 默认打开；
 *          2 OSP_NOT_IMPLEMENTED表示不支持
 */
OspStatusCode_e ospCliCloseCoredump(bool isCloseCoredump)
{
#ifdef PS3OS_COREDUMP
    rtems_close_coredump(isCloseCoredump);
    return OSP_SUCCESSFUL;
#else
    (void)isCloseCoredump;
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   查询coredump功能是否关闭
 * @param   *isCloseCoredump [out], true/false
 * @return  执行结果
 * @warning 无
 * @note    1 默认打开；
 *          2 OSP_NOT_IMPLEMENTED表示不支持
 *          3 OSP_INVALID_ADDRESS表示入参为空
 */
OspStatusCode_e ospCliGetCoredumpIsClose(bool *isCloseCoredump)
{
#ifdef PS3OS_COREDUMP
    if (isCloseCoredump == NULL) {
        return OSP_INVALID_ADDRESS;
    } else {
        *isCloseCoredump = rtems_coredump_is_close();
        return OSP_SUCCESSFUL;
    }
#else
    (void)isCloseCoredump;
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   设置coredump导出方法
 * @param   OSP_COREDUMP_FUN_XXX
 * @return  执行结果
 * @warning 无
 * @note    1 OSP_INVALID_NUMBER表示入参错误
 *          2 OSP_NOT_IMPLEMENTED表示不支持
 */
OspStatusCode_e ospCliSetFunType(int coredumpFunType)
{
#ifdef PS3OS_COREDUMP
    rtems_status_code ret;
    OspStatusCode_e ospRet;
    ret = rtems_coredump_set_fun_type(coredumpFunType);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
#else
    (void)coredumpFunType;
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   查询coredump导出方法
 * @param   void
 * @return  OSP_COREDUMP_FUN_XXX:导出方法
 * @warning 无
 * @note    1 OSP_NOT_IMPLEMENTE表示不支持
 */
int ospCliGetFunType(void)
{
#ifdef PS3OS_COREDUMP
    return rtems_coredump_get_fun_type();
#else
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   配置coredump onf分区信息
 * @param   is_minicore
 *              true : minicore
 *              false : not minicore
 * @return
 * @warning 无
 */
void ospCliSetCoredumpMinicore(bool is_minicore)
{
    rtems_set_minicore(is_minicore);
}

/**
 * @brief   判断是否配置minicore
 * @param
 * @return  true : minicore
 *          false : not minicore
 * @warning 无
 */
bool ospCliCoredumpIsMinicore(void)
{
    return rtems_coredump_is_minicore();
}



/**
 * @brief   配置coredump到mem的起始地址和长度信息
 * @param   memStart: coredump mem起始位置
 * @param   size: coredump mem大小
 * @return  执行结果
 * @warning 无
 * @note    1 OSP_SUCCESSFUL表示配置成功
 *          2 OSP_NOT_IMPLEMENTE表示不支持
 *          3 OSP_INVALID_SIZE示配置参数失败
 *          4 需先配置dump到mem区域在选择dump到mem
 */
OspStatusCode_e ospCliSetMemCoredumpArea(uint32_t memStart, uint32_t size)
{
#ifdef PS3OS_COREDUMP
    int ret;
    ret = rtems_coredump_memarea_set(memStart, size);
    if (ret) {
        return OSP_INVALID_SIZE;
    }
    return OSP_SUCCESSFUL;
#else
    (void)memStart;
    (void)size;
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   配置coredump flash分区信息
 * @param   regionStart: coredump flash分区起始位置,需4k对齐
 * @param   size: coredump flash分区大小,需4k对齐
 * @return  执行结果
 * @warning 无
 * @note    1 OSP_SUCCESSFUL表示配置成功
 *          2 OSP_NOT_IMPLEMENTE表示不支持
 *          3 OSP_INVALID_SIZE示配置参数失败
 */
OspStatusCode_e ospCliSetFlashCoredumpRegion(uint32_t regionStart, uint32_t size)
{
#ifdef PS3OS_COREDUMP
    int ret;

    ret = rtems_coredump_flash_region_set_v2(regionStart, size);
    if (ret) {
        return OSP_INVALID_SIZE;
    }
    return OSP_SUCCESSFUL;
#else
    (void)regionStart;
    (void)size;
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   打印当前调用栈
 * @param   void
 * @return  void
 * @warning 无
 * @note
 */
void ospCliDumpStack(void)
{
    dump_stack();
}

/**
 * @brief   打印当前调用栈
 * @param   dumpType
 *              frame输出类型
 * @param   memInfo
 *              当输出到mem时需要指定输出内存地址
 * @param   size
 *              memInfo的大小, 至少大于等于 OSP_DUMP_STACK_MEM_PER_MINISIZE
 * @return  success
 *              OSP_SUCCESSFUL
 * @warning 无
 * @note
 */
OspStatusCode_e ospCliDumpStackExtension(OspDumpStackType_E dumpType, char *memInfo,uint32_t size)
{
    if ( memInfo == NULL  && (  dumpType == OSP_DUMP_STACK_MEM || dumpType == OSP_DUMP_STACK_COM_MEM  ) )
    {
        return OSP_INVALID_ADDRESS;
    }

    if ( size < OSP_DUMP_STACK_MEM_PER_MINISIZE && (  dumpType == OSP_DUMP_STACK_MEM || dumpType == OSP_DUMP_STACK_COM_MEM  ) )
    {
        return OSP_INVALID_SIZE;
    }

    DumpStackMemInfo_s tmp_memInfo;

    tmp_memInfo.serialBuf = memInfo;
    tmp_memInfo.index = 0;
    tmp_memInfo.serialBufSize = size;

    dump_stack_extension(dumpType, &tmp_memInfo);
    return OSP_SUCCESSFUL;
}

/**
 * @brief   设置backtrace输出类型
 * @param   dumpType
 *              frame输出类型
 * @param   memInfo
 *              当输出到mem时需要指定输出内存地址
 * @param   perSize
 *              每个记录fatal信息的buf大小 , 至少大于 OSP_DUMP_STACK_MEM_PER_MINISIZE
 * @param   memInfoSize
 *              memInfo 的总大小
 * @param   numberMemInfo
 *             memInfo 中包含多少个 记录fatal信息的buf memInfo(total) = perSize * numberMemInfo
 *             numberMemInfo 至少大于 等于当前核数
 * @param  number of DumpStackMemInfo_s
 * @return  success
 *              OSP_SUCCESSFUL
 * @warning 无
 * @note
 */
OspStatusCode_e ospCliDumpStackExtensionSetType(OspDumpStackType_E dumpType, char *memInfo, uint32_t perSize, uint32_t memInfoSize, uint32_t numberMemInfo)
{
    if ( dumpType < OSP_DUMP_STACK_COM || dumpType >= OSP_DUMP_STACK_LAST )
    {
        return OSP_UNSATISFIED;
    }

    if ( dumpType == OSP_DUMP_STACK_COM && memInfo != NULL )
    {
        return OSP_UNSATISFIED;
    }

    if ( dumpType == OSP_DUMP_STACK_MEM || dumpType == OSP_DUMP_STACK_COM_MEM  )
    {
        if ( memInfo == NULL)
        {
            return OSP_INVALID_ADDRESS;
        }

        if ( perSize <  OSP_DUMP_STACK_MEM_PER_MINISIZE )
        {
            return OSP_INVALID_SIZE;
        }

        if ( memInfoSize  < OSP_DUMP_STACK_MEM_PER_MINISIZE * numberMemInfo )
        {
            return OSP_INVALID_SIZE;
        }

#ifdef PS3OS_SMP
        if ( numberMemInfo == 0  )
        {
            return OSP_INVALID_NUMBER;
        }
#else
        uint32_t cpuCount = rtems_scheduler_get_processor_maximum();
        if ( cpuCount < numberMemInfo )
        {
            return OSP_INVALID_NUMBER;
        }
#endif

    }

    bool ret = set_dump_stack_type_and_meminfo(dumpType, memInfo, perSize, numberMemInfo);
    if (ret == false)
    {
        return OSP_NO_MEMORY;
    }

    return OSP_SUCCESSFUL;
}

OspSectionInfo_s ospSectionInfo[OSP_MAX_SECTION] = {
    [OSP_TEXT_SECTION] = {
        .begin = (uintptr_t)bsp_section_text_begin,
        .end = (uintptr_t)bsp_section_text_end,
        .size = (uintptr_t)bsp_section_text_size,
    },
    [OSP_START_SECTION] = {
        .begin = (uintptr_t)bsp_section_start_begin,
        .end = (uintptr_t)bsp_section_start_end,
        .size = (uintptr_t)bsp_section_start_size,
    },
    [OSP_RODATA_SECTION] = {
        .begin = (uintptr_t)bsp_section_rodata_begin,
        .end = (uintptr_t)bsp_section_rodata_end,
        .size = (uintptr_t)bsp_section_rodata_size,
    },
    [OSP_FAST_TEXT_SECTION] = {
        .begin = (uintptr_t)bsp_section_fast_text_begin,
        .end = (uintptr_t)bsp_section_fast_text_end,
        .size = (uintptr_t)bsp_section_fast_text_size,
    },
    [OSP_FAST_DATA_SECTION] = {
        .begin = (uintptr_t)bsp_section_fast_data_begin,
        .end = (uintptr_t)bsp_section_fast_data_end,
        .size = (uintptr_t)bsp_section_fast_data_size,
    },
#if defined(PS3OS_COREDUMP)
    [OSP_COREDUMP_SECTION] = {
        .begin = (uintptr_t)bsp_section_coredump_begin,
        .end = (uintptr_t)bsp_section_coredump_end,
        .size = (uintptr_t)-1,
    },
#else
    [OSP_COREDUMP_SECTION] = {
        .begin = (uintptr_t)0,
        .end = (uintptr_t)0,
        .size = (uintptr_t)0,
    },
#endif
    [OSP_BSS_SECTION] = {
        .begin = (uintptr_t)bsp_section_bss_begin,
        .end = (uintptr_t)bsp_section_bss_end,
        .size = (uintptr_t)bsp_section_bss_size,
    },
    [OSP_DATA_SECTION] = {
        .begin = (uintptr_t)bsp_section_data_begin,
        .end = (uintptr_t)bsp_section_data_end,
        .size = (uintptr_t)bsp_section_data_size,
    },
#if defined(PS3OS_HBA_V200)
    [OSP_TRANSLATION_TABLE_SECTION] = {
        .begin = (uintptr_t)bsp_translation_table_base,
        .end = (uintptr_t)bsp_translation_table_end,
        .size = (uintptr_t)-1,
    },
#else
    [OSP_TRANSLATION_TABLE_SECTION] = {
        .begin = 0,
        .end = 0,
        .size = 0,
    },
#endif
};


/**
 * @brief  获取 section 信息
 * @param  secton 内容
 * @return  section info
 * @warning 无
 * @note
 */
OspSectionInfo_s const *ospCliGetSectionInfo(OSP_SECTION_TYPE_E sectionType)
{
    if (ospSectionInfo[sectionType].size == (uintptr_t)-1)
    {
        ospSectionInfo[sectionType].size = ospSectionInfo[sectionType].end - ospSectionInfo[sectionType].begin;
    }

    return &ospSectionInfo[sectionType];
}

