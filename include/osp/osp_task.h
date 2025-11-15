/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_task.h
 * @author  tianye@tianye.com
 * @date    2020.08.04
 * @brief   porting task
 * @note    NA
 */

#ifndef __OSP_TASK_H__
#define __OSP_TASK_H__

#include <osp_attr.h>
#include <osp_mode.h>
#include <osp_status.h>
#include <osp_types.h>
#include <osp_priority.h>
#include <osp_cpu.h>
#include <sched.h>

#ifdef __cplusplus
extern "C" {
#endif

///< 任务状态
#define OSP_STATES_READY                           0x00000000 ///< 就绪
#define OSP_STATES_WAITING_FOR_MUTEX               0x00000001 ///< 等待互斥锁
#define OSP_STATES_WAITING_FOR_SEMAPHORE           0x00000002 ///< 等待信号量
#define OSP_STATES_WAITING_FOR_EVENT               0x00000004 ///< 等待事件
#define OSP_STATES_WAITING_FOR_SYSTEM_EVENT        0x00000008 ///< 等待系统事件(如：若启动了telnet服务，该任务会定义系统事件)
#define OSP_STATES_WAITING_FOR_MESSAGE             0x00000010 ///< 等待消息
#define OSP_STATES_WAITING_FOR_CONDITION_VARIABLE  0x00000020 ///< 等待posix条件变量
#define OSP_STATES_WAITING_FOR_FUTEX               0x00000040 ///< 等待futex
#define OSP_STATES_WAITING_FOR_BSD_WAKEUP          0x00000080 ///< 等待BSD唤醒
#define OSP_STATES_WAITING_FOR_TIME                0x00000100 ///< 等待定时器超时
#define OSP_STATES_WAITING_FOR_PERIOD              0x00000200 ///< 等待周期计时器超时
#define OSP_STATES_WAITING_FOR_SIGNAL              0x00000400 ///< 等待信号量
#define OSP_STATES_WAITING_FOR_BARRIER             0x00000800 ///< 等待barrier
#define OSP_STATES_WAITING_FOR_RWLOCK              0x00001000 ///< 等待读写锁
#define OSP_STATES_WAITING_FOR_JOIN_AT_EXIT        0x00002000 ///< 任务执行时等待pthread join(posix)
#define OSP_STATES_WAITING_FOR_JOIN                0x00004000 ///< 等待pthread join(posix)
#define OSP_STATES_SUSPENDED                       0x00008000 ///< 任务被暂停
#define OSP_STATES_WAITING_FOR_SEGMENT             0x00010000 ///< 任务等待region分配
#define OSP_STATES_LIFE_IS_CHANGING                0x00020000 ///< 任务重新开始
#define OSP_STATES_DEBUGGER                        0x08000000 ///< 任务正在被调试器调试（暂时没有使用）
#define OSP_STATES_INTERRUPTIBLE_BY_SIGNAL         0x10000000 ///< 任务处于可重点阻塞状态
#define OSP_STATES_WAITING_FOR_RPC_REPLY           0x20000000 ///< 任务等待MPCI请求
#define OSP_STATES_ZOMBIE                          0x40000000 ///< 任务退出处于等待被系统回收状态
#define OSP_STATES_DORMANT                         0x80000000 ///< 任务被创建但是还没有开始状态

#define OSP_MAX_BACKTRACE_CNT 10

typedef struct OspTaskBt {
    OspID id;
    OspName name;
    uint32_t cnt;
    uint32_t *addr[OSP_MAX_BACKTRACE_CNT];
} OspTaskBt_s;

///< typedef rtems_tcb OspTcb;

typedef void (*OspTaskEntry)(OspTaskArgument);
///< typedef bool (*OspTaskVisitor)(OspTcb *tcb, void *arg);

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
                              OspMode initialModes, OspAttribute attributeSet, OspID *id);

/**
 * @brief   设置任务名字
 * @param   id ,                task的id
 * @param   name ,              task名称
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSetTaskName(OspID id, OspName name);

/**
 * @brief   获取任务名字
 * @param   id ,                task的id
 * @param   name ,              task名称
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospGetTaskName(OspID id, OspName *name);

/**
 * @brief   获取指定任务是否就绪
 * @param   id ,                task的id
 * @param   bool ,              task是否就绪
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskIsReady(OspID id, bool *isReady);

/**
 * @brief   获取任务id
 * @param   name ,      task名称
 * @param   node ,      指定搜索的节点，如SEARCH_ALL_NODES等
 * @param   id [out],   task的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskIdent(OspName name, uint32_t node, OspID *id);

/**
 * @brief   调用者获取自己的任务id
 * @param   NA
 * @return  OspID 任务id
 * @warning NA
 * @note    NA
 */
OspID ospTaskSelf(void);

/**
 * @brief   开始任务
 * @param   id ,                task的id
 * @param   entryPoint [in],    任务函数指针
 * @param   argument [in/out],  任务参数
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskStart(OspID id, OspTaskEntry entryPoint, OspTaskArgument argument);



/**
 * @brief   重新开始任务
 * @param   id ,                task的id
 * @param   argument [in/out],  任务参数
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskRestart(OspID id, OspTaskArgument argument);

/**
 * @brief   删除任务
 * @param   id ,                task的id
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskDelete(OspID id);

/**
 * @brief   任务退出
 * @param   NA
 * @return  NA
 * @warning NA
 * @note    NA
 */
void ospTaskExit(void);

/**
 * @brief   挂起任务
 * @param   id ,                task的id
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskSuspend(OspID id);

/**
 * @brief   恢复任务
 * @param   id ,                task的id
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskResume(OspID id);

/**
 * @brief   检查任务是否被挂起
 * @param   id ,                task的id
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskIsSuspended(OspID id);

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
OspStatusCode_e ospTaskSetPriority(OspID id, OspTaskPriority_e newPriority, OspTaskPriority_e *oldPriority);

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
OspStatusCode_e ospTaskGetPriority(OspID id, OspTaskPriority_e *priority);

/**
 * @brief   设置任务模式
 * @param   modeSet ,             任务模式
 * @param   mask ,                任务模式掩码
 * @param   previousModeSet [out],任务的旧模式
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskMode(OspMode modeSet, OspMode mask, OspMode *previousModeSet);

/**
 * @brief   任务休眠指定单位的ticks
 * @param   ticks ,       休眠ticks
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskWakeAfter(OspInterval ticks);

/**
 * @brief   任务切换
 * @param
 * @return
 * @warning NA
 * @note    NA
 */
void ospTaskCondResched();

/**
 * @brief   任务休眠到指定时间
 * @param   timeBuffer [in], 休眠截止时间
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskWakeWhen(OspTimeOfDay_s *timeBuffer);

/**
 * @brief   获取任务亲核性
 * @param   taskId ,      任务ID
 * @param   cpusetSize ,  sizeof(cpu_set_t)
 * @param   cpuset [out], cpu集
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskGetAffinity(OspID taskId, size_t cpusetSize, cpu_set_t *cpuset);

/**
 * @brief   设置任务亲核性
 * @param   taskId ,      任务ID
 * @param   cpusetSize ,  sizeof(cpu_set_t)
 * @param   cpuset [in],  cpu集
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskSetAffinity(OspID taskId, size_t cpusetSize, const cpu_set_t *cpuset);

/**
 * @brief   Get scheduler of a task
 * @param   taskId ,      任务ID
 * @param   schedulerId,  schedulerId
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskGetScheduler(OspID taskId, OspID *schedulerId);

/**
 * @brief   Set scheduler of a task
 * @param   taskId ,      任务ID
 * @param   schedulerId,  schedulerId
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTaskSetScheduler(OspID taskId, OspID schedulerId, OspTaskPriority_e priority);

/**
 * @brief   Disables thread dispatching.
 * @return  OspPerCPUControl, per cpu control
 * @warning NA
 * @note    return The current processor
 */
OspPerCPUControl ospThreadDispatchDisable(void);

/**
 * @brief   Enables thread dispatching.
 * @param   cpuSelf, The current processor per cpu control
 * @return
 * @warning NA
 * @note    May perform a thread dispatch if necessary as a side-effect.
 */
void ospThreadDispatchEnable(OspPerCPUControl cpuSelf);

/**
 * @brief   Get interrupted task backtrace.
 * @param   task backtrace info
 * @return
 * @warning only can be used in isr
 * @note    none
 */
OspStatusCode_e ospGetInterruptedTaskBt(OspTaskBt_s *taskBt);

#ifdef __cplusplus
}
#endif

#endif
