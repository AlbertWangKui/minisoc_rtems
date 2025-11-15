/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_interrupt.h
 * @author  lichenxiang
 * @date    2020.09.08
 * @brief   porting interrupt
 * @note    NA
 */

#ifndef __OPS_INTERRUPT_H
#define __OPS_INTERRUPT_H

#include <osp_status.h>
#include <osp_types.h>
#include <osp_mode.h>
#include <osp_options.h>
#include <inner/osp_inner_interrupt.h>
#include <osp_cpuopts.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef InnerOspInterruptLevel_t OspInterruptLevel_t;
typedef InnerOspInterruptLock_t OspInterruptLock_t;
typedef InnerOspInterruptLockContext_t OspInterruptLockContext_t;
typedef void (*OspInterruptHandler)(void *);

typedef enum OspTriggerMode { OSP_LEVEL_SENSITIVE, OSP_EDGE_TRIGGERED } OspTriggerMode_e;

///< 中断优先级
typedef enum OspInterruptPriority {
    INTERRUPT_PRIORITY_LOWEST,  ///< 用户最低优先级
    INTERRUPT_PRIORITY_LOW,     ///< 用户次低优先级
    INTERRUPT_PRIORITY_NORMAL,  ///< 用户正常优先级
    INTERRUPT_PRIORITY_HIGH,    ///< 用户次高优先级
    INTERRUPT_PRIORITY_HIGHEST, ///< 用户最高优先级
} OspInterruptPriority_e;

#define OSP_IPI_1 1
#define OSP_IPI_2 2
#define OSP_IPI_3 3
#define OSP_IPI_5 5
#define OSP_IPI_6 6
#define OSP_IPI_7 7
#define OSP_IPI_8 8
#define OSP_IPI_9 9
#define OSP_IPI_10 10
#define OSP_IPI_11 11
#define OSP_IPI_12 12
#define OSP_IPI_13 13
#define OSP_IPI_14 14
#define OSP_IPI_15 15
#define OSP_IPI_LAST 15

///< 软件触发中断过滤枚举
typedef enum OspIrqTargetFilter {
    OSP_IRQ_TO_ALL_IN_LIST,         ///< 发给targetslist定义的所有core
    OSP_IRQ_TO_ALL_EXCEPT_SELF,     ///< 发给系统中除触发者之外的所有core
    OSP_IRQ_TO_SELF,                ///< 发给触发的core
    OSP_IRQ_TO_BUTT,                ///< 无效值
} OspIrqTargetFilter_e;

/**
 * @brief   Acquires an interrupt lock.
 *
 * Interrupts will be disabled.  On SMP configurations this function acquires
 * an SMP lock.
 *
 * This function can be used in thread and interrupt context.
 *
 * @param   _lock [in/out], The interrupt lock
 * @param   _lock_context [in/out], The local interrupt lock context for an
 * acquire and release pair.
 * @return  NA
 * @warning NA
 * @note    NA
 */
#define OSP_INTERRUPT_LOCK_ACQUIRE(_lock, _lock_context) INNER_OSP_INTERRUPT_LOCK_ACQUIRE(_lock, _lock_context)

/**
 * @brief   Releases an interrupt lock.
 *
 * The interrupt status will be restored.  On SMP configurations this function
 * releases an SMP lock.
 *
 * @param   _lock [in/out], The interrupt lock
 * @param   _lock_context [in/out], The local interrupt lock context for an
 * acquire and release pair.
 * @return  NA
 * @warning NA
 * @note    NA
 */
#define OSP_INTERRUPT_LOCK_RELEASE(_lock, _lock_context) INNER_OSP_INTERRUPT_LOCK_RELEASE(_lock, _lock_context)

/**
 * @brief   Acquires an interrupt lock in the corresponding interrupt service
 * routine.
 *
 * The interrupt status will remain unchanged.  On SMP configurations this
 * function acquires an SMP lock.
 *
 * In case the corresponding interrupt service routine can be interrupted by
 * higher priority interrupts and these interrupts enter the critical section
 * protected by this lock, then the result is unpredictable.
 *
 * @param   _lock [in/out], The interrupt lock
 * @param   _lock_context [in/out], The local interrupt lock context for an
 * acquire and release pair.
 * @return  NA
 * @warning NA
 * @note    NA
 */
#define OSP_INTERRUPT_LOCK_ACQUIRE_ISR(_lock, _lock_context) INNER_OSP_INTERRUPT_LOCK_ACQUIRE_ISR(_lock, _lock_context)

/**
 * @brief   Releases an interrupt lock in the corresponding interrupt service
 * routine.
 *
 * The interrupt status will remain unchanged.  On SMP configurations this
 * function releases an SMP lock.
 *
 * @param   _lock [in/out], The interrupt lock
 * @param   _lock_context [in/out], The local interrupt lock context for an
 * acquire and release pair.
 * @return  NA
 * @warning NA
 * @note    NA
 */
#define OSP_INTERRUPT_LOCK_RELEASE_ISR(_lock, _lock_context) INNER_OSP_INTERRUPT_LOCK_RELEASE_ISR(_lock, _lock_context)

/**
 * @brief   Initializes an interrupt lock.
 *
 * Concurrent initialization leads to unpredictable results.
 *
 * @param   _lock [in/out], The interrupt lock.
 * @param   _name [in], The name for the interrupt lock.  This name must be a
 * string persistent throughout the life time of this lock.  The name is only
 * used if profiling is enabled.
 * @return  NA
 * @warning NA
 * @note    NA
 */
#define OSP_INTERRUPT_LOCK_INITIALIZE(_lock, _name) INNER_OSP_INTERRUPT_LOCK_INITIALIZE(_lock, _name)

/**
 * @brief   Destroys an interrupt lock.
 *
 * Concurrent destruction leads to unpredictable results.
 *
 * @param   _lock [in/out], The interrupt lock.
 * @return  NA
 * @warning NA
 * @note    NA
 */
#define OSP_INTERRUPT_LOCK_DESTROY(_lock) INNER_OSP_INTERRUPT_LOCK_DESTROY(_lock)

/**
 * @brief   This macro disables the interrupts on the current processor.
 *
 * On SMP configurations this will not ensure system wide mutual exclusion.
 * Use interrupt locks instead.
 *
 * @param   _isr_cookie [in], The previous interrupt level is returned.  The
 * type of this variable must be InnerOspInterruptLevel_t.
 * @return  NA
 * @warning NA
 * @note    NA
 */
#define OSP_INTERRUPT_LOCAL_DISABLE(_isr_cookie) INNER_OSP_INTERRUPT_LOCAL_DISABLE(_isr_cookie)

/**
 * @brief   This macro restores the previous interrupt level on the current
 * processor.
 * @param   _isr_cookie [in], The previous interrupt level returned by
 * OSP_INTERRUPT_LOCAL_DISABLE().
 * @return  NA
 * @warning NA
 * @note    NA
 */
#define OSP_INTERRUPT_LOCAL_ENABLE(_isr_cookie) INNER_OSP_INTERRUPT_LOCAL_ENABLE(_isr_cookie)

/**
 * @brief   Initializer for static initialization of interrupt locks.
 * @param   _name , The name for the interrupt lock.  It must be a string.  The
 * name is only used if profiling is enabled.
 * @return  NA
 * @warning NA
 * @note    NA
 */
#define OSP_INTERRUPT_LOCK_INITIALIZER(_name) INNER_OSP_INTERRUPT_LOCK_INITIALIZER(_name)

#if !defined(PS3OS_SMP)

/**
 * @brief   使能中断
 * @param   previousLevel [in],
 * 需要传入需要设置的cpsr值，这个值必须是ospInterruptEnables的返回值
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospInterruptEnable(OspInterruptLevel_t previousLevel);

/**
 * @brief   禁止中断
 * @param   无
 * @return  返回当前cpsr的值 ，
 * 在调用ospInterruptEnables的时候需要把这个参数作为入参传入
 * @warning 无
 * @note    无
 */
OspInterruptLevel_t ospInterruptDisable(void);

/**
 * @brief   相当于连续调用 ospInterruptEnable 和 ospInterruptDisable
 * @param   previousLevel [in],
 * 需要传入需要设置的cpsr值，这个值必须是ospInterruptEnables的返回值
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospInterruptFlash(OspInterruptLevel_t previousLevel);


/**
 * @brief   获取中断是否打开
 * @param   无
 * @return  true(disabled) or false(enabled)
 * @warning 无
 * @note    无
 */
bool ospInterruptIsDisabled(void);

#endif

/**
 * @brief   判断当前是否在中断上下文
 * @param   无
 * @return  true or false
 * @warning 无
 * @note    无
 */
bool ospInterruptIsInProgress(void);

/**
 * @brief   获取中断等级
 * @param   level , 任务模式, 例如 OSP_DEFAULT_MODES
 * @return  返回被 OSP_INTERRUPT_MASK 过滤的值(中断等级)
 * @warning 无
 * @note    无
 */
OspMode ospInterruptLevelBody(uint32_t level);

/**
 * @brief   注册中断处理函数
 * @param   vector ,硬件中断号
 * @param   info [in] ,描述中断处理入口的字符串
 * @param   options ,选择中断处理方式，有以下选项：
            OSP_INTERRUPT_UNIQUE  (最常用)当前中断号只对应一个中断处理函数
            OSP_INTERRUPT_SHARED  当前中断号可被多个中断处理函数共用
            OSP_INTERRUPT_REPLACE 当前要注册的@handler会强制替换最初的@handler
 * @param   handler ,待注册的中断处理函数
 * @param   arg [in/out],中断处理函数的参数
 * @return  注册操作的结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospInterruptHandlerInstall(OspVector vector, const char *info, OspOption options,
                                           OspInterruptHandler handler, void *arg);

/**
 * @brief   删除中断处理函数
 * @param   vector ,硬件中断号
 * @param   info [in] ,描述中断处理入口的字符串
 * @param   handler ,待删除的中断处理函数
 * @param   void *arg [in/out],中断处理函数的参数
 * @return  删除操作的结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospInterruptHandlerRemove(OspVector vector, OspInterruptHandler handler, void *arg);

/**
 * @brief   设置中断控制器触发方式
 * @param   id ,硬件中断号
 * @param   mode  ,触发方式
 * @return
 * @warning 无
 * @note    无
 */
void ospInterruptIDSetTriggerMode(uint32_t id, OspTriggerMode_e mode);

/**
 * @brief   设置中断优先级
 * @param   vector ,   硬件中断号
 * @param   Priority , 中断优先级
 * @return  操作结果
 * @warning 1. 当前对中断嵌套的支持如下：
 *             sas-expander/pcie-switch 的 FPGA 硬件平台
 *             pcie-switch 的 EMU 硬件平台
 *             pcie-switch 的 HAPS 硬件平台
 *             xilinx-zynq 的 QEMU 模拟平台
 *          2. 禁止配置timer0(系统滴答)和uart0(console终端)的中断优先级
            3. 对于不支持中断嵌套的平台，本接口会返回 OSP_INTERNAL_ERROR
 * @note    无
 */
OspStatusCode_e ospInterruptSetPriority(OspVector vector, OspInterruptPriority_e priority);

/**
 * @brief   设置中断亲和性
 * @param   vector , 硬中断号
 * @param   affinity_size  ,affinity bit域的大小
 * @param   affinity, bit域 ，每个bit 代表亲和一个core
 * @return
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospInterruptSetAffinity(OspVector vector, size_t affinity_size,const cpu_set_t *affinity);

/**
 * @brief 软件触发核间中断
 *
 * @Param vector  按照规划0~3预留给os，4~15业务可以按照规划使用
 * @Param filter  枚举值 详细示意参考OspIrqTargetFilter_e
 * @Param targets 在filter为 OSP_IRQ_TO_ALL_IN_LIST时有效，共8bit，每bit表示一个core，bit0为1表示core0
 *
 * @Returns
 */
OspStatusCode_e ospInterruptGenSoftIrq(OspVector vector, OspIrqTargetFilter_e filter, uint8_t targets);

/**
 * @brief   使能中断向量
 * @param   vector ,   硬件中断号
 * @return  无
 * @warning 无
 * @note    无
 */
void ospInterruptVectorEnable(OspVector vector);

/**
 * @brief   去使能中断向量
 * @param   vector ,   硬件中断号
 * @return  无
 * @warning 无
 * @note    无
 */
void ospInterruptVectorDisable(OspVector vector);

/**
 * @brief   清理特定vector中断对应的中断控制器信息
 * @param   vector ,   硬件中断号
 * @return  无
 * @warning 无
 * @note    无
 */
void ospInterruptVectorUninit(OspVector vector);

void ospIPISendBroadcast( uint32_t ipi );

#ifdef __cplusplus
}
#endif

#endif
