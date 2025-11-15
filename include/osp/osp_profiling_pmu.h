/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_profiling_pmu.h
 * @author  pengqianheng
 * @date    2020.12.10
 * @brief   使用 PMU 监测 CPU 的性能
 * @note    当前只支持 ARMv7 (单核)的 PMUv2
 */

#ifndef _OSP_PROFILING_PMU_H_
#define _OSP_PROFILING_PMU_H_

#include <bsp.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OSP_PROFILING_INFINITE (0xffffffff)

/**
 * 支持监测的事件类型
 */
typedef enum OspProfilingEvent {
    OSP_PROFILING_L1_ICACHE_REFILL, ///< Instruction cache miss
    OSP_PROFILING_L1_ICACHE_ACCESS, ///< Instruction cache access
    OSP_PROFILING_L1_DCACHE_REFILL, ///< Data cache miss
    OSP_PROFILING_L1_DCACHE_ACCESS, ///< Data cache access
    OSP_PROFILING_MEM_READ,         ///< 读内存的指令(load)执行次数
    OSP_PROFILING_MEM_WRITE,        ///< 写内存的指令(store)执行次数
    OSP_PROFILING_IMM_BRANCH,       ///< 跳转指令(branch)的执行次数
    OSP_PROFILING_EXC_TAKEN,        ///< 发生异常的次数
    OSP_PROFILING_EXC_EXECUTED,     ///< 异常返回的次数(例如
                                    ///< RFE, MOVS PC, LDM Rn, {..,PC}^ )
    OSP_PROFILING_CID_WRITE,        ///< Change to Context ID executed
    OSP_PROFILING_BRANCH_MIS_PRED,  ///< 分支预测失败
    OSP_PROFILING_BRANCH_PRED,      ///< 分支预测成功

    OSP_PROFILING_L1D_TLB_REFILL_RD, ///< level 1 data tlb miss, read
    OSP_PROFILING_L1D_TLB_REFILL_WR, ///< level 1 data tlb miss, write
    OSP_PROFILING_L1D_TLB_REFILL,    ///< level 1 data tlb miss 
    OSP_PROFILING_L1I_TLB_REFILL,    ///< level 1 inst tlb miss
    OSP_PROFILING_L2D_CACHE_REFILL,    ///< level 2 data cache miss

    OSP_PROFILING_EVENT_MAX
} OspProfilingEvent_e;

/**
 * @brief   命令 PMU 开始执行事件监测
 * @param   duration , 监测时长，单位：秒
 * @param   queue [in], 待测事件类型的队列，类型参考 OspProfilingEvent_e
 * @param   length , 待测事件的长度，受 CPU 硬件限制，最多支持2个可选类型
 * @return  1. OSP_OK                  成功
 *          2. OSP_PROFILING_QUEUE_LEN_IVNA    length 参数无效
 *          3. OSP_PROFILING_QUEUE_EMPTY       queue 参数无效
 * @warning 非阻塞，不支持同时开启多个监测任务
 * @note    通常情况下，duration时间到后会自动停止监测并输出结果
 *          若想手动控制监测时长，可向 duraton 传入 OSP_PROFILING_INFINITE
 *          然后在希望停止的地方调用 ospProfilingStop() 即可打印监测结果
 */
int ospProfilingStart(unsigned int duration, unsigned int *queue, unsigned int length);

/**
 * @brief   停止 PMU 监测并输出结果
 * @param   NA
 * @return  NA
 * @warning 非阻塞
 * @note    NA
 */
void ospProfilingStop(void);

#if defined(PS3OS_EXPANDER_V200)
#define OSP_MAX_PMU_USER_EVENT  (3)
#define CPU_COUNT (1)
#elif defined(PS3OS_SWITCH_V300)
#define OSP_MAX_PMU_USER_EVENT  (3)
#define CPU_COUNT (1)
#elif defined(PS3OS_SWITCH_V200)
#define OSP_MAX_PMU_USER_EVENT  (3)
#define CPU_COUNT (1)
#elif defined(PS3OS_HBA_V200)
#define OSP_MAX_PMU_USER_EVENT  (5)
#define CPU_COUNT (4)
#elif defined(PS3OS_ZYNQMP)
#define OSP_MAX_PMU_USER_EVENT  (5)
#define CPU_COUNT (1)
#elif defined(PS3OS_NETCHIP)
#define OSP_MAX_PMU_USER_EVENT  (3)
#define CPU_COUNT (1)
#elif defined(PS3OS_IBCHIP)
#define OSP_MAX_PMU_USER_EVENT  (3)
#define CPU_COUNT (1)
#elif defined(PS3OS_NIC_100G)
#define OSP_MAX_PMU_USER_EVENT  (3)
#define CPU_COUNT (1)
#else
#define OSP_MAX_PMU_USER_EVENT  (3)
#define CPU_COUNT (1)
#endif

typedef struct OspPMUUserEvt {
    uint32_t evt_queue[OSP_MAX_PMU_USER_EVENT];
    uint32_t evt_cnt;
} OspPMUUserEvt_s;

typedef struct OspPMUPerfData {
    uint64_t cycles[CPU_COUNT];
    uint64_t inst[CPU_COUNT];
    uint64_t events[CPU_COUNT][OSP_MAX_PMU_USER_EVENT];
} OspPMUPerfData_s;

/**
 * @brief   业务流程中开启 PMU 相关事件统计
 * @param   queue [in], 待测事件类型的队列，类型参考 OspProfilingEvent_e
 * @param   length , 待测事件的长度，受PMU版本限制，最多支持5个可选类型
 * @return  1. 0        成功
 *          2. -1       失败
 * @warning 不支持同时开启多个监测任务，debug接口，调用者保证不能发生并发
 * @note    NA
 */
int ospPMUPerfStart(OspPMUUserEvt_s *pmuUserEvt);

/**
 * @brief   业务流程中停止 PMU 监测
 * @param   NA
 * @return  1. 0        成功
 *          2. -1       失败
 * @warning NA
 * @note    NA
 */
int ospPMUPerfStop(OspPMUPerfData_s *pmuPerData);

#ifdef __cplusplus
}
#endif

#endif
