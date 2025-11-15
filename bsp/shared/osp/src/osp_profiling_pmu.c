/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_profiling_pmu.c
 * @author  pengqianheng
 * @date    2020.12.12
 * @brief   封装   PMU 监测的对外接口
 * @note    目前针对的是 Cortex-R5 的处理器
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <rtems.h>
#include <rtems/counter.h>
#include <rtems/profiling.h>
#include <rtems/thread.h>
#include <osp_task.h>
#include <bsp.h>
#include <osp_profiling_pmu.h>
#include <osp_errno.h>
#include <osp_status.h>
#include <osp_cpuopts.h>
#ifdef PS3OS_SMP
#include <rtems/score/smpimpl.h>
#endif

#define PMUCTL_L1D_TLB_REFILL_RD 0x004c
#define PMUCTL_L1D_TLB_REFILL_WR 0x004d

#if defined(PS3OS_EXPANDER_V200) || defined(PS3OS_NETCHIP) \
    || defined(PS3OS_IBCHIP) || defined(PS3OS_HBA_V200) || defined(PS3OS_IBCHIP_V100)\
    || defined(PS3OS_ZYNQMP) || defined(PS3OS_SWITCH_V200) || defined(PS3OS_NIC_100G) \
    || defined(PS3OS_SWITCH_V300)

#include <bsp/arm-pmu.h>
#include <bsp/irq.h>
#include <bsp/irq-info.h>

#if defined(PS3OS_EXPANDER_V200) || defined(PS3OS_NETCHIP) || defined(PS3OS_IBCHIP) || defined(PS3OS_NIC_100G) || defined(PS3OS_SWITCH_V300) || defined(PS3OS_IBCHIP_V100)
#include <osp_sys_clock.h>
#endif

/**
 * PMU功能的相关定义
 */
#if defined(PS3OS_EXPANDER_V200)
#define MAX_PMU_COUNTER (4)
static uint32_t irq_num = SGR5_IRQ_CPU_PMU;
#elif defined(PS3OS_SWITCH_V300)
#define MAX_PMU_COUNTER (4)
static uint32_t irq_num = SGR5_IRQ_CPU_PMU;
#elif defined(PS3OS_SWITCH_V200)
#define MAX_PMU_COUNTER (4)
static uint32_t irq_num = SGR5_IRQ_CPU_PMU;
#elif defined(PS3OS_HBA_V200)
#define MAX_PMU_COUNTER (6)
static uint32_t irq_num = SGA72_IRQ_PMU;
#elif defined(PS3OS_ZYNQMP)
#define MAX_PMU_COUNTER (6)
static uint32_t irq_num = ZYNQMP_IRQ_PMU;
#elif defined(PS3OS_NETCHIP)
#define MAX_PMU_COUNTER (4)
static uint32_t irq_num = SGR5_IRQ_CPU_PMU;
#elif defined(PS3OS_IBCHIP) || defined(PS3OS_IBCHIP_V100)
#define MAX_PMU_COUNTER (4)
static uint32_t irq_num = SGR5_IRQ_CPU_PMU;
#elif defined(PS3OS_NIC_100G)
#define MAX_PMU_COUNTER (4)
static uint32_t irq_num = SGR5_IRQ_CPU_PMU;
#endif

/* 性能监测的事件类型和事件描述 */
typedef struct OspProfilingMap {
    uint32_t eventType;
    const char eventBriefInfo[16];
} OspProfilingMap_t;

/* 性能监测功能的相关配置 */
typedef struct OspProfilingConfig {
    uint32_t duration;                                  ///< 监测时长
    OspProfilingEvent_e queue[MAX_PMU_COUNTER]; ///< 用户指定的事件
    uint64_t toNanoSecsScaler;                          ///< 计数器的数值转换为纳秒的换算比例
    uint32_t length;
} OspProfilingConfig_t;

typedef struct OspProfilingInfo {
    rtems_id taskID[CPU_COUNT];
    uint32_t completed[CPU_COUNT];///< 当前CPU核是否完成
    uint64_t events_info[CPU_COUNT][MAX_PMU_COUNTER + 1]; ///< 记录counter的计数用于最后打印
    uint32_t overflow[CPU_COUNT][MAX_PMU_COUNTER + 1];
    uint32_t HeaderReported;
} OspProfilingInfo_t;

OspProfilingInfo_t g_OspProfilingInfo;

static rtems_mutex g_pmu_mutex;

///< PMU 监测功能的配置
static OspProfilingConfig_t ospProfilingConf;

///< PMU 事件类型映射表( 当前只支持 ARMv7 )
static OspProfilingMap_t armv7ProfilingMapTable[OSP_PROFILING_EVENT_MAX] = {
    [OSP_PROFILING_L1_ICACHE_REFILL] =
    {
        .eventType      = ARMV7_PMUCTR_L1_ICACHE_REFILL,
        .eventBriefInfo = "I-Cache Miss",
    },
    [OSP_PROFILING_L1_ICACHE_ACCESS] =
    {
        .eventType      = ARMV7_PMUCTR_L1_ICACHE_ACCESS,
        .eventBriefInfo = "I-Cache Access",
    },
    [OSP_PROFILING_L1_DCACHE_REFILL] =
    {
        .eventType      = ARMV7_PMUCTR_L1_DCACHE_REFILL,
        .eventBriefInfo = "D-Cache Miss",
    },

    [OSP_PROFILING_L1_DCACHE_ACCESS] =
    {
        .eventType      = ARMV7_PMUCTR_L1_DCACHE_ACCESS,
        .eventBriefInfo = "D-Cache Access",
    },
    [OSP_PROFILING_MEM_READ] =
    {
        .eventType      = ARMV7_PMUCTR_MEM_READ,
        .eventBriefInfo = "Load",
    },
    [OSP_PROFILING_MEM_WRITE] =
    {
        .eventType      = ARMV7_PMUCTR_MEM_WRITE,
        .eventBriefInfo = "Store",
    },
    [OSP_PROFILING_IMM_BRANCH] =
    {
        .eventType      = ARMV7_PMUCTR_PC_IMM_BRANCH,
        .eventBriefInfo = "Branch Excuted",
    },
    [OSP_PROFILING_EXC_TAKEN] =
    {
        .eventType      = ARMV7_PMUCTR_EXC_TAKEN,
        .eventBriefInfo = "Except Taken",
    },
    [OSP_PROFILING_EXC_EXECUTED] =
    {
        .eventType      = ARMV7_PMUCTR_EXC_EXECUTED,
        .eventBriefInfo = "Except Return",
    },
    [OSP_PROFILING_CID_WRITE] =
    {
        .eventType      = ARMV7_PMUCTR_CID_WRITE,
        .eventBriefInfo = "Change CID",
    },
    [OSP_PROFILING_BRANCH_MIS_PRED] =
    {
        .eventType      = ARMV7_PMUCTR_PC_BRANCH_MIS_PRED,
        .eventBriefInfo = "Branch Miss",
    },
    [OSP_PROFILING_BRANCH_PRED] =
    {
        .eventType      = ARMV7_PMUCTR_PC_BRANCH_PRED,
        .eventBriefInfo = "Branch Predict",
    },
    [OSP_PROFILING_L1D_TLB_REFILL_RD] =
    {
        .eventType      = PMUCTL_L1D_TLB_REFILL_RD,
        .eventBriefInfo = "L1 DTLB RD Miss",
    },
    [OSP_PROFILING_L1D_TLB_REFILL_WR] =
    {
        .eventType      = PMUCTL_L1D_TLB_REFILL_WR,
        .eventBriefInfo = "L1 DTLB WR Miss",
    },
    [OSP_PROFILING_L1D_TLB_REFILL] =
    {
        .eventType      = ARMV7_PMUCTR_DTLB_REFILL,
        .eventBriefInfo = "L1 DTLB Miss",
    },
    [OSP_PROFILING_L1I_TLB_REFILL] =
    {
        .eventType      = ARMV7_PMUCTR_ITLB_REFILL,
        .eventBriefInfo = "L1 ITLB Miss",
    },
    [OSP_PROFILING_L2D_CACHE_REFILL] =
    {
        .eventType      = ARMV7_PMUCTR_L2_CACHE_REFILL,
        .eventBriefInfo = "L2-Dcache Miss",
    },
};

/**
 * @brief   统一申请profilling资源
 * @param
 * @return  NA
 * @warning
 * @note
 */
static OspStatusCode_e ospProfilingResourceInit(void)
{
    rtems_mutex_init(&g_pmu_mutex, "pmu_mutex");

    memset(ospProfilingConf.queue, 0, sizeof(ospProfilingConf.queue));
    memset(g_OspProfilingInfo.taskID, 0, sizeof(g_OspProfilingInfo.taskID));
    memset(g_OspProfilingInfo.completed, 0, sizeof(g_OspProfilingInfo.completed));
    memset(g_OspProfilingInfo.events_info, 0, sizeof(g_OspProfilingInfo.events_info));
    memset(g_OspProfilingInfo.overflow, 0, sizeof(g_OspProfilingInfo.overflow));

    return OSP_SUCCESSFUL;

}

/**
 * @brief   统一释放profilling资源
 * @param
 * @return  NA
 * @warning
 * @note
 */
static void ospProfilingResourceUninit(void)
{
    g_OspProfilingInfo.HeaderReported = 0;
    rtems_mutex_destroy(&g_pmu_mutex);
}

/**
 * @brief   PMU 中断处理函数
 * @param   args [in], 传入中断处理函数的参数
 * @return  NA
 * @warning 不能在该函数内调用 printf
 * @note    args 可以忽略
 */
static void ospPmuIrqHandler(void *args)
{
    (void)args;

    uint32_t cpu_index_self = _SMP_Get_current_processor();
    uint32_t flag = pmu_read_counter_overflow_flag( );

    if((1<<31) & flag)
    {
        (g_OspProfilingInfo.overflow[cpu_index_self][0])++;
        pmu_clear_cycle_overflow_flag( );
    }

    for(uint32_t i = 0; i < MAX_PMU_COUNTER; i++)
    {
        if((1<<i) & flag)
        {
            (g_OspProfilingInfo.overflow[cpu_index_self][i+1])++;
            pmu_clear_event_overflow_flag(i);
        }
    }
}

/**
 * @brief   计算计数值到纳秒的换算比例
 * @param   frequency , 计数器的频率
 * @return  NA
 * @warning NA
 * @note    NA
 */
static void ospCalculateScaler(unsigned int frequency)
{
    uint64_t ns_per_s = UINT64_C(1000000000);

    ospProfilingConf.toNanoSecsScaler = ((ns_per_s << 32) + frequency - 1) / frequency;
}

/**
 * @brief   将计数器的 tick 值换算为纳秒
 * @param   ticks , 计数器的 tick 数值
 * @return  NA
 * @warning NA
 * @note    NA
 */
static uint64_t ospConvertTicksToNanoseconds(unsigned int ticks)
{
    return (uint32_t)((ticks * ospProfilingConf.toNanoSecsScaler) >> 32);
}

/**
 * @brief   映射事件类型
 * @param   event , 事件类型
 * @return  NA
 * @warning NA
 * @note    NA
 */
static unsigned int ospEventTypeMapping(OspProfilingEvent_e event)
{
    unsigned int type = 0;

    if(event < OSP_PROFILING_EVENT_MAX) {
        type = armv7ProfilingMapTable[event].eventType;
    } else {
        printf("[Profiling] Unknow Event Type!\n");
    }

    return type;
}

/**
 * @brief   映射事件描述
 * @param   event , 事件类型
 * @return  NA
 * @warning NA
 * @note    NA
 */
static const char *ospEventBriefMapping(OspProfilingEvent_e event)
{
    const char *ptrBrief;

    if(event < OSP_PROFILING_EVENT_MAX) {
        ptrBrief = armv7ProfilingMapTable[event].eventBriefInfo;
    } else {
        ptrBrief = NULL;
    }

    return ptrBrief;
}

/**
 * @brief   打印分割线
 * @param   length , 分割线的长度
 * @return  NA
 * @warning NA
 * @note    分割线以 1 个 - 为单位长度
 */
static inline void ospPrintLine(uint32_t length)
{
    for(uint32_t i = 0; i < length; i++) {
        printf("-");
    }
    printf("\n");
}

/**
 * @brief   遍历各个表项
 * @param   NA
 * @return  NA
 * @warning NA
 * @note    NA
 */
static void ospProfilingIterate(void)
{
    uint32_t cpu_index_self = _SMP_Get_current_processor();

    ///< v7 pmu cpu cycle计数是32位，也可能溢出
    g_OspProfilingInfo.events_info[cpu_index_self][0] = pmu_read_cycle_counter( ) +
        (uint64_t)0xffffffff * g_OspProfilingInfo.overflow[cpu_index_self][0];
    ///< event 计数32位，可能溢出
    g_OspProfilingInfo.events_info[cpu_index_self][1] = pmu_read_event_counter(0) +
        (uint64_t)0xffffffff * g_OspProfilingInfo.overflow[cpu_index_self][1];

    const char *ptrInfo = NULL;
    for(uint32_t i = 0; i < ospProfilingConf.length; i++) {
        ptrInfo = ospEventBriefMapping(ospProfilingConf.queue[i]);
        if(ptrInfo != NULL) {
            ///< event 计数32位，可能溢出
            g_OspProfilingInfo.events_info[cpu_index_self][2+i] = pmu_read_event_counter(1 + i) +
                (uint64_t)0xffffffff * g_OspProfilingInfo.overflow[cpu_index_self][2 + i];
        } else {
            continue;
        }
    }

    g_OspProfilingInfo.completed[cpu_index_self] = 0;
}

/**
 * @brief   按照CPU顺序打印结果
 * @param   NA
 * @return  NA
 * @warning NA
 * @note    NA
 */
static void ospProfilingPrintResult(void)
{
    uint32_t cpu_num = _SMP_Get_processor_maximum();
    uint64_t cycles;
    uint64_t inst;
    uint64_t events;
    float cpi;
    const char *ptrInfo = NULL;

    for (uint32_t i=0; i<cpu_num; i++)
    {
        cycles = g_OspProfilingInfo.events_info[i][0];
        printf("%-5d | %-15s | %-15llu|%-17llu| %-10s| %-8s| %-6s\n", i, "CPU Cycles", cycles, ospConvertTicksToNanoseconds(cycles),
                "-", "-", "-");
        inst = g_OspProfilingInfo.events_info[i][1];
        cpi = ((float)cycles / (float)inst);
        double incidence = ((double)inst / (double)cycles) * 100;
        printf("%-5d | %-15s | %-15llu|%-17llu| %-9.4f%%| %-7d%%| %-6.4f\n", i, "Instructions", inst, ospConvertTicksToNanoseconds(cycles),
                incidence, 100, cpi);

        for (uint32_t j=0; j<ospProfilingConf.length; j++)
        {
            ptrInfo = ospEventBriefMapping(ospProfilingConf.queue[j]);
            if(ptrInfo == NULL)
            {
                continue;
            }
            events = g_OspProfilingInfo.events_info[i][2 + j];
            double incidence = ((double)events / (double)cycles) * 100;
            if(ospProfilingConf.queue[j] == OSP_PROFILING_MEM_READ ||
                    ospProfilingConf.queue[j] == OSP_PROFILING_MEM_WRITE ||
                    ospProfilingConf.queue[j] == OSP_PROFILING_IMM_BRANCH)
            {
                double ratio = ((double)events / (double)inst);
                printf("%-5d | %-15s | %-15llu|%-17llu| %-9.4f%%| %-7.4f%%| %-6.4f\n", i, ptrInfo, events,
                        ospConvertTicksToNanoseconds(cycles), incidence, ratio * 100, ratio * cpi);
            }
            else
            {
                printf("%-5d | %-15s | %-15llu|%-17llu| %-9.4f%%| %-8s| %-6s\n", i, ptrInfo, events,
                        ospConvertTicksToNanoseconds(cycles), incidence, "-", "-");
            }
        }
    }
}

/**
 * @brief   打印整体的监测结果
 * @param   NA
 * @return  NA
 * @warning NA
 * @note    NA
 */
static void ospProfileReport(void)
{
    ///< PMU 报表的表头
    static const char reportHeader[] = {
        "------------------------------------------------------------------------------------------\n"
            "                            Performance Monitor Information\n"
            "Note: 1. Incidence       = Event / CPU Cycles\n"
            "      2. Ratio           = Event / Instructions\n"
            "      3. CPI             = CPU Cycles / Instructions\n"
            "      4. Averal Disabled = Total Disabled / Disabled Count\n"
            "      5. Averal Int Time = Total Int Time / Interrupt Count\n"
            "      6. During Disabled time, thread can't be dispatched\n"
            "------+-----------------+----------------+-----------------+-----------+---------+---------\n"
            "CPUID | Event           | Counts         | Time (ns)       | Incidence | Ratio   | CPI     \n"
            "------+-----------------+----------------+-----------------+-----------+---------+---------\n"};
    uint32_t cpu_num = _SMP_Get_processor_maximum();
    uint32_t completed = 0;

    rtems_mutex_lock(&g_pmu_mutex);

    if ( !g_OspProfilingInfo.HeaderReported)
    {
        g_OspProfilingInfo.HeaderReported = 1;
        ///< 打印表头
        printf("%s", reportHeader);
    }
    ///< 记录各个表
    ospProfilingIterate( );

    for(uint32_t i=0; i<cpu_num; i++)
    {
        completed = completed || g_OspProfilingInfo.completed[i];
    }

    ///< 打印表尾
    if (!completed)
    {
        ospProfilingPrintResult();
        ospPrintLine(91);
    }
    rtems_mutex_unlock(&g_pmu_mutex);
    if (!completed)
    {
        ospProfilingResourceUninit();
    }
}

static void ospPMUDisableAllCnt(void)
{
    pmu_disable_cycle_counter( );
    pmu_disable_cycle_interrupt( );
    pmu_all_counters_stop( );
    for(uint32_t i = 0; i < MAX_PMU_COUNTER; i++) {
        pmu_disable_event_counter(i);
        pmu_disable_event_interrupt(i);
    }
}

static void ospPMUResetAllCnt(void)
{
    pmu_reset_cycle_counter( );
    pmu_reset_event_counter( );
}

static void ospPMUConfigAndEnableCnt(void *args)
{
    OspProfilingConfig_t* data = (OspProfilingConfig_t*)args;

    ///< 配置 cycle 计数器
    // pmu_write_cycle_counter(ospPreservePmuCycleCounts(perfDuration));
    pmu_enable_cycle_counter( );
    pmu_enable_cycle_interrupt( );

    ///< 配置默认的 event 计数器
    pmu_set_event_type(0, ARMV7_PMUCTR_INSTR_EXECUTED);
    pmu_enable_event_interrupt(0);
    pmu_enable_event_counter(0);

    ///< 配置可选的 event 计数器
    for(uint32_t i = 0; i < data->length; i++) {
        pmu_set_event_type(1 + i, ospEventTypeMapping(data->queue[i]));
        pmu_enable_event_counter(1 + i);
        pmu_enable_event_interrupt(1 + i);
    }
}

/**
 * @brief   执行 PMU 监测任务
 * @param   ignored , 传入任务的参数
 * @return  NA
 * @warning NA
 * @note    入参可忽略
 */
static void ospDoProfiling(rtems_task_argument arg)
{
    rtems_event_set event;
    uint64_t wait = 0;
    uint32_t current;
    uint32_t latest;
    uint32_t delta = 0;
    uint32_t cpu_index_self = _SMP_Get_current_processor();

    g_OspProfilingInfo.completed[cpu_index_self] = 1;
    wait = ospProfilingConf.duration * rtems_clock_get_ticks_per_second( );
    if(ospProfilingConf.duration == OSP_PROFILING_INFINITE || wait >= OSP_PROFILING_INFINITE) {
        wait = OSP_PROFILING_INFINITE;
        printf("\r\nPerformance monitor start (Estimated: Inf profilingID: "
                "0x%08X)\n",g_OspProfilingInfo.taskID[cpu_index_self]);
    } else {
        printf("\r\nPerformance monitor start (Estimated: %ds profilingID: "
                "0x%08X)\n",
                ospProfilingConf.duration, g_OspProfilingInfo.taskID[cpu_index_self]);
    }

    ospPMUDisableAllCnt();
    ospPMUResetAllCnt();
    ospPMUConfigAndEnableCnt((void *)arg);

    pmu_all_counters_start( );
    current = time(NULL);
    do {
        if(RTEMS_SUCCESSFUL == rtems_event_receive(RTEMS_ALL_EVENTS, RTEMS_WAIT | RTEMS_EVENT_ANY,
                    RTEMS_MILLISECONDS_TO_TICKS(50), &event)) {
            //stop 时间使用30BIT通知
            if((1<<30) & event) {
                break;
            }
        }
        latest = time(NULL);
        delta  = latest - current;
    } while(delta < ospProfilingConf.duration);
    pmu_all_counters_stop( );

    ///< 报告监测结果
    ospProfileReport( );

    ///< 释放 PMU 资源
    ospPMUDisableAllCnt();

    rtems_task_exit( );
}

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

static void ospPmuUsage()
{
    printf("\nCount Events:\n"
            "\t0   Instruction cache miss\n"
            "\t1   Instruction cache access\n"
            "\t2   Data cache miss\n"
            "\t3   Data cache access\n"
            "\t4   Memory load inst executed\n"
            "\t5   Memory store inst executed\n"
            "\t6   Jump inst executed\n"
            "\t7   Exception taken\n"
            "\t8   Exception return\n"
            "\t9   Change to Context ID executed\n"
            "\t10  Branch predict successfully\n"
            "\t11  Branch predict unsuccessfully\n"
            "\t12  Level 1 data tlb miss, read\n"
            "\t13  Level 1 data tlb miss, write\n"
            "\t14  Level 1 data tlb miss\n"
            "\t15  Level 1 inst tlb miss\n"
          );
}

int ospProfilingStart(unsigned int duration, unsigned int *queue, unsigned int length)
{
    OspErrno_e status = OSP_OK;
    uint32_t cpu_num = _SMP_Get_processor_maximum();
    cpu_set_t cpuset;

    if(duration == 0) {
        printf("Performance monitor didn't start.\n");
        goto exit;
    }

    if(queue == NULL || length == 0) {
        printf("Performance monitor failed to start! Invalid queue.\n");
        status = OSP_PROFILING_QUEUE_EMPTY;
        goto exit;
    }

    if(length > MAX_PMU_COUNTER - 1) {
        printf("Performance monitor failed to start! Invalid length: %d\n", length);
        status = OSP_PROFILING_QUEUE_LEN_IVNA;
        goto exit;
    }

    if(ospProfilingResourceInit() != OSP_SUCCESSFUL)
    {
        printf("Performance profiling rescorece failed!\n");
        status = OSP_NO_MEMORY;
        goto exit;
    }

    ///< 配置 cycle   计数器的 tick 数到纳秒的转换比例
#if defined(PS3OS_EXPANDER_V200) || defined(PS3OS_NETCHIP) || defined(PS3OS_IBCHIP) || defined(PS3OS_NIC_100G) || defined(PS3OS_IBCHIP_V100) ||  defined(PS3OS_SWITCH_V300)
    ospCalculateScaler(ospGetSysClock());
#else
    ospCalculateScaler(BSP_PLL_OUT_CLOCK);
#endif
    ///< 配置length
    ospProfilingConf.length = length;
    ///< 配置测试时长
    ospProfilingConf.duration = duration;
    ///< 配置可选的 event 计数器
    for(unsigned int i=0; i<length; i++)
    {
        ospProfilingConf.queue[i] = queue[i];
    }

    for (uint32_t cpu_index = 0; cpu_index < cpu_num; cpu_index++)
    {

        ///< 开启 PMU 监测任务
        assert(RTEMS_SUCCESSFUL == rtems_task_create(rtems_build_name('P', 'E', 'R', 'F'), RTEMS_MINIMUM_PRIORITY, 256,
                    RTEMS_DEFAULT_MODES, RTEMS_LOCAL, &(g_OspProfilingInfo.taskID[cpu_index])));
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_index, &cpuset);
        assert(OSP_SUCCESSFUL == ospTaskSetAffinity(g_OspProfilingInfo.taskID[cpu_index], sizeof(cpuset), &cpuset));

        assert(RTEMS_SUCCESSFUL == rtems_task_start(g_OspProfilingInfo.taskID[cpu_index], ospDoProfiling, (rtems_task_argument)&ospProfilingConf));
    }

    ///< 注册 PMU 中断处理函数
    rtems_interrupt_handler_install(irq_num, "PMU-", RTEMS_INTERRUPT_UNIQUE, ospPmuIrqHandler, NULL);

exit:
    ospPmuUsage();
    return status;
}

/**
 * @brief   停止 PMU 监测并输出结果

 * @param   NA
 * @return  NA
 * @warning 非阻塞
 * @note    NA
 */
void ospProfilingStop(void)
{
    uint32_t cpu_num = _SMP_Get_processor_maximum();

    for (uint32_t cpu_index = 0; cpu_index < cpu_num; cpu_index++)
    {
        rtems_event_send(g_OspProfilingInfo.taskID[cpu_index], 1<<30);
    }
}

/**
 * @brief   执行 PMU 监测任务
 * @param   ignored , 传入任务的参数
 * @return  NA
 * @warning NA
 * @note    入参可忽略
 */
static void ospPerfStart(void *args)
{
    ospPMUDisableAllCnt();
    ospPMUResetAllCnt();
    ospPMUConfigAndEnableCnt(args);
    pmu_all_counters_start( );
}

static void ospPerfStop(void *args)
{
    (void)args;

    uint32_t cpu_index_self = _SMP_Get_current_processor();

    ///< 释放 PMU 资源
    ospPMUDisableAllCnt();

    ///< cpu cycle计数是64位，理论上不会溢出，所以这里不做溢出计算处理
    g_OspProfilingInfo.events_info[cpu_index_self][0] = pmu_read_cycle_counter( );
    ///< event 计数32位，可能溢出
    g_OspProfilingInfo.events_info[cpu_index_self][1] = pmu_read_event_counter(0) +
        (uint64_t)0xffffffff * g_OspProfilingInfo.overflow[cpu_index_self][1];
    const char *ptrInfo = NULL;
    for(uint32_t i = 0; i < ospProfilingConf.length; i++) {
        ptrInfo = ospEventBriefMapping(ospProfilingConf.queue[i]);
        if(ptrInfo != NULL) {
            g_OspProfilingInfo.events_info[cpu_index_self][2 + i] = pmu_read_event_counter(1 + i) +
                (uint64_t)0xffffffff * g_OspProfilingInfo.overflow[cpu_index_self][2 + i];
        } else {
            continue;
        }
    }
}

int ospPMUPerfStart(OspPMUUserEvt_s *pmuUserEvt)
{
    int ret = 0;

    if(pmuUserEvt == NULL || pmuUserEvt->evt_cnt == 0) {
        printf("Performance monitor failed to start! Invalid queue.\n");
        ret = -1;
        goto exit;
    }
    if(pmuUserEvt->evt_cnt > (MAX_PMU_COUNTER - 1)) {
        printf("Performance monitor failed to start! Invalid length: %d\n", pmuUserEvt->evt_cnt);
        ret = -1;
        goto exit;
    }
    if(ospProfilingResourceInit() != OSP_SUCCESSFUL) {
        printf("Performance profiling rescorece failed!\n");
        ret = -1;
        goto exit;
    }

    ///< 配置length
    ospProfilingConf.length = pmuUserEvt->evt_cnt;
    ///< 配置可选的 event 计数器
    for(unsigned int i = 0; i < ospProfilingConf.length; i++) {
        ospProfilingConf.queue[i] = pmuUserEvt->evt_queue[i];
    }
    ///< 配置 cycle   计数器的 tick 数到纳秒的转换比例
#if defined(PS3OS_EXPANDER_V200) || defined(PS3OS_NETCHIP) || defined(PS3OS_IBCHIP) || defined(PS3OS_NIC_100G) || defined(PS3OS_IBCHIP_V100)  || defined(PS3OS_SWITCH_V300)
    ospCalculateScaler(ospGetSysClock());
#else
    ospCalculateScaler(BSP_PLL_OUT_CLOCK);
#endif
    ///< 注册 PMU 中断处理函数
    rtems_interrupt_handler_install(irq_num, "PMU-", RTEMS_INTERRUPT_UNIQUE, ospPmuIrqHandler, NULL);
#if defined(PS3OS_SMP)
    _SMP_Othercast_action(ospPerfStart, &ospProfilingConf);
#endif
    ospPerfStart(&ospProfilingConf);

exit:
    return ret;
}

/**
 * @brief   停止 PMU 监测并输出结果

 * @param   NA
 * @return  NA
 * @warning 非阻塞
 * @note    NA
 */
int ospPMUPerfStop(OspPMUPerfData_s *pmuPerfData)
{
    int ret = 0;

    ospPerfStop(NULL);
#if defined(PS3OS_SMP)
    _SMP_Othercast_action(ospPerfStop, NULL);
#endif
    rtems_interrupt_handler_remove(irq_num, ospPmuIrqHandler, NULL);

    if (pmuPerfData == NULL) {
        ret = -1;
        goto exit;
    }

    ///< 报告监测结果
    const char *ptrInfo = NULL;
    for (uint32_t i = 0; i < _SMP_Get_processor_maximum(); i++) {
        pmuPerfData->cycles[i] = g_OspProfilingInfo.events_info[i][0];
        pmuPerfData->inst[i] = g_OspProfilingInfo.events_info[i][1];
        for (uint32_t j = 0; j < ospProfilingConf.length; j++) {
            ptrInfo = ospEventBriefMapping(ospProfilingConf.queue[j]);
            if(ptrInfo == NULL) {
                continue;
            }
            pmuPerfData->events[i][j] = g_OspProfilingInfo.events_info[i][2 + j];
        }
    }

exit:
    ospProfilingResourceUninit();
    return ret;
}

#endif
