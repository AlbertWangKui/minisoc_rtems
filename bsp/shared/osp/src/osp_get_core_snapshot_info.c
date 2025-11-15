#include <osp_get_core_snapshot_info.h>
#include <osp_scheduler_smp.h>

#include <rtems.h>
#include <rtems/score/percpu.h>
#include <rtems/score/isrlock.h>
#include <rtems/score/threadq.h>
#include <rtems/rtems/types.h>
#include <rtems/score/smp.h>
#include <osp_cpuopts.h>
#include <bsp/irq-generic.h>
#include <rtems/rtems/support.h>
#include <rtems/bspIo.h>
#include <inttypes.h>
#include <osp_status.h>
#include <rtems/counter.h>
#include <string.h>
#include <rtems/score/threadimpl.h>
#include <bspopts.h>
#ifdef CFG_ASAN_SHADOW_OFFSET
#include <asan.h>
#endif


#define CHECKE_DELAY_TIME (1000 * 1000 * 500)

#if defined(PS3OS_HBA_V200)
static volatile  uint64_t * dl_raddr  = (volatile uint64_t *)(0x1420000 + 0x20);
static volatile  uint64_t * dl_waddr  = (volatile uint64_t *)(0x1420000 + 0x28);
static volatile  uint32_t * dl_cnt    = (volatile uint32_t *)(0x1420000 + 0x34);

static volatile  uint64_t * bus_raddr = (volatile uint64_t *)(0x1420000 + 0x38);
static volatile  uint64_t * bus_waddr = (volatile uint64_t *)(0x1420000 + 0x40);
static volatile  uint32_t * bus_cnt   = (volatile uint32_t *)(0x1420000 + 0x4c);
#endif

extern volatile uint32_t    Clock_driver_ticks;
static void drawBannerHeader(unsigned int coreID , unsigned int currentCoreID)
{
    printk("==================================================================================================\n");

    printk("CPU(%u) print Snapshot for cpu (%u) \n",currentCoreID, coreID);

    printk("=============================================================================\n");
    printk("\n\n");
}

static void drawBannerTail()
{
    printk("=============================================================================\n");
    printk("\n\n");
}

static void drawAllTimeTickDetal()
{
    printk("CPU Tick info : \n");
    printk("  Clock_driver_ticks : %u\n",Clock_driver_ticks);

    uint32_t cpu_max;
    cpu_max = rtems_configuration_get_maximum_processors();

    for ( uint32_t cpu_index = 0 ; cpu_index < cpu_max ; ++cpu_index )
    {
        Per_CPU_Control          *cpu;
        cpu = _Per_CPU_Get_by_index( cpu_index );
        printk("  CORE %u : %llu(tiks)\n",cpu_index,cpu->Watchdog.ticks);
    }

    printk("\n\n");
}

static bool getAndDrawInInterruptContext(Per_CPU_Control *cpu)
{
    bool isrInProgress = (cpu->isr_nest_level != 0);
    printk("In Interrupt Context : %s\n", isrInProgress ? "true": "false");
    printk("\n\n");
    return isrInProgress;
}

static void drawCoreSchedDetail(Per_CPU_Control *cpu)
{

    printk("Cpu thread dispatch details :\n");
    printk("  Thread dispatch disable level : %u \n", cpu->thread_dispatch_disable_level);
    printk("  dispatch necessary : %s\n",cpu->dispatch_necessary ? "true":"false");

    printk("\n\n");
}


static void drawInterruptStats(unsigned int coreID ,Per_CPU_Control *cpu)
{
#ifdef RTEMS_PROFILING
    printk("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printk("ID         MTDDT           MIT           MID                  TDDC                 TTDDT                    IC                   TIT                   \n");
    printk("%02d"  "    0x%08" PRIx32 "    0x%08" PRIx32 "    0x%08" PRIx32 "    0x%016" PRIu64 "    0x%016" PRIu64 "    0x%016" PRIu64 "    0x%016" PRIu64 "     \n" ,
            coreID,
            cpu->Stats.max_thread_dispatch_disabled_time,
            cpu->Stats.max_interrupt_time,
            cpu->Stats.max_interrupt_delay,
            cpu->Stats.thread_dispatch_disabled_count,
            cpu->Stats.total_thread_dispatch_disabled_time,
            cpu->Stats.interrupt_count,
            cpu->Stats.total_interrupt_time
          );
    printk("\n\n");
#else
    (void)coreID;
    (void)cpu;
#endif
}


static void drawExecutingThread(Per_CPU_Control *cpu)
{
    char strName[5] = "";

    rtems_name_to_characters(
            cpu->executing->Object.name.name_u32,
            &strName[ 0 ],
            &strName[ 1 ],
            &strName[ 2 ],
            &strName[ 3 ]
            );

    printk("-----------------------------------------\n");
    printk("TID         NAME           STATE\n");
    printk("0x%08" PRIx32 "    %s      %u\n",
            cpu->executing->Object.id,
            strName,
            cpu->executing->current_state
            );

    printk("\n\n");
}

static bool queryTaskVisitor(Thread_Control *the_thread, void *arg)
{
    CoreSnapshotInfo_s * returnInfo = (CoreSnapshotInfo_s *)arg;

    if (the_thread->Object.id == returnInfo->queryThreadInfo.queryThreadID)
    {
        returnInfo->queryThreadInfo.switchInTime = the_thread->switch_in_time;
        returnInfo->queryThreadInfo.switchOutTime = the_thread->switch_out_time;
        return true;
    }

    return false;
}

static void getQueryThreadInfo(CoreSnapshotInfo_s * returnInfo)
{
    if (!returnInfo)
        return;
    _Thread_Iterate(queryTaskVisitor, (void*)returnInfo);
}


static void drawRawDeadLock()
{
#if defined(PS3OS_HBA_V200)

    printk("Raw Dead Lock Info : \n");
    printk("-----------------------------------------\n");

    uint64_t last_dl_raddr = *dl_raddr;
    uint64_t last_dl_waddr = *dl_waddr;
    uint32_t last_dl_cnt   = *dl_cnt;
    uint64_t last_be_raddr = *bus_raddr;
    uint64_t last_be_waddr = *bus_waddr;
    uint32_t last_be_cnt   = *bus_cnt;

    printk("DL : [%llu]-[%llu]-[%u]",last_dl_raddr , last_dl_waddr , last_dl_cnt);
    printk("BE : [%llu]-[%llu]-[%u]",last_be_raddr , last_be_waddr , last_be_cnt);

#endif


    printk("\n\n");
}


/**
 * @brief   打印 coreid 对应core 的调试信息
 *              中断信息
 *              cpu 超时信息
 *              是否有中断风暴
 *          返回业务最后执行任务的栈信息和任务切换时间信息
 * @param   coreID [in],  指定需要打印信息的coreid
 * @param   returnInfo [out], 指向获取任务切换时间和栈信息的存放地址
 *                            这里返回的是出先wdt时检测到coreid 指定的core上最后执行的任务的相关信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e  ospGetCoreSnapshotInfo(unsigned int coreID , CoreSnapshotInfo_s * returnInfo)
{
#if defined(PS3OS_SMP)
    if (coreID >= ospSchedulerGetProcessorMaximum())
    {
        return OSP_INVALID_ID;
    }
#else
    if (coreID != 0)
    {
        return OSP_INVALID_ID;
    }
#endif

    Per_CPU_Control          *cpu;
    uint32_t  currentCoreID = _SMP_Get_current_processor();
    cpu = _Per_CPU_Get_by_index( coreID );

    drawBannerHeader(coreID, currentCoreID);


    bool isrInProgress = getAndDrawInInterruptContext(cpu);

    getQueryThreadInfo(returnInfo);

    if (returnInfo && returnInfo->queryStackInfo.stack)
    {
        if (isrInProgress)
        {
            returnInfo->queryStackInfo.length = cpu->interrupt_stack_high - cpu->interrupt_stack_low;
#ifdef CFG_ASAN_SHADOW_OFFSET
            asan_memcpy_unchecked(returnInfo->queryStackInfo.stack, cpu->interrupt_stack_low,returnInfo->queryStackInfo.length);
#else
            memcpy(returnInfo->queryStackInfo.stack, cpu->interrupt_stack_low,returnInfo->queryStackInfo.length);
#endif
        }
        else
        {
            returnInfo->queryStackInfo.length = cpu->executing->Start.Initial_stack.size;
#ifdef CFG_ASAN_SHADOW_OFFSET
            asan_memcpy_unchecked(returnInfo->queryStackInfo.stack, cpu->executing->Start.Initial_stack.area, returnInfo->queryStackInfo.length);
#else
            memcpy(returnInfo->queryStackInfo.stack, cpu->executing->Start.Initial_stack.area, returnInfo->queryStackInfo.length);
#endif
        }
    }

    drawExecutingThread(cpu);

    drawCoreSchedDetail(cpu);

    drawRawDeadLock();


    drawInterruptStats(coreID , cpu);
    drawAllTimeTickDetal();

    if (coreID != currentCoreID && currentCoreID != 0)
    {
        printk("\n\n ##Delay 500ms show Interrupt again : \n\n");

        rtems_counter_delay_nanoseconds( CHECKE_DELAY_TIME );

        drawInterruptStats(coreID , cpu);
        drawAllTimeTickDetal();
    }

    drawBannerTail();

    return OSP_SUCCESSFUL;
}
