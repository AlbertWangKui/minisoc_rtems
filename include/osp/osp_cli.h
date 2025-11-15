#ifndef __OSP_CLI_H__
#define __OSP_CLI_H__

#include <stdint.h>
#include <stdbool.h>
#include <osp_status.h>
#include <osp_types.h>
#include <osp_priority.h>
#include <osp_memory.h>
#include <osp_interrupt.h>
#include <osp_timer.h>
#include <osp_cpuopts.h>
#include <inner/osp_inner_cli.h>

#ifdef __cplusplus
extern "C" {
#endif

///< 任务信息
typedef struct OspTaskInfo {
    OspID taskID;                           ///< 任务ID
    OspName taskName;                       ///< 任务名称
    OspName schedulerName;                  ///< 任务所属调度器名称
    OspTaskPriority_e taskPriority;         ///< 任务优先级(如果不是通过osp接口创建的任务，优先级无效)
    OspStatesControl taskState;             ///< 任务当前状态，参考osp_task.h的OSP_STATES_xxx
    OspEventSet events;                     ///< 任务收到的事件
    uintptr_t stackLow;                     ///< 任务堆栈的低地址
    uintptr_t stackHigh;                    ///< 任务堆栈的高地址
    uintptr_t stackCurr;                    ///< 任务当前栈
    uint32_t maxStackUse;                   ///< 任务栈历史最大开销
    uint32_t cpuUseSec;                     ///< 任务执行时间统计(seconds)，ospCliResetCpuUsage重新开始统计
    uint32_t cpuUseNanosec;                 ///< 任务执行时间统计(nanoseconds)，ospCliResetCpuUsage重新开始统计
    uint32_t cpuUseIval;                    ///< 任务执行时间占比(整数部分)，ospCliResetCpuUsage重新开始统计
    uint32_t cpuUseFval;                    ///< 任务执行时间占比(小数部分,表示千分数，如5表示0.005)，ospCliResetCpuUsage重新开始统计
#ifdef PS3OS_SMP
    int64_t switchInTime;                   ///< 任务最近一次切入时间
    int64_t switchOutTime;                  ///< 任务最近一次切出时间
#endif
    uint64_t nvcsw;                         ///< 任务主动切换次数 , 有可能溢出，并且实际切换后才会计数
    uint64_t nivcsw;                        ///< 任务被动切换次数 , 有可能溢出，并且实际切换后才会计数
} OspTaskInfo_s;

///< cpu core信息
typedef struct OspCpuCoreInfo {
    OspID cpuCoreIndex;                     ///< cpu core索引
    bool cpuCoreOnline;                     ///< cpu core是否在线
    OspName schedulerName;                  ///< cpu core所属调度器名称
    uintptr_t stackLow;                     ///< 中断堆栈的低地址(每个cpu core一个中断栈)
    uintptr_t stackHigh;                    ///< 中断堆栈的高地址
    uintptr_t stackCurr;                    ///< 中断当前栈
} OspCpuCoreInfo_s;

///< cpu运行信息
typedef struct OspCpuUsage
{
    int32_t cpuIndex;       ///< cpu index
    int64_t totalTime;      ///< cpu运行总时长
    int64_t idleTime;       ///< 单个cpu上idle 任务运行时长
}OspCpuUsage_s;

///< 内存信息
typedef struct OspMemoryInfo {
    bool isUnified;                         ///< os资源管理对象内存和通用内存是否统一管理
    OspHeapInformationBlock_s heapInfo;     ///< 内存信息
} OspMemoryInfo_s;

///< 系统已经注册的中断信息
typedef struct OspRegInterInfo {
   OspVector vector;                        ///< 中断号
   OspOption options;                       ///< 中断属性(共享/非共享/替换)
   OspInterruptHandler handler;             ///< 中断处理程序地址
   void *handlerArg;                        ///< 中断处理程序参数地址
} OspRegInterInfo_s;

///< 系统中正在使用的信号量信息
typedef struct OspSemaphoreInfo {
    OspID id;                               ///< 信号量ID
    OspName name;                           ///< 信号量名称
    OspAttribute attribute;                 ///< 信号量属性
    uint32_t maxCount;                      ///< 信号量最大计数
    uint32_t curCount;                      ///< 信号量当前基础
    OspID holderId;                         ///< 信号量拥有者(二值信号量才有意义)
} OspSemaphoreInfo_s;

///< 系统中正在使用的消息队列信息
typedef struct OspMessageQueueInfo {
    OspID id;                               ///< 消息队列ID
    OspName name;                           ///< 消息队列名称
    OspAttribute attribute;                 ///< 消息队列属性
    uint32_t numberOfPendingMessages;       ///< 消息队列待处理消息数
    uint32_t maximumPendingMessages;        ///< 消息队列最大消息请求数
    size_t maximumMessageSize;              ///< 消息队列最大消息长度
} OspMessageQueueInfo_s;

///< 系统中正在使用的timer信息
typedef struct OspTimerInfo {
    OspID id;                               ///< 定时器ID
    OspName name;                           ///< 定时器名称
    ospTimerServiceRoutineEntry routine;    ///< 定时器回调函数
    uint32_t initTime;                      ///< 定时器初始化时间
    uint32_t startTime;                     ///< 定时器开始时间
    uint32_t stopTime;                      ///< 定时器取消或者结束时间
} OspTimerInfo_s;

///< 系统中正在使用的region信息
typedef struct OspRegionInfo {
    OspID id;                               ///< region ID
    OspName name;                           ///< region名称
    OspAttribute attribute;                 ///< region属性
    void *startAddr;                        ///< region起始地址
    uint32_t length;                        ///< region长度
    uint32_t pageSize;                      ///< region page长度
    uint32_t usedBlocks;                    ///< region使用的block数
} OspRegionInfo_s;

///< 如果系统资源规格的bit31被置1，表示该资源规格是无限的，否则就是有限的
#define OSP_UNLIMITED_OBJECTS 0x80000000U
///< 系统资源规格
typedef struct OspSystemResourceSpec {
    uint32_t    maxTasks;                   ///< 系统当前任务规格
    uint32_t    maxTimers;                  ///< 系统当前定时器规格
    uint32_t    maxSemaphores;              ///< 系统当前信号量规格
    uint32_t    maxMessageQueues;           ///< 系统当前消息队列规格
    uint32_t    maxRegions;                 ///< 系统当前region规格
    uint32_t    microsecondsPerTick;        ///< 系统时钟滴答精度(单位us)
} OspSystemResourceSpec_s;

///< 系统性能统计信息
typedef struct OspProfInfo {
    uint32_t cpuCoreIndex;                  ///< cpu core索引
    uint32_t maxThreadDispatchDisabledTime; ///< 任务调度关闭的最大时间，单位纳秒
    uint32_t maxInterruptTime;              ///< 中断处理程序执行的最大时间，单位纳秒
} OspProfInfo_s;

typedef enum OSP_SECTION_TYPE {
    OSP_TEXT_SECTION = 0,
    OSP_START_SECTION ,
    OSP_RODATA_SECTION ,
    OSP_FAST_TEXT_SECTION,
    OSP_FAST_DATA_SECTION,
    OSP_COREDUMP_SECTION,
    OSP_BSS_SECTION,
    OSP_DATA_SECTION,
    OSP_TRANSLATION_TABLE_SECTION,
    OSP_MAX_SECTION
}OSP_SECTION_TYPE_E;

typedef struct OspSectionInfo {
    uintptr_t begin;
    uintptr_t end;
    uintptr_t size;
}OspSectionInfo_s;

typedef enum OspDumpStackType {
    OSP_DUMP_STACK_COM,
    OSP_DUMP_STACK_MEM,
    OSP_DUMP_STACK_COM_MEM,
    OSP_DUMP_STACK_LAST
} OspDumpStackType_E;

#define OSP_DUMP_STACK_MEM_PER_MINISIZE (1024 * 4)



/**
 * @brief   获取系统当前任务数量
 * @param   taskCnt [out],  系统当前任务数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTaskCnt(uint32_t *taskCnt);

/**
 * @brief   重置任务cpu利用率统计
 * @param   void
 * @return  执行结果
 * @warning 无
 * @note    任务的cpu利用率信息由接口ospCliGetTaskInfo给出，该接口只是重新开始统计cpu利用率
 */
OspStatusCode_e ospCliResetCpuUsage(void);

/**
 * @brief   获取任务最大栈开销
 * @param   stack [in],  任务栈
 * @return  最大栈开销
 * @warning 无
 * @note    无
 */
uint32_t ospCliGetMaxStackUse(const Thread_Control *thread);

/**
 * @brief   获取系统任务信息
 * @param   taskCnt 任务数量
 * @param   taskInfo [out], 任务信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTaskInfo(uint32_t taskCnt, OspTaskInfo_s *taskInfo);

/**
 * @brief   通过任务id获取系统任务信息
 * @param   id 任务id
 * @param   taskInfo [out], 任务信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTaskInfoByID(OspID id, OspTaskInfo_s *taskInfo);

/**
 * @brief   获取系统当前cpu core数量
 * @param   cpuCoreCnt [out], 系统当前cpu core数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetCpuCoreCnt(uint32_t *cpuCoreCnt);

/**
 * @brief   获取系统cpu core信息
 * @param   cpuCoreCnt cpu core数量
 * @param   cpuCoreInfo [out], cpu core信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetCpuCoreInfo(uint32_t cpuCoreCnt, OspCpuCoreInfo_s *cpuCoreInfo);

/**
 * @brief   获取系统cpu uptime信息和cpu上idle任务运行总时长
 * @param   cnt cpu core数量
 * @param   cpuUsage [out], cpu uptime和idle任务时长信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetCpuRunningTime(uint32_t cnt, OspCpuUsage_s *cpuUsage);

/**
 * @brief   获取系统内存统计
 * @param   memoryInfo 内存统计信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetMemoryInfo(OspMemoryInfo_s *memoryInfo);

/**
 * @brief   获取系统workspace统计
 * @param   memoryInfo 内存统计信息
 * @return  执行结果
 * @warning 无
 * @note    如果workspace和malloc内存是一个内存堆，这该接口和ospCliGetMemoryInfo接口一致
 */
OspStatusCode_e ospCliGetWorkspaceInfo(OspMemoryInfo_s *memoryInfo);

/**
 * @brief   获取注册到系统中的中断数量
 * @param   regInterCnt [out], 系统当前注册的中断数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetRegisteredInterCnt(uint32_t *regInterCnt);

/**
 * @brief   获取系统已注册的中断信息
 * @param   regInterCnt 已经注册的数量
 * @param   regInterInfo [out], 已经注册的中断信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetRegInterInfo(uint32_t regInterCnt, OspRegInterInfo_s *regInterInfo);

/**
 * @brief   获取系统中的信号量数量
 * @param   semaphoreCnt [out], 系统当前的信号量数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetSemaphoreCnt(uint32_t *semaphoreCnt);

/**
 * @brief   获取系统中的信号量信息
 * @param   semaphoreCnt 系统中的信号量数量
 * @param   semaphoreInfo [out], 系统中的信息量信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetSemaphoreInfo(uint32_t semaphoreCnt, OspSemaphoreInfo_s *semaphoreInfo);

/**
 * @brief   获取系统中的消息队列数量
 * @param   messageQueueCnt [out], 系统当前的消息队列数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetMessageQueueCnt(uint32_t *messageQueueCnt);

/**
 * @brief   获取系统中的消息队列信息
 * @param   messageQueueCnt 系统中的消息队列数量
 * @param   messageQueueInfo [out], 系统中的消息队列信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetMessageQueueInfo(uint32_t messageQueueCnt, OspMessageQueueInfo_s *messageQueueInfo);

/**
 * @brief   获取系统中的timer数量
 * @param   timerCnt [out], 系统当前的timer数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTimerCnt(uint32_t *timerCnt);

/**
 * @brief   获取系统中的timer信息
 * @param   timerCnt 系统中的timer数量
 * @param   timerInfo [out], 系统中的timer信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetTimerInfo(uint32_t timerCnt, OspTimerInfo_s *timerInfo);

/**
 * @brief   获取系统中的region数量
 * @param   regionCnt [out], 系统当前的region数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetRegionCnt(uint32_t *regionCnt);

/**
 * @brief   获取系统中的region信息
 * @param   regionCnt 系统中的region数量
 * @param   regionInfo [out], 系统中的region信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetRegionInfo(uint32_t regionCnt, OspRegionInfo_s *regionInfo);

/**
 * @brief   获取系统中的资源规格
 * @param   resourceSpec [out], 系统中的资源规格
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetSystemResourceSpec(OspSystemResourceSpec_s *resourceSpec);

/**
 * @brief   获取系统性能统计信息
 * @param   cpuCoreCnt cpu core数量
 * @param   profInfo [out], 每个cpu core的prof信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCliGetProfInfo(uint32_t cpuCnt, OspProfInfo_s *profInfo);

/**
 * @brief   关闭coredump功能
 * @param   true/false
 * @return  执行结果
 * @warning 无
 * @note    1 默认打开；
 *          2 OSP_NOT_IMPLEMENTED表示不支持
 */
OspStatusCode_e ospCliCloseCoredump(bool isCloseCoredump);

/**
 * @brief   查询coredump功能是否关闭
 * @param   *isCloseCoredump [out], true/false
 * @return  执行结果
 * @warning 无
 * @note    1 默认打开；
 *          2 OSP_NOT_IMPLEMENTED表示不支持
 *          3 OSP_INVALID_ADDRESS表示入参为空
 */
OspStatusCode_e ospCliGetCoredumpIsClose(bool *isCloseCoredump);

#define OSP_COREDUMP_FUN_DMA        OSP_INNER_COREDUMP_FUN_DMA
#define OSP_COREDUMP_FUN_MEM        OSP_INNER_COREDUMP_FUN_MEM
#define OSP_COREDUMP_FUN_CON        OSP_INNER_COREDUMP_FUN_CON
#define OSP_COREDUMP_FUN_GDMA       OSP_INNER_COREDUMP_FUN_GDMA
#define OSP_COREDUMP_FUN_FLASH       OSP_INNER_COREDUMP_FUN_FLASH
#define OSP_COREDUMP_FUN_FLASH_V2       OSP_INNER_COREDUMP_FUN_FLASH_V2

/**
 * @brief   设置coredump导出方法
 * @param   OSP_COREDUMP_FUN_XXX
 * @return  执行结果
 * @warning 无
 * @note    1 OSP_INVALID_NUMBER表示入参错误
 *          2 OSP_NOT_IMPLEMENTED表示不支持
 */
OspStatusCode_e ospCliSetFunType(int coredumpFunType);

/**
 * @brief   查询coredump导出方法
 * @param   void
 * @return  OSP_COREDUMP_FUN_XXX:导出方法
 * @warning 无
 * @note    1 OSP_NOT_IMPLEMENTE表示不支持
 */
int ospCliGetFunType(void);

/**
 * @brief   配置coredump onf分区信息
 * @param   is_minicore
 *              true : minicore
 *              false : not minicore
 * @return
 * @warning 无
 */
void ospCliSetCoredumpMinicore(bool is_minicore);

/**
 * @brief   判断是否配置minicore
 * @param
 * @return  true : minicore
 *          false : not minicore
 * @warning 无
 */
bool ospCliCoredumpIsMinicore(void);

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
OspStatusCode_e ospCliSetMemCoredumpArea(uint32_t memStart, uint32_t size);

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
OspStatusCode_e ospCliSetFlashCoredumpRegion(uint32_t regionStart, uint32_t size);

/**
 * @brief   打印当前调用栈
 * @param   void
 * @return  void
 * @warning 无
 * @note
 */
void ospCliDumpStack(void);

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
OspStatusCode_e ospCliDumpStackExtension(OspDumpStackType_E dumpType, char *memInfo,uint32_t size);

/**
 * @brief   设置backtrace输出类型
 * @param   dumpType
 *              frame输出类型
 * @param   memInfo
 *              当输出到mem时需要指定输出内存地址
 * @param   perSize
 *              每个记录fatal信息的buf大小 , 至少大于 OSP_DUMP_STACK_MEM_PER_MINISIZE
 * @param   numberMemInfo
 *             memInfo 中包含多少个 记录fatal信息的buf memInfo(total) = perSize * numberMemInfo
 *             numberMemInfo 至少大于 等于当前核数
 * @param  number of DumpStackMemInfo_s
 * @return  success
 *              OSP_SUCCESSFUL
 * @warning 无
 * @note
 */
OspStatusCode_e ospCliDumpStackExtensionSetType(OspDumpStackType_E dumpType, char *memInfo, uint32_t perSize, uint32_t memInfoSize, uint32_t numberMemInfo);



/**
 * @brief  获取 section 信息
 * @param  secton 内容
 * @return  section info
 * @warning 无
 * @note
 */
OspSectionInfo_s const *ospCliGetSectionInfo(OSP_SECTION_TYPE_E sectionType);

#ifdef __cplusplus
}
#endif

#endif
