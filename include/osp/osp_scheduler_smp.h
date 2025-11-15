/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_scheduler_smp.h
 * @author  lichenxiang
 * @date    2021.03.06
 * @brief   smp scheduler
 * @note    NA
 */

#ifndef __OSP_SCHEDULER_SMP_H
#define __OSP_SCHEDULER_SMP_H

#include <osp_attr.h>
#include <osp_mode.h>
#include <osp_status.h>
#include <osp_types.h>
#include <osp_priority.h>
#include <sched.h>
#include <rtems/rtems/tasks.h>

#ifdef __cplusplus
extern "C" {
#endif


#define SCHEDULER_GLOBAL  ((uint32_t)('G') << 24 | (uint32_t)('L') << 16 | (uint32_t)('B') << 8 | (uint32_t)('L'))
#define SCHEDULER_SUBSET0 ((uint32_t)('S') << 24 | (uint32_t)('B') << 16 | (uint32_t)('T') << 8 | (uint32_t)('0'))
#define SCHEDULER_SUBSET1 ((uint32_t)('S') << 24 | (uint32_t)('B') << 16 | (uint32_t)('T') << 8 | (uint32_t)('1'))
#define SCHEDULER_SUBSET2 ((uint32_t)('S') << 24 | (uint32_t)('B') << 16 | (uint32_t)('T') << 8 | (uint32_t)('2'))

#define CPU_COUNT 4

/**
 * @brief   Get ID of a scheduler
 * @param   name ,                name The scheduler name.
 * @param   id,                   id The scheduler identifier associated with the name.
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSchedulerIdent(OspName name, OspID *id);

/**
 * @brief   Get ID of a scheduler by processor
 * @param   cpuIndex ,                cpuIndex The processor index.
 * @param   id,                        id The scheduler identifier associated with the processor index.
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSchedulerIdentByProcessor(uint32_t cpuIndex, OspID *id);

/**
 * @brief   Get ID of a scheduler by processor set
 * @param   cpuSetSize ,                cpuSetSize Size of the specified processor set buffer in bytes.  This value must
 * be positive.
 * @param   cpuSet,                     cpuSet The processor set to identify the scheduler.
 * @param   id,                         id The scheduler identifier associated with the processor set.
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSchedulerIdentByProcessorSet(size_t cpuSetSize, const cpu_set_t *cpuSet, OspID *id);

/**
 * @brief   Get current processor index
 * @return  The index of the current processor.
 * @warning NA
 * @note    NA
 */

static inline uint32_t ospSchedulerGetProcessor(void)
{
    return rtems_scheduler_get_processor( );
}

/**
 * @brief   Get processor maximum
 * @return  The processor maximum supported by the system.
 * @warning NA
 * @note    NA
 */
uint32_t ospSchedulerGetProcessorMaximum(void);

/**
 * @brief   Gets the set of processors owned by the specified scheduler instance.
 * @param   schedulerId ,                Identifier of the scheduler instance.
 * @param   cpuSetSize,                   Size of the specified processor set buffer in
 * @param   cpuSet,                       cpuSet The processor set owned by the scheduler.  A set bit in
 *                                        the processor set means that this processor is owned by the scheduler and a
 *                                        cleared bit means the opposite.
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSchedulerGetProcessorSet(OspID schedulerId, size_t cpuSetSize, cpu_set_t *cpuSet);

/**
 * @brief   Add processor to a scheduler
 * @param   schedulerId ,                Identifier of the scheduler instance.
 * @param   cpuIndex,                    Index of the processor to add.
 *
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSchedulerAddProcessor(OspID schedulerId, uint32_t cpuIndex);

/**
 * @brief   Remove processor from a scheduler
 * @param   schedulerId ,                Identifier of the scheduler instance.
 * @param   cpuIndex,                    Index of the processor to add.
 *
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */

OspStatusCode_e ospSchedulerRemoveProcessor(OspID schedulerId, uint32_t cpuIndex);

#ifdef __cplusplus
}
#endif

#endif
