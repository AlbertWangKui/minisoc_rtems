/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_scheduler_smp.c
 * @author  lichenxiang
 * @date    2021.03.06
 * @brief   smp scheduler
 * @note    NA
 */
#include <osp_scheduler_smp.h>

#include <rtems/rtems/tasks.h>
#include <rtems/score/schedulerimpl.h>

#include "include/osp_inner_common.h"

/**
 * @brief   Get ID of a scheduler
 * @param   name ,                name The scheduler name.
 * @param   id,                   id The scheduler identifier associated with the name.
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSchedulerIdent(OspName name, OspID *id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_scheduler_ident(name, id);
    ospRet = ospConvertReturnValue(ret);

    return ospRet;
}

/**
 * @brief   Get ID of a scheduler by processor
 * @param   cpuIndex ,                cpuIndex The processor index.
 * @param   id,                        id The scheduler identifier associated with the processor index.
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSchedulerIdentByProcessor(uint32_t cpuIndex, OspID *id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_scheduler_ident_by_processor(cpuIndex, id);
    ospRet = ospConvertReturnValue(ret);

    return ospRet;
}

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
OspStatusCode_e ospSchedulerIdentByProcessorSet(size_t cpuSetSize, const cpu_set_t *cpuSet, OspID *id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_scheduler_ident_by_processor_set(cpuSetSize, cpuSet, id);
    ospRet = ospConvertReturnValue(ret);

    return ospRet;
}


/**
 * @brief   Get processor maximum
 * @return  The processor maximum supported by the system.
 * @warning NA
 * @note    NA
 */
uint32_t ospSchedulerGetProcessorMaximum(void)
{

    return rtems_scheduler_get_processor_maximum( );
}

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
OspStatusCode_e ospSchedulerGetProcessorSet(OspID schedulerId, size_t cpuSetSize, cpu_set_t *cpuSet)
{

    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_scheduler_get_processor_set(schedulerId, cpuSetSize, cpuSet);
    ospRet = ospConvertReturnValue(ret);

    return ospRet;
}

/**
 * @brief   Add processor to a scheduler
 * @param   schedulerId ,                Identifier of the scheduler instance.
 * @param   cpuIndex,                    Index of the processor to add.
 *
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospSchedulerAddProcessor(OspID schedulerId, uint32_t cpuIndex)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_scheduler_add_processor(schedulerId, cpuIndex);
    ospRet = ospConvertReturnValue(ret);

    return ospRet;
}

/**
 * @brief   Remove processor from a scheduler
 * @param   schedulerId ,                Identifier of the scheduler instance.
 * @param   cpuIndex,                    Index of the processor to add.
 *
 * @return  OspStatusCode_e           返回值
 * @warning NA
 * @note    NA
 */

OspStatusCode_e ospSchedulerRemoveProcessor(OspID schedulerId, uint32_t cpuIndex)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_scheduler_remove_processor(schedulerId, cpuIndex);
    ospRet = ospConvertReturnValue(ret);

    return ospRet;
}
