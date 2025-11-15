/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_system_error.c
 * @author  hulong
 * @date    2022.03.17
 * @brief   封装获取flash中的system error信息
 * @note    NA
 */

#include <string.h>
#include <stdio.h>
#include <bsp.h>
#include <osp_appinit.h>
#include <osp_system_error.h>

/**
 * @brief   Function config snapshot
 * @param   addr, size
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCrashSnapshotConfig(OSP_SNAPSHOT_RECORD_TYPE_E record_type, uint32_t addr, uint32_t size)
{
    uint32_t ret = OSP_NOT_IMPLEMENTED;
    if (record_type != OSP_SNAPSHOT_2_FLASH)
    {
        return ret;
    }
#if defined(PS3OS_HBA_V200)
    ret = bsp_crash_snapshot_area_config(addr, size);
    if ( ret == 0 )
    {
        return OSP_SUCCESSFUL;
    }

    return OSP_UNSATISFIED;
#else
    (void)addr;
    (void)size;
    return ret;
#endif
}

#if defined(PS3OS_HBA_V200)
#if defined(RTEMS_SMP)
#include <rtems/score/smpimpl.h>
#include <osp_scheduler_smp.h>

typedef void ( *SMP_Action_handler )( void *arg );
void _SMP_Broadcast_action(
  SMP_Action_handler  handler,
  void               *arg
);

static void ospSmpGetCpuL1CacheSingleEccCnt(void *arg)
{
    uint32_t *cntArray = (uint32_t *) arg;
    int cpuid;

    cpuid = rtems_scheduler_get_processor();
    cntArray[cpuid] = bsp_cpu_cache_get_single_ecc_error_cnt(OSP_SYSERR_CPU_CACHE_LEVEL_1);
}

static void ospSmpClearCpuL1CacheSingleEccCnt(void *arg)
{
    arg = arg;
    bsp_cpu_cache_clear_single_ecc_error_cnt(OSP_SYSERR_CPU_CACHE_LEVEL_1);
}

static uint32_t ospGetPerCpuL1CacheSingleEccErrorCnt(void)
{
    uint32_t cntArray[CPU_COUNT];
    uint32_t cntSum = 0;
    int cpuid;
    int i;

    memset(cntArray, 0, sizeof(cntArray));
    cpuid = rtems_scheduler_get_processor();

    cntArray[cpuid] = bsp_cpu_cache_get_single_ecc_error_cnt(OSP_SYSERR_CPU_CACHE_LEVEL_1);

    _SMP_Othercast_action(ospSmpGetCpuL1CacheSingleEccCnt, cntArray);
    for (i=0; i<CPU_COUNT; i++) {
        cntSum += cntArray[i];
    }
    return cntSum;
}

static void ospClearPerCpuL1CacheSingleEccErrorCnt(void)
{
   bsp_cpu_cache_clear_single_ecc_error_cnt(OSP_SYSERR_CPU_CACHE_LEVEL_1);
   _SMP_Othercast_action(ospSmpClearCpuL1CacheSingleEccCnt, NULL);
}
#else
static uint32_t ospGetPerCpuL1CacheSingleEccErrorCnt(void)
{
    return bsp_cpu_cache_get_single_ecc_error_cnt(OSP_SYSERR_CPU_CACHE_LEVEL_1);
}
static void ospClearPerCpuL1CacheSingleEccErrorCnt(void)
{
    return bsp_cpu_cache_clear_single_ecc_error_cnt(OSP_SYSERR_CPU_CACHE_LEVEL_1);
}
#endif
#endif

/**
 * @brief   Function get system error info recorded in flash
 * @param
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospGetSystemErrorInfo(OSPSystemErrorInfo *sys_err_info)
{
    int32_t ret = OSP_UNSATISFIED;

    sys_err_info = sys_err_info;
    return ret;

}

/**
 * @brief   Function get cpu level cache single ecc err count
 * @param   level[in]: level num of cache
 * @return  ecc err count
 * @warning 无
 * @note    无
 */
uint32_t ospGetCpuCacheSingleEccErrorCnt(int level)
{
    uint32_t cnt = 0;
#if defined(PS3OS_HBA_V200)
    switch (level) {
        case OSP_SYSERR_CPU_CACHE_LEVEL_1:
            cnt = ospGetPerCpuL1CacheSingleEccErrorCnt();
            break;

        case OSP_SYSERR_CPU_CACHE_LEVEL_2:
            cnt = bsp_cpu_cache_get_single_ecc_error_cnt(level);
            break;

        default:
            printf("ERROR: cpu cache get single ecc level(%d) err.\n", level);
            break;
    }
#else
    level = level;
#endif
    return cnt;
}

/**
 * @brief   Function clear cpu level cache single ecc err count
 * @param   level[in]: level num of cache
 * @return  void
 * @warning 无
 * @note    无
 */
void ospClearCpuCacheSingleEccErrorCnt(int level)
{
#if defined(PS3OS_HBA_V200)
    switch (level) {
        case OSP_SYSERR_CPU_CACHE_LEVEL_1:
            ospClearPerCpuL1CacheSingleEccErrorCnt();
            break;

        case OSP_SYSERR_CPU_CACHE_LEVEL_2:
            bsp_cpu_cache_clear_single_ecc_error_cnt(level);
            break;

        default:
            printf("ERROR: cpu cache clear single ecc level(%d) err.\n", level);
            break;
    }
#else
    level = level;
#endif
}
