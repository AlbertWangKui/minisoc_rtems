 /**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_system_error.h
 * @author  hulong
 * @date    2021.04.21
 * @brief   get system error recorded info
 * @note    NA
 */

#ifndef __OSP_SYSTEM_ERROR_H
#define __OSP_SYSTEM_ERROR_H

#include <inner/osp_inner_system_error.h>
#include <osp_status.h>
#include <osp_types.h>

//__BEGIN_DECLS
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t eccErrorStat;
    uint32_t eccErrorCnt;
    uint64_t eccErrorAddr;
} OSPSystemErrorInfo;

enum OSPSystemErrorType {
    OSP_SYSERR_TYPE_OCM = 0,
    OSP_SYSERR_TYPE_HAC0,
    OSP_SYSERR_TYPE_HAC1,
    OSP_SYSERR_TYPE_DDR0,
    OSP_SYSERR_TYPE_DDR1,
    OSP_SYSERR_TYPE_CACHEL1,
    OSP_SYSERR_TYPE_CACHEL2,
    OSP_SYSERR_TYPE_NR,
};

#define OSP_SYSERR_CPU_CACHE_LEVEL_1 OSP_INNER_SYSERR_CPU_CACHE_LEVEL_1
#define OSP_SYSERR_CPU_CACHE_LEVEL_2 OSP_INNER_SYSERR_CPU_CACHE_LEVEL_2

typedef enum OSP_SNAPSHOT_RECORD_TYPE {
    OSP_SNAPSHOT_2_FLASH,
} OSP_SNAPSHOT_RECORD_TYPE_E;

/**
 * @brief   Function config snapshot
 * @param   addr, size
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospCrashSnapshotConfig(OSP_SNAPSHOT_RECORD_TYPE_E record_type, uint32_t addr, uint32_t size);

/**
 * @brief   Function get system error info recorded in flash
 * @param
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospGetSystemErrorInfo(OSPSystemErrorInfo *sys_err_info);

/**
 * @brief   Function get cpu level cache single ecc err count
 * @param   level[in]: level num of cache
 * @return  ecc err count
 * @warning 无
 * @note    无
 */
uint32_t ospGetCpuCacheSingleEccErrorCnt(int level);

/**
 * @brief   Function clear cpu level cache single ecc err count
 * @param   level[in]: level num of cache
 * @return  void
 * @warning 无
 * @note    无
 */
void ospClearCpuCacheSingleEccErrorCnt(int level);

#ifdef __cplusplus
}
#endif
//__END_DECLS
#endif
