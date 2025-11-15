/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_cache.h
 * @author  pengqianheng
 * @date    2020.12.22
 * @brief   封装   cache 操作的对外接口
 * @note    目前针对的是 Cortex-R5 的处理器
 */

#ifndef __OSP_CACHE_H
#define __OSP_CACHE_H

#include <stddef.h>
#include <osp_common.h>

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief   刷新本地cache
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospCacheFlushEntireData( void );

/**
 * @brief   clean and invalidate 本地cache
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospCacheCleanAndInvalidateEntireData( void );

/**
 * @brief   对指定的内存区域执行 clean & invalidate cache 的操作
 * @param   addr [in], 内存地址
 * @param   bytes, 数据长度( 单位: 字节 )
 * @return  无
 * @warning NA
 * @note    NA
 */
void PS3OS_OSP_PERF_SECTION ospCacheCleanAndInvalidateDcacheArea(void *addr, size_t bytes);

/**
 * @brief   对指定的内存区域执行 clean cache 的操作
 * @param   addr [in], 内存地址
 * @param   bytes, 数据长度( 单位: 字节 )
 * @return  无
 * @warning NA
 * @note    NA
 */
void PS3OS_OSP_PERF_SECTION ospCacheCleanDcacheArea(void *addr, size_t bytes);

/**
 * @brief   对指定的内存区域执行 invalidate cache 的操作
 * @param   addr [in], 内存地址
 * @param   bytes, 数据长度( 单位: 字节 )
 * @return  无
 * @warning NA
 * @note    NA
 */
void PS3OS_OSP_PERF_SECTION ospCacheInvalidateDcacheArea(void *addr, size_t bytes);

/**
 * @brief   分配一块与 cache line 的 size 对齐的内存
 * @param   bytes, 数据长度( 单位: 字节 )
 * @return  1. 若成功，返回内存地址
            2. 若失败，返回 NULL
 * @warning NA
 * @note    use free() to release the memory allocated by this function.
 */
void *ospCacheAlignedMalloc(size_t bytes);

#ifdef __cplusplus
}
#endif

#endif
