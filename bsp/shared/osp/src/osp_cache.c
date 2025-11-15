/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_cache.c
 * @author  pengqianheng
 * @date    2020.12.22
 * @brief   封装   cache 操作的对外接口
 * @note    目前针对的是 Cortex-R5 的处理器
 */

#include <stddef.h>

#include <rtems.h>
#include <libcpu/arm-cp15.h>

#include <osp_errno.h>
#include <rtems/rtems/cache.h>
#include <bspopts.h>
#include <osp_cache.h>


/**
 * @brief   刷新本地cache
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospCacheFlushEntireData( void )
{
    rtems_cache_flush_entire_data();
}

void ospCacheCleanAndInvalidateEntireData( void )
{
    rtems_cache_clean_and_invalidate_entire_data();
}

#if defined(PS3OS_HBA_V200) || defined(PS3OS_HBA_V200_DEBUG)
#define OSP_CACHE_LINE_SIZE 64
#endif


/**
 * @brief   对指定的内存区域执行 clean & invalidate cache 的操作
 * @param   addr [in], 内存地址
 * @param   bytes, 数据长度( 单位: 字节 )
 * @return  无
 * @warning NA
 * @note    NA
 */
#if defined(PS3OS_HBA_V200) || defined(PS3OS_HBA_V200_DEBUG)
void PS3OS_OSP_PERF_SECTION ospCacheCleanAndInvalidateDcacheArea(void *addr, size_t bytes)
{
    if ( bytes != 0 ) {
        uint32_t       adx       = (uint32_t) addr
            & ~(OSP_CACHE_LINE_SIZE - 1);
        const uint32_t ADDR_LAST =
            (uint32_t)( (size_t) addr + bytes - 1 );

        for (; adx <= ADDR_LAST; adx += OSP_CACHE_LINE_SIZE) {
            /* Store and invalidate the Data cache line */
            arm_cp15_data_cache_clean_and_invalidate_line( (void*)adx );
        }
        /* Wait for L1 store to complete */
        _ARM_Data_synchronization_barrier();
    }
}
#else
void ospCacheCleanAndInvalidateDcacheArea(void *addr, size_t bytes)
{
    rtems_cache_flush_multiple_data_lines(addr, bytes);
}
#endif

/**
 * @brief   对指定的内存区域执行 clean cache 的操作
 * @param   addr [in], 内存地址
 * @param   bytes, 数据长度( 单位: 字节 )
 * @return  无
 * @warning NA
 * @note    NA
 */
void PS3OS_OSP_PERF_SECTION ospCacheCleanDcacheArea(void *addr, size_t bytes)
{
    const void *finalAddress;

    if(bytes == 0) ///< Do nothing if number of bytes to flush is zero
        goto exit;

#if defined(PS3OS_HBA_V200) || defined(PS3OS_HBA_V200_DEBUG)
    size_t cacheLineSize = OSP_CACHE_LINE_SIZE;
#else
    size_t cacheLineSize = rtems_cache_get_maximal_line_size();
#endif
    finalAddress = (void *)((size_t)addr + bytes - 1);
    addr         = (void *)((size_t)addr & ~(cacheLineSize - 1));
    while(addr <= finalAddress) {
        arm_cp15_data_cache_clean_line(addr);
        addr = (void *)((size_t)addr + cacheLineSize);

    }
    ///< Wait for L1 clean to complete
    _ARM_Data_synchronization_barrier( );

exit:
    return;
}

/**
 * @brief   对指定的内存区域执行 invalidate cache 的操作
 * @param   addr [in], 内存地址
 * @param   bytes, 数据长度( 单位: 字节 )
 * @return  无
 * @warning NA
 * @note    NA
 */
#if defined(PS3OS_HBA_V200) || defined(PS3OS_HBA_V200_DEBUG)
void PS3OS_OSP_PERF_SECTION ospCacheInvalidateDcacheArea(void *addr, size_t bytes)
{
    if ( bytes != 0 ) {
        uint32_t       adx = (uint32_t) addr
            & ~(OSP_CACHE_LINE_SIZE - 1);
        const uint32_t end =
            (uint32_t)( (size_t)addr + bytes -1);


        /* Back starting address up to start of a line and invalidate until end */
        for (;
                adx <= end;
                adx += OSP_CACHE_LINE_SIZE) {
            /* Invalidate the Instruction cache line */
            arm_cp15_data_cache_invalidate_line( (void*)adx );
        }
        /* Wait for L1 invalidate to complete */
        _ARM_Data_synchronization_barrier();
    }
}
#else
void ospCacheInvalidateDcacheArea(void *addr, size_t bytes)
{
    rtems_cache_invalidate_multiple_data_lines(addr, bytes);
}
#endif

/**
 * @brief   分配一块与 cache line 的 size 对齐的内存
 * @param   bytes, 数据长度( 单位: 字节 )
 * @return  1. 若成功，返回内存地址
            2. 若失败，返回 NULL
 * @warning NA
 * @note    use free() to release the memory allocated by this function.
 */
void *ospCacheAlignedMalloc(size_t bytes)
{
    void *ptr;

    if(bytes == 0) {
        ptr = NULL;
        goto exit;
    }

    ptr = rtems_cache_aligned_malloc(bytes);

exit:
    return ptr;
}
