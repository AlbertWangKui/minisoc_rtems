/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_memory.h
 * @author  tianye@tianye.com
 * @date    2020.08.04
 * @brief   porting rtems memory
 * @note    NA
 */

#ifndef __OSP_MEMORY_H__
#define __OSP_MEMORY_H__

#include <osp_attr.h>
#include <osp_options.h>
#include <osp_status.h>
#include <osp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void OspTask;

/**
 * @brief Information about blocks.
 */
typedef struct OspHeapInformation {
    /**
     * @brief Number of blocks of this type.
     */
    uintptr_t number;

    /**
     * @brief Largest block of this type.
     */
    uintptr_t largest;

    /**
     * @brief Total size of the blocks of this type.
     */
    uintptr_t total;
} OspHeapInformation_s;

typedef struct OspHeapStatistics {
    /**
     * @brief Lifetime number of bytes allocated from this heap.
     *
     * This value is an integral multiple of the page size.
     */
    uint64_t lifetimeAllocated;

    /**
     * @brief Lifetime number of bytes freed to this heap.
     *
     * This value is an integral multiple of the page size.
     */
    uint64_t lifetimeFreed;

    /**
     * @brief Size of the allocatable area in bytes.
     *
     * This value is an integral multiple of the page size.
     */
    uintptr_t size;

    /**
     * @brief Current free size in bytes.
     *
     * This value is an integral multiple of the page size.
     */
    uintptr_t freeSize;

    /**
     * @brief Minimum free size ever in bytes.
     *
     * This value is an integral multiple of the page size.
     */
    uintptr_t minFreeSize;

    /**
     * @brief Current number of free blocks.
     */
    uint32_t freeBlocks;

    /**
     * @brief Maximum number of free blocks ever.
     */
    uint32_t maxFreeBlocks;

    /**
     * @brief Current number of used blocks.
     */
    uint32_t usedBlocks;

    /**
     * @brief Maximum number of blocks searched ever.
     */
    uint32_t maxSearch;

    /**
     * @brief Total number of searches.
     */
    uint32_t searches;

    /**
     * @brief Total number of successful allocations.
     */
    uint32_t allocs;

    /**
     * @brief Total number of failed allocations.
     */
    uint32_t failedAllocs;

    /**
     * @brief Total number of successful frees.
     */
    uint32_t frees;

    /**
     * @brief Total number of successful resizes.
     */
    uint32_t resizes;
} OspHeapStatistics_s;

/**
 * @brief Information block returned by _Heap_Get_information().
 */
typedef struct OspHeapInformationBlock {
    OspHeapInformation_s free;
    OspHeapInformation_s used;
    OspHeapStatistics_s stats;
} OspHeapInformationBlock_s;

/**
 * 内存区域类型枚举
 */
typedef enum OspMemType {
    OSP_MEM_INVALID = 0,
    OSP_MEM_SRAM_OS_USED = 1,                 ///< OS已经使用的SRAM
    OSP_MEM_SRAM_USER_CACHEABLE = 2,          ///< 用户可用cacheable SRAM区域
    OSP_MEM_SRAM_USER_NON_CACHEABLE = 3,      ///< 用户可用non cacheable SRAM区域
    OSP_MEM_SRAM_USER_SBR_NON_CACHEABLE = 4,  ///< 用户可用sbr non cacheable SRAM区域
    OSP_MEM_SRAM_BOOTLOADER_NON_CACHEABLE = 5,///< bootloader使用non cacheable SRAM区域
    OSP_MEM_BUTT = 255,
} OspMemType_e;

/**
 * 内存区域描述结构体
 */
typedef struct OspMemAreaInfo {
    OspMemType_e memType;               ///< 内存区域类型
    size_t startAddr;                   ///< 起始地址(按4KB对齐)
    size_t len;                         ///< 长度(B)
} OspMemAreaInfo_s;

/**
 * @brief   创建partition
 * @param   name ,                 partition名称
 * @param   startingAddress [in],  partition起始地址
 * @param   length ,               partition长度
 * @param   bufferSize ,           partition的buffer的长度
 * @param   attributeSet ,         partition的属性，如全局/局部等
 * @param   id [out],              输出partition的id
 * @return  OspStatusCode_e       返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospPartitionCreate(OspName name, void *startingAddress, uintptr_t length, size_t bufferSize,
                                   OspAttribute attributeSet, OspID *id);

/**
 * @brief   获取指定name的partition的id
 * @param   name ,           partition名称
 * @param   node ,           指定搜索的节点，如OBJECTS_SEARCH_ALL_NODES等
 * @param   id [out],        partition名称对应的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    1、partition的名称可以不唯一，这种情况下返回的是第一个找到的对象
 */
OspStatusCode_e ospPartitionIdent(OspName name, uint32_t node, OspID *id);

/**
 * @brief   获取指定id的partition
 * @param   id [in],         待删除partition的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospPartitionDelete(OspID id);

/**
 * @brief   从指定分区中获取buffer
 * @param   id [in],         指定的partition的id
 * @param   buffer [out],    获取的buffer
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospPartitionGetBuffer(OspID id, void **buffer);

/**
 * @brief   将buffer返回到指定partition
 * @param   id [in],         指定的partition的id
 * @param   buffer [in],     返回的buffer
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospPartitionReturnBuffer(OspID id, void *buffer);

/**
 * @brief   创建region
 * @param   name ,                 region名称
 * @param   startingAddress [in],  region起始地址
 * @param   length ,               region长度
 * @param   pageSize ,             页大小
 * @param   attributeSet ,         region的属性，如全局/局部等
 * @param   id [out],              输出region的id
 * @return  OspStatusCode_e       返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionCreate(OspName name, void *startingAddress, uintptr_t length, uintptr_t pageSize,
                                OspAttribute attributeSet, OspID *id);

/**
 * @brief   获取指定name的region的id
 * @param   name ,           region名称
 * @param   id [out],        region名称对应的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    1、region的名称可以不唯一，这种情况下返回的是第一个找到的对象
 */
OspStatusCode_e ospRegionIdent(OspName name, OspID *id);

/**
 * @brief   删除指定id的region
 * @param   id [in],         待删除region的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionDelete(OspID id);

/**
 * @brief   扩展指定region
 * @param   id [in],                待扩展region的id
 * @param   startingAddress [in],   待扩展物理地址
 * @param   length ,                待扩展物理地址长度
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionExtend(OspID id, void *startingAddress, uintptr_t length);

/**
 * @brief   从指定region获取segment
 * @param   id ,                指定region的id
 * @param   size ,              segment大小
 * @param   optionSet ,         option set，如WAIT、NO_WAIT等
 * @param   timeout ,           超时时间，如果配置的是NO_TIMEOUT，则会永远等待
 * @param   segment [out],      segment地址
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionGetSegment(OspID id, uintptr_t size, OspOption optionSet, OspInterval timeout, void **segment);

/**
 * @brief   将segment返回到指定region
 * @param   id ,               指定region的id
 * @param   segment [in]       segment地址
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionReturnSegment(OspID id, void *segment);

/**
 * @brief   获取指定region的指定segment的大小
 * @param   id ,               指定region的id
 * @param   segment [in],      指定segment地址
 * @param   size [out],        大小
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionGetSegmentSize(OspID id, void *segment, uintptr_t *size);

/**
 * @brief   修改segment大小
 * @param   id ,               指定region的id
 * @param   segment [in],      指定segment地址
 * @param   newSize ,          待修改大小
 * @param   oldSize [out],     修改前大小
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionResizeSegment(OspID id, void *segment, uintptr_t newSize, uintptr_t *oldSize);

/**
 * @brief   获取region基本信息
 * @param   id ,               指定region的id
 * @param   theInfo [out],     region信息
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionGetInformation(OspID id, OspHeapInformationBlock_s *theInfo);

/**
 * @brief   获取region中的free信息
 * @param   id ,               指定region的id
 * @param   theInfo [out],     region的free信息
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionGetFreeInformation(OspID id, OspHeapInformationBlock_s *theInfo);

/**
 * @brief   获取指定类型内存区域的数量
 * @param   memType ,           类型
 * @param   count [out],        数量
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospGetMemAreaCount(OspMemType_e memType, size_t *count);

/**
 * @brief   获取指定类型内存区域信息
 * @param   memType ,           类型
 * @param   *memInfoBuf [out],  用户提供buf,存放内存区域信息
 * @param   bufLen,             用户提供buf长度
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospGetMemAreaInfo(OspMemType_e memType, OspMemAreaInfo_s *memInfoBuf, size_t bufLen);

#ifdef __cplusplus
}
#endif

#endif
