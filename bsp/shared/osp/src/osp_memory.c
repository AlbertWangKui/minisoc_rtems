/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_memory.c
 * @author  tianye
 * @date    2020.08.29
 * @brief   封装rtems操作系统内存管理对外接口
 * @note    NA
 */

#include <osp_attr.h>
#include <osp_options.h>
#include <osp_memory.h>
#include <rtems/rtems/status.h>
#include <rtems/rtems/part.h>
#include <rtems/rtems/region.h>
#include <rtems/score/heapinfo.h>
#include <bsp/linker-symbols.h>
#include <bspopts.h>
#include "include/osp_inner_common.h"
#include "common_defines.h"

extern OspMemAreaInfo_s gMemAreaInfo[];
extern U32 OspMemAreaInfoCntGet(void);

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
                                   OspAttribute attributeSet, OspID *id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_partition_create(name, startingAddress, length, bufferSize, attributeSet, id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   获取指定name的partition的id
 * @param   name ,           partition名称
 * @param   node ,           指定搜索的节点，如OBJECTS_SEARCH_ALL_NODES等
 * @param   id [out],        partition名称对应的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    1、partition的名称可以不唯一，这种情况下返回的是第一个找到的对象
 */
OspStatusCode_e ospPartitionIdent(OspName name, uint32_t node, OspID *id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_partition_ident(name, node, id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   获取指定id的partition
 * @param   id [in],         待删除partition的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospPartitionDelete(OspID id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_partition_delete(id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   从指定分区中获取buffer
 * @param   id [in],         指定的partition的id
 * @param   buffer [out],    获取的buffer
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospPartitionGetBuffer(OspID id, void **buffer)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_partition_get_buffer(id, buffer);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   将buffer返回到指定partition
 * @param   id [in],         指定的partition的id
 * @param   buffer [in],     返回的buffer
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospPartitionReturnBuffer(OspID id, void *buffer)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_partition_return_buffer(id, buffer);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

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
                                OspAttribute attributeSet, OspID *id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_create(name, startingAddress, length, pageSize, attributeSet, id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   获取指定name的region的id
 * @param   name ,           region名称
 * @param   id [out],        region名称对应的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    1、region的名称可以不唯一，这种情况下返回的是第一个找到的对象
 */
OspStatusCode_e ospRegionIdent(OspName name, OspID *id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_ident(name, id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   获取指定id的region
 * @param   id [in],         待删除region的id
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionDelete(OspID id)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_delete(id);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   扩展指定region
 * @param   id [in],                待扩展region的id
 * @param   startingAddress [in],   待扩展物理地址
 * @param   length ,                待扩展物理地址长度
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionExtend(OspID id, void *startingAddress, uintptr_t length)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_extend(id, startingAddress, length);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

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
OspStatusCode_e ospRegionGetSegment(OspID id, uintptr_t size, OspOption optionSet, OspInterval timeout, void **segment)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_get_segment(id, size, optionSet, timeout, segment);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   将segment返回到指定region
 * @param   id ,               指定region的id
 * @param   segment [in]       segment地址
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionReturnSegment(OspID id, void *segment)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_return_segment(id, segment);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   获取指定region的指定segment的大小
 * @param   id ,               指定region的id
 * @param   segment [in],      指定segment地址
 * @param   size [out],        大小
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionGetSegmentSize(OspID id, void *segment, uintptr_t *size)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_get_segment_size(id, segment, size);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

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
OspStatusCode_e ospRegionResizeSegment(OspID id, void *segment, uintptr_t newSize, uintptr_t *oldSize)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_resize_segment(id, segment, newSize, oldSize);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   获取region基本信息
 * @param   id ,               指定region的id
 * @param   theInfo [out],     region信息
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionGetInformation(OspID id, OspHeapInformationBlock_s *theInfo)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_get_information(id, (Heap_Information_block *)theInfo);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   获取region中的free信息
 * @param   id ,               指定region的id
 * @param   theInfo [out],     region的free信息
 * @return  OspStatusCode_e   返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospRegionGetFreeInformation(OspID id, OspHeapInformationBlock_s *theInfo)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet;

    ret    = rtems_region_get_free_information(id, (Heap_Information_block *)theInfo);
    ospRet = ospConvertReturnValue(ret);
    return ospRet;
}

/**
 * @brief   获取指定类型内存区域的数量
 * @param   memType ,           类型
 * @param   count [out],        数量
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospGetMemAreaCount(OspMemType_e memType, size_t *count)
{
    OspStatusCode_e ospStatus;
    int i;
    size_t cnt;

    if (memType <= OSP_MEM_INVALID || memType >= OSP_MEM_BUTT || count == NULL) {
        ospStatus = OSP_INVALID_ID;
        goto exit;
    }

    cnt = 0;
    for (i = 0; i < OspMemAreaInfoCntGet(); i++) {
        if (memType == gMemAreaInfo[i].memType) {
            cnt++;
        }
    }
    *count = cnt;
    ospStatus = OSP_SUCCESSFUL;

exit:
    return ospStatus;
}

/**
 * @brief   获取指定类型内存区域信息
 * @param   memType ,           类型
 * @param   *memInfoBuf [out],  用户提供buf,存放内存区域信息
 * @param   bufLen,             用户提供buf长度
 * @return  OspStatusCode_e     返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospGetMemAreaInfo(OspMemType_e memType, OspMemAreaInfo_s *memInfoBuf, size_t bufLen)
{
    int i;
    size_t cnt;
    OspStatusCode_e ospStatus;

    if (memType <= OSP_MEM_INVALID || memType >= OSP_MEM_BUTT || memInfoBuf == NULL) {
        ospStatus = OSP_INVALID_ID;
        goto exit;
    }
    ///< 如果用户提供的buf不够返回失败
    ospStatus = ospGetMemAreaCount(memType, &cnt);
    if (ospStatus != OSP_SUCCESSFUL) {
        goto exit;
    }
    if (cnt * sizeof(OspMemAreaInfo_s) > bufLen) {
        ospStatus = OSP_INVALID_SIZE;
        goto exit;
    }

    cnt = 0;
    for (i = 0; i < OspMemAreaInfoCntGet(); i++) {
        if (memType == gMemAreaInfo[i].memType) {
            memInfoBuf[cnt].memType = gMemAreaInfo[i].memType;
            memInfoBuf[cnt].startAddr = gMemAreaInfo[i].startAddr;
            memInfoBuf[cnt].len = gMemAreaInfo[i].len;
            cnt++;
        }
    }
    ospStatus = OSP_SUCCESSFUL;

exit:
    return ospStatus;
}
