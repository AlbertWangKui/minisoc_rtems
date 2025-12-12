/*
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file partition_table.c
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/08/04
 * @brief 分区表操作
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "partition_table.h"
#include "bsp_config.h"
#include "drv_spi_api.h"
#include "log_msg.h"

static U32 flashLoad(U32 offset, U8 *pData, U32 size)
{
    struct channel_info chan_info = { .channel_id = 0 };

    if(flash_read(&chan_info, pData, size, offset) != 0) {
        return 0;
    }
    return size;
}

static S32 ptHeaderCheck(U32 type, PtTableHeader_s *pHeader)
{
    #ifdef PT_HEADER_CHECK
    /* Validate type parameter */
    if(type > 1) {
        return -EINVAL;
    }

    if(pHeader->magic != PT_TABLE_HEADER_MAGIC) {
        return -EIO;
    }
    if(pHeader->headerSize != PARTITION_TABLE_HEADER_LEN) {
        return -EIO;
    }
    if(pHeader->primaryPtMemStart != PT_PRIMARY_HEADER_OFFSET) {
        return -EIO;
    }
    if(pHeader->secondaryPtMemStart != PT_SECONDARY_HEADER_OFFSET) {
        return -EIO;
    }
    if(pHeader->regionCount == 0) {
        return -EIO;
    }
    if(pHeader->regionLen == 0) {
        return -EIO;
    }
    if(pHeader->regionCount > MAX_REGION_COUNT) {
        return -EIO;
    }
    if(pHeader->regionLen != pHeader->regionCount * REGION_DESC_LEN) {
        return -EIO;
    }
    if(pHeader->regionDataAddrEnd <= pHeader->regionDataAddrStart) {
        return -EIO;
    }
    #endif

    /* checksum doesn't need to check because it will be checked in bootloader*/

    return EXIT_SUCCESS;
}

S32 ptHeaderGet(U32 type, PtTableHeader_s *pHeader)
{
    U32 offset = 0;

    if (NULL == pHeader) {
        return -EINVAL;
    }

    /* Validate type parameter */
    if (type > 1) {
        return -EINVAL;
    }

    /* 0: primary header, 1: secondary header */
    offset = (type == 0) ? PT_PRIMARY_HEADER_OFFSET : PT_SECONDARY_HEADER_OFFSET;

    if (flashLoad(offset, (U8 *)pHeader, sizeof(PtTableHeader_s)) != sizeof(PtTableHeader_s)) {
        return -EIO;
    }

    if (ptHeaderCheck(type, pHeader) != EXIT_SUCCESS) {
        return -EIO;
    }

    return EXIT_SUCCESS;
}

S32 regionOffsetGet(U32 type, U32 regionId, U32 *pOffset, U32 *pSize)
{
    S32 ret;
    U8 buf[MAX_PT_BUF_SIZE];
    PtTableHeader_s header;
    U32 regionCount = 0;
    U32 i;

    if (NULL == pOffset || NULL == pSize) {
        return -EINVAL;
    }

    /* Validate type parameter */
    if (type > 1) {
        return -EINVAL;
    }

    ret = ptHeaderGet(type, &header);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    regionCount = header.regionCount;
    U32 ptStartAddr = (type == 0 ? PT_PRIMARY_HEADER_OFFSET : PT_SECONDARY_HEADER_OFFSET) + PARTITION_TABLE_HEADER_LEN;
    U32 loaded = 0;
    U32 bufRegionCount = MAX_PT_BUF_SIZE / REGION_DESC_LEN;
    if (bufRegionCount == 0) {
        return -ENOMEM;
    }

    while (loaded < regionCount) {
        U32 remain = regionCount - loaded;
        U32 loadCount = (remain > bufRegionCount) ? bufRegionCount : remain;
        U32 loadSize = loadCount * REGION_DESC_LEN;

        /* Check for overflow in address calculation */
        if (loaded > (UINT32_MAX / REGION_DESC_LEN) ||
            (loaded * REGION_DESC_LEN) > (UINT32_MAX - ptStartAddr)) {
            return -ERANGE;
        }

        S32 flashRet = flashLoad(ptStartAddr + loaded * REGION_DESC_LEN, buf, loadSize);
        if (flashRet != loadSize) {
            return -EIO;
        }

        for (i = 0; i < loadCount; i++) {
            PtRegionDesc_s *pRegion = (PtRegionDesc_s *)(buf + i * REGION_DESC_LEN);
            if (pRegion->regionId == regionId) {
                *pOffset = pRegion->offset;
                *pSize = pRegion->size;
                return EXIT_SUCCESS;
            }
        }
        loaded += loadCount;
    }

    return -ENOENT;
}

/**
 * @brief 获取分区数据
 * @param [in] regionId 分区ID
 * @param [in] offset 偏移量
 * @param [out] pData 数据缓冲区
 * @param [in] size 数据大小，最多返回size个字节
 * @return 读取到的数据大小，0表示失败
 */
U32 regionDataGet(U32 type, U32 regionId, U32 offset, U8 *pData, U32 size)
{
    S32 ret;
    U32 regionOffset = 0;
    U32 regionSize = 0;

    if (NULL == pData || size == 0) {
        return 0;
    }

    /* Validate type parameter */
    if (type > 1) {
        return 0;
    }

    ret = regionOffsetGet(type, regionId, &regionOffset, &regionSize);
    if (ret != EXIT_SUCCESS) {
        return 0;
    }

    /* Check offset boundary */
    if (offset >= regionSize) {
        return 0;
    }

    /* Adjust size to not exceed region boundary */
    U32 maxReadSize = regionSize - offset;
    if (size > maxReadSize) {
        size = maxReadSize;
    }

    /* Check for overflow in address calculation */
    if (offset > (UINT32_MAX - regionOffset)) {
        return 0;
    }

    return flashLoad(regionOffset + offset, pData, size);
}

#ifdef CONFIG_PARTITION_TABLE_DUMP
S32 dumpRegionInfo(U32 type)
{
    S32 ret;
    PtTableHeader_s header;
    PtRegionDesc_s region;

    /* Validate type parameter */
    if (type > 1) {
        return -EINVAL;
    }

    ret = ptHeaderGet(type, &header);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    LOGI("header.magic: 0x%08X\n", header.magic);
    LOGI("header.headerSize: 0x%08X\n", header.headerSize);
    LOGI("header.primaryPtMemStart: 0x%08X\n", header.primaryPtMemStart);
    LOGI("header.secondaryPtMemStart: 0x%08X\n", header.secondaryPtMemStart);
    LOGI("header.regionDataAddrStart: 0x%08X\n", header.regionDataAddrStart);
    LOGI("header.regionDataAddrEnd: 0x%08X\n", header.regionDataAddrEnd);
    LOGI("header.regionCount: 0x%08X\n", header.regionCount);
    LOGI("header.regionLen: 0x%08X\n", header.regionLen);
    LOGI("header.checksumDesc: 0x%08X\n", header.checksumDesc);
    LOGI("header.checksumHeader: 0x%08X\n", header.checksumHeader);

    U32 ptStartAddr = (type == 0) ? PT_PRIMARY_HEADER_OFFSET : PT_SECONDARY_HEADER_OFFSET;
    ptStartAddr += PARTITION_TABLE_HEADER_LEN;

    for (U32 i = 0; i < header.regionCount; i++) {
        /* Check for overflow in address calculation */
        if (i > (UINT32_MAX / REGION_DESC_LEN) ||
            (i * REGION_DESC_LEN) > (UINT32_MAX - ptStartAddr)) {
            LOGI("Error: Address overflow at region %d\n", i);
            return -ERANGE;
        }

        S32 flashRet = flashLoad(ptStartAddr + i * REGION_DESC_LEN, (U8 *)&region, REGION_DESC_LEN);
        if (flashRet != REGION_DESC_LEN) {
            LOGI("Error: Failed to load region %d\n", i);
            return -EIO;
        }

        LOGI("region %d: offset=0x%08X, size=0x%08X, valid=%d\n", region.regionId, region.offset, region.size, region.valid);
    }

    return EXIT_SUCCESS;
}

#endif
