/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file sbr_api.c
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/08/01
 * @brief SBR image management
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "sbr_api.h"
#include "verify.h"
#include "log_msg.h"
#include "partition_table.h"
#include "drv_spi_api.h"
#include "bsp_device.h"

#ifdef CONFIG_SBR_IMG_IN_RAM
extern U8 _binary_sbr_bin_start[];
extern U8 _binary_sbr_bin_end[];
static U8 *pSbrImg = _binary_sbr_bin_start;  /* SBR镜像在RAM中的指针 */
#endif

#ifndef CONFIG_SBR_IMG_IN_RAM
static U32 sbrRegionOffset = 0, sbrRegionSize = 0;
#endif
static U32 bitMapRegionOffset = 0, bitMapRegionSize = 0;
static U32 usrSettingsRegionOffset = 0, usrSettingsRegionSize = 0;
static U32 defaultUsrSettingsRegionOffset = 0, defaultUsrSettingsRegionSize = 0;

/**
 * @brief 从SBR镜像的指定偏移处读取指定长度的数据到缓冲区
 * @param offset 镜像偏移位置
 * @param buf 目标缓冲区
 * @param len 要读取的长度
 * @return 实际读取的长度，失败返回0
 */
static U32 sbrLoad(U32 offset, U8 *buf, U32 len)
{
     U32 readLen = 0;
#ifndef CONFIG_SBR_IMG_IN_RAM
     RegionID_e sbrRegionId = 0;
#endif

     /* 参数检查 */
     if (buf == NULL || len == 0) {
         return 0;
     }

#ifdef CONFIG_SBR_IMG_IN_RAM
     /* RAM模式：直接从内存中读取 */
     if (pSbrImg == NULL) {
         return 0;  /* SBR镜像指针为空 */
     }

     /* 从内存中复制数据 */
     memcpy(buf, pSbrImg + offset, len);
     readLen = len;  /* 设置返回值 */

#else
     sbrRegionId = getSbrRegionId();
     if ((sbrRegionId != REGION_ID_SBR_A) && (sbrRegionId != REGION_ID_SBR_B)) {
        LOGE("get sbr region id failed\r\n");
        return 0;
     }
     if(sbrRegionOffset == 0 || sbrRegionSize == 0) {
        if(regionOffsetGet(0, sbrRegionId, &sbrRegionOffset, &sbrRegionSize) != EXIT_SUCCESS) {
            LOGE("get sbr region offset failed\r\n");
            return 0;
        }
     }
     struct channel_info chan_info = { .channel_id = 0 };
     if(flash_read(&chan_info, buf, len, offset + sbrRegionOffset) != 0) {
         LOGE("read sbr data failed\r\n");
         return 0;
     } else {
         readLen = len;
     }
#endif

     return readLen;  /* 返回实际读取的长度 */
}

static U32 sbrSave(U32 offset, U8 *buf, U32 len)
{
     U32 writeLen = 0;
#ifndef CONFIG_SBR_IMG_IN_RAM
     RegionID_e sbrRegionId = 0;
#endif

     /* 参数检查 */
     if (buf == NULL || len == 0) {
         return 0;
     }

#ifdef CONFIG_SBR_IMG_IN_RAM
     /* RAM模式：直接写入内存 */
     if (pSbrImg == NULL) {
         return 0;  /* SBR镜像指针为空 */
     }

     /* 写入数据到内存 */
     memcpy(pSbrImg + offset, buf, len);
     writeLen = len;
#else
     if(sbrRegionOffset == 0 || sbrRegionSize == 0) {
        sbrRegionId = getSbrRegionId();
        if ((sbrRegionId != REGION_ID_SBR_A) && (sbrRegionId != REGION_ID_SBR_B)) {
            LOGE("get sbr region id failed\r\n");
            return 0;
        }
        if(regionOffsetGet(0, sbrRegionId, &sbrRegionOffset, &sbrRegionSize) != EXIT_SUCCESS) {
            LOGE("get sbr region offset failed\r\n");
            return 0;
        }
     }
     struct channel_info chan_info = { .channel_id = 0 };
     ///< erase the flash first
     if (flash_erase(&chan_info, offset + sbrRegionOffset, len, true) != 0) {
         LOGE("erase sbr data failed\r\n");
         return 0;
     }
     ///< write the data to the flash
     if (flash_write(&chan_info, buf, len, offset + sbrRegionOffset) != 0) {
         LOGE("write sbr data failed\r\n");
         return 0;
     } else {
         writeLen = len;
     }
#endif

     return writeLen;  /* 返回实际写入的长度 */
}

 static S32 sbrUpdateMetaSize(void)
 {
    sbrHeader_s header;
    S32 ret = EXIT_SUCCESS;
    U16 sbrMetaSize;

    /* 读取头部信息 */
    if (sbrLoad(0, (U8 *)&header, sizeof(sbrHeader_s)) != sizeof(sbrHeader_s)) {
        ret = -EIO;
        goto exit;
    }

    /* 计算sbrMetaSize: sizeof(sbrHeader_s) + entryCount * sizeof(sbrEntry_s) */
    sbrMetaSize = sizeof(sbrHeader_s) + header.entryCount * sizeof(sbrEntry_s);

    /* 更新header中的sbrMetaSize */
    header.sbrMetaSize = sbrMetaSize;

    /* 写回header */
    if(sbrSave(0, (U8 *)&header, sizeof(sbrHeader_s)) != sizeof(sbrHeader_s)) {
        ret = -EIO;
        goto exit;
    }

    ret = EXIT_SUCCESS;
exit:
    return ret;
 }

S32 sbrValidate(void)
{
    sbrHeader_s header;
    U16 expectedMetaSize;

    /* 读取头部信息 */
    if (sbrLoad(0, (U8 *)&header, sizeof(sbrHeader_s)) != sizeof(sbrHeader_s)) {
        return -ENOENT;  /* 文件不存在或读取失败 */
    }

    /* 验证魔数 */
    if (header.magic != SBR_MAGIC) {
        return -EINVAL;
    }

    /* 验证头部ID */
    if (header.hdrID != SBR_HEADER_ID) {
        return -EINVAL;
    }

    /* 计算期望的sbrMetaSize: sizeof(sbrHeader_s) + entryCount * sizeof(sbrEntry_s) */
    expectedMetaSize = sizeof(sbrHeader_s) + header.entryCount * sizeof(sbrEntry_s);

    /* 验证sbrMetaSize */
    if (header.sbrMetaSize != expectedMetaSize) {
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief 读取指定设备的SBR配置
 * @param devID 设备ID
 * @param Cfg 配置数据缓冲区
 * @return 返回实际读取的数据长度，失败返回0
 */
U32 devSbrRead(DevList_e devID, void *CfgBuf, U32 offset, U32 len)
{
    U8 buffer[MAX_SBR_BUF_SIZE];
    sbrHeader_s header = {0};
    sbrEntry_s *entryPtr = NULL;
    U32 i = 0;
    U32 readSize = 0, actualRead = 0, entriesPerRead = 0;
    U32 currentOffset = 0;

    if (CfgBuf == NULL || len == 0) {
        errno = EINVAL;
        return 0;
    }

    /* 读取头部信息 */
    if (sbrLoad(HEADER_OFFSET_SBR, (U8 *)&header, sizeof(sbrHeader_s)) != sizeof(sbrHeader_s)) {
        errno = EIO;
        return 0;  /* 文件不存在或读取失败 */
    }

    /* 计算大小并检查溢出 */
    if (header.entryCount > (UINT32_MAX / sizeof(sbrEntry_s))) {
        errno = EINVAL;
        return 0;  /* 防止整数溢出 */
    }

    /* 验证魔数 */
    if (header.magic != SBR_MAGIC) {
        errno = EINVAL;
        return 0;
    }

    /* 验证头部ID */
    if (header.hdrID != SBR_HEADER_ID) {
        errno = EINVAL;
        return 0;
    }

    /* 计算可以一次性读取的完整entries数量 */
    entriesPerRead = MAX_SBR_BUF_SIZE / sizeof(sbrEntry_s);

    /* 缓冲区太小，无法读取任何entry */
    if (entriesPerRead == 0) {
        errno = EINVAL;
        return 0;
    }

    /* 分批读取entries */
    for (i = 0; i < header.entryCount; i += entriesPerRead) {
        /* 计算本次读取的entries数量 */
        U32 currentBatchSize = (header.entryCount - i < entriesPerRead) ?
                              (header.entryCount - i) : entriesPerRead;

        /* 计算读取大小（当前批次的entries） */
        readSize = currentBatchSize * sizeof(sbrEntry_s);

        /* 一次性读取头部和当前批次的entries */
        actualRead = sbrLoad(currentOffset + sizeof(sbrHeader_s) + HEADER_OFFSET_SBR, buffer, readSize);
        if (actualRead != readSize) {
            errno = EIO;
            return 0;  /* 读取失败 */
        } else {
            currentOffset += actualRead;
        }

        /* 在当前批次中查找目标设备 */
        for (U32 j = 0; j < currentBatchSize; j++) {
            entryPtr = (sbrEntry_s *)(buffer + j * sizeof(sbrEntry_s));

            /* 检查设备ID和有效性 */
            if (entryPtr->devID == devID && entryPtr->valid == 1) {
                /* 计算数据在文件中的偏移位置 */
                U32 dataOffset = header.sbrMetaSize + HEADER_OFFSET_SBR + entryPtr->offset + offset;
                /* 检查偏移是否超出设备配置大小 */
                if (offset >= entryPtr->size) {
                    errno = EINVAL;
                    return 0;
                }

                /* 计算实际可读取的数据大小 */
                U32 availableSize = entryPtr->size - offset;
                U32 dataSize = len > availableSize ? availableSize : len;

                /* 读取配置数据 */
                if (sbrLoad(dataOffset, (U8 *)CfgBuf, dataSize) != dataSize) {
                    errno = EIO;
                    return 0;  /* 读取失败 */
                } else {
                    return dataSize;
                }
            }
        }
    }

    errno = EINVAL;
    return 0;  /* 未找到对应的设备配置 */
}

U32 devSbrWrite(DevList_e devID, void *CfgBuf, U32 offset, U32 len)
{
    sbrHeader_s header;
    U8 buffer[MAX_SBR_BUF_SIZE];
    U32 entriesSize, entriesPerRead, i, readSize, actualRead;
    U32 currentOffset;
    sbrEntry_s *entryPtr = NULL;

    if (CfgBuf == NULL || len == 0) {
        errno = EINVAL;
        return 0;
    }

    /* 读取头部信息 */
    if (sbrLoad(0, (U8 *)&header, sizeof(sbrHeader_s)) != sizeof(sbrHeader_s)) {
        errno = ENOENT;
        return 0;
    }

    /* 验证魔数和头部ID */
    if (header.magic != SBR_MAGIC || header.hdrID != SBR_HEADER_ID) {
        errno = EINVAL;
        return 0;
    }

    /* 计算entries区大小并检查溢出 */
    if (header.entryCount > (UINT32_MAX / sizeof(sbrEntry_s))) {
        errno = EINVAL;
        return 0;  /* 防止整数溢出 */
    }
    entriesSize = header.entryCount * sizeof(sbrEntry_s);

    /* 每次能读取多少个entry */
    entriesPerRead = MAX_SBR_BUF_SIZE / sizeof(sbrEntry_s);
    if (entriesPerRead == 0) {
        errno = EINVAL;
        return 0;
    }

    currentOffset = 0;

    /* 分批读取entries */
    for (i = 0; i < header.entryCount; i += entriesPerRead) {
        U32 currentBatchSize = (header.entryCount - i < entriesPerRead) ?
                              (header.entryCount - i) : entriesPerRead;
        readSize = currentBatchSize * sizeof(sbrEntry_s);

        /* 读取当前批次的entries */
        actualRead = sbrLoad(sizeof(sbrHeader_s) + currentOffset, buffer, readSize);
        if (actualRead != readSize) {
            errno = EIO;
            return 0;
        }

        /* 查找目标设备 */
        for (U32 j = 0; j < currentBatchSize; j++) {
            entryPtr = (sbrEntry_s *)(buffer + j * sizeof(sbrEntry_s));
            if (entryPtr->devID == devID && entryPtr->valid == 1) {
                /* 检查写入长度是否超过entry的size */
                U32 writeLen = (offset + len > entryPtr->size) ? (entryPtr->size - offset) : len;
                U32 dataOffset = sizeof(sbrHeader_s) + entriesSize + entryPtr->offset;
                if (sbrSave(dataOffset + offset, (U8 *)CfgBuf, writeLen) != writeLen) {
                    LOGE("write sbr data failed\r\n");
                    errno = EIO;
                    return 0;
                } else {
                    if(sbrUpdateMetaSize() != EXIT_SUCCESS) {
                        LOGE("update sbr meta size failed\r\n");
                        errno = EIO;
                        return 0;
                    }
                    return writeLen;
                }
            }
        }
        currentOffset += readSize;
    }

    errno = EINVAL;
    return 0;
}

U32 usrSettingGet(U32 offset, U8 *buf, U8 len)
{
    struct channel_info chan_info = { .channel_id = 0 };
    U8 userChunk[MAX_CHUNK_SIZE], defaultChunk[MAX_CHUNK_SIZE];
    U8 i;

    if (buf == NULL || len == 0 || len > MAX_CHUNK_SIZE) {
        LOGE("%s: invalid parameters\r\n", __func__);
        return 0;
    }

    if (usrSettingsRegionOffset == 0 || usrSettingsRegionSize == 0) {
        if(regionOffsetGet(0, REGION_ID_USER_SETTINGS, &usrSettingsRegionOffset, &usrSettingsRegionSize) != EXIT_SUCCESS) {
            LOGE("get user settings region offset failed\r\n");
            return 0;
        }
    }

    if (defaultUsrSettingsRegionOffset == 0 || defaultUsrSettingsRegionSize == 0) {
        if(regionOffsetGet(0, REGION_ID_DEFAULT_USER_SETTINGS_A, &defaultUsrSettingsRegionOffset, &defaultUsrSettingsRegionSize) != EXIT_SUCCESS) {
            LOGE("get default user settings region offset failed\r\n");
            return 0;
        }
    }

    if (bitMapRegionOffset == 0 || bitMapRegionSize == 0) {
        if(regionOffsetGet(0, REGION_ID_USER_SETTINGS_BITMAP, &bitMapRegionOffset, &bitMapRegionSize) != EXIT_SUCCESS) {
            LOGE("get bitmap region offset failed\r\n");
            return 0;
        }
    }

    if(flash_read(&chan_info, userChunk, len, usrSettingsRegionOffset + offset) != 0) {
        LOGE("read user settings data failed\r\n");
        return 0;
    }

    if(flash_read(&chan_info, defaultChunk, len, defaultUsrSettingsRegionOffset + offset) != 0) {
        LOGE("read default user settings data failed\r\n");
        return 0;
    }

    U32 startBitmapByteOffset = offset / 8;
    U32 endBitmapByteOffset = (offset + len - 1) / 8;
    U32 bitmapBytesToRead = endBitmapByteOffset - startBitmapByteOffset + 1;
    U8 bitmapBytes[8] = {0};

    if (bitmapBytesToRead > 8) bitmapBytesToRead = 8;

    if(flash_read(&chan_info, bitmapBytes, bitmapBytesToRead, bitMapRegionOffset + startBitmapByteOffset) != 0) {
        LOGE("read bitmap bytes failed\r\n");
        memset(bitmapBytes, 0, bitmapBytesToRead);
        return 0;
    }

    for (i = 0; i < len; i++) {
        U32 absoluteBitOffset = offset + i;
        U32 bitmapByteIndex = absoluteBitOffset / 8;
        U32 bitIndexInByte = absoluteBitOffset % 8;
        U32 relativeBitmapIndex = bitmapByteIndex - startBitmapByteOffset;

        if (relativeBitmapIndex < bitmapBytesToRead &&
            (bitmapBytes[relativeBitmapIndex] & (1U << bitIndexInByte))) {
            buf[i] = defaultChunk[i];
        } else {
            buf[i] = userChunk[i];
        }
    }

    return len;
}

U32 usrSettingSet(U32 offset, U8 *buf, U8 len)
{
    struct channel_info chan_info = { .channel_id = 0 };
    U8 i;

    if (buf == NULL || len == 0 || len > MAX_CHUNK_SIZE) {
        LOGE("%s: invalid parameters\r\n", __func__);
        return 0;
    }

    if (usrSettingsRegionOffset == 0 || usrSettingsRegionSize == 0) {
        if(regionOffsetGet(0, REGION_ID_USER_SETTINGS, &usrSettingsRegionOffset, &usrSettingsRegionSize) != EXIT_SUCCESS) {
            LOGE("get user settings region offset failed\r\n");
            return 0;
        }
    }

    if (bitMapRegionOffset == 0 || bitMapRegionSize == 0) {
        if(regionOffsetGet(0, REGION_ID_USER_SETTINGS_BITMAP, &bitMapRegionOffset, &bitMapRegionSize) != EXIT_SUCCESS) {
            LOGE("get bitmap region offset failed\r\n");
            return 0;
        }
    }

    if(flash_erase(&chan_info, usrSettingsRegionOffset + offset, len, true) != 0) {
        LOGE("erase user settings data failed\r\n");
        return 0;
    }

    if(flash_write(&chan_info, buf, len, usrSettingsRegionOffset + offset) != 0) {
        LOGE("write user settings data failed\r\n");
        return 0;
    }

    U32 startBitmapByteOffset = offset / 8;
    U32 endBitmapByteOffset = (offset + len - 1) / 8;
    U32 bitmapBytesToRead = endBitmapByteOffset - startBitmapByteOffset + 1;
    U8 bitmapBytes[8] = {0};

    if (bitmapBytesToRead > 8) bitmapBytesToRead = 8;

    if(flash_read(&chan_info, bitmapBytes, bitmapBytesToRead, bitMapRegionOffset + startBitmapByteOffset) != 0) {
        LOGE("%s: read bitmap bytes failed, user settings map may be corrupted\r\n", __func__);
        memset(bitmapBytes, 0, bitmapBytesToRead);
    }

    for (i = 0; i < len; i++) {
        U32 absoluteBitOffset = offset + i;
        U32 bitmapByteIndex = absoluteBitOffset / 8;
        U32 bitIndexInByte = absoluteBitOffset % 8;
        U32 relativeBitmapIndex = bitmapByteIndex - startBitmapByteOffset;

        if (relativeBitmapIndex < bitmapBytesToRead) {
            bitmapBytes[relativeBitmapIndex] &= ~(1U << bitIndexInByte);
        }
    }

    if(flash_erase(&chan_info, bitMapRegionOffset + startBitmapByteOffset, bitmapBytesToRead, true) != 0) {
        LOGE("%s: erase bitmap bytes failed, user settings map may be corrupted\r\n", __func__);
        return 0;
    }

    if(flash_write(&chan_info, bitmapBytes, bitmapBytesToRead, bitMapRegionOffset + startBitmapByteOffset) != 0) {
        LOGE("%s: write bitmap bytes failed, user settings map may be corrupted\r\n", __func__);
        return 0;
    }

    return len;
}

S32 usrBitMapReset(void)
{
    struct channel_info chan_info = { .channel_id = 0 };

    /* 获取位图区域偏移和大小 */
    if(bitMapRegionOffset == 0 || bitMapRegionSize == 0) {
        if(regionOffsetGet(0, REGION_ID_USER_SETTINGS_BITMAP, &bitMapRegionOffset, &bitMapRegionSize) != EXIT_SUCCESS) {
            LOGE("get bitmap region offset failed\r\n");
            return -EXIT_FAILURE;
        }
    }

    /* 擦除位图区域 */
    if(flash_erase(&chan_info, bitMapRegionOffset, bitMapRegionSize, true) != 0) {
        LOGE("%s: erase bitmap region failed\r\n", __func__);
        return -EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

