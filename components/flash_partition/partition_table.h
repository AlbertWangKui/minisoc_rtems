/*
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file partition_table.h
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/08/04
 * @brief 分区表头定义
 */

#ifndef __PARTITION_TABLE_H__
#define __PARTITION_TABLE_H__

#include "common_defines.h"

#define MAX_PT_BUF_SIZE    128
#undef PT_HEADER_CHECK

#define SYS_SBR_REGION_REG_ADDR     (0xBE1004DC)
#define PT_TABLE_HEADER_MAGIC       (0x327f68ac)
#define PT_PRIMARY_HEADER_OFFSET    (0x10000)
#define PT_SECONDARY_HEADER_OFFSET  (0x14000)
#define MAX_REGION_COUNT            (509)     /* 要求header+regionDesc不能超过16K bytes: (16384-64)/32=509 */
#define PARTITION_TABLE_HEADER_LEN  (sizeof(PtTableHeader_s))
#define REGION_DESC_LEN             (sizeof(PtRegionDesc_s))
#define REG_VALUE_SBR_REGION_A      (0x0)
#define REG_VALUE_SBR_REGION_B      (0x1)

typedef enum {
    REGION_ID_BOOTLOADER_A = 1,          /* bootloader分区A*/
    REGION_ID_BOOTLOADER_B = 2,          /* bootloader分区B*/
    REGION_ID_DEFAULT_FRU,               /* default fru分区*/
    REGION_ID_USER_FRU,                  /* user fru分区*/
    REGION_ID_OPTIONROM_A,               /* optionrom分区A*/
    REGION_ID_OPTIONROM_B,               /* optionrom分区B*/
    REGION_ID_FIRMWARE_A,                /* firmware分区A*/
    REGION_ID_FIRMWARE_B,                /* firmware分区B*/
    REGION_ID_SBR_A = 25,                     /* SBR分区A*/
    REGION_ID_SBR_B,                     /* SBR分区B*/
    REGION_ID_DEFAULT_USER_SETTINGS_A,   /* default user settings分区A*/
    REGION_ID_DEFAULT_USER_SETTINGS_B,   /* default user settings分区B*/
    REGION_ID_USER_SETTINGS,             /* user settings分区*/
    REGION_ID_USER_SETTINGS_BITMAP,      /* user settings bitmap分区*/
    REGION_ID_SYS_LOG,                   /* system log分区*/
    REGION_ID_USER_LOG,                  /* user log分区*/
    REGION_ID_CORE_LOG,                  /* coredump log分区*/
    REGION_ID_SWAP,                      /* swap分区*/
    REGION_ID_MAX
} RegionID_e;

typedef struct {
    U32 magic;
    U32 headerSize; /* 头大小: PARTITION_TABLE_HEADER_LEN */
    U32 primaryPtMemStart; /* 主分区表起始地址: PT_PRIMARY_HEADER_OFFSET */
    U32 secondaryPtMemStart; /* 次分区表起始地址: PT_SECONDARY_HEADER_OFFSET */
    U32 regionDataAddrStart; /* region数据区起始地址 */
    U32 regionDataAddrEnd; /* region数据区结束地址 */
    U32 regionCount; /* 分区数量: 分区个数,max=MAX_REGION_COUNT*/
    U32 regionLen; /* 分区表长度: regionCount * REGION_DESC_LEN */
    U32 reserved[6]; /* 保留字段 */
    U32 checksumDesc; /* region desc区域校验和 */
    U32 checksumHeader; /* PtTableHeader_s的校验和 */
} PtTableHeader_s;

typedef struct {
    U32 regionId;
    U32 offset;
    U32 size;
    U8 valid;        /* 使用固定大小类型确保跨平台兼容 */
    U8 reserved1[3]; /* 填充到4字节对齐 */
    U32 reserved[4];
} __attribute__((packed)) PtRegionDesc_s;

/* Compile-time size validation */
_Static_assert(sizeof(PtTableHeader_s) == 64, "PtTableHeader_s must be 64 bytes");
_Static_assert(sizeof(PtRegionDesc_s) == 32, "PtRegionDesc_s must be 32 bytes");
_Static_assert(MAX_REGION_COUNT * sizeof(PtRegionDesc_s) + sizeof(PtTableHeader_s) <= 16384,
               "Partition table size must not exceed 16KB");

S32 ptHeaderGet(U32 type, PtTableHeader_s *pHeader);
S32 regionOffsetGet(U32 type, U32 regionId, U32 *pOffset, U32 *pSize);
U32 regionDataGet(U32 type, U32 regionId, U32 offset, U8 *pData, U32 size);

#ifdef CONFIG_PARTITION_TABLE_DUMP
S32 dumpRegionInfo(U32 type);
#endif

RegionID_e getSbrRegionId(void);

#endif /* __PARTITION_TABLE_H__ */
