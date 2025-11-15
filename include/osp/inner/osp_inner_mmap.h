/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_mmap.h
 * @author  tanzhengrong
 * @date    2021.12.10
 * @brief   porting inner mmu mmap
 * @note    NA
 */
#ifndef __OSP_INNER_MMAP_H__
#define __OSP_INNER_MMAP_H__

#ifndef __OSP_MMAP_H__
#error "include osp_mmap.h instead include this header file"
#endif

#include <bsp/arm-cp15-start.h> 

#ifdef __cplusplus
extern "C" {
#endif


#define OSP_INNER_MMU_MMAP_FLAG_READ_ONLY              ARMV7_MMU_READ_ONLY
#define OSP_INNER_MMU_MMAP_FLAG_READ_ONLY_CACHED       ARMV7_MMU_READ_ONLY_CACHED
#define OSP_INNER_MMU_MMAP_FLAG_READ_WRITE             ARMV7_MMU_READ_WRITE
#define OSP_INNER_MMU_MMAP_FLAG_READ_WRITE_CACHED      ARMV7_MMU_READ_WRITE_CACHED
#define OSP_INNER_MMU_MMAP_FLAG_DATA_READ_ONLY         ARMV7_MMU_DATA_READ_ONLY        
#define OSP_INNER_MMU_MMAP_FLAG_DATA_READ_ONLY_CACHED  ARMV7_MMU_DATA_READ_ONLY_CACHED
#define OSP_INNER_MMU_MMAP_FLAG_DATA_READ_WRITE        ARMV7_MMU_DATA_READ_WRITE     
#define OSP_INNER_MMU_MMAP_FLAG_DATA_READ_WRITE_CACHED ARMV7_MMU_DATA_READ_WRITE_CACHED
#define OSP_INNER_MMU_MMAP_FLAG_CODE                   ARMV7_MMU_CODE
#define OSP_INNER_MMU_MMAP_FLAG_CODE_CACHED            ARMV7_MMU_CODE_CACHED
#define OSP_INNER_MMU_MMAP_FLAG_DEVICE                 ARMV7_MMU_DEVICE
#define OSP_INNER_MMU_MMAP_FLAG_DEVICE_READ_ONLY       ARMV7_MMU_DEVICE_READ_ONLY
#define OSP_INNER_MMU_MMAP_FLAG_DEVICE_NO_ACCESS       ARMV7_MMU_DEVICE_NO_ACCESS 

#define OSP_INNER_MMU_MMAP_FLAG_STRONGLY_ORDERED_READ_ONLY       ARMV7_MMU_STRONGLY_ORDERED_READ_ONLY 
#define OSP_INNER_MMU_MMAP_FLAG_STRONGLY_ORDERED                 ARMV7_MMU_STRONGLY_ORDERED 


#define OSP_INNER_MMU_MMAP_KIND_SEC       ARM_MMU_MMAP_KIND_SEC
#define OSP_INNER_MMU_MMAP_KIND_SMALL     ARM_MMU_MMAP_KIND_SMALL

typedef arm_cp15_start_section_config osp_mmapinit_item;

#define OSP_INNER_MMAPINIT_ITEM(module, begin, end, flags, kind)        \
                        RTEMS_MMAPINIT_ITEM(module, begin, end, flags, kind)

#ifdef __cplusplus
}
#endif

#endif

