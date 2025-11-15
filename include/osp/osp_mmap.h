/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_mmap.h
 * @author  tanzhengrong
 * @date    2021.12.10
 * @brief   mmu mmap
 * @note    NA
 */

#ifndef __OSP_MMAP_H__
#define __OSP_MMAP_H__

#include <inner/osp_inner_linkersets.h>
#include <inner/osp_inner_mmap.h>

#ifdef __cplusplus
extern "C" {
#endif


///< mmu内存属性
#define OSP_MMU_MMAP_FLAG_READ_ONLY              OSP_INNER_MMU_MMAP_FLAG_READ_ONLY
#define OSP_MMU_MMAP_FLAG_READ_ONLY_CACHED       OSP_INNER_MMU_MMAP_FLAG_READ_ONLY_CACHED
#define OSP_MMU_MMAP_FLAG_READ_WRITE             OSP_INNER_MMU_MMAP_FLAG_READ_WRITE
#define OSP_MMU_MMAP_FLAG_READ_WRITE_CACHED      OSP_INNER_MMU_MMAP_FLAG_READ_WRITE_CACHED
#define OSP_MMU_MMAP_FLAG_DATA_READ_ONLY         OSP_INNER_MMU_MMAP_FLAG_DATA_READ_ONLY
#define OSP_MMU_MMAP_FLAG_DATA_READ_ONLY_CACHED  OSP_INNER_MMU_MMAP_FLAG_DATA_READ_ONLY_CACHED
#define OSP_MMU_MMAP_FLAG_DATA_READ_WRITE        OSP_INNER_MMU_MMAP_FLAG_DATA_READ_WRITE
#define OSP_MMU_MMAP_FLAG_DATA_READ_WRITE_CACHED OSP_INNER_MMU_MMAP_FLAG_DATA_READ_WRITE_CACHED
#define OSP_MMU_MMAP_FLAG_CODE                   OSP_INNER_MMU_MMAP_FLAG_CODE
#define OSP_MMU_MMAP_FLAG_CODE_CACHED            OSP_INNER_MMU_MMAP_FLAG_CODE_CACHED
#define OSP_MMU_MMAP_FLAG_DEVICE                 OSP_INNER_MMU_MMAP_FLAG_DEVICE
#define OSP_MMU_MMAP_FLAG_DEVICE_READ_ONLY       OSP_INNER_MMU_MMAP_FLAG_DEVICE_READ_ONLY
#define OSP_MMU_MMAP_FLAG_DEVICE_NO_ACCESS       OSP_INNER_MMU_MMAP_FLAG_DEVICE_NO_ACCESS
#define OSP_MMU_MMAP_FLAG_STRONGLY_ORDERED       OSP_INNER_MMU_MMAP_FLAG_STRONGLY_ORDERED

#define OSP_MMU_MMAP_FLAG_STRONGLY_ORDERED_READ_ONLY   OSP_INNER_MMU_MMAP_FLAG_STRONGLY_ORDERED_READ_ONLY

///< 1m 页表
#define OSP_MMU_MMAP_KIND_SEC       OSP_INNER_MMU_MMAP_KIND_SEC
///< 4k 页表
#define OSP_MMU_MMAP_KIND_SMALL     OSP_INNER_MMU_MMAP_KIND_SMALL


/**
 * @brief   定义一段地址空间映射
 * @param   module [in], 地址空间模块名
 * @param   begin [in], 起始地址
 * @param   end [in], 结束地址
 * @param   flags [in], mmu内存属性,类型参考 OSP_MMU_MMAP_FLAG_*
 * @param   kind [in], 页表大小, 类型参可 OSP_MMU_MMAP_KIND_*
 * @warning NA
 * @note    NA
 */
#define OSP_MMAPINIT_ITEM(module, begin, end, flags, kind)  \
                 OSP_INNER_MMAPINIT_ITEM(module, begin, end, flags, kind)

/**
 * @brief   定义一段设备类型地址空间映射
 * @param   module [in], 地址空间模块名
 * @param   begin [in], 起始地址
 * @param   end [in], 结束地址
 * @param   kind [in], 页表大小, 类型参可 OSP_MMU_MMAP_KIND_*
 * @warning NA
 * @note    NA
 */
#define OSP_MMAPINIT_DEVICE_ITEM(module, begin, end, kind)  \
                 OSP_MMAPINIT_ITEM(module, begin, end, OSP_MMU_MMAP_FLAG_DEVICE, kind)


unsigned int ospSetTranslationTableEntries(const void *begin,const void *end,unsigned section_flags);
#ifdef __cplusplus
}
#endif

#endif

