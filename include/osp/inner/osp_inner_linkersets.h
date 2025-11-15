/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_linkersets.h
 * @author  lichenxiang
 * @date    2020.09.08
 * @brief   链接器使用的宏
 * @note    NA
 */

#ifndef __OPS_INNER_LINKERSETS_H
#define __OPS_INNER_LINKERSETS_H

#if (! defined __OSP_APPINIT_H ) && (! defined __OSP_MMAP_H__ ) 
#error "include osp_appinit.h or osp_mmap.h instead include this header file"
#endif

#include <rtems/linkersets.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OSP_INNER_LINKER_ROSET(set, type) RTEMS_LINKER_ROSET(set, type)

#define OSP_LINKER_ROSET_ITEM_ORDERED(set, type, item, order) RTEMS_LINKER_ROSET_ITEM_ORDERED(set, type, item, order)

#define OSP_USED RTEMS_USED

#ifdef __cplusplus
}
#endif

#endif
