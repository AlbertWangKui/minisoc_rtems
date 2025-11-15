/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_system_error.h
 * @author  tanzhengrong
 * @date    2023.10.20
 * @brief   os system err define
 * @note    NA
 */

#ifndef __OSP_INNER_SYSTEM_ERROR_H
#define __OSP_INNER_SYSTEM_ERROR_H

#ifndef __OSP_SYSTEM_ERROR_H
#error "include osp_system_error.h instead include this header file"
#endif

#include <bsp.h>

#ifdef __cplusplus
extern "C" {
#endif


#ifdef SYSERR_CPU_CACHE_LEVEL_2
#define OSP_INNER_SYSERR_CPU_CACHE_LEVEL_1  SYSERR_CPU_CACHE_LEVEL_1 
#define OSP_INNER_SYSERR_CPU_CACHE_LEVEL_2  SYSERR_CPU_CACHE_LEVEL_2
#else
#define OSP_INNER_SYSERR_CPU_CACHE_LEVEL_1  1 
#define OSP_INNER_SYSERR_CPU_CACHE_LEVEL_2  2
#endif
#ifdef __cplusplus
}
#endif

#endif
