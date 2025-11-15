
/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_asan.h
 * @author  lichenxiang
 * @date    2021.11.15
 * @brief   os asan define
 * @note    NA
 */

#ifndef __OSP_INNER_ASAN_H
#define __OSP_INNER_ASAN_H

#ifndef __OSP_ASAN_H
#error "include osp_asan.h instead include this header file"
#endif

#include <asan.h>

#ifdef __cplusplus
extern "C" {
#endif


#define INNER_OSP_ASAN_USER_DATA_READ_ZONE ASAN_USER_DATA_READ_ZONE 
#define INNER_OSP_ASAN_USER_HEAP_READ_ZONE ASAN_USER_HEAP_READ_ZONE 

#ifdef __cplusplus
}
#endif

#endif
