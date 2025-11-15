/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_mutex.h
 * @author  lichenxiang
 * @date    2020.08.04
 * @brief   porting rtems mutex
 * @note    NA
 */

#ifndef __OSP_INNER_MUTER_H
#define __OSP_INNER_MUTER_H

#ifndef __OSP_MUTEX_H
#error "include osp_mutex.h instead include this header file"
#endif

#include <osp_status.h>
#include <osp_types.h>
#include <rtems/thread.h>

///< __BEGIN_DECLS
#ifdef __cplusplus
extern "C" {
#endif

typedef rtems_mutex InnerOspMutex_t;
typedef rtems_recursive_mutex InnerOspRecursiveMutex_t;

#define INNER_OSP_MUTEX_INITIALIZER(name) RTEMS_MUTEX_INITIALIZER(name)

#define INNER_OSP_RECURSIVE_MUTEX_INITIALIZER(name) RTEMS_RECURSIVE_MUTEX_INITIALIZER(name)

#ifdef __cplusplus
}
#endif
///< __END_DECLS
#endif
