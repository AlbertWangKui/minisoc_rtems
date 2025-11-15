/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_cpuopts.h
 * @author  lichenxiang
 * @date    2021.03.02
 * @brief   os inner config define
 * @note    NA
 */

#ifndef __OSP_INNER_CPUOPTS_H
#define __OSP_INNER_CPUOPTS_H

#ifndef __OSP_CPUOPTS_H
#error "include osp_cpuopts.h instead include this header file"
#endif

#include <rtems/score/cpuopts.h>

#ifdef __cplusplus
extern "C" {
#endif


#ifdef RTEMS_SMP
#define PS3OS_SMP
#endif

#ifdef RTEMS_PROFILING
#define OS_SUPPORT_PROFILING 
#endif

#ifdef RTEMS_COREDUMP
#define PS3OS_COREDUMP
#endif

#ifdef __cplusplus
}
#endif

#endif
