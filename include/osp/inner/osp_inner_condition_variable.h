/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_condition_variable.h
 * @author  tianye
 * @date    2021.07.01
 * @brief   porting rtems condition variable
 * @note    NA
 */

#ifndef __OSP_INNER_CONDITION_VARIABLE_H__
#define __OSP_INNER_CONDITION_VARIABLE_H__

#ifndef __OSP_CONDITION_VARIABLE_H__
#error "include osp_condition_variable.h instead include this header file"
#endif

#include <osp_status.h>
#include <osp_types.h>
#include <rtems/thread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef rtems_condition_variable InnerOspConditonVariable_t;

#define INNER_OSP_CONDITION_VARIABLE_INITIALIZER(name) RTEMS_CONDITION_VARIABLE_INITIALIZER(name)

#ifdef __cplusplus
}
#endif

#endif
