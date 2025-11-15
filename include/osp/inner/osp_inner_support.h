/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_support.h
 * @author  pengqianheng
 * @date    2020.10.13
 * @brief   porting inner support
 * @note    NA
 */

#include <rtems/rtems/support.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INNER_MILLISECONDS_TO_TICKS(_ms) RTEMS_MILLISECONDS_TO_TICKS(_ms)

#define INNER_MICROSECONDS_TO_TICKS(_us) RTEMS_MICROSECONDS_TO_TICKS(_us)

#ifdef __cplusplus
}
#endif
