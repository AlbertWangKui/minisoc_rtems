/**
 * Copyright (C), 2021, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_sys_clock.h
 * @author  tianye
 * @date    2021.11.10
 * @brief   NA
 * @note    NA
 */

#ifndef __OSP_SYS_CLOCK_H__
#define __OSP_SYS_CLOCK_H__

#include <bsp.h>
#ifdef PS3OS_EXPANDER_V200
    #include "osp_sys_clock_expander_v200.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif
#if defined(PS3OS_NETCHIP) || defined(PS3OS_NIC_100G) || defined(PS3OS_SWITCH_V300) || defined(PS3OS_IBCHIP) || defined(PS3OS_IBCHIP_V100)
/**
 * @brief   获取系统频率
 * @param   none
 * @return  HZ
 * @warning NA
 * @note    NA
 */
uint32_t ospGetSysClock(void);
#endif

#ifdef __cplusplus
}
#endif
#endif
