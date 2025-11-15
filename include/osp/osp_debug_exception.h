/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_debug_exception.h
 * @author  tanzhengrong
 * @date    2023.9.27
 * @brief   debug exception
 * @note    NA
 */

#ifndef __OSP_DEBUG_EXCEPTION_H__
#define __OSP_DEBUG_EXCEPTION_H__

#include <inner/osp_inner_debug_exception.h>
#include <osp_status.h>

#ifdef __cplusplus
extern "C" {
#endif


#define OSP_DEBUG_WATCHPOINT_READ  OSP_INNER_DEBUG_WATCHPOINT_READ
#define OSP_DEBUG_WATCHPOINT_WRITE OSP_INNER_DEBUG_WATCHPOINT_WRITE
#define OSP_DEBUG_WATCHPOINT_BOTH  OSP_INNER_DEBUG_WATCHPOINT_BOTH

typedef OSPInnerDebugBeakpointInfo OSPDebugBeakpointInfo;
typedef OSPInnerDebugWatchpointInfo OSPDebugWatchpointInfo;
typedef OSPInnerDebugExceptionInfo OSPDebugExceptionInfo;

/**
 * @brief   使能调试异常
 * @param   void
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospDebugExceptionEnable(void);

/**
 * @brief   失能调试异常
 * @param   void
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
void ospDebugExceptionDisable(void);

/**
 * @brief   调试异常全局信息获取
 * @param   info[in]，信息结构指针，调用者提供内存空间
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospDebugExceptionInfoGet(OSPDebugExceptionInfo *info);

/**
 * @brief   配置断点调试异常
 * @param   info[in]，断点相关配置信息
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospDebugExceptionBreakpointConfig(OSPDebugBeakpointInfo *info);

/**
 * @brief   配置观察点调试异常
 * @param   info[in]，观察点相关配置信息
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    addr&len must in 8byte boundary
 */
OspStatusCode_e ospDebugExceptionWatchpointConfig(OSPDebugWatchpointInfo *info);

#ifdef __cplusplus
}
#endif

#endif
