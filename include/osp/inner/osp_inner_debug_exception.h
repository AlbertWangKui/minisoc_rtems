
/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_debug_exception.h
 * @author  tanzhengrong
 * @date    2023.9.27
 * @brief   porting inner debug exception
 * @note    NA
 */
#ifndef __OSP_INNER_DEBUG_EXCEPTION_H__
#define __OSP_INNER_DEBUG_EXCEPTION_H__


#include "rtems/hw_debug_exception.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * typedef enum {
 *  DEBUG_WATCHPOINT_READ = 1,
 *  DEBUG_WATCHPOINT_WRITE = 2,
 *  DEBUG_WATCHPOINT_BOTH = 3,
 * } watchpoint_type;
 */
#define OSP_INNER_DEBUG_WATCHPOINT_READ  DEBUG_WATCHPOINT_READ
#define OSP_INNER_DEBUG_WATCHPOINT_WRITE DEBUG_WATCHPOINT_WRITE
#define OSP_INNER_DEBUG_WATCHPOINT_BOTH  DEBUG_WATCHPOINT_BOTH

/**
 * typedef struct {
 *   uintptr_t addr;
 *   bool enable;
 *   uint8_t num;
 * } debug_beakpoint_info;
 */
typedef debug_beakpoint_info OSPInnerDebugBeakpointInfo;

/**
 * typedef struct {
 *    uintptr_t addr;
 *    uint32_t len;
 *    watchpoint_type type;
 *    bool enable;
 *    uint8_t num; 
 *  } debug_watchpoint_info;
 */
typedef debug_watchpoint_info OSPInnerDebugWatchpointInfo;

/**
 * typedef struct {
 *   bool enable;
 *   uint8_t breakpoint_num;    ///< 支持的最大断点数
 *   uint8_t watchpoint_num;    ///< 支持的最大观察点数
 *   debug_beakpoint_info breakpoints[ARCH_HW_BREAKPOINT_MAX];
 *   debug_watchpoint_info watchpoints[ARCH_HW_WATCHPOINT_MAX];
 * } debug_exception_info;
 */
typedef debug_exception_info OSPInnerDebugExceptionInfo;

#ifdef __cplusplus
}
#endif

#endif
