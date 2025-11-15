/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_debug_exception.c
 * @author  tanzhengrong
 * @date    2023.9.27
 * @brief   封装调试异常相关接口
 * @note    NA
 */

#include <osp_debug_exception.h>

/**
 * @brief   使能调试异常
 * @param   void
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospDebugExceptionEnable(void)
{
    if ( 0 != rtems_debug_exception_enable()){
        return OSP_INTERNAL_ERROR;
    } else {
        return OSP_SUCCESSFUL;
    }
}

/**
 * @brief   失能调试异常
 * @param   void
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
void ospDebugExceptionDisable(void)
{
    rtems_debug_exception_disable();
}

/**
 * @brief   调试异常全局信息获取
 * @param   info[in]，信息结构指针，调用者提供内存空间
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospDebugExceptionInfoGet(OSPDebugExceptionInfo *info)
{
    if ( 0 != rtems_debug_exception_info_get(info)) {
        return OSP_INVALID_ADDRESS;
    } else {
        return OSP_SUCCESSFUL;
    }
}

/**
 * @brief   配置断点调试异常
 * @param   info[in]，断点相关配置信息
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospDebugExceptionBreakpointConfig(OSPDebugBeakpointInfo *info)
{
    if ( 0 != rtems_debug_exception_breakpoint_config(info)) {
        return OSP_INVALID_ADDRESS;
    } else {
        return OSP_SUCCESSFUL;
    }
}

/**
 * @brief   配置观察点调试异常
 * @param   info[in]，观察点相关配置信息
 * @return  OspStatusCode_e 返回值
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospDebugExceptionWatchpointConfig(OSPDebugWatchpointInfo *info)
{
    if ( 0 != rtems_debug_exception_watchpoint_config(info)) {
        return OSP_INVALID_ADDRESS;
    } else {
        return OSP_SUCCESSFUL;
    }
}
