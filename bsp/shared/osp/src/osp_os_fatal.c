/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co., Ltd.
 * @file osp_os_fatal.c
 * @brief os fatal notify
 * @author tianye
 * @version
 * @date 2024-05-08
 */

#include <osp_os_fatal.h>
#include <osp_status.h>
#include <bspopts.h>
#if defined(PS3OS_NIC_100G) || defined(PS3OS_IB_SWITCH) || defined(PS3OS_EXPANDER_V200) || defined(PS3OS_SWITCH_V200) \
    || defined(PS3OS_SWITCH_V300) || defined(PS3OS_NETCHIP) || defined(PS3OS_IBCHIP) || defined(PS3OS_IBCHIP_V100)
#include <bsp/bspfatal.h>
#endif

OspStatusCode_e ospOsFatalNotifyFuncReg(OspOsFatalNotify func)
{
#if defined(PS3OS_NIC_100G) || defined(PS3OS_IB_SWITCH) || defined(PS3OS_EXPANDER_V200) || defined(PS3OS_SWITCH_V200) \
    || defined(PS3OS_SWITCH_V300) || defined(PS3OS_NETCHIP) || defined(PS3OS_IBCHIP) || defined(PS3OS_IBCHIP_V100)
    bsp_fatal_notify_func_reg((bsp_fatal_notify)func);
    return OSP_SUCCESSFUL;
#else
    (void)func;
    return OSP_NOT_IMPLEMENTED;
#endif
}
