/**
 * Copyright (C), 2022, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_os_reset.c
 * @author  tianye
 * @date    2022.03.25
 * @brief   封装os热复位对外接口
 * @note    NA
 */

#include <osp_os_reset.h>
#include <osp_status.h>
#include <rtems/rtems/intr.h>
#include <bsp.h>
#if defined(PS3OS_HBA_R5)  || \
    defined(PS3OS_EXPANDER_V200) || defined(PS3OS_HBA_V200) || defined(PS3OS_NETCHIP) || defined(PS3OS_SWITCH_V300) ||\
    defined(PS3OS_SWITCH_V200) || defined(PS3OS_IBCHIP) || defined(PS3OS_NIC_100G) || defined(PS3OS_IBCHIP_V100) ||\
    defined(PS3OS_IB_SWITCH)
#include <bsp/bsposreset.h>
#endif

/**
 * @brief   os热复位信息配置
 * @param   osResetInfo 调用者传递复位信息
 * @return  不支持的BSP返回OSP_NOT_IMPLEMENTED，支持的BSP该接口成功
 * @warning NA
 * @note    调用者配置复位信息（如当前镜像的flash起始地址、镜像头长度等）
 *
 */
OspStatusCode_e ospOsResetInfoConfig(OspOsResetInfo_s osResetInfo)
{
#if defined(PS3OS_HBA_R5)  || \
    defined(PS3OS_EXPANDER_V200) || defined(PS3OS_HBA_V200) || defined(PS3OS_NETCHIP) || defined(PS3OS_SWITCH_V300) ||\
    defined(PS3OS_SWITCH_V200) || defined(PS3OS_IBCHIP) || (defined(PS3OS_NIC_100G) && !defined(PS3OS_NIC_100G_SLAVE)) || defined(PS3OS_IBCHIP_V100) ||\
    defined(PS3OS_IB_SWITCH)
    os_reset_info_s resetinfo;
    resetinfo.flash_boot_addr = osResetInfo.flashBootAddr;
    resetinfo.header_len = osResetInfo.headerLen;
    bsp_os_reset_info_config(resetinfo);
    return OSP_SUCCESSFUL;
#else
    osResetInfo = osResetInfo;
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   os热复位类型配置
 * @param   osResetType 调用者传递复位类型
 * @return  不支持的BSP返回OSP_NOT_IMPLEMENTED，支持的BSP该接口成功
 * @warning NA
 * @note    调用者配置复位类型（RESET_BY_OS_SELF or RESET_BY_SPECIAL_EXTENSION）
 *
 */
OspStatusCode_e ospOsResetTypeConfig(OspOsResetType_e osResetType)
{
#if defined(PS3OS_HBA_R5)  || \
    defined(PS3OS_HBA_V200)
    bsp_os_reset_type_config((int) osResetType);
    return OSP_SUCCESSFUL;
#else
    osResetType = osResetType;
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   os热复位接口执行接口
 * @param   void
 * @return  不支持的BSP返回OSP_NOT_IMPLEMENTED，支持的BSP该接口不返回
 * @warning 只能在0核或者0核中断上下文调用，且接口不返回
 * @note    调用者保证能够发生热复位；
 *
 */
OspStatusCode_e ospOsResetAction(void)
{
#if defined(PS3OS_HBA_R5)  || \
    defined(PS3OS_EXPANDER_V200) || defined(PS3OS_HBA_V200) || defined(PS3OS_NETCHIP) || defined(PS3OS_SWITCH_V300) ||\
    defined(PS3OS_SWITCH_V200) || defined(PS3OS_IBCHIP) || (defined(PS3OS_NIC_100G) && !defined(PS3OS_NIC_100G_SLAVE)) || defined(PS3OS_IBCHIP_V100) ||\
    defined(PS3OS_IB_SWITCH)
    bsp_os_reset_action();
    return OSP_SUCCESSFUL;
#else
    return OSP_NOT_IMPLEMENTED;
#endif
}

/**
 * @brief   业务注册os复位通知链接口
 * @param   void
 * @return  不支持的BSP返回OSP_NOT_IMPLEMENTED，支持的BSP该接口成功
 * @warning 通知链接口一定要保证能够在中断上下文执行
 * @note    NA
 *
 */
OspStatusCode_e ospOsResetNotifyChainFuncReg(OspOsResetNotifyChain func)
{
#if defined(PS3OS_HBA_R5) || \
    defined(PS3OS_EXPANDER_V200) || defined(PS3OS_HBA_V200) || defined(PS3OS_NETCHIP) || defined(PS3OS_SWITCH_V300) ||\
    defined(PS3OS_SWITCH_V200) || defined(PS3OS_IBCHIP) || defined(PS3OS_NIC_100G) || defined(PS3OS_IB_SWITCH) || defined(PS3OS_IBCHIP_V100)
    bsp_os_reset_notify_chain_func_reg((bsp_os_reset_notify_chain)func);
    return OSP_SUCCESSFUL;
#else
    func = func;
    return OSP_NOT_IMPLEMENTED;
#endif
}
