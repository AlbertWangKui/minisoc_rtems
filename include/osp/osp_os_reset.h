/**
 * Copyright (C), 2022, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_os_reset.h
 * @author  tianye
 * @date    2022.03.25
 * @brief   os reset
 * @note    NA
 */

#ifndef __OSP_OS_RESET_H__
#define __OSP_OS_RESET_H__

#include <stdint.h>
#include <osp_status.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct OspOsResetInfo {
    uint32_t flashBootAddr;     ///< 当前镜像启动分区地址
    uint32_t headerLen;         ///< 镜像产品头长度
} OspOsResetInfo_s;

///< 通知链函数指针定义
typedef void (*OspOsResetNotifyChain)(void);

typedef enum {
    RESET_BY_OS_SELF = 0,
    RESET_BY_SPECIAL_EXTENSION,
}OspOsResetType_e;


/**
 * @brief   os热复位信息配置
 * @param   osResetInfo 调用者传递复位信息
 * @return  不支持的BSP返回OSP_NOT_IMPLEMENTED，支持的BSP该接口成功
 * @warning NA
 * @note    调用者配置复位信息（如当前镜像的flash起始地址、镜像头长度等）
 *
 */
OspStatusCode_e ospOsResetInfoConfig(OspOsResetInfo_s osResetInfo);

/**
 * @brief   os热复位类型配置
 * @param   osResetType 调用者传递复位类型
 * @return  不支持的BSP返回OSP_NOT_IMPLEMENTED，支持的BSP该接口成功
 * @warning NA
 * @note    调用者配置复位类型（RESET_BY_OS_SELF or RESET_BY_SPECIAL_EXTENSION）
 *
 */
OspStatusCode_e ospOsResetTypeConfig(OspOsResetType_e osResetType);


/**
 * @brief   os热复位接口执行接口
 * @param   void
 * @return  不支持的BSP返回OSP_NOT_IMPLEMENTED，支持的BSP该接口不返回
 * @warning 只能在0核或者0核中断上下文调用，且接口不返回
 * @note    调用者保证能够发生热复位；
 *
 */
OspStatusCode_e ospOsResetAction(void);

/**
 * @brief   业务注册os复位通知链接口
 * @param   void
 * @return  不支持的BSP返回OSP_NOT_IMPLEMENTED，支持的BSP该接口成功
 * @warning 通知链接口一定要保证能够在中断上下文执行
 * @note    NA
 *
 */
OspStatusCode_e ospOsResetNotifyChainFuncReg(OspOsResetNotifyChain func);

#ifdef __cplusplus
}
#endif

#endif
