/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_ocm_ecc_api.h
 * @author zhangxin3@starsmicrosystem.com
 * @date 2025/09/22
 * @brief eccapi定义
 */

#ifndef __DRV_OCM_ECC_API_H__
#define __DRV_OCM_ECC_API_H__

#include "common_defines.h"
#include "bsp_device.h"

typedef enum ocmEccErrType {
    OCM_ECC_SEC = 0, ///< 单bit
    OCM_ECC_DED = 1, ///< 多bit
    OCM_ECC_ETYPE_MAX,
} ocmEccErrType_e;

/**
 * ECC 错误回调函数类型，参数为本次错误的信息
 * errType 错误类型
 * errAddr 错误地址
 * errData 错误数据
 */
typedef void (*pOcmEccCallback)(U8 errType, U32 errAddr, U64 errData);

/**
 * @brief 初始化
 * @param [in] devId 设备ID
 * @return 0表示成功，<0表示错误
 */
S32 ocmEccInit(DevList_e devId);

/**
 * @brief 去初始化
 * @param [in] devId 设备ID
 * @return 0表示成功，<0表示错误
 */
S32 ocmEccDeinit(DevList_e devId);

/**
 * @brief 注册中断回调
 * @param [in] devId 设备ID
 * @param [in] func中断回调
 * @return 0表示成功，<0表示参数错误
 */
S32 ocmEccIrqCbRegister(DevList_e devId, pOcmEccCallback func);

/**
 * @brief 注销中断回调
 * @param [in] devId 设备ID
 * @return 0表示成功，<0表示参数错误
 */
S32 ocmEccIrqCbDeregister(DevList_e devId);

/**
 * @brief 获取错误计数
 * @param [in] devId 设备ID
 * @param [in] eType表示单bit/多bit
 * @return >= 0表示错误计数个数，<0表示参数错误
 */
S32 ocmEccErrCntGet(DevList_e devId, ocmEccErrType_e eType);

/**
 * @brief 清除错误计数
 * @param [in] devId 设备ID
 * @param [in] eType表示单bit/多bit
 * @return 0表示成功，<0表示参数错误
 */
S32 ocmEccErrCntClear(DevList_e devId, ocmEccErrType_e eType);

/**
 * @brief 获取最近的错误数据和地址
 * @param [in] devId 设备ID
 * @param [in] eType表示单bit/多bit
 * @param [out] pErrAddr返回错误数据的地址
 * @param [out] pErrData返回错误数据
 * @return 0表示成功，<0表示参数错误
 */
S32 ocmEccLastErrGet(DevList_e devId, ocmEccErrType_e eType, U32 *pErrAddr, U64 *pErrData);

#endif
