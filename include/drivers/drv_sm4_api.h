/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_SM4_api.h
 * @author sangwei (sangwei@starsmicrosystem.com)
 * @date 2025/09/25
 * @brief
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/09/25   sangwei         the first version
 *
 *
 */

#ifndef __DRV_SM4_API_H__
#define __DRV_SM4_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common_defines.h"
#include "bsp_device.h"

typedef struct sm4Params{
    U32 srcAddr; ///< 表示sm4输入数据的开始地址;
    U32 destAddr; ///< 表示sm4加密后的数据的存储起始地址;
    U32 len; ///< 表示sm4输入数据的长度;
    U32 keys[4]; ///< 表示sm4 加密用的keys；
    U32 cnt[4]; ///< 表示sm4 加密用的cnt；
    U32 cntLen[4]; ///< 表示sm4 加密用的cnt_len；
} sm4TransactionDesc_s;

typedef enum sm4Mode {
    SM4_MODE_HW = 0, ///< 表示keys、cnt、cnt_len全部来自硬件;
    SM4_MODE_SW = 1, ///< 表示keys、cnt、cnt_len全部来自软件;
    SM4_MODE_KEY_HW_AND_CNT_SW = 2, ///< 表示keys来自硬件、cnt、cnt_len来自软件;
    SM4_MODE_INVALID
}  sm4Mode_e;

/**
 * @brief SM4 init
 * @param [in] device id
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sm4Init(DevList_e devId);

/**
 * @brief SM4 deinit
 * @param [in] device id
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sm4DeInit(DevList_e devId);

/**
 * @brief SM4 data encrypt
 * @param [in] device id
 *             sm4TransactionDesc_s,
 *             sm4Mode_e
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 */
S32 sm4EncryptData(DevList_e devId,  sm4TransactionDesc_s *transactionDesc,  sm4Mode_e mode);

#endif /* __DRV_SM4_API_H__ */
