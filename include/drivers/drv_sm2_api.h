/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 * @file    drv_sm2_api.h
 * @author  pengjb@starsmicrosystem.com
 * @date    2025/09/30
 * @brief   SM2 cryptographic driver API
 */

#ifndef __DRV_SM2_API_H__
#define __DRV_SM2_API_H__

typedef void (*sm2IrqCallBack)(void *arg);

typedef enum _sm2KeyMode {
    SM2_KEY_SOFT_MODE = 0,
    SM2_KEY_EFUSE_MODE= 1,
    SM2_KEY_MODE_MAX = 2,
} sm2KeyMode;

/**
 * @brief   SM2驱动初始化
 * @param[in] devId 控制器编号
 * @return  EXIT_SUCCESS成功 / -EBUSY获取锁失败 / -EINVAL参数无效 / -EIO硬件操作失败 / -ENOMEM内存分配失败
 */
S32 sm2Init(DevList_e devId);

/**
 * @brief   中断方式实现SM2验签
 * @param[in] devId   控制器编号
 * @param[in] signBuf 签名数据，共64bytes, 前32bytes是sign_r，后32bytes是sign_s
 * @param[in] pkey    公钥数据，共64bytes, 前32bytes是keys_x，后32bytes是keys_y；如果keys来源于eFuse，参数直接写成NULL
 * @param[in] dataIn  存储sm3计算的hash值, hash值是256bit
 * @return  EXIT_SUCCESS验签请求发送成功 / -EINVAL参数无效 / -EBUSY获取锁失败 / -EIO硬件操作失败
 */
S32 sm2DoVerify(DevList_e devId, U32 *signBuf, U32 *pkey, U32 *dataIn);

/**
 * @brief   查询方式实现SM2验签
 * @param[in] devId   控制器编号
 * @param[in] dataIn  存储sm3计算的hash值, hash值是256bit
 * @param[in] signBuf 签名数据，共64bytes, 前32bytes是sign_r，后32bytes是sign_s
 * @param[in] pkey    公钥数据，共64bytes, 前32bytes是keys_x，后32bytes是keys_y；如果keys来源于eFuse，参数直接写成NULL
 * @param[in] timeout 轮询超时时间，单位为1毫秒
 * @return  EXIT_SUCCESS验签成功 / -EINVAL参数无效 / -EBUSY获取锁失败 / -EIO硬件操作失败 / -ETIMEDOUT操作超时
 */
S32 sm2DoVerifyByPolling(DevList_e devId, U32 *dataIn, U32 *signBuf, U32 *pkey, U32 timeout);

/**
 * @brief   注册SM2中断回调函数
 * @param[in] devId       控制器编号
 * @param[in] callback    中断回调函数接口
 * @param[in] callbackArg 中断回调函数参数
 * @return  EXIT_SUCCESS注册成功 / -EINVAL参数无效 / -EBUSY获取锁失败 / -EIO硬件操作失败
 */
S32 sm2CallbackRegister(DevList_e devId, sm2IrqCallBack callback, void *callbackArg);

/**
 * @brief   SM2驱动去初始化
 * @param[in] devId 控制器编号
 * @return  EXIT_SUCCESS去初始化成功 / -EINVAL参数无效 / -EIO硬件操作失败或设备未初始化 / -EBUSY获取锁失败
 */
S32 sm2DeInit(DevList_e devId);

#endif /* __DRV_SM2_API_H__ */