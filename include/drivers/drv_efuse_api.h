#ifndef __DRV_EFUSE_API_H__
#define __DRV_EFUSE_API_H__

#include "common_defines.h"
#include "bsp_device.h"

///< ELD设备枚举
typedef enum EfuseEldNum {
    EFUSE_ELD_0 = 0,
    EFUSE_ELD_1,
    EFUSE_ELD_2,
    EFUSE_ELD_3,
    EFUSE_ELD_4,
    EFUSE_ELD_5,
    EFUSE_ELD_6,
    EFUSE_ELD_7,
    EFUSE_ELD_8,
    EFUSE_ELD_9,
    EFUSE_ELD_10,
    EFUSE_ELD_11,
    EFUSE_ELD_12,
    EFUSE_ELD_13,
    EFUSE_ELD_14,
    EFUSE_ELD_15,
    EFUSE_ELD_16,
} EfuseEldNum_e;

/**
 * @brief    初始化软件资源和复位硬件
 * @param    [in] devId: efuse device ID
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 efuseInit(DevList_e devId);

/**
 * @brief    反初始化
 * @param    devId: efuse device ID
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 efuseDeInit(DevList_e devId);

/**
 * @brief    efuse读操作
 * @param    [in] eldNum eld号
 * @param    [in] offset 偏移
 * @param    [in] len 数据长度 有多少个32bit的数据。
 * @param    [out] pDest 读到的数据buf
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 * @warning 非阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 * @note     原流程，eld2的bit127被置1后，不只eld写会失败，eld读也会失败，
 *           与硬件核对操作流程后删除了不需要opcode、start等操作（PR2517）
 */
S32 efuseEldGet(DevList_e devId,EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pDest);

/**
 * @brief    efuse写操作
 * @param    [in] eldNum eld号
 * @param    [in] index 偏移
 * @param    [in] len 数据长度
 * @param    [in] pSrc 写入的数据
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 efuseEldSet(DevList_e devId,EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pSrc);

 /**
 * @brief    sm2 key 烧录
 * @param    [in] devId efuse id
 * @param    [in] buf 待写入缓冲区
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 efuseWriteSm2Key(DevList_e devId, U8 *buf);

 /**
 * @brief    sm4 key 烧录
 * @param    [in] devId efuse id
 * @param    [in] buf 待写入缓冲区
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 efuseWriteSm4Key(DevList_e devId, U8 *buf);

#endif
