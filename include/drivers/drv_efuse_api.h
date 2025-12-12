#ifndef __DRV_EFUSE_API_H__
#define __DRV_EFUSE_API_H__

#include "common_defines.h"
#include "bsp_device.h"

///< ELD设备枚举
#if defined(CONFIG_DRIVER_EFUSE_V1)
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
#elif defined(CONFIG_DRIVER_EFUSE_V2)
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
    EFUSE_ELD_17,
    EFUSE_ELD_18,
    EFUSE_ELD_19,
    EFUSE_ELD_20,
    EFUSE_ELD_21,
    EFUSE_ELD_22,
    EFUSE_ELD_23,
    EFUSE_ELD_24,
    EFUSE_ELD_25,
    EFUSE_ELD_26,
    EFUSE_ELD_27,
} EfuseEldNum_e;
#endif

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
 * @param    [in] devId efuse device ID
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
S32 efuseEldRead(DevList_e devId, EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pDest);

/**
 * @brief    efuse写操作
 * @param    [in] devId efuse device ID
 * @param    [in] eldNum eld号
 * @param    [in] offset 偏移
 * @param    [in] len 数据长度
 * @param    [in] pSrc 写入的数据
 * @return   EXIT_SUCCESS 成功
 *           -EXIT_FAILURE 失败
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 efuseEldProgram(DevList_e devId, EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pSrc);

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

#if defined(CONFIG_USING_EFUSE_V2)

/**
 * @brief    efuse clear操作
 * @param    [in] devId: efuse device ID
 * @param    [in] eldNum:eld号
 * @return   EXIT_SUCCESS 成功
 * @return   -EINVAL        参数错误（eldNum非法）
 * @return   -EBUSY         资源忙
 * @return   -EIO           IO错误（命令启动或状态检测失败）
 * @note     ELD0，ELD26，ELD27不对minisoc开放读写操作
 */
S32 efuseEldClear(DevList_e devId, EfuseEldNum_e eldNum);

/**
 * @brief    efuse sense读操作
 * @param    [in] devId: efuse device ID
 * @param    [in] eldNum:eld号
 * @param    [in] offset:偏移量
 * @param    [in] len:数据长度 有多少个32bit的数据。
 * @param    [out] pDest:读到的数据buf
 * @return   EXIT_SUCCESS 成功
 * @return   -EINVAL        参数错误（pDest为NULL或eldNum非法）
 * @return   -EBUSY         资源忙
 * @return   -EIO           设备/IO错误（命令启动或状态检测失败）
 * @note     ELD0，ELD26，ELD27不对minisoc开放读写操作
 */
S32 efuseEldSenseRead(DevList_e devId, EfuseEldNum_e eldNum, U32 offset, U32 len, U32 *pDest);

/**
 * @brief    efuse verify操作
 * @param    [in] devId: efuse device ID
 * @param    [in] eldNum: eld号
 * @return   EXIT_SUCCESS 成功
 * @return   -EINVAL        参数错误（eldNum非法）
 * @return   -EBUSY         资源忙
 * @return   -EIO           IO错误（命令启动或状态检测失败）
 * @note     ELD0，ELD26，ELD27不对minisoc开放读写操作
 */
S32 efuseEldVerify(DevList_e devId, EfuseEldNum_e eldNum);
#endif  ///< CONDIF_USING_EFUSE_V2
#endif
