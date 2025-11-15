/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_crg.h
 * @author
 * @date 2025/06/06
 * @brief none
 * @version v1.0
 */

#ifndef __BSP_CRG_H__
#define __BSP_CRG_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief pll初始化
 * @param [in] none
 * @param [out] none
 * @return none
 */
void pllInit(void);

/**
 * @brief 设置异常向量的映射
 * @param [in] none
 * @param [out] none
 * @return none
 */
void exceptionAddressRemap(void);

/**
 * @brief 获取外设的时钟频率
 * @param [in] devId,设备ID
 * @param [out] hz, 时钟频率，单位为赫兹
 * @return none
 */
S32 peripsClockFreqGet(DevList_e devId, U32 *Hz);

/**
 * @brief 软件配置power on复位
 * @param [in] 无
 * @return 无
 * @warning 阻塞；不可重入; OS启动前和启动后；可用于中断上下文；可以用于线程上下文
 */
void softPorReset(void);

/**
 * @brief 软件配置system复位
 * @param [in] 无
 * @return 无
 * @warning 阻塞；不可重入; OS启动前和启动后；可用于中断上下文；可以用于线程上下文
 */
void softSysReset(void);

/**
 * @brief "hotreset"外设
 * @param [in] none
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 peripsReset(DevList_e devId);

/**
 * @brief 外设保持复位状态
 * @param [in] devId,设备ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 peripsResetHold(DevList_e devId);

/**
 * @brief 外设保持复位释放
 * @param [in] devId,设备ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 peripsResetRelease(DevList_e devId);

/**
 * @brief 指定外设时钟使能
 * @param [in] devId,设备ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 peripsClockEnable(DevList_e devId);

/**
 * @brief 关闭指定外设的时钟
 * @param [in] devId,设备ID
 * @param [out] none
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
S32 peripsClockDisable(DevList_e devId);

#ifdef __cplusplus
}
#endif

#endif