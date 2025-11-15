/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_uart_api.h
 * @author taohb@starsmicrosystem.com
 * @date 2025/06/07
 * @brief uart api definitions.
 */

#ifndef __DRV_UART_API_H__
#define __DRV_UART_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common_defines.h"
#include "bsp_device.h"

typedef void (*UartTxCallback)(DevList_e devID, U32 state);
typedef void (*UartRxCallback)(DevList_e devID, U32 state, U8 *buf, U32 size);

typedef enum UartStopLength {
    UART_STOP_1 = 1,
    UART_STOP_2,
} UartStopLength_e;

typedef enum UartParityType {
    UART_PARITY_NONE = 0,
    UART_PARITY_ODD,
    UART_PARITY_EVEN,
} UartParityType_e;

typedef enum UartDataLength {
    UART_DATA_LEN_5 = 5,
    UART_DATA_LEN_6,
    UART_DATA_LEN_7,
    UART_DATA_LEN_8,
} UartDataLength_e;

typedef struct UartBasicSetting {
    U32 baud;
    UartDataLength_e dataLen;
    UartParityType_e parity;
    UartStopLength_e stopLen;
    Bool enableTxInterrupt;
    Bool enableRxInterrupt;
} UartBasicSetting_s;

/**
 * @brief 获取uart的配置
 * @param [in] devID uart号
 * @param [out] cfg uart配置
 * @return EXIT_SUCCESS 表示成功, 负数表示失败
 * @warning 阻塞；可重入; OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 uartGetCfg(DevList_e devID, UartBasicSetting_s *cfg);

/**
 * @brief 获取uart的配置
 * @param [in] devID uart号
 * @param [out] cfg uart配置
 * @return EXIT_SUCCESS 表示成功, 负数表示失败
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 uartSetCfg(DevList_e devID, UartBasicSetting_s *cfg);

/**
 * @brief 向指定uart发送指定大小的数据
 * @param [in] devID uart号
 * @param [in] buf 发送缓冲区
 * @param [in] size 发送字节数
 * @param [in] timeout 超时时间，单位为ms, 如果该值为负数，表示死等
 * @return EXIT_SUCCESS 表示成功, 负数表示失败
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 uartSend(DevList_e devID, U8 *buf, U32 size, S32 timeout);

/**
 * @brief 以查询方式（不可用于中断方式），从uart接受数据
 * @param [in] devID uart号
 * @param [in] buf 接收缓冲区
 * @param [in] size 预期接收字节数
 * @param [in] timeout 超时时间，单位为ms, 如果该值为负数，表示死等
 * @return 实际接收的字节数
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 uartReceive(DevList_e devID, U8 *buf, U32 size, S32 timeout);

/**
 * @brief 注册发送完成回调函数
 * @param [in] devID uart号
 * @param [in] txCallback 发送完成回调函数，该回调函数工作于中断上下文
 * @return EXIT_SUCCESS 表示成功, 负数表示失败
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 uartRegisterTxCallback(DevList_e devID,
    UartTxCallback txCallback);

/**
 * @brief 注册接收回调函数
 * @param [in] devID uart号
 * @param [in] rxCallback 接收回调函数，该回调函数工作于中断上下文
 * @return EXIT_SUCCESS 表示成功, 负数表示失败
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 uartRegisterRxCallback(DevList_e devID,
    UartRxCallback rxCallback);

/**
 * @brief uart初始化函数
 * @param [in] uartID：uart号
 * @return EXIT_SUCCESS is success, negative return is failure
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 uartInit(DevList_e devID);

/**
 * @brief uart deinit
 * @param [in] uartID：uart号
 * @return EXIT_SUCCESS 表示成功, 负数表示失败
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 uartDeinit(DevList_e devID);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_API_H__ */
