/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sgpio_api.h
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/06/05
 * @brief
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/06/05   yangzhl3        the first version
 *
 *
 */

#ifndef __DRV_SGPIO_API_H__
#define __DRV_SGPIO_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common_defines.h"
#include "bsp_device.h"

#ifdef USING_SGPIO_V2_0 // sheshou
typedef enum {
    SGPIO_MODE_NORMAL = 0,
    SGPIO_MODE_GP,
} SgpioMode_e;

typedef enum {
    SGPIO_LINK_DOWN_S0 = 0,     /* off */
    SGPIO_LINK_INIT_S1,         /* yellow on */
    SGPIO_LINK_ACT_IDLE_S2,     /* green on */
    SGPIO_LINK_ACT_X8_S3,       /* green blink in freq C 2hz */
    SGPIO_LINK_ACT_X4_S4,       /* green blink in freq B 1hz */
    SGPIO_LINK_ACT_OTHER_S5,    /* green blink in freq A 0.5hz */
    SGPIO_PORT_ERR_S6,          /* yellow blink in freq C 2hz */
    SGPIO_LOCATE_S7,            /* yellow blink in freq A 0.5hz */
    SGPIO_HARD_MODE_S8,         /* 软硬结合模式 */
    SGPIO_STATE_MAX,
} SgpioState_e;

/**
 @note portNum[in]

param portNum[in] with corresponding drive and link in functions blow:

    sgpio_get_led
    sgpio_get_led_hw
    sgpio_set_led

    >> relationship on SheShou
    =======================================================================

    | portNum | sgpio drive | link      | ib portNum | note             |
    | ------- | ----------- | --------- | ---------- | ---------------- |
    | 0       | all(0~95)   | -         | -          | config all drive |
    | 1       | 0           | link0 x8  | 1          |
    | 2       | 1           | link0 x4  | 2          |
    | 3       | 2           | link1 x8  | 3          |
    | 4       | 3           | link1 x4  | 4          |
    | ...     | ...         | ...       | ...        |
    | 79      | 78          | link38 x8 | 79         |
    | 80      | 79          | link38 x4 | 80         |
    | 81      | 80          | vhca      | 0          | SheShou CS only  |
    | 82 ~ 96 | 81 ~ 95     | -         | -          | reserved         |

    >> relationship on TianQin
    =======================================================================
    TODO:

    >> relationship on TianHe
    =======================================================================
    TODO:

 *
 */

/**
 * @brief sgpio init
 * @param [in] device id
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioInit(DevList_e devId);

/**
 * @brief sgpio deinit
 * @param [in] device id
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioDeInit(DevList_e devId);

/**
 * @brief  Set sgpio led state
 * @param [in] id sgpio controller id
 *              should asigned to 0 when only 1 controller instance
 * @param [in] portNum see @note portNum[in]
 * @param [in] state
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioSetLedState(DevList_e devId, U8 portNum, SgpioState_e state);

/**
 * @brief Get sgpio drive count
 * @param [in] device id
 * @param [out] count, drive count
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioGetDriveCount(DevList_e devId, U32 *count);

/**
 * @brief Set sgpio drive count
 * @param [in] device id
 * @param [in] count, drive count
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioSetDriveCount(DevList_e devId, U32 count);

/**
 * @brief  Get sgpio led state
 * @details none
 * @param [in] devId sgpio controller id
 * @param [in|out] portNum see @note portNum[in]
 * @param [in|out] pLink current led state
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioGetLedState(DevList_e devId, U8 portNum, SgpioState_e *pLink);

/**
 * @brief  Get sgpio led state(event input) from hardware
 *          Hardware event input form link
 * @details none
 * @param [in] id
 * @param [in] drv_id
 * @param [out] pLink
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioGetLedLinkState(DevList_e dev_id, U8 portNum, U8 *pLinkState);
#elif defined(CONFIG_USING_SGPIO_V1_0)
/**
 * SGPIO与PHY指示灯映射关系示意图：
 * 
 * ┌─────────────────────────────────────────────────────────┐
 * │              PHY 集合 (最多288个) 4x72                    │
 * └─────────────────────────────────────────────────────────┘
 *                              │
 *                ┌─────────────┼─────────────┐
 *                │             │             │
 *        ┌───────▼───────┐ ┌──▼───┐ ┌───────▼───────┐
 *        │  mux 0        │ | ...  │ │      mux 3    │
 *        │  Select * 64  │ │      │ │  Select * 64  │
 *        └───────┬───────┘ └──────┘ └───────┬───────┘
 *                │ 72-64 max                │ 72-64 max
 *                │ (最多64,最少4)            │ (最多64,最少4)
 *        ┌───────▼───────┐         ┌───────▼───────┐
 *        │   SGPIO_0     │         │   SGPIO_3     │
 *        │  64 drivers   │   ...   │  64 drivers   │
 *        └───────┬───────┘         └───────┬───────┘
 *                │                         │
 *        ┌───────▼───────┐         ┌───────▼───────┐
 *        │ 指示灯控制      │         │ 指示灯控制     │
 *        │  Activity ◉   │         │  Activity ◉   │
 *        │  Locate   ◉   │         │  Locate   ◉   │
 *        │  Error    ◉   │         │  Error    ◉   │
 *        └───────────────┘         └───────────────┘
 * 
 * 系统架构说明：
 *   1. 系统包含4组SGPIO控制器(SGPIO_0 ~ SGPIO_3)，每个SGPIO组可提供4-64个驱动(driver)
 *   2. 用于控制PHY的指示灯(Activity/Locate/Error)。
 * 映射流程：
 *   1. 从PHY集合中选择4-64个PHY进行映射（通过pinmux配置)
 *   2. 每个SGPIO组最多驱动64个PHY，最少驱动4个PHY（通过软件配置）
 *   3. 通过pinmux配置具体的映射关系
 * 
 * 指示灯控制方式：
 *   ◉ Activity: 硬件自动闪烁(信号：nvme_activity) + 软件可配置频率
 *   ◉ Locate:   完全由软件控制
 *   ◉ Error:    完全由软件控制
 * 
 * 规格总结：
 *   +───────────────┬──────────────┬──────────────┐
 *   │  SGPIO组数量   │  驱动范围     │   总计PHY数   │
 *   +───────────────┼──────────────┼──────────────┤
 *   │       4       │    4-64      │   16-256     │
 *   +───────────────┴──────────────┴──────────────┘
 */

/**
 * @brief LED类型定义
 */
typedef enum {
    SGPIO_LED_ERROR = 0,    ///< Error LED
    SGPIO_LED_LOCATE,       ///< Locate LED
    SGPIO_LED_ACTIVITY,      ///< Activity LED
    SGPIO_LED_MAX
} SgpioLedType_e;

/**
 * @brief LED状态定义
 */
typedef enum {
    SGPIO_LED_OFF = 0,           ///< 关闭
    SGPIO_LED_ON,                ///< 常亮
    SGPIO_LED_BLINK_A_HIGH_LOW,  ///< Pattern A闪烁，先高后低
    SGPIO_LED_BLINK_A_LOW_HIGH,  ///< Pattern A闪烁，先低后高
    SGPIO_LED_BLINK_B_HIGH_LOW,  ///< Pattern B闪烁，先高后低
    SGPIO_LED_BLINK_B_LOW_HIGH,  ///< Pattern B闪烁，先低后高
    SGPIO_LED_EVENT_RISING,      ///< 上升沿触发(仅Activity LED)
    SGPIO_LED_EVENT_FALLING,     ///< 下降沿触发(仅Activity LED)
    SGPIO_LED_STATE_MAX
} SgpioLedState_e;

/**
 * @brief sgpio init
 * @param [in] device id
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioInit(DevList_e devId);

/**
 * @brief sgpio deinit
 * @param [in] device id
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioDeInit(DevList_e devId);

/**
 * @brief Get sgpio drive count
 * @param [in] device id
 * @param [out] count, drive count
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioGetDriveCount(DevList_e devId, U32 *count);

/**
 * @brief Set sgpio drive count
 * @param [in] device id
 * @param [in] count, drive count
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioSetDriveCount(DevList_e devId, U32 count);

/**
 * @brief  Set sgpio driver led state directly
 * @param [in] devId sgpio controller id
 * @param [in] phyNum PHY编号
 * @param [in] ledType LED类型 (Error/Locate/Activity)
 * @param [in] ledState LED状态
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 *          -EINVAL : -22
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioSetLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, SgpioLedState_e ledState);

/**
 * @brief  Get sgpio driver led state directly
 * @param [in] devId sgpio controller id
 * @param [in] phyNum PHY编号
 * @param [in] ledType LED类型 (Error/Locate/Activity)
 * @param [out] pLedState LED状态指针
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 *          -EINVAL : -22
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 sgpioGetLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, SgpioLedState_e *pLedState);

/**
 * @brief  Set sgpio general purpose driver led state directly
 * @details This function operates on txgp registers for General Purpose mode.
 *          Each PHY is controlled by 3 bits for error, locate, and activity.
 *          It handles cases where a PHY's 3 bits span across two registers.
 * @param [in] devId sgpio controller id
 * @param [in] phyNum PHY编号
 * @param [in] ledType LED类型 (SGPIO_LED_ERROR/SGPIO_LED_LOCATE/SGPIO_LED_ACTIVITY)
 * @param [in] ledOn  true: turn on, false: turn off
 * @return  EXIT_SUCCESS : 0
 *          -EBUSY : -16
 *          -EINVAL : -22
 */
S32 sgpioSetGpLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, bool ledOn);

/**
 * @brief 获取GP模式下指定PHY的LED状态
 * @param [in] devId sgpio控制器ID
 * @param [in] phyNum PHY编号
 * @param [in] ledType LED类型 (SGPIO_LED_ERROR/SGPIO_LED_LOCATE/SGPIO_LED_ACTIVITY)
 * @param [out] pLedOn 指向bool变量，返回true表示点亮，false表示熄灭
 * @return EXIT_SUCCESS: 0
 *         -EBUSY: -16
 *         -EINVAL: -22
 */
S32 sgpioGetGpLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, bool *pLedOn);

/**
* @brief 设置PCIe活动映射到SGPIO控制器
* 此函数用于将PCIe活动映射到指定的SGPIO控制器上的某个driver。
* @param devId SGPIO控制器ID，枚举类型DevList_e
* @param phyNum PCIe PHY编号
* @param sgpioActivitySel 要映射的SGPIO driver编号
* @return 函数执行结果，成功返回EXIT_SUCCESS，失败返回错误码
*         - EINVAL：无效的参数
*         - EIO：获取SGPIO驱动数量失败
*/
S32 sgpioSetPcieActivityMapping(DevList_e devId, U32 phyNum, U8 sgpioActivitySel);

/**
* @brief 导出SGPIO控制器的PCIe活动映射表
* 该函数将导出SGPIO控制器的PCIe活动映射表，显示每个驱动器（Driver）对应的PHY编号。
* @param devId 设备ID，表示要操作的SGPIO控制器。
* @return 返回值类型为S32，成功时返回EXIT_SUCCESS，失败时返回相应的错误码。
*/
S32 sgpioDumpPcieActivityMapping(DevList_e devId);

#else
 /* for feature */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SGPIO_API_H__ */
