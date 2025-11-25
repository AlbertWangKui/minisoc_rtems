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
 * 2025/11/15   liuzhsh1        add sgpio version 1.0 for Tianhe
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
 * @brief SGPIO设备初始化函数
 *
 * 初始化指定设备的SGPIO控制器，包括设备锁定验证、驱动匹配检查、
 * 内存分配、设备配置获取、硬件复位、驱动安装、pinmux配置和硬件初始化等步骤。
 *
 * @param devId 设备ID，指定要初始化的SGPIO设备
 *
 * @return 成功返回EXIT_SUCCESS(0)，失败返回负数错误码：
 *         -EBUSY   设备忙或已初始化
 *         -EINVAL  设备不匹配或无效
 *         -ENOMEM  内存分配失败
 *         -EIO     设备配置获取失败、复位失败或驱动安装失败
 */
S32 sgpioInit(DevList_e devId);

/*
 * sgpioDeInit - SGPIO设备反初始化函数
 *
 * 此函数用于反初始化指定的SGPIO设备，包括禁用设备、注销中断服务程序、
 * 卸载驱动、复位外设和关闭时钟等操作。
 *
 * @param devId: 设备ID，指定要反初始化的SGPIO设备
 *
 * @return: 执行结果
 *          EXIT_SUCCESS - 反初始化成功
 *          -EBUSY      - 设备忙，锁定失败
 *          -EINVAL     - 设备ID不匹配SGPIO驱动
 *          -ENODEV     - 设备未初始化
 *          -EIO        - 设备驱动获取失败或驱动卸载失败
 */
S32 sgpioDeInit(DevList_e devId);

/**
 * @brief 获取SGPIO驱动计数
 *
 * @param devId 设备ID
 * @param count 用于存储驱动计数的指针
 *
 * @return 执行状态码
 *   - EXIT_SUCCESS 成功
 *   - -EBUSY 设备忙，获取锁失败
 *   - -ENODEV 设备未初始化
 *   - -EINVAL 无效参数或设备不匹配
 *   - -EIO 获取设备驱动数据失败
 */
S32 sgpioGetDriveCount(DevList_e devId, U32 *count);

/**
 * @brief 设置SGPIO驱动器的驱动器数量
 *
 * 该函数用于配置指定SGPIO设备的驱动器数量，包括设置SGPIO配置寄存器
 * 和厂商特定寄存器中的驱动器数量字段。
 *
 * @param devId 设备ID，指定要配置的SGPIO设备
 * @param count 要设置的驱动器数量，必须在有效范围内
 *              (SGPIO_DRIVER_COUNT_MIN 到 sbrCfg.driveNum)
 *
 * @return 执行状态码：
 *         - EXIT_SUCCESS: 操作成功完成
 *         - -EBUSY: 设备忙，无法获取锁
 *         - -EIO: 设备未初始化或不匹配，或获取设备驱动失败
 *         - -EINVAL: 驱动器数量参数超出有效范围
 */
S32 sgpioSetDriveCount(DevList_e devId, U32 count);

/**
 * @brief 设置SGPIO LED状态
 *
 * @param devId 设备ID
 * @param phyNum 物理编号
 * @param ledType LED类型(SGPIO_LED_ERROR/ACTIVITY/LOCATE)
 * @param ledState LED状态(SGPIO_LED_STATE_OFF/ON/BLINK)
 *
 * @return 成功返回EXIT_SUCCESS，失败返回错误码:
 *         -EBUSY: 设备忙
 *         -ENODEV: 设备未初始化
 *         -EINVAL: 参数无效
 *         -EIO: 获取驱动数据失败
 */
S32 sgpioSetLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, SgpioLedState_e ledState);

/**
 * @brief 获取SGPIO LED的状态
 *
 * @param devId 设备ID，标识要操作的SGPIO设备
 * @param phyNum 物理编号，指定要查询的驱动器编号
 * @param ledType LED类型，指定要查询的LED类型（错误/定位/活动）
 * @param pLedState 输出参数，用于返回LED状态
 *
 * @return 执行结果：
 *         - EXIT_SUCCESS: 成功
 *         - -EBUSY: 设备忙，获取锁失败
 *         - -ENODEV: 设备未初始化
 *         - -EINVAL: 参数无效或设备不匹配
 *         - -EIO: 获取设备驱动数据失败
 */
S32 sgpioGetLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, SgpioLedState_e *pLedState);

/**
 * @brief  Set sgpio general purpose driver led state directly
 *
 * 该函数用于控制SGPIO设备的LED指示灯状态，支持活动灯、定位灯和错误灯三种类型。
 * 每个物理端口对应3个LED位(活动、定位、错误)，通过寄存器位操作控制LED的开关。
 *
 * @param devId    设备ID，指定要操作的SGPIO设备
 * @param phyNum   物理端口号，范围0到驱动器数量-1
 * @param ledType  LED类型，支持SGPIO_LED_ACTIVITY(活动)、SGPIO_LED_LOCATE(定位)、SGPIO_LED_ERROR(错误)
 * @param ledOn    LED开关状态，true表示开启，false表示关闭
 *
 * @return 执行结果，EXIT_SUCCESS表示成功，负数表示错误码：
 *         -EBUSY  设备忙，无法获取锁
 *         -ENODEV 设备未初始化
 *         -EINVAL 参数无效或设备不匹配
 *         -EIO    获取设备驱动数据失败
 */
S32 sgpioSetGpLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, bool ledOn);

/**
 * 获取GP模式下指定PHY的LED状态
 *
 * @param devId    设备ID，指定要操作的SGPIO设备
 * @param phyNum   物理端口号，范围0到SGPIO_DRIVER_COUNT_MAX-1
 * @param ledType  LED类型，指定要查询的LED种类（错误/定位/活动）
 * @param pLedOn   输出参数，返回LED状态（true表示点亮，false表示熄灭）
 *
 * @return 执行结果代码：
 *         - EXIT_SUCCESS: 操作成功
 *         - -EBUSY: 设备忙，获取锁失败
 *         - -ENODEV: 设备未初始化或不存在
 *         - -EINVAL: 参数无效或设备不匹配
 *         - -EIO: 获取设备驱动数据失败
 */
S32 sgpioGetGpLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, bool *pLedOn);

/**
 * @brief 设置SGPIO活动映射关系
 *
 * 该函数用于配置SGPIO控制器中指定驱动器与PHY编号的映射关系。
 * 通过读写SGPIO_TOP_CRG_MAP_REG相关寄存器来实现映射配置。
 *
 * @param devId SGPIO设备ID，取值范围：DEVICE_SGPIO0 到 (DEVICE_SGPIO0 + SGPIO_MAX_NUM - 1)
 * @param phyNum PCIe PHY编号，取值范围：0-255
 * @param sgpioActivitySel SGPIO驱动器选择，取值范围：0到(设备驱动器数量-1, 最大63)
 *
 * @return 执行状态码
 *   - EXIT_SUCCESS: 操作成功
 *   - -EBUSY: 设备忙，获取锁失败
 *   - -ENODEV: 设备未初始化
 *   - -EINVAL: 参数无效
 *   - -EIO: 获取驱动器数量失败
 */
S32 sgpioSetPcieActivityMapping(DevList_e devId, U32 phyNum, U8 sgpioActivitySel);

/**
 * @brief 打印SGPIO控制器中PCIe活动映射表（驱动器到PHY的映射）
 * 
 * @param devId 设备ID，必须是有效的SGPIO设备ID
 * 
 * @return 执行状态：
 *         - EXIT_SUCCESS: 成功
 *         - -EBUSY: 设备忙，获取锁失败
 *         - -ENODEV: 设备未初始化
 *         - -EINVAL: 无效的设备ID
 *         - -EIO: 获取驱动器数量失败
 * 
 * @note 函数会打印出SGPIO控制器中所有驱动器到PHY的映射关系表，
 *       格式为"Driver\tPHY"，最多显示64个驱动器。
 */
S32 sgpioDumpPcieActivityMapping(DevList_e devId);

#else
 /* for feature */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SGPIO_API_H__ */
