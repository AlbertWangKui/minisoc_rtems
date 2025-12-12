/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_gpio.h
 * @author zhaorui (zhaorui@starsmicrosystem.com)
 * @date 2025/11/7
 * @brief  gpio driver header
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/10/22   zhaorui         the first version
 *
 */

#ifndef __DRV_GPIO_H__
#define __DRV_GPIO_H__

#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_sbr.h"
#include "sbr_api.h"
#include "drv_gpio_api.h"

#define GPIO_MAX_BIT_INDEX_REG 31
#define GPIO_PIN_COUNT  32
#define GPIO_LOCK_TIMEOUT_MS 1000

typedef volatile struct {
    volatile U32 dataOut;            ///< 0x00: 0-output 0, 1-output 1
    volatile U32 dir;                ///< 0x04: 0-input, 1-output
    volatile U32 rsvd08[10];         ///< 0x08 ~ 0x2c reserved
    volatile U32 irqEn;              ///< 0x30: 1-en, 0-dis
    volatile U32 irqMask;            ///< 0x34: 0-unmask, 1-mask
    volatile U32 irqType;            ///< 0x38: 0-level, 1-edge
    volatile U32 irqPol;             ///< 0x3C: 0-low/neg, 1-high/pos
    volatile U32 irqStat;             ///< 0x40: edge int need to clear
    volatile U32 irqRaw;              ///< 0x44: unmasked int_stat
    volatile U32 debounce;            ///< 0x48: debounce
    volatile U32 eoi;                 ///< 0x4C: write 1 to clear int_stat
    volatile U32 dataIn;              ///< 0x50: input data
    volatile U32 rsvd54[3];           ///< 0x54~0x5c reserved
    volatile U32 levelIntSync;        ///< 0x60: level int sync to system int
    volatile U32 rsvd64;              ///< 0x64 reserved
    volatile U32 intBothedge;         ///< 0x68: 0-single, 1-both
} GpioReg_s;

typedef struct {
    gpioIrqCallBack cb;              ///< 中断回调函数指针
    void *arg;                        ///< 回调函数参数
} GpioIrqCbItem_s;

typedef struct GpioDrvData {
    SbrGpioCfg_s sbrCfg;              ///< SBR配置信息
    U8 isInit;                        ///< 初始化标志
    DevList_e devId;                  ///< 设备ID
    GpioIrqCbItem_s irqCbList[32];    ///< 中断回调列表
} GpioDrvData_s;

#endif /* __DRV_GPIO_H__ */