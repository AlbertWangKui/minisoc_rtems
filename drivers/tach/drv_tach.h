/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co.,Ltd
 * @file    drv_tach.h
 * @author  yangkl@starsmicrosystem.com
 * @date    2025/01/01
 * @brief   Tach driver internal definitions
 */

#ifndef __DRV_TACH_H__
#define __DRV_TACH_H__

#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

#define TACH_CTRL_REG      0x00
#define TACH_TIME_CTRL_REG 0x04
#define TACH_STATE_REG     0x08
#define TACH_INT_STA_REG   0x0C
#define TACH_INT_RAW_REG   0x10
#define TACH_INT_EN_REG    0x14
#define TACH_INT_INJ_REG   0x18

#define TACH_DEFAULT_MEASURE_TIME_S      (1)          ///< Default measurement time in seconds
#define TACH_MAX_TIMEOUT_REG_VALUE       (0xFFFFFFFF) ///< Maximum timeout value for capture width
#define TACH_MAX_TIMEOUT_VALUE_S         (10)         ///< Maximum timeout value for capture width
#define TACH_LOCK_TIMEOUT_MS             (1000)       ///< Default timeout value for capture width
#define TACH_POLL_INTERVAL_MS            (10)         ///< Polling interval in milliseconds

typedef enum TachInputFilter {
    TACH_NO_FILTER = 0,      ///< No filter
    TACH_2_CLK_FILTER,       ///< 2 clock filter
    TACH_4_CLK_FILTER,       ///< 4 clock filter
    TACH_8_CLK_FILTER,       ///< 8 clock filter
    TACH_FILTER_MAX,         ///< Maximum filter value
} TachInputFilter_e;

typedef union TachCtrlReg {
    struct __attribute__((aligned(4))) {
        U32 capEn : 1;           ///< Capture enable
        U32 rsvd2 : 3;           ///< Reserved
        U32 polaritySel : 1;     ///< Polarity selection
        U32 rsvd1 : 3;           ///< Reserved
        U32 filterSel : 2;       ///< Filter selection
        U32 speedMeasureSel : 1; ///< Speed measure selection
        U32 rsvd0 : 21;          ///< Reserved
    } bit;
    U32 all;                     ///< All bits
} TachCtrlReg_u;

typedef union TachStateReg {
    struct __attribute__((aligned(4))) {
        U32 tachCnt : 16;        ///< Tachometer count
        U32 capOver : 1;         ///< Capture over flag
        U32 rsvd : 15;           ///< Reserved
    } bit;
    U32 all;                     ///< All bits
} TachStateReg_u;

typedef union TachIntStaReg {
    struct __attribute__((aligned(4))) {
        U32 tachIrq : 1;         ///< Tachometer interrupt status
        U32 rsvd : 31;          ///< Reserved
    } bit;
    U32 all;                     ///< All bits
} TachIntStaReg_u;

typedef union TachIntRawReg {
    struct __attribute__((aligned(4))) {
        U32 tachIrqRaw : 1;      ///< Tachometer interrupt raw
        U32 rsvd : 31;          ///< Reserved
    } bit;
    U32 all;                     ///< All bits
} TachIntRawReg_u;

typedef union TachIntEnReg {
    struct __attribute__((aligned(4))) {
        U32 tachIrqEn : 1;       ///< Tachometer interrupt enable
        U32 rsvd : 31;          ///< Reserved
    } bit;
    U32 all;                     ///< All bits
} TachIntEnReg_u;

typedef union TachIntInjReg {
    struct __attribute__((aligned(4))) {
        U32 tachIrqInj : 1;      ///< Tachometer interrupt inject
        U32 rsvd : 31;          ///< Reserved
    } bit;
    U32 all;                     ///< All bits
} TachIntInjReg_u;

typedef struct {
    volatile TachCtrlReg_u       ctrl;      ///< Control register
    volatile U32                 timeCtrl;  ///< Time control register
    volatile TachStateReg_u      state;     ///< State register
    volatile TachIntStaReg_u     intStat;   ///< Interrupt status register
    volatile TachIntRawReg_u     intRaw;    ///< Interrupt raw register
    volatile TachIntEnReg_u      intEn;     ///< Interrupt enable register
    volatile TachIntInjReg_u     intInj;    ///< Interrupt inject register
} __attribute__((packed, aligned(4))) TachReg_s;

typedef void (*tachIrqCallBack)(void *arg);

typedef struct TachDrvData {
    SbrTachCfg_s      sbrCfg;      ///< SBR configuration from hardware
    tachIrqCallBack   callback;    ///< Interrupt callback function pointer
    void             *arg;         ///< User argument for callback function
    U32               clk;         ///< Clock frequency in Hz
    U32               busy;        ///< Flag to indicate if device is busy
} TachDrvData_s;

#endif /* __DRV_TACH_H__ */
