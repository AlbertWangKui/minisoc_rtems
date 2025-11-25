/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_tach.h
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025
 * @brief  tach driver internal definitions
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2024         tach_driver     Created initial version
 */

#ifndef __DRV_TACH_H__
#define __DRV_TACH_H__

#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

/* TACH register offset definitions */
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
#define TACH_POLL_INTERVAL_MS            (10)  ///< Polling interval in milliseconds

typedef enum TachInputFilter {
    TACH_NO_FILTER = 0,
    TACH_2_CLK_FILTER,
    TACH_4_CLK_FILTER,
    TACH_8_CLK_FILTER,
    TACH_FILTER_MAX,
} TachInputFilter_e;

typedef union TachCtrlReg {
    struct __attribute__((aligned(4))){
        U32 capEn : 1;
        U32 rsvd2 : 3;
        U32 polaritySel : 1;
        U32 rsvd1 : 3;
        U32 filterSel : 2;
        U32 speedMeasureSel : 1;
        U32 rsvd0 : 21;
    } bit;
    U32 all;
} TachCtrlReg_u;

typedef union TachStateReg {
    struct __attribute__((aligned(4))) {
        U32 tachCnt : 16;
        U32 capOver : 1;
        U32 rsvd : 15;
    } bit;
    U32 all;
} TachStateReg_u;

typedef union TachIntStaReg {
    struct __attribute__((aligned(4))) {
        U32 tachIrq : 1;
        U32 rsvd : 31;
    } bit;
    U32 all;
} TachIntStaReg_u;

typedef union TachIntRawReg {
    struct __attribute__((aligned(4))) {
        U32 tachIrqRaw : 1;
        U32 rsvd : 31;
    } bit;
    U32 all;
} TachIntRawReg_u;

typedef union TachIntEnReg {
    struct __attribute__((aligned(4))) {
        U32 tachIrqEn : 1;
        U32 rsvd : 31;
    } bit;
    U32 all;
} TachIntEnReg_u;

typedef union TachIntInjReg {
    struct __attribute__((aligned(4))) {
        U32 tachIrqInj : 1;
        U32 rsvd : 31;
    } bit;
    U32 all;
} TachIntInjReg_u;

typedef struct {
    volatile TachCtrlReg_u       ctrl;      ///< 0x00
    volatile U32                 timeCtrl;  ///< 0x04
    volatile TachStateReg_u      state;     ///< 0x08
    volatile TachIntStaReg_u     intStat;   ///< 0x0C
    volatile TachIntRawReg_u     intRaw;    ///< 0x10
    volatile TachIntEnReg_u      intEn;     ///< 0x14
    volatile TachIntInjReg_u     intInj;    ///< 0x18
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