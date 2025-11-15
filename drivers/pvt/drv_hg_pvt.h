/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_hg_pvt.h
 * @author taohb@starsmicrosystem.com
 * @date 2024.08.27
 * @brief hg pvt driver.
 */

#ifndef __DRV_HG_PVT_H__
#define __DRV_HG_PVT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common_defines.h"
#include "drv_hg_pvt_api.h"

#include "bsp_sbr.h"
#include "sbr_api.h"

#define DTS_TEMPERATURE_THRESHOLD_MAX (0x7fff)

typedef union TmonTempValueReg {
    volatile struct {
        U32 expectTempValue : 11; /* [10:0]: expected temperature value */
        U32 valid : 1; /* [11]: valid, 1 is valid, 0 is invalid */
        U32 realTempValue : 12; /* [23:12]: real temperature value */
        U32 : 8; /* [31:24]: Reserved */
    } fields;
    volatile U32 dword;
} TmonTempValueReg_u;

typedef union TmonChSoftIrqReg {
    volatile struct {
        U32 tmon : 1; /* [0]: tmon soft irq */
        U32 dts : 1; /* [1]: dts soft irq */
        U32 cntrsram : 1; /* [2]: cntrsram soft irq */
        U32 tsram : 1; /* [3]: tsram soft irq */
        U32 : 28; /* [31:4]: Reserved */
    } fields;
    volatile U32 dword;
} TmonChSoftIrqReg_u;

typedef union DtsChSoftIrqReg {
    volatile struct {
        U32 dts : 1; /* [0]: dts soft irq */
        U32 : 5; /* [5:1]: Reserved */
        U32 tmonThreshold : 8; /* [13:6]: tmon threshold */
        U32 : 2; /* [15:14]: Reserved */
        U32 tmonThresholdEn : 2; /* [17:16]: 10: enable dts modify, 00: disable dts modify */
        U32 : 14; /* [31:18]: Reserved */
    } fields;
    volatile U32 dword;
} DtsChSoftIrqReg_u;

typedef union ChSoftIrqReg {
    TmonChSoftIrqReg_u tmon;
    DtsChSoftIrqReg_u dts;
} ChSoftIrqReg_u;

typedef union TmonThresholdReg {
    volatile struct {
        U32 : 6; /* [5:0]: Reserved */
        U32 tmonThreshold : 8; /* [13:6]: tmon threshold */
        U32 : 18; /* [31:14]: Reserved */
    } fields;
    volatile U32 dword;
} TmonThresholdReg_u;

typedef union dtsThresholdReg {
    volatile struct {
        U32 thremTripLimit : 16; /* [15:0]: ThermTripLimit */
        U32 : 16; /* [31:16]: Reserved */
    } fields;
    volatile U32 dword;
} DtsThresholdReg_u;

typedef volatile struct HgPvtchSoftIrqReg {
    /* 0x00: Softirq Status Register */
    ChSoftIrqReg_u irqStatusReg;
    /* 0x04: Softirq Mask Register */
    ChSoftIrqReg_u irqMaskReg;
    /* 0x08: Softirq Control Register */
    ChSoftIrqReg_u irqCtrlReg;
    /* 0x0C: Softirq Clear Register */
    ChSoftIrqReg_u irqClearReg;
} HgPvtChSoftIrqReg_u;

typedef struct HgPvtTmonChipInfo {
    Bool channelExist;
    U32 testPoints;
    U32 defalutTempTh;
} HgPvtTmonChipInfo_s;

typedef struct HgPvtDtsChipInfo {
    Bool channelExist;
    U32 tempTestPoints;
    U32 volTestPoints;
    U32 configLatency;
    U32 defaultTempTh;
} HgPvtDtsChipInfo_s;

typedef struct PvtDrvData {
    SbrPvtChCfg_s sbrCfg;             /* SBR配置 */
    U32 channel;                      /* 通道号 */
    HgPvtTmonChipInfo_s tmonInfo;     /* TMON信息 */
    HgPvtDtsChipInfo_s dtsInfo;       /* DTS信息 */
    dtsTempAlarmIsrCb dtsTempAlarmIsrCb; /* DTS温度报警回调 */
} PvtDrvData_s;

/**
 * @brief set tmon temperature threshold.
 * @param [in] channel, channel id.
 * @param [in] thresholdRaw, unit is Centigrade.
 * @return EXIT_FAILURE or EXIT_SUCCESS
 */
S32 tmonThresholdSet(U32 channel, U8 thresholdRaw);

/**
 * @brief set dts temperature threshold.
 * @param [in] channel, channel id.
 * @param [in] tempThreshold, unit is Centigrade.
 * @return EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsTempThresholdSet(U32 channel, S16 tempThreshold);

/**
 * @brief register dts temperature alarm callback which is in isr.
 * @param [in] channel, channel id.
 * @param [in] isrCb, isr callback.
 * @return EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsTemAlarmCallbackRegister(U32 channel,
    dtsTempAlarmIsrCb isrCb);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_HG_PVT_H__ */
