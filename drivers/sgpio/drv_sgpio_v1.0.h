/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sgpio_v1.0.h
 * @author liuzhsh1 (liuzhsh1@starsmicrosystem.com)
 * @date 2025/09/22
 * @brief SGPIO (Serial GPIO) 驱动头文件，适用于 v1.0 硬件
 *        定义 SGPIO 控制器的寄存器结构、常量和数据类型
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/09/22   liuzhsh1        the first version
 *
 *
 */

#ifndef __DRV_SGPIO_V1P0_H__
#define __DRV_SGPIO_V1P0_H__

#include "common_defines.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

/*
 * +----------------+---------+---------+
 * | Address        | Size    | Module  |
 * +----------------+---------+---------+
 * | 0xB806_0000    | 4KB*4   | SGPIO0  |
 * | 0xB806_4000    | 4KB*4   | SGPIO1  |
 * | 0xB806_8000    | 4KB*4   | SGPIO2  |
 * | 0xB806_C000    | 4KB*4   | SGPIO3  |
 * +----------------+---------+---------+
 *
 * +----------------+------------------+
 * | Address Range  | Description      |
 * +----------------+------------------+
 * | 0x0000–0x1FFF  | I2C              |
 * | 0x2000–0x2FFF  | 8485             |
 * | 0x3400–0x37FF  | 1005             |
 * | 0x3800–0x3FFF  | read-only        |
 * +----------------+------------------+
 */

#define SGPIO_MAX_NUM (4) /* 最大支持4个SGPIO控制器 */

#define SGPIO_BASE_ADDR  0xb8060000
#define SGPIO_DEVICE_OFFSET 0x4000
#define SGPIO_8485_REG_OFFSET 0x2000
#define SGPIO_1005_REG_OFFSET 0x3400

#define SGPIO_TOP_CRG_REG_BASE    (0xb8000000UL)
/* SGPIO to PHY Mapping Registers */
#define SGPIO_TOP_CRG_MAP_REG_ADDR      (SGPIO_TOP_CRG_REG_BASE + 0xc700)
#define SGPIO_TOP_CRG_MAP_BLOCK_SIZE    0x40

#define SGPIO_DRIVER_COUNT_MIN    (4)
#define SGPIO_DRIVER_COUNT_MAX    (64)

#define SGPIO_PINMUX_BASE 0xB8040000UL
#define SGPIO_TOP_CRG_REG_OFFSET 0xC680UL

/**
 * 配置最高频率为100KHZ, 最小频率为32HZ, 频率范围为[32HZ,100KHZ]
 */
#define SGPIO_MAX_CLK 100000
#define SGPIO_MIN_CLK 32

#define SGPIO_IRQ_NUM (385) /* SGPIO irq number SGPIO0 385, SGPIO1 386, SGPIO2 387, SGPIO3 388 */
#define SGPIO_IRQ_PRI (127) /* SGPIO irq priority */

#define SGPIO_START_INT 0x01
#define SGPIO_STOP_INT 0x02
#define SGPIO_DRV_ERR_INT 0x04

#define SGPIO_START_INT_CLR 4
#define SGPIO_STOP_INT_CLR 5
#define SGPIO_DRV_ERR_INT_CLR 6

/* SGPIO cfg1Timing register bit field definitions */
#define SGPIO_CFG1_STRETCH_VALUE_MAX 0x0F
#define SGPIO_CFG1_PATTERN_VALUE_MAX  0x0F
#define SGPIO_CFG1_STRETCH_ACTIVITY_OFF_SHIFT   28
#define SGPIO_CFG1_STRETCH_ACTIVITY_OFF_MASK    (0xF << SGPIO_CFG1_STRETCH_ACTIVITY_OFF_SHIFT)
#define SGPIO_CFG1_STRETCH_ACTIVITY_ON_SHIFT    24
#define SGPIO_CFG1_STRETCH_ACTIVITY_ON_MASK     (0xF << SGPIO_CFG1_STRETCH_ACTIVITY_ON_SHIFT)
#define SGPIO_CFG1_FORCE_ACTIVITY_OFF_SHIFT     20
#define SGPIO_CFG1_FORCE_ACTIVITY_OFF_MASK      (0xF << SGPIO_CFG1_FORCE_ACTIVITY_OFF_SHIFT)
#define SGPIO_CFG1_FORCE_ACTIVITY_ON_SHIFT      16
#define SGPIO_CFG1_FORCE_ACTIVITY_ON_MASK       (0xF << SGPIO_CFG1_FORCE_ACTIVITY_ON_SHIFT)
#define SGPIO_CFG1_PATTERN_B_BLINK_FREQ_SHIFT   12
#define SGPIO_CFG1_PATTERN_B_BLINK_FREQ_MASK    (0xF << SGPIO_CFG1_PATTERN_B_BLINK_FREQ_SHIFT)
#define SGPIO_CFG1_PATTERN_A_BLINK_FREQ_SHIFT   8
#define SGPIO_CFG1_PATTERN_A_BLINK_FREQ_MASK    (0xF << SGPIO_CFG1_PATTERN_A_BLINK_FREQ_SHIFT)

/**
 * 源时钟6.25MHZ
 */
#define SGPIO_SOURCE_CLK  (6250000UL) /* 6.25MHz */

typedef enum {
    SGPIO_STA1005 = 0,
    SGPIO_SFF8485,
} SgpioProtocol_e;

typedef enum {
    SGPIO_MODE_GP_WHEN_TX_SLOAD_IS_ODD = 0,
    SGPIO_MODE_GP_WHEN_TX_SLOAD_IS_NOT_ZERO,
} SgpioGpMode_e;

typedef union __attribute__((packed)) {
    struct {
        U32 sff8485Sel0 : 1;          /* [bit0] 0: STA1005 protocol, 1: SFF8485 protocol */
        U32 sff8485Sel1 : 1;          /* [bit1] 0: STA1005 protocol, 1: SFF8485 protocol */
        U32 sff8485Sel2 : 1;          /* [bit2] 0: STA1005 protocol, 1: SFF8485 protocol */
        U32 sff8485Sel3 : 1;          /* [bit3] 0: STA1005 protocol, 1: SFF8485 protocol */
        U32 rsv0 : 8;                 /* [11:4] */
        U32 sgpioBlinkSel : 2;        /* [13:12] 00: SGPIO0, 01: SGPIO1, 10: SGPIO2, 11: SGPIO3 */
        U32 rsv1 : 18;                /* [31:14] */
    } fields;
    U32 dword;
} SgpioTopCfg_u;

typedef union __attribute__((packed)) {
    struct {
        U32 rcv0 : 8;           /* [7:0] */
        U32 version : 4;        /* [11:8] */
        U32 rcv1 : 4;           /* [15:12] */
        U32 gpRegCount : 4;     /* [19:16], rst_val=0x1 */
        U32 cfgRegCount : 3;    /* [22:20], rst_val=0x2 */
        U32 sgpioEn : 1;        /* [23] */
        U32 sdriveCount : 7;    /* [30:24], rstVal=0x4 */
        U32 rcv2 : 1;           /* [31] */
    } fields;
    U32 dword;
} SgpioCfg0_u;

typedef union __attribute__((packed)) {
    struct {
        U32 rsv0 : 8; /* [7:0] rsv0 */
        U32 blinkGenRateA : 4; /* [11:8], rstVal=0x0, a: 1/8s */
        U32 blinkGenRateB : 4; /* [15:12], rstVal=0x0, b: 1/8s */
        U32 maxActOn : 4; /* [19:16], rstVal=0x2, maxActOn: 2/4s */
        U32 forceActOff : 4; /* [23:20], rstVal=0x1, forceActOff: 1/8s */
        U32 strechActOn : 4; /* [27:24], rstVal=0x0, strechActOn: 1/64s */
        U32 strechActOff : 4; /* [31:28], rstVal=0x0, strechActOff: 1/64s */
    } fields;
    U32 dword;
} SgpioCfg1_u;

typedef union __attribute__((packed)) {
    struct {
        U32 nlRxDrive4n0 : 3;     /* [2:0] */
        U32 rcv0 : 5;           /* [7:3] */
        U32 nlRxDrive4n1 : 3;     /* [10:8] */
        U32 rcv1 : 5;           /* [15:11] */
        U32 nlRxDrive4n2 : 3;     /* [18:16] */
        U32 rcv2 : 5;           /* [23:19] */
        U32 nlRxDrive4n3 : 3;     /* [26:24] */
        U32 rcv3 : 5;           /* [31:27] */
    } fields;
    U32 dword;
} SgpioRx_u;

typedef union __attribute__((packed)) {
    struct {
        /*
         * config state under software mode
         * bit[0:8]: [2:0]-OD0, [5:3]-OD1, [8:6]-OD2
         * code-state: 000b-off, 001b-on, 010b-blink freq A
         *             011b-blink B, 100b-blink C, others-off
         */
        U32 nlTxDrive4n0 : 8;     /* [7:0] driver 4n SDataOut value */
        U32 nlTxDrive4n1 : 8;     /* [15:8] driver 4n+1 SDataOut value */
        U32 nlTxDrive4n2 : 8;     /* [23:16] driver 4n+2 SDataOut value */
        U32 nlTxDrive4n3 : 8;     /* [31:24] driver 4n+3 SDataOut value */
    } fields;
    U32 dword;
} SgpioTx_u;

typedef union __attribute__((packed)) {
    struct {
        U32 rsv0 : 16;       /* [15:0], rstVal=0x0 */
        U32 gpRxCfgCnt : 8;  /* [23:16], rstVal=0xff */
        U32 rsv1 : 8;        /* [31:24] */
    } fields;
    U32 dword;
} SgpioRxGpCfg_u;

typedef union __attribute__((packed)) {
    U32 dword; /* 3bits per link*/
} SgpioRxGp_u;

typedef union __attribute__((packed)) {
    struct {
        U32 rsv0 : 16; /* [15:0]  rstVal=0x0 */
        U32 gpTxCfgCnt : 8; /* [23:16] rstVal=0xff */
        U32 gpTxSload : 4; /* [27:24] rstVal=0x0 normal mode must be 0x0 */
        U32 rsv1 : 4; /* [31:28] rstVal=0x0 */
    } fields;
    U32 dword;
} SgpioTxGpCfg_u;

typedef union __attribute__((packed)) {
    U32 dword; /* 3bits per link*/
} SgpioTxGp_u;

typedef union __attribute__((packed)) {
    struct {
        U32 firstSioDevice : 1;  /* [0], rstVal=0x0 */
        U32 sioMode : 1;        /* [1], rstVal=0x0 */
        U32 rcv0 : 1;            /* [2] */
        U32 sioClkEn : 1;     /* [3], rstVal=0x0 */
        U32 sioCtrlDoutOden : 1; /* [4], rstVal=0x0 */
        U32 sioClkFilterEn : 1; /* [5], rstVal=0x0 */
        U32 sioCtrlOden : 1;   /* [6], rstVal=0x0 */
        U32 clkDivide : 21;    /* [27:7], rstVal=0x0 */
        U32 rsv0 : 2;       /* [29:28] */
        U32 stsSioClkoutEn : 1; /* [30], rstVal=0x0 */
        U32 restFsm : 1;      /* [31], rstVal=0x0 */
    } fields;
    U32 dword;
} SgpioVendorSpec0_u;

typedef union __attribute__((packed)) {
    struct {
        U32 sgpioDeviceCnt : 7; /* [6:0], rstVal=0x0 */
        U32 rsv0 : 1;          /* [7] */
        U32 sgpioModeCtrlSel : 1; /* [8], rstVal=0x0 */
        U32 sloadMode : 1;   /* [9], rstVal=0x0 */
        U32 gpModeCtrlSel : 1; /* [10], rstVal=0x0 */
        U32 dinLatchMode : 1; /* [11], rstVal=0x0 */
        U32 sdoutPolaritySel : 1; /* [12], rstVal=0x0 */
        U32 sloadPolaritySel : 1; /* [13], rstVal=0x0 */
        U32 rsv1 : 18;            /* [31:14] */
    } fields;
    U32 dword;
} SgpioVendorSpec1_u;

typedef union __attribute__((packed)) {
    struct {
        U32 blinkOutSel : 3; /* [2:0] 00: SGPIO0, 01: SGPIO1, 10: SGPIO2, 11: SGPIO3 */
        U32 blinkOutOen : 1; /* [3]  1: disable, 0: enable */
        U32 mstBlinkSel : 1; /* [4]  0: from external (sio_blink_i), 1: from sioBlinkReg */
        U32 blinkMask : 1; /* [5]  1: disable blink function, 0: enable */
        U32 rsv0 : 26;       /* [31:6] */
    } fields;
    U32 dword;
} SgpioBlinkSel_u;

typedef volatile struct {
    SgpioCfg0_u sgpioCfg0;               /* 0x0 */
    SgpioCfg1_u sgpioCfg1;               /* 0x4 */
    U32 rsv0[62];                        /* 0x8~0xff */
    SgpioRx_u rxn[16];                   /* 0x100~0x13c */     
    U32 rsv1[48];                        /* 0x140~0x1ff */      
    SgpioTx_u txn[16];                   /* 0x200~0x23c */
    U32 rsv2[48];                        /* 0x240~0x2ff */
    SgpioRxGpCfg_u rxgpCfg;              /* 0x300 */
    SgpioRxGp_u rxgp[6];                 /* 0x304~0x318 */
    U32 rsv3[57];                        /* 0x31c~0x3ff */
    SgpioTxGpCfg_u txgpCfg;              /* 0x400 */
    SgpioTxGp_u txgp[6];                 /* 0x404~0x418 */
    U32 rsv6[57];                        /* 0x41c~0x4ff */
    SgpioVendorSpec0_u vendorSpec0;      /* 0x500 */
    SgpioVendorSpec1_u vendorSpec1;      /* 0x504 */
    U32 rsv7[78];                        /* 0x508~0x63f */
    U32 driverStrobe0;                   /* 0x640 strobe signal for driver 0-31 */
    U32 driverStrobe1;                   /* 0x644 strobe signal for driver 32-63 */
    U32 rsv9[46];                        /* 0x648~0x6ff */
    U32 sioIntStatus;                    /* 0x700 SGPIO interrupt status register */
    U32 sioIntCfg;                       /* 0x704 SGPIO interrupt configuration register */
    U32 rsv10[62];                       /* 0x708~0x7ff */
    SgpioBlinkSel_u sioBlinkSel;         /* 0x800 SGPIO blink selection register */
    U32 sioBlinkReg;                     /* 0x804 SGPIO blink register */
    U32 sioClkPeriod;                    /* 0x808 SGPIO clock period register */
    U32 rsv11[7];                        /* 0x80c~0x824 */
    U32 us100Cnt;                        /* 0x828 100us counter register */
} SgpioReg_s;

typedef struct __attribute__((packed)) sgpioDrvData {
    SbrSgpioCfg_s sbrCfg;
} SgpioDrvData_s;

#endif /* __DRV_SGPIO_V1P0_H__ */
