/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sgpio_v2.0.h
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

#ifndef __DRV_SGPIO_H__
#define __DRV_SGPIO_H__

#include "common_defines.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

#define LED_OD2(pattern)                    (pattern << 6)
#define LED_OD1(pattern)                    (pattern << 3)
#define LED_OD0(pattern)                    (pattern << 0)

#define LED_DRIVE_PATTERN(od2, od1, od0)    (LED_OD2(od2) | LED_OD1(od1) | LED_OD0(od0))

enum LEDPattern { /* defined info from hardware */
    LED_OFF     = 0x0,
    LED_ON      = 0x1,
    LED_BLINK_A = 0x2,
    LED_BLINK_B = 0x3,
    LED_BLINK_C = 0x4,
};

typedef enum __sgpio_type {
    SGPIO_ERROR = 0,
    SGPIO_LOCATE,
    SGPIO_ACTIVE
} SgpioLed_e;

typedef enum {
    /**
     * @brief  sgpio Drive(3 leds in a drive group) pattern
     * @details On FPGA, [8:6]-OD2 not used, [5:3]-OD1 red, [2:0]-OD0 green.
     *              For fpga, use red to replace yellow
     *          On ASIC,  [8:6]-OD2 not used, [5:3]-OD1 yellow, [2:0]-OD0 green
     *
     * @note    OD2 led not be used in Sheshou
     *          ODx means LEDx(ODx == LEDx)
     */
    /* 9b000 000 000 -- off */
    SGPIO_LINK_DOWN_P0      = LED_DRIVE_PATTERN(LED_OFF, LED_OFF, LED_OFF),
    /* 9b001 000 000 -- red/yellow on */
    SGPIO_LINK_INIT_P1      = LED_DRIVE_PATTERN(LED_OFF, LED_ON, LED_OFF),
    /* 9b000 001 000 -- green on */
    SGPIO_LINK_ACT_IDLE_P2  = LED_DRIVE_PATTERN(LED_OFF, LED_OFF, LED_ON),
    /* 9b000 100 000 -- green blink in freq C 2hz */
    SGPIO_LINK_ACT_X8_P3    = LED_DRIVE_PATTERN(LED_OFF, LED_OFF, LED_BLINK_C),
    /* 9b000 011 000 -- green blink in freq B 1hz */
    SGPIO_LINK_ACT_X4_P4    = LED_DRIVE_PATTERN(LED_OFF, LED_OFF, LED_BLINK_B),
    /* 9b000 010 000 -- green blink in freq A 0.5hz */
    SGPIO_LINK_ACT_OTHER_P5 = LED_DRIVE_PATTERN(LED_OFF, LED_OFF, LED_BLINK_A),
    /* 9b100 000 000 -- red/yellow blink in freq C 2hz */
    SGPIO_PORT_ERR_P6       = LED_DRIVE_PATTERN(LED_OFF, LED_BLINK_C, LED_OFF),
    /* 9b010 000 000 -- red/yellow blink in freq A 0.5hz */
    SGPIO_LOCATE_P7         = LED_DRIVE_PATTERN(LED_OFF, LED_BLINK_A, LED_OFF),
    /* for error check */
    SGPIO_PATTERN_MAX = 0x1F,
} SgpioDrvPattern_e;

typedef union {
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

typedef union {
    struct {
        U32 blinkDivA : 16; /* [15:0], rstVal=0xbb8, 150mhz--a:0.5hz */
        U32 blinkDivB : 16; /* [31:16], rstVal=0x5dc, b:1hz */
    } fields;
    U32 dword;
} SgpioCfg1_u;

typedef union {
    struct {
        U32 onMin : 16;    /* [15:0], rstVal=0xbb8 */
        U32 offMin : 16;   /* [31:16], rstVal=0xbb8 */
    } fields;
    U32 dword;
} SgpioCfg2_u;

typedef union {
    struct {
        U32 blinkDivC : 16;     /* [15:0], rstVal=0x2ee, c:2hz */
        U32 rcv : 16;           /* [31:16] */
    } fields;
    U32 dword;
} SgpioCfg4_u;

typedef union {
    struct {
        U32 nlRxDrive0 : 3;     /* [2:0] */
        U32 rcv0 : 5;           /* [7:3] */
        U32 nlRxDrive1 : 3;     /* [10:8] */
        U32 rcv1 : 5;           /* [15:11] */
        U32 nlRxDrive2 : 3;     /* [18:16] */
        U32 rcv2 : 5;           /* [23:19] */
        U32 nlRxDrive3 : 3;     /* [26:24] */
        U32 rcv3 : 5;           /* [31:27] */
    } fields;
    U32 dword;
} SgpioRx_u;

typedef union {
    struct {
        /*
         * config state under software mode
         * bit[0:8]: [2:0]-OD0, [5:3]-OD1, [8:6]-OD2
         * code-state: 000b-off, 001b-on, 010b-blink freq A
         *             011b-blink B, 100b-blink C, others-off
         */
        U32 nlTxDrive0 : 9;     /* [8:0] */
        U32 rcv0 : 7;           /* [15:9] */
        U32 nlTxDrive1 : 9;     /* [24:16] */
        U32 rcv1 : 7;           /* [31:25] */
    } fields;
    U32 dword;
} SgpioTx_u;

typedef union {
    struct {
        U32 gpRxCfgCount : 8;       /* [7:0], rstVal=0xff */
        U32 rcv : 24;               /* [31:8] */
    } fields;
    U32 dword;
} SgpioRxGpCfg_u;

typedef union {
    struct {
        U32 gpTxCfgCount : 8;       /* [7:0], rstVal=0xff */
        U32 gpTxSload : 4;          /* [11:8] */
        U32 rcv : 20;               /* [31:12] */
    } fields;
    U32 dword;
} SgpioTxGpCfg_u;

typedef union {
    struct {
        U32 sloadoutOden : 1;  /* [0] */
        U32 sdoutOden : 1;     /* [1] */
        U32 sclkoutOden : 1;   /* [2] */
        U32 sdinOden : 1;      /* [3] */
        U32 resetFsm : 1;      /* [4] */
        U32 clkDiv : 21;       /* [25:5], rstVal=0x96 */
        U32 rcv : 6; /* [31:26] */
    } fields;
    U32 dword;
} SgpioOutCtrl_u;

typedef union {
    struct {
        U32 sloadPolaritySel : 1;   /* [0] */
        U32 sdoutPolaritySel : 1;   /* [1] */
        U32 gpModeCtrlSel : 1;      /* [2] */
        U32 sloadMode : 1;          /* [3] */
        U32 rcv : 28; /* [31:4] */
    } fields;
    U32 dword;
} SgpioModeCtrl_u;

typedef union {
    struct {
        U32 linkTxMatch : 1;    /* [0] */
        U32 driveCtrl : 1;      /* [1] */
        U32 txGp : 1;           /* [2] */
        U32 txNl : 1;           /* [3] */
        U32 rcv : 28;           /* [31:4] */
    } fields;
    U32 dword;
} SgpioQuickCfg_u;

typedef union {
    struct {
        /*
         * bit[0:5]: [1:0]-OD0, [3:2]-OD1, [5:4]-OD2
         * code-discription: 00b-soft ctrl, high low
         *                   01b-soft ctrl, low high
         *                   10b-soft&hard ctrl, high low
         *                   11b-soft&hard ctrl, low high
         */
        U8 driveCtrl : 6;   /* [5:0], rstVal=0x2a */
        U8 rcv0 : 2;        /* [7:6] */
    } fields;
    U8 dword;
} SgpioDriveCtrl_u;

typedef union {
    struct {
        /*
         * config state under hardware mode
         * bit[0:8]: [2:0]-OD0, [5:3]-OD1, [8:6]-OD2
         * code-state: 000b-off, 001b-on, 010b-blink freq A
         *             011b-blink B, 100b-blink C, others-off
         *
         * bit[15:9]: reserved
         */
        U16    stateX[8];
    } fields;
    U32 dword[4];
} SgpioLinkTxMatch_u;

typedef union {
    struct {
        U32 rxOverflow : 1;     /* [0] */
        U32 rxdataValid : 1;    /* [1] */
        U32 outOfRange : 1;     /* [2] */
        U32 rcv : 29; /* [31:3] */
    } fields;
    U32 dword;
} SgpioIntStatus_u;

typedef union {
    struct {
        U32 rxOverflowPending : 1;    /* [0] */
        U32 rxdataValidPending : 1;   /* [1] */
        U32 out_of_range_pending : 1; /* [2] */
        U32 rcv : 29; /* [31:3] */
    } fields;
    U32 dword;
} SgpioIntPending_u;

typedef union {
    struct {
        U32 rxOverflowMask : 1;     /* [0] */
        U32 rxdataValidMask : 1;    /* [1] */
        U32 rcv0 : 1;               /* [2] */
        U32 rxOverflowClr : 1;      /* [3] */
        U32 rcv1 : 28; /* [31:4] */
    } fields;
    U32 dword;
} SgpioIntCtrl_u;

typedef union {
    struct {
        U32 size : 4;   /* [3:0], rstVal=0x2 */
        U32 level : 4;  /* [7:4] */
        U32 full : 1;   /* [8] */
        U32 empty : 1;  /* [9], rstVal=0x1 */
        U32 rcv1 : 22;  /* [31:10] */
    } fields;
    U32 dword;
} SgpioRxfifoStatue_u;

typedef union {
    struct {
        U32 l0_sta : 4; /* [3:0] */
        U32 l1_sta : 4; /* [7:4] */
        U32 l2_sta : 4; /* [11:8] */
        U32 l3_sta : 4; /* [15:12] */
        U32 l4_sta : 4; /* [19:16] */
        U32 l5_sta : 4; /* [23:20] */
        U32 l6_sta : 4; /* [27:24] */
        U32 l7_sta : 4; /* [31:28] */
    } fields;
    U32 dword;
} SgpioLinkStatus_u;

typedef volatile struct {
    SgpioCfg0_u cfg0;               /* 0x0 */
    SgpioCfg1_u cfg1;               /* 0x4 */
    SgpioCfg2_u cfg2;               /* 0x8 */
    U32 cfg3;                       /* 0xC, rstVal=0x96 */
    SgpioCfg4_u cfg4;               /* 0x10 */
    U32 resved_14[59];              /* 0x14~0xff */
    SgpioRx_u rxnl[24];             /* 0x100~0x15c */
    U32 resved_160[40];             /* 0x160~0x1ff */
    SgpioTx_u txnl[48];             /* 0x200~0x2bc */
    U32 resved_2c0[16];             /* 0x2c0~0x2ff */
    SgpioRxGpCfg_u rxgpCfg;         /* 0x300 */
    SgpioTxGpCfg_u txgpCfg;         /* 0x304 */
    U32 resved_308[62];             /* 0x308~0x3ff */
    U32 rxgp[9];                    /* 0x400~0x420 */
    U32 resved_424[55];             /* 0x424~0x4ff */
    U32 txgp[9];                    /* 0x500~0x520 */
    U32 resved_524[55];             /* 0x524~0x5ff */
    SgpioOutCtrl_u outCtrl;         /* 0x600 */
    SgpioModeCtrl_u modeCtrl;       /* 0x604 */
    SgpioQuickCfg_u qkCfg;          /* 0x608 */
    U32 resved_60c[61];             /* 0x60c~0x6ff */
    SgpioDriveCtrl_u drvCtrl[96];   /* 0x700~0x75c */
    U32 resved_760[40];             /* 0x760~0x7ff */
    SgpioLinkTxMatch_u txMatch[96]; /* 0x800~0xdfc */

    SgpioIntStatus_u intSta;        /* 0xe00 */
    SgpioIntCtrl_u intCtrl;         /* 0xe04 */
    SgpioRxfifoStatue_u rxFifoSta;  /* 0xe08 */
    SgpioIntPending_u intPending;   /* 0xe0c */
    U32 resved_e10[60];             /* 0xe10~0xeff */
    SgpioLinkStatus_u linkSta[12];  /* 0xf00~0xf2c */
} SgpioReg_s;

typedef struct sgpioDrvData {
    SbrSgpioCfg_s sbrCfg;
} SgpioDrvData_s;

#endif /* __DRV_SGPIO_H__ */
