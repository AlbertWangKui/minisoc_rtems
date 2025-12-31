/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_trng.h
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/05/19
 * @brief
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/05/19   yangzhl3        porting from rtems_minisoc
 *
 *
 */
#ifndef __DRV_TRNG_DW_H__
#define __DRV_TRNG_DW_H__

#include "common_defines.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

/* offset: 0x0 */
typedef union {
    struct {

        /**
         * @brief  cmd: WO(write only)
         * @details
         *          0x0: NOP
         *          0x1: GEN_NOISE
         *          0x2: GEN_NONECE
         *          0x3: CREATE_STATE
         *          0x4: RENEW_STATE
         *          0x5: REFRESH_ADDIN
         *          0x6: GEN_RANDOM
         *          0x7: ADVANCE_STATE
         *          0x8: RUN_KAT
         *          0xf: ZEROIZE
         */
        U32 cmd : 4;                /* [3:0] */
        U32 : 28;
    } fields;
    U32 dword;
} TrngCtrlReg_u;

/* offset: 0x4 */
typedef union {
    struct {
        U32 sec_alg : 1;            /* [0] */
        U32 : 2;
        U32 pred_resist : 1;        /* [3] */
        U32 addin_present : 1;      /* [4] */
        U32 kat_vec : 2;            /* [6:5] */
        U32 kat_sel : 2;            /* [8:7] */
        U32 : 23;
    } fields;
    U32 dword;
} TrngModeReg_u;

/* offset: 0x8 */
typedef union {
    struct {
        U32 nonce : 1;              /* [0] */
        U32 mission_mode : 1;       /* [1] */
        U32 max_rejects : 8;        /* [9:2] */
        U32 : 6;                    /* [15:] */
        U32 indiv_ht_disable : 8;   /* [23:16] */
        U32 : 7;
        U32 noise_collect : 1;      /* [31] */
    } fields;
    U32 dword;
} TrngSmodeReg_u;

/* offset: 0xc */
typedef union {
    struct {
        U32 last_cmd : 4;               /* [3:0] */
        U32 sec_alg : 1;                /* [4] */
        U32 nonce : 1;                  /* [5] */
        U32 mission_mode : 1;           /* [6] */
        U32 drbg_state : 2;             /* [8:7] */
        U32 startup_test_stuck : 1;     /* [9] */
        U32 startup_test_in_prog : 1;   /* [10] */
        U32 : 20;
        U32 busy : 1; /* bit_31 */
    } fields;
    U32 dword;
} TrngStatReg_u;

/* offset: 0x10 */
typedef union {
    struct {
        U32 zeroized : 1;       /* [0] */
        U32 kat_completed : 1;  /* [1] */
        U32 noise_rdy : 1;      /* [2] */
        U32 alarms : 1;         /* [3] */
        U32 done : 1;           /* [4] */
        U32 : 26;
        U32 glbl : 1;           /* [31] */
    } fields;
    U32 dword;
} TrngIeReg_u;

/* offset: 0x14 */
typedef union {
    struct {
        U32 zeroized : 1;       /* [0] */
        U32 kat_completed : 1;  /* [1] */
        U32 noise_rdy : 1;      /* [2] */
        U32 alarms : 1;         /* [3] */
        U32 done : 1;           /* [4] */
        U32 : 27;
    } fields;
    U32 dword;
} TrngIstatReg_u;

/* offset: 0x18 */
typedef union {
    struct {
        U32 failed_test_id : 4;     /* [3:0] */
        U32 illegal_cmd_seq : 1;    /* [4] */
        U32 failed_tseed_st_ht : 1; /* [5] */
        U32 : 26;
    } fields;
    U32 dword;
} TrngAlarmsReg_u;

/* offset: 0x1c */
typedef union {
    struct {
        U32 rel_num : 16;   /* [15:0] */
        U32 ext_ver : 8;    /* [23:16] */
        U32 : 4;
        U32 ext_enum : 4;   /* [31:28] */
    } fields;
    U32 dword;
} TrngCorekitRelReg_u;

/* offset: 0x20 */
typedef union {
    struct {
        U32 secure_rst_state : 1;       /* [0] */
        U32 diag_level_st_hlt : 3;      /* [3:1] */
        U32 diag_level_cpl800 : 3;      /* [6:4] */
        U32 diag_level_ns : 1;          /* [7] */
        U32 ps_present : 1;             /* [8] */
        U32 aes256 : 1;                 /* [9] */
        U32 : 22;
    } fields;
    U32 dword;
} TrngFeatureReg_u;

/* offset: 0xf0 */
typedef union {
    struct {
        U32 core_type : 2;                  /* [1:0] */
        U32 : 5;
        U32 bg8 : 1;                        /* [7] */
        U32 cdc_sync_depth : 2;             /* [9:8] */
        U32 backgroud_noise : 1;            /* [10] */
        U32 edu_present : 1;                /* [11] */
        U32 aes_datapath : 1;               /* [12] */
        U32 aes_max_key_size : 1;           /* [13] */
        U32 personalization_str : 1;        /* [14] */
        U32 : 17;
    } fields;
    U32 dword;
} TrngBuildCfg0Reg_u;

/* offset: 0xf4 */
typedef union {
    struct {
        U32 num_raw_noise_blks : 8;         /* [7:0] */
        U32 sticky_startup : 1;             /* [8] */
        U32 : 3;
        U32 auto_coprelation_test : 1;      /* [12] */
        U32 monobit_test : 1;               /* [13] */
        U32 run_test : 1;                   /* [14] */
        U32 poker_test : 1;                 /* [15] */
        U32 raw_ht_adap_test : 3;           /* [18:16] */
        U32 raw_ht_rep_test : 1;            /* [19] */
        U32 ent_src_req_smpl_test : 3;      /* [22:20] */
        U32 ent_src_rep_test : 1;           /* [23] */
        U32 ent_src_rep1_test : 7;          /* [30:24] */
        U32 : 1;
    } fields;
    U32 dword;
} TrngBuildCfg1Reg_u;

typedef volatile struct __trng_reg {
    volatile TrngCtrlReg_u           ctrl;           /* 0x0 */
    volatile TrngModeReg_u           mode;           /* 0x4 */
    volatile TrngSmodeReg_u          smode;          /* 0x8 */
    volatile TrngStatReg_u           stat;           /* 0xC */
    volatile TrngIeReg_u             ie;             /* 0x10 */
    volatile TrngIstatReg_u          istat;          /* 0x14 */
    volatile TrngAlarmsReg_u         alarms;         /* 0x18 */
    volatile TrngCorekitRelReg_u     corekit_rel;    /* 0x1C */
    volatile TrngFeatureReg_u        features;       /* 0x20 */
    volatile U32                     rand[4];        /* 0x24 */
    volatile U32                     npa_data[16];   /* 0x34 */
    volatile U32                     seed[12];       /* 0x74 */
    volatile U32                     reserved0[11];  /* 0xA4 */
    volatile U32                     tts;            /* 0xD0 */
    volatile U32                     reserved1[7];   /* 0xD4 */
    volatile TrngBuildCfg0Reg_u      build_cfg0;     /* 0xF0 */
    volatile TrngBuildCfg1Reg_u      build_cfg1;     /* 0xF4 */
} TrngReg_s;

typedef enum {
    CMD_TYPE_NOP = 0,
    CMD_TYPE_GEN_NOISE,
    CMD_TYPE_GEN_NONECE,
    CMD_TYPE_CREATE_STATE,
    CMD_TYPE_RENEW_STATE,
    CMD_TYPE_REFRESH_ADDIN,
    CMD_TYPE_GEN_RANDOM,
    CMD_TYPE_ADVANCE_SATE,
    CMD_TYPE_RUN_KAT,
    CMD_TYPE_ZEROIZE = 0xf,
    CMD_TYPE_RESERVED,
} CmdType_e;

typedef enum {
    SYNC_DEPTH_CDC_2 = 2,
    SYNC_DEPTH_CDC_3 = 3,
    SYNC_DEPTH_CDC_4 = 4,
    SYNC_DEPTH_CDC_RESERVED,
} CDC_SYNC_DEPTH_e;

typedef enum {
    CORE_TYPE_BASE_TRNG = 0,
    CORE_TYPE_BASE_TRNG_WITH_ESM_NONCE,
    CORE_TYPE_BASE_NIST_TRNG,
    CORE_TYPE_BASE_NIST_TRNG_WITH_EDU,
    CORE_TYPE_RESERVED,
} CoreType_e;

typedef struct {
    SbrTrngCfg_s sbrCfg;
} TrngDrvData_s;

#endif /* _DRV_TRNG_H */
