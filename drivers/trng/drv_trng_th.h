/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv-trng-th.h
 * @author dingwei (dingwei@starsmicrosystem.com)
 * @date 2025/11/26
 * @brief TRNG driver private header
 */
#ifndef __DRV_TRNG_TH_H__
#define __DRV_TRNG_TH_H__

#include "common_defines.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

#define TRNG_LOCK_TIMEOUT_MS           (1000U)
#define TRNG_WAIT_TIMEOUT_MS           (100U)
#define TRNG_POLL_DELAY_US             (1000U)
#define TRNG_MAX_RANDOM_WORDS          (2048U)

#define TRNG_IE_ZEROIZED               (1U << 0)
#define TRNG_IE_KAT_COMPLETE           (1U << 1)
#define TRNG_IE_NOISE                  (1U << 2)
#define TRNG_IE_ALARM                  (1U << 3)
#define TRNG_IE_DONE                   (1U << 4)
#define TRNG_IE_COUNT                  (1U << 5)
#define TRNG_IE_GLBL                   (1U << 31)
#define TRNG_IE_ALL                    (TRNG_IE_ZEROIZED | TRNG_IE_KAT_COMPLETE | \
                                        TRNG_IE_NOISE | TRNG_IE_ALARM | \
                                        TRNG_IE_DONE | TRNG_IE_COUNT | TRNG_IE_GLBL)

#define TRNG_ENTROPY_ALL               (0x00000FFFU)

/**
 * @brief TRNG command types
 */
typedef enum {
    CMD_NOP = 0,
    CMD_GEN_NOISE,
    CMD_GEN_NONCE,
    CMD_CREATE_STATE,
    CMD_RENEW_STATE,
    CMD_REFRESH_ADDIN,
    CMD_GEN_RANDOM,
    CMD_ADVANCE_STATE,
    CMD_RUN_KAT,
    CMD_ZEROIZE = 0xF,
} TrngCmd_e;

/**
 * @brief TRNG register definitions
 */
typedef union {
    struct {
        U32 cmd : 4;
        U32 : 28;
    } b;
    U32 w;
} TrngCtrl_u;

typedef union {
    struct {
        U32 secAlg : 1;
        U32 : 2;
        U32 predResist : 1;
        U32 addinPresent : 1;
        U32 katVec : 2;
        U32 katSel : 2;
        U32 : 23;
    } b;
    U32 w;
} TrngMode_u;

typedef union {
    struct {
        U32 nonce : 1;
        U32 secureEn : 1;
        U32 maxRejects : 8;
        U32 : 2;
        U32 autoEn : 1;
        U32 : 1;
        U32 bgEnLock : 1;
        U32 xorAll : 1;
        U32 bgEnable : 12;
        U32 : 4;
    } b;
    U32 w;
} TrngSmode_u;

typedef union {
    struct {
        U32 lastCmd : 4;
        U32 secAlg : 1;
        U32 nonceMode : 1;
        U32 secure : 1;
        U32 drbgState : 2;
        U32 seedEmpty : 1;
        U32 randEmpty : 1;
        U32 randAlmostEmpty : 1;
        U32 fsmState : 4;
        U32 : 15;
        U32 busy : 1;
    } b;
    U32 w;
} TrngStat_u;

typedef union {
    struct {
        U32 zeroized : 1;
        U32 katComplete : 1;
        U32 noise : 1;
        U32 alarm : 1;
        U32 done : 1;
        U32 count : 1;
        U32 : 25;
        U32 glbl : 1;
    } b;
    U32 w;
} TrngIe_u;

typedef union {
    struct {
        U32 zeroized : 1;
        U32 katComplete : 1;
        U32 noise : 1;
        U32 alarm : 1;
        U32 done : 1;
        U32 count : 1;
        U32 : 26;
    } b;
    U32 w;
} TrngIstat_u;

typedef union {
    struct {
        U32 : 10;
        U32 rdSeedSelect : 1;
        U32 : 20;
        U32 rdseedPriHigh : 1;
    } b;
    U32 w;
} TrngMux_u;

/**
 * @brief TRNG register map
 */
typedef volatile struct {
    TrngCtrl_u   ctrl;                  ///< 0x00
    TrngMode_u   mode;                  ///< 0x04
    TrngSmode_u  smode;                 ///< 0x08
    TrngStat_u   stat;                  ///< 0x0C
    TrngIe_u     ie;                    ///< 0x10
    TrngIstat_u  istat;                 ///< 0x14
    U32          alarms;                ///< 0x18
    U32          corekitRel;            ///< 0x1C
    U32          features;              ///< 0x20
    U32          rand[4];               ///< 0x24
    U32          npaData[16];           ///< 0x34
    U32          seed[12];              ///< 0x74
    U32          iaRdata;               ///< 0xA4
    U32          iaWdata;               ///< 0xA8
    U32          iaAddr;                ///< 0xAC
    U32          iaCmd;                 ///< 0xB0
    U32          randCount;             ///< 0xB4
    U32          randCountThreshold;    ///< 0xB8
    U32          rdrandValue;           ///< 0xBC
    U32          rdseedValue;           ///< 0xC0
    U32          terminate;             ///< 0xC4
    U32          desctle;               ///< 0xC8
    U32          reserved1[2];          ///< 0xCC-0xD0
    U32          seedAvail;             ///< 0xD4
    U32          seedAlmostEmptyLevel;  ///< 0xD8
    U32          randAlmostEmptyLevel;  ///< 0xDC
    U32          reserved2[6];          ///< 0xE0-0xF4
    TrngMux_u    trngMux;               ///< 0xF8
} TrngReg_s;

/**
 * @brief TRNG driver private data
 */
typedef struct {
    SbrTrngCfg_s sbrCfg;                ///< SBR configuration
} TrngDrvData_s;

#endif /* __DRV_TRNG_TH_H__ */

