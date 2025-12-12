/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sgpio_v2.0.c
 * @author liugh (liugh@starsmicrosystem.com)
 * @date 2025/06/01
 * @brief  sgpio driver for SheShou
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2023/07/06   liugh           the first version
 * 2025/06/01   yangzhl3        add Json & remove IOC interface
 *
 */

#include "common_defines.h"
#include "log_msg.h"
#include "udelay.h"
#include "drv_sgpio_v2.0.h"
#include "drv_sgpio_api.h"
#include "bsp_api.h"
#include "stringto_wrapper.h"
#include "bsp_config.h"
#include "bsp_api.h"

#define SGPIO_QUICK_CFG         (0xf)
#define SGPIO_PINMUX_OFFSET     (0x1D0)
#define SGPIO_MIN_GROUP_SOFT    (2)
#define SGPIO_MIN_GROUP         (4)
#define SGPIO_DIV_0P5HZ         (50000)
#define SGPIO_DIV_1HZ           (100000)
#define SGPIO_DIV_2HZ           (200000)
#define SGPIO_DIV_100K          (1000000)
#define SGPIO_MAX_DRIVE         (96)

typedef struct {
    SgpioDrvPattern_e   pattern;    /* hardware interface */
    SgpioState_e        state;      /* user interface */
} SgpioP2SItem_s;

static SgpioP2SItem_s sgpio_p2s_tab[] = {
    {SGPIO_LINK_DOWN_P0,        SGPIO_LINK_DOWN_S0      },
    {SGPIO_LINK_INIT_P1,        SGPIO_LINK_INIT_S1      },
    {SGPIO_LINK_ACT_IDLE_P2,    SGPIO_LINK_ACT_IDLE_S2  },
    {SGPIO_LINK_ACT_X8_P3,      SGPIO_LINK_ACT_X8_S3    },
    {SGPIO_LINK_ACT_X4_P4,      SGPIO_LINK_ACT_X4_S4    },
    {SGPIO_LINK_ACT_OTHER_P5,   SGPIO_LINK_ACT_OTHER_S5 },
    {SGPIO_PORT_ERR_P6,         SGPIO_PORT_ERR_S6       },
    {SGPIO_LOCATE_P7,           SGPIO_LOCATE_S7         },
    {SGPIO_PATTERN_MAX,         SGPIO_HARD_MODE_S8      },
};

static SgpioState_e pattern2State(SgpioDrvPattern_e pattern)
{
    for (int i = 0; i < ARRAY_SIZE(sgpio_p2s_tab); i++) {
        if (sgpio_p2s_tab[i].pattern == pattern) {
            return sgpio_p2s_tab[i].state;
        }
    }
    return SGPIO_PATTERN_MAX;
}

static SgpioDrvPattern_e state2Pattern(SgpioState_e state)
{
    for (int i = 0; i < ARRAY_SIZE(sgpio_p2s_tab); i++) {
        if (sgpio_p2s_tab[i].state == state) {
            return sgpio_p2s_tab[i].pattern;
        }
    }
    /* can not be reached */
    return SGPIO_LINK_DOWN_P0;
}

__attribute__((unused)) static S32 sgpioDevEnable(SgpioDrvData_s *sgpioDrvData)
{
    S32 ret = EXIT_SUCCESS;

    if (NULL == sgpioDrvData) {
        ret = -EINVAL;
        goto out;
    }
    ((SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr)->cfg0.fields.sgpioEn = 1;
out:
    return ret;
}

__attribute__((unused)) static S32 sgpioDevDisable(SgpioDrvData_s *sgpioDrvData)
{
    S32 ret = EXIT_SUCCESS;

    if (NULL == sgpioDrvData) {
        ret = -EINVAL;
        goto out;
    }
    ((SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr)->cfg0.fields.sgpioEn = 0;
out:
    return ret;
}

__attribute__((unused)) static S32 sgpioSloadSet(SgpioDrvData_s *sgpioDrvData, U32 value)
{
    S32 ret = EXIT_SUCCESS;

    if (NULL == sgpioDrvData) {
        ret = -EINVAL;
        goto out;
    }
    ((SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr)->txgpCfg.fields.gpTxSload = value;
out:
    return ret;
}

/**
 * @brief  sub function of sgpioLedSet
 * @details none
 * @param [in] dev no param check
 * @param [in] portNum
 */
static void sgpioLinkMatchDefaultSet(SgpioDrvData_s *sgpioDrvData, U8 portNum)
{
    SgpioReg_s *reg = (SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr;
    reg->txMatch[portNum].fields.stateX[0] = SGPIO_LINK_DOWN_P0;
    reg->txMatch[portNum].fields.stateX[1] = SGPIO_LINK_INIT_P1;
    reg->txMatch[portNum].fields.stateX[2] = SGPIO_LINK_ACT_IDLE_P2;
    reg->txMatch[portNum].fields.stateX[3] = SGPIO_LINK_ACT_X8_P3;
    reg->txMatch[portNum].fields.stateX[4] = SGPIO_LINK_ACT_X4_P4;
    reg->txMatch[portNum].fields.stateX[5] = SGPIO_LINK_ACT_OTHER_P5;
    reg->txMatch[portNum].fields.stateX[6] = SGPIO_PORT_ERR_P6;
    reg->txMatch[portNum].fields.stateX[7] = SGPIO_LOCATE_P7;
}

static S32 sgpioLedSet(DevList_e devId, U8 portNum, SgpioState_e state)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 clk = 0;
    U32 link = 0;
    bool cfgAll = false;
    bool softHardMode = false;
    U32 pattern;

    if (getDevDriver(devId, (void**)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto exit;
    }
    reg = (SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr;

    if (portNum > sgpioDrvData->sbrCfg.driveNum) {
        LOGE("%s: portNum %d is out of range\r\n", __func__, portNum);
        ret = -EINVAL;
        goto exit;
    }

    if (state >= SGPIO_STATE_MAX) {
        LOGE("%s: state %d is out of range\r\n", __func__, state);
        ret = -EINVAL;
        goto exit;
    }

    if (portNum == 0) {
        cfgAll = true;
    } else {
        cfgAll = false;
        link = (portNum - 1) / 2;
    }

    if (peripsClockFreqGet(devId, &clk) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }
    reg->cfg0.fields.sgpioEn = 0;
    reg->cfg1.fields.blinkDivA = clk / SGPIO_DIV_0P5HZ;
    reg->cfg1.fields.blinkDivB = clk / SGPIO_DIV_1HZ ;
    reg->cfg4.fields.blinkDivC = clk / SGPIO_DIV_2HZ ;
    reg->cfg2.fields.onMin = clk / SGPIO_DIV_0P5HZ;
    reg->cfg2.fields.offMin = clk / SGPIO_DIV_0P5HZ;
    reg->cfg3 = clk / SGPIO_DIV_100K;

    pattern = state2Pattern(state);
    if (state == SGPIO_HARD_MODE_S8) {
        softHardMode = true;
    }

    if (softHardMode == false) {
        if (!cfgAll) { /* 设置指定的port */
            if (((portNum - 1) % 2) == 0) {
                reg->txnl[link].fields.nlTxDrive0 = pattern;
            } else {
                reg->txnl[link].fields.nlTxDrive1 = pattern;
            }

            reg->drvCtrl[(portNum - 1)].fields.driveCtrl = 0x0;
        } else { /* 设置全部port */
            for (link = 0; link < sgpioDrvData->sbrCfg.driveNum; link++) {
                if ((link % 2) == 0) {
                    reg->txnl[link / 2].fields.nlTxDrive0 = pattern;
                } else {
                    reg->txnl[link / 2].fields.nlTxDrive1 = pattern;
                }
            }
            reg->drvCtrl[0].fields.driveCtrl = 0;
            reg->drvCtrl[1].fields.driveCtrl = 0;
            reg->drvCtrl[2].fields.driveCtrl = 0;
            reg->drvCtrl[3].fields.driveCtrl = 0;
            reg->qkCfg.fields.driveCtrl = 1;
        }
    } else { /* 软硬结合模式 */
        if (!cfgAll) { /* yaohj 设置指定port */
            sgpioLinkMatchDefaultSet(sgpioDrvData, portNum - 1);
            reg->cfg2.fields.onMin = 1;
            reg->cfg2.fields.offMin = 1;
            reg->drvCtrl[portNum - 1].fields.driveCtrl = 0x3f;

            if (((portNum - 1) % 2) == 0) {
                reg->txnl[link].fields.nlTxDrive0 = 0;
            } else {
                reg->txnl[link].fields.nlTxDrive1 = 0;
            }
        } else { /* 设置全部port */
            for (portNum = 0; portNum < sgpioDrvData->sbrCfg.driveNum; portNum++) {
                sgpioLinkMatchDefaultSet(sgpioDrvData, portNum);
            }
            for (link = 0; link < sgpioDrvData->sbrCfg.driveNum; link++) {
                if ((link % 2) == 0) {
                    reg->txnl[link / 2].fields.nlTxDrive0 = 0;
                } else {
                    reg->txnl[link / 2].fields.nlTxDrive1 = 0;
                }
            }
            reg->cfg2.fields.onMin = 1;
            reg->cfg2.fields.offMin = 1;
            reg->drvCtrl[0].fields.driveCtrl = 0x3f;
            reg->drvCtrl[1].fields.driveCtrl = 0x3f;
            reg->drvCtrl[2].fields.driveCtrl = 0x3f;
            reg->drvCtrl[3].fields.driveCtrl = 0x3f;
            reg->qkCfg.fields.driveCtrl = 1;
        }
    }

    reg->outCtrl.fields.clkDiv = clk / SGPIO_DIV_100K;
    reg->modeCtrl.fields.gpModeCtrlSel = 0;
    reg->cfg0.fields.sdriveCount = sgpioDrvData->sbrCfg.driveNum;
    reg->cfg0.fields.sgpioEn = 1;

exit:
    return ret;
}

static S32 sgpioGetDevCfg(DevList_e devId, SbrSgpioCfg_s *sbrCfg)
{
    /* 从SBR读取SGPIO配置 */
    if (devSbrRead(devId, sbrCfg, 0, sizeof(SbrSgpioCfg_s)) != sizeof(SbrSgpioCfg_s)) {
        LOGE("%s: failed to read SBR config for device %d\r\n",
             __func__, devId);
        return -EIO;
    }

    /* 验证配置数据的有效性 */
    if (sbrCfg->regAddr == NULL) {
        LOGE("%s: invalid register address in SBR config\r\n", __func__);
        return -EINVAL;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("sgpio: SBR dump - regAddr:%p, mode:%u, maxClk:%u, cfgClk:%u, driveNum:%u, reserved:0x%08x\r\n",
         sbrCfg->regAddr, sbrCfg->mode, sbrCfg->maxClk, sbrCfg->cfgClk, sbrCfg->driveNum, sbrCfg->reserved);
#endif

    if (sbrCfg->driveNum == 0 || sbrCfg->driveNum > SGPIO_MAX_DRIVE) {
        LOGE("%s: invalid drive number %d in SBR config\r\n", __func__, sbrCfg->driveNum);
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief sgpio init
 * @param [in] device id
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 sgpioInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_V2_SGPIO)) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == true) {
        /* if repeat init do nothing */
        ret = -EBUSY;
        goto exit;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    sgpioDrvData = (SgpioDrvData_s*)calloc(1, sizeof(SgpioDrvData_s));
    if (sgpioDrvData == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    if (sgpioGetDevCfg(devId, &sgpioDrvData->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freeMem;
    }

    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freeMem;
    }
    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freeMem;
    }

    /* 配置sgpio 可控制phy数量 */
    SgpioReg_s *reg = (SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr;
    reg->cfg0.fields.sgpioEn = 0;
    reg->modeCtrl.fields.gpModeCtrlSel = sgpioDrvData->sbrCfg.mode;
    reg->cfg0.fields.sdriveCount = sgpioDrvData->sbrCfg.driveNum;

    if (drvInstall(devId, sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freeMem;
    }

    ret = EXIT_SUCCESS;

freeMem:
    if (sgpioDrvData != NULL && ret != EXIT_SUCCESS) {
        free(sgpioDrvData);
        sgpioDrvData = NULL;
    }

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief sgpio deinit
 * @param [in] device id
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 sgpioDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;

    if (!isDrvMatch(devId, DRV_ID_STARS_V2_SGPIO)) {
        return -EINVAL;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (drvUninstall(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    peripsReset(devId);
    peripsClockDisable(devId);

    ret = EXIT_SUCCESS;
exit:
    /* unlock */
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief  Set sgpio led state
 * @param [in] id sgpio controller id
 *              should asigned to 0 when only 1 controller instance
 * @param [in] portNum see @note portNum[in]
 * @param [in] state
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 sgpioSetLedState(DevList_e devId, U8 portNum, SgpioState_e state)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_V2_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void**)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (portNum > sgpioDrvData->sbrCfg.driveNum) {
        LOGE("%s: portNum %d is out of range\r\n", __func__, portNum);
        ret = -EINVAL;
        goto exit;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (sgpioLedSet(devId, portNum, state) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto unlock;
    }
unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief Get sgpio drive count
 * @param [in] device id
 * @param [out] count, drive count
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 sgpioGetDriveCount(DevList_e devId, U32 *count)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;

    if (count == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V2_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void**)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    *count = ((SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr)->cfg0.fields.sdriveCount;

    ret = EXIT_SUCCESS;
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief Set sgpio drive count
 * @param [in] device id
 * @param [in] count, drive count
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 sgpioSetDriveCount(DevList_e devId, U32 count)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_V2_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void**)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (count > sgpioDrvData->sbrCfg.driveNum) {
        return -EINVAL;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    ((SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr)->cfg0.fields.sdriveCount = count;

    ret = EXIT_SUCCESS;
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief Get sgpio led state
 * @param [in] id sgpio controller id
 *              should asigned to 0 when only 1 controller instance
 * @param [in] portNum see @note portNum[in]
 * @param [out] pState current led state
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 sgpioGetLedState(DevList_e devId, U8 portNum, SgpioState_e *pState)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg;
    U32 link = 0;
    U32 state = 0;
    bool softHardMode = false;

    if (!isDrvMatch(devId, DRV_ID_STARS_V2_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void**)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (portNum > sgpioDrvData->sbrCfg.driveNum) {
        LOGE("%s: portNum %d is out of range\r\n", __func__, portNum);
        ret = -EINVAL;
        goto exit;
    }

    reg = (SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr;
    link = (portNum - 1) / 2;

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (((portNum - 1) % 2) == 0) {
        state = reg->txnl[link].fields.nlTxDrive0;
    } else {
        state = reg->txnl[link].fields.nlTxDrive1;
    }

    if (reg->drvCtrl[portNum - 1].fields.driveCtrl == 0x3f) {
        softHardMode = true;
    }

    /* 寄存器读取的SGPIO state转换为软件定义的枚举体 */
    state = pattern2State(state);

    if (state < SGPIO_STATE_MAX) {
        *pState = (SgpioState_e)state;
    } else {
        /* nothing to do */
    }

    if (softHardMode) {
        *pState = SGPIO_HARD_MODE_S8;
    }

    ret = EXIT_SUCCESS;
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief  Get sgpio led state(event input) from hardware
 *          Hardware event input form link
 * @details none
 * @param [in] id
 * @param [in] portNum
 * @param [out] pState
 * @return
 */
S32 sgpioGetLedLinkState(DevList_e devId, U8 portNum, U8 *out)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 index = 0;
    U32 offset = 0;

    if (out == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V2_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void**)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (portNum > sgpioDrvData->sbrCfg.driveNum) {
        LOGE("%s: portNum %d is out of range\r\n", __func__, portNum);
        ret = -EINVAL;
        goto exit;
    }

    reg = (SgpioReg_s*)sgpioDrvData->sbrCfg.regAddr;

    index = (portNum - 1) / 8;
    offset = (portNum - 1) % 8;
    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }
    *out = (U8)(reg->linkSta[index].dword >> (4 * offset)) & 0xf;

    ret = EXIT_SUCCESS;
    devUnlockByDriver(devId);
exit:
    return ret;
}
