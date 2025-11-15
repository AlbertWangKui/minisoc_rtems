/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sgpio_v1.0.c
 * @author liuzhsh1 (liuzhsh1@starsmicrosystem.com)
 * @date 2025/09/22
 * @brief  sgpio driver for tianhe
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/09/22   liuzhsh1        the first version
 *
 */

#include "common_defines.h"
#include "log_msg.h"
#include "udelay.h"
#include "drv_sgpio_v1.0.h"
#include "drv_sgpio_api.h"
#include "bsp_api.h"
#include "stringto_wrapper.h"
#include "bsp_config.h"
#include "bsp_api.h"
#include "osp_interrupt.h"

/**
 * @brief SGPIO ISR Handler function pointer type
 */
typedef void (*SgpioIsrHandler)(void *arg);

/**
 * @brief LED状态编码映射表
 */
static const U8 sgpioLedPatternMap[][8] = {
    // Error LED (3 bits)
    [SGPIO_LED_ERROR] = {
        [SGPIO_LED_ON] = 0,
        [SGPIO_LED_OFF] = 1,
        [SGPIO_LED_BLINK_A_HIGH_LOW] = 2,
        [SGPIO_LED_BLINK_A_LOW_HIGH] = 3,
        [SGPIO_LED_BLINK_B_HIGH_LOW] = 6,
        [SGPIO_LED_BLINK_B_LOW_HIGH] = 7
    },
    // Locate LED (2 bits)
    [SGPIO_LED_LOCATE] = {
        [SGPIO_LED_ON] = 0,
        [SGPIO_LED_OFF] = 1,
        [SGPIO_LED_BLINK_A_HIGH_LOW] = 2,
        [SGPIO_LED_BLINK_A_LOW_HIGH] = 3,
    },
    // Activity LED (3 bits)
    [SGPIO_LED_ACTIVITY] = {
        [SGPIO_LED_ON] = 0,
        [SGPIO_LED_OFF] = 1,
        [SGPIO_LED_BLINK_A_HIGH_LOW] = 2,
        [SGPIO_LED_BLINK_A_LOW_HIGH] = 3,
        [SGPIO_LED_EVENT_RISING] = 4,
        [SGPIO_LED_EVENT_FALLING] = 5,
        [SGPIO_LED_BLINK_B_HIGH_LOW] = 6,
        [SGPIO_LED_BLINK_B_LOW_HIGH] = 7
    }
};

// TODO： need replace when soc top pinmux api ready
void sgpioPinmuxCfg()
{
    U32 pinmuxAddr = 0xB8040000;
    U32 addr;
    U32 pinmuxVal;
    U32 offsets[] = { 0x208, 0x210, 0x218, 0x220, 0x228, 0x230, 0x238, 0x240 };
    int i;

    for (i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        addr = pinmuxAddr + offsets[i];
        pinmuxVal = reg32Read(addr);
        pinmuxVal &= ~(0xf);
        pinmuxVal |= 5;
        reg32Write(addr, pinmuxVal);
    }
}

static S32 sgpioDevCfgGet(DevList_e devId, SbrSgpioCfg_s *sbrCfg)
{
#if 0
    /* 从SBR读取SGPIO配置 */
    if (devSbrRead(devId, sbrCfg, 0, sizeof(SbrSgpioCfg_s)) != sizeof(SbrSgpioCfg_s)) {
        LOGE("%s: failed to read SBR config for device %d\r\n",
             __func__, devId);
        return -EIO;
    }
#else // TODO: 临时配置，后续删除
    sbrCfg->regAddr = (void *)(SGPIO_BASE_ADDR + (devId - DEVICE_SGPIO0) * SGPIO_DEVICE_OFFSET +
        SGPIO_REG_OFFSET);
    sbrCfg->driveNum = 64;
    sbrCfg->cfgClk = 100000; // 100KHz
    sbrCfg->ActivityMaskhigh = 0xffffffff;
    sbrCfg->ActivityMasklow = 0xffffffff;
    sbrCfg->blinkEn = 0; // 0: enable, 1: disable
    sbrCfg->blinkMode = 1; // 0: from external input trig, 1: sio self trigger
    sbrCfg->blinkFreq = 4; // blink频率, 单位Hz
    sbrCfg->blinkOutSel = 0; // 0: sgpio0, 1: sgpio1, 2: sgpio2, 3: sgpio3
    sbrCfg->cfg1Timing = 0x00427300;
    sbrCfg->mode = 0; // 0: normal, 1: GP
    sbrCfg->irqEn = 1; // enable irq
    sbrCfg->irqNum = SGPIO_IRQ_NUM; // irq number
    sbrCfg->irqPrio = SGPIO_IRQ_PRI; // irq priority
#endif
    /* 验证配置数据的有效性 */
    if (sbrCfg->regAddr == NULL) {
        LOGE("%s: invalid register address in SBR config\r\n", __func__);
        return -EINVAL;
    }

#ifdef CONFIG_DUMP_SBR
    LOGD("SGPIO Debug Config:\n"
         "  regAddr: %p\n"
         "  driveNum: %u\n"
         "  cfgClk: %u Hz\n"
         "  ActivityMaskhigh: 0x%08x\n"
         "  ActivityMasklow: 0x%08x\n"
         "  blinkEn: %u\n"
         "  blinkMode: %u\n"
         "  blinkFreq: %u Hz\n"
         "  cfg1Timing: 0x%08x\n"
         "  blinkOutSel: %u\n"
         "  mode: %u\n"
         "  irqEn: %u\n"
         "  irqNum: %u\n"
         "  irqPrio: %u",
        sbrCfg->regAddr, sbrCfg->driveNum, sbrCfg->cfgClk, sbrCfg->ActivityMaskhigh,
        sbrCfg->ActivityMasklow, sbrCfg->blinkEn, sbrCfg->blinkMode, sbrCfg->blinkFreq,
        sbrCfg->cfg1Timing, sbrCfg->blinkOutSel, sbrCfg->mode, sbrCfg->irqEn, sbrCfg->irqNum,
        sbrCfg->irqPrio);
#endif

    if (sbrCfg->driveNum < SGPIO_DRIVER_COUNT_MIN || sbrCfg->driveNum > SGPIO_DRIVER_COUNT_MAX) {
        LOGE("%s: invalid drive number %d in SBR config\r\n", __func__, sbrCfg->driveNum);
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}

static void sgpioIsr(void *arg)
{
    U32 intClr = 0;
    SgpioReg_s *sgpioRegs = (SgpioReg_s *)arg;

    if (sgpioRegs == NULL) {
        return;
    }

    LOGD("sgpio int status: 0x%08x\r\n", sgpioRegs->sioIntStatus);
    if (sgpioRegs->sioIntStatus & 0x1) {
        LOGD("sgpio sio start int\r\n");
        SET_BIT(intClr, 4);
    }
    if (sgpioRegs->sioIntStatus & 0x2) {
        LOGD("sgpio sio stop int\r\n");
        SET_BIT(intClr, 5);
    }
    if (sgpioRegs->sioIntStatus & 0x4) {
        LOGD("sgpio driver out of range int\r\n");
        SET_BIT(intClr, 6);
    }
    /* clear interrupt */
    sgpioRegs->sioIntCfg |= intClr;
}

static S32 sgpioRegisterIsr(DevList_e devId, SgpioIsrHandler handler, U16 irqNum, U16 irqPrio)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;

    if (handler == NULL) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);

    sgpioRegs->sioIntCfg = 0;

    ospInterruptHandlerInstall(irqNum + (devId - DEVICE_SGPIO0), "sgpio 1.0", OSP_INTERRUPT_UNIQUE,
        (void (*)(void *))handler, (void *)sgpioRegs);
    ospInterruptSetPriority(irqNum + (devId - DEVICE_SGPIO0), irqPrio);

    return EXIT_SUCCESS;
}

/**
* @brief 使能指定的SGPIO设备
*
* 该函数用于使能指定的SGPIO设备。
*
* @param devId 设备标识符，用于指定需要使能的SGPIO设备
* @return 如果函数执行成功，返回EXIT_SUCCESS；如果获取设备驱动失败，返回-EIO
*/
static S32 sgpioEnable(DevList_e devId)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;

    if (isDrvInit(devId) == false) {
        return -ENODEV;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);
    sgpioRegs->sgpioCfg0.fields.sgpioEn = 1;

    return EXIT_SUCCESS;
}

/**
* @brief 禁用指定的SGPIO设备
*
* 该函数用于禁用指定的SGPIO设备。
*
* @param devId 设备ID，类型为DevList_e
*
* @return 返回值类型为S32，表示操作结果。
*         - EXIT_SUCCESS（0）：操作成功
*         - -EIO：输入/输出错误
*/
static S32 sgpioDisable(DevList_e devId)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;

    if (isDrvInit(devId) == false) {
        return -ENODEV;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);
    sgpioRegs->sgpioCfg0.fields.sgpioEn = 0;

    return EXIT_SUCCESS;
}

/**
* @brief 使能SGPIO设备Sload功能,级联场景，只有first sio device的initiator配置为1，级联 device的initiator配置为0。
* @param devId 设备ID
* @return 如果函数调用成功，返回EXIT_SUCCESS；如果失败，返回-EIO
*/
static S32 sgpioSloadEnable(DevList_e devId)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);

    sgpioRegs->vendorSpec0.fields.firstSioDevice = 1;
    return EXIT_SUCCESS;
}

/**
* @brief 设置SGPIO设备的SLOAD值
*
* 设置指定SGPIO设备的SLOAD值。GP模式下的SLOAD {L3, L2, L1, L0}, NORMAL模式下的SLOAD必须为0。
*
* @param devId 设备ID，用于指定要设置的SGPIO设备
* @param sloadValue 要设置的SLOAD值
*
* @return 成功时返回EXIT_SUCCESS，失败时返回-EIO
*/
static S32 sgpioSetGpTxSload(DevList_e devId, U32 sloadValue)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);

    /** 
     * SGPIO_MODE_GP_WHEN_TX_SLOAD_IS_ODD ==> gp_tx_sload 为奇数的时候进入GP模式, 偶数进入NORMAL模式
     * SGPIO_MODE_GP_WHEN_TX_SLOAD_IS_NOT_ZERO ==> gp_tx_sload 不为0的时候进入GP模式, 为0进入NORMAL模式 
     */
    sgpioRegs->vendorSpec1.fields.gpModeCtrlSel = SGPIO_MODE_GP_WHEN_TX_SLOAD_IS_ODD;
    sgpioRegs->txgpCfg.fields.gpTxSload = sloadValue;
    return EXIT_SUCCESS;
}

/**
* @brief 设置SGPIO的活动定时参数
*
* 设置SGPIO的活动定时参数，包括拉伸活动关闭时间、拉伸活动开启时间、强制活动关闭时间和最大活动开启时间。
*
* @param sgpioRegs 指向SGPIO寄存器的指针
* @param stretchActOff 拉伸活动关闭时间（单位：1/64s），范围为0-15
* @param stretchActOn 拉伸活动开启时间（单位：1/64s），范围为0-15
* @param forceActOff 强制活动关闭时间（单位：1/8s），范围为0-15
* @param maxActOn 最大活动开启时间（单位： 1/4s），范围为0-15
*
* @return 返回操作结果，成功返回EXIT_SUCCESS，失败返回-EINVAL
*/
static S32 sgpioSetActivityTiming(
    SgpioReg_s *sgpioRegs, U8 stretchActOff, U8 stretchActOn, U8 forceActOff, U8 maxActOn)
{
    U32 currentValue;
    U32 newValue;
    const U32 MASK = 0xFFFF0000; // 掩码，覆盖 [31:16] 位

    if (sgpioRegs == NULL) {
        return -EINVAL;
    }

    if (stretchActOff > 15 || stretchActOn > 15 || forceActOff > 15 || maxActOn > 15) {
        LOGE("invalid timing param %d, %d, %d, %d\r\n", stretchActOff, stretchActOn, forceActOff,
            maxActOn);
        return -EINVAL;
    }

    // 准备要写入的新值
    newValue =
        (stretchActOff << 28) | (stretchActOn << 24) | (forceActOff << 20) | (maxActOn << 16);

    // 读-改-写
    currentValue = sgpioRegs->sgpioCfg1.dword; // 1. 读取当前值
    currentValue &= ~MASK; // 2. 清除目标位域
    currentValue |= newValue; // 3. 设置新值
    sgpioRegs->sgpioCfg1.dword = currentValue; // 4. 写回
    return EXIT_SUCCESS;
}

/**
* @brief 设置SGPIO的闪烁速率
*
* 设置SGPIO寄存器的闪烁速率，用于控制LED等设备的闪烁频率。
*
* @param sgpioRegs 指向SgpioReg_s结构体的指针，包含SGPIO的配置寄存器
* @param patternARate 模式A的闪烁速率，取值范围为0到15, 单位 1/8s
* @param patternBrate 模式B的闪烁速率，取值范围为0到15，单位 1/8s
*
* @return 如果成功，返回EXIT_SUCCESS；如果参数无效，返回-EINVAL
*/
static S32 sgpioSetBlinkRates(SgpioReg_s *sgpioRegs, U8 patternARate, U8 patternBrate)
{
    U32 currentValue;
    U32 newValue;
    const U32 MASK = 0x0000FF00; // 掩码，覆盖 [15:8] 位

    if (sgpioRegs == NULL) {
        return -EINVAL;
    }

    if (patternARate > 15 || patternBrate > 15) {
        LOGE("invalid blink rate param %d, %d\r\n", patternARate, patternBrate);
        return -EINVAL;
    }
    // 准备要写入的新值
    newValue = (patternARate << 8) | (patternBrate << 12);

    // 读-改-写
    currentValue = sgpioRegs->sgpioCfg1.dword; // 1. 读取当前值
    currentValue &= ~MASK; // 2. 清除目标位域
    currentValue |= newValue; // 3. 设置新值
    sgpioRegs->sgpioCfg1.dword = currentValue; // 4. 写回

    return EXIT_SUCCESS;
}

/**
* @brief 设置SGPIO配置1
*
* 根据设备ID和配置参数设置SGPIO的配置1。
*
* @param devId 设备ID，枚举类型DevList_e
* @param cfg 配置参数，无符号32位整数
*
* @return 返回值类型S32，成功返回EXIT_SUCCESS，失败返回-EIO
*
* @details
* 4. 调用sgpioSetActivityTiming函数设置活动定时参数。
* 5. 调用sgpioSetBlinkRates函数设置闪烁速率参数。
* 6. 最后返回EXIT_SUCCESS表示成功。
*/
static S32 sgpioSetCfg1(DevList_e devId, U32 cfg)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    SgpioCfg1_u sgpioCfg1;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);

    sgpioCfg1.dword = cfg;

    sgpioSetActivityTiming(sgpioRegs, sgpioCfg1.fields.strechActOff, sgpioCfg1.fields.strechActOn,
        sgpioCfg1.fields.forceActOff, sgpioCfg1.fields.maxActOn);
    sgpioSetBlinkRates(sgpioRegs, sgpioCfg1.fields.blinkGenRateA, sgpioCfg1.fields.blinkGenRateB);
    //sgpioRegs->sgpioCfg1.dword = 0x427300;
    return EXIT_SUCCESS;
}

/**
* @brief 驱动SGPIO引脚活动掩码
*
* 驱动SGPIO引脚活动掩码，用于设置SGPIO驱动器的输出引脚的活动掩码。
*
* @param devId 设备ID，用于指定操作的设备
* @param driverHigh32Mask 高32位驱动掩码，用于设置高32位驱动器的活动掩码(0 表示不驱动, 1 表示驱动)
* @param driverLow32Mask 低32位驱动掩码，用于设置低32位驱动器的活动掩码(0 表示不驱动, 1 表示驱动)
*
* @return 成功返回EXIT_SUCCESS，失败返回-EIO
*/
static S32 sgpioDriveActivityMask(DevList_e devId, U32 driverHigh32Mask, U32 driverLow32Mask)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);

    sgpioRegs->driverStrobe0 = driverLow32Mask;
    sgpioRegs->driverStrobe1 = driverHigh32Mask;
    return EXIT_SUCCESS;
}

/**
 * @brief 配置sgpio 工作频率
 * @param [in] sgpioPortNum sgpio port
 * @param [in] hz 配置的频率, 32Hz ~ 100kHz
 * @param [out] none
 * @return EXIT_SUCCESS, EXIT_FAILURE
 */
static S32 sgpioSetSClock(DevList_e devId, U32 hz)
{
    U32 clkDiv;
    U32 clkPeriod;
    U32 us100Cnt;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_FAILURE;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);

    if (hz > SGPIO_MAX_CLK || hz < SGPIO_MIN_CLK) {
        ret = EXIT_FAILURE;
    } else {
        /**
         * 分频计算公式hz = source clk/(((clkdivide * 8) + 1) *2)
         */
        clkDiv = ((SGPIO_SOURCE_CLK / (2 * hz)) - 1) / 8;
        sgpioRegs->vendorSpec0.fields.clkDivide = clkDiv;

        /**
         * 10e9 / hz, clock blink period is base on clock output
         */
        clkPeriod = 1000000000 / hz; /* clock period unit: ns */
        sgpioRegs->sioClkPeriod = clkPeriod;

        /**
         * 必须为10KHz， freq(10KHz) = source clk / (us100Cnt)
        */
        us100Cnt = (SGPIO_SOURCE_CLK / 10000);
        sgpioRegs->us100Cnt = us100Cnt;

        ret = EXIT_SUCCESS;
    }
    return ret;
}

/**
* @brief 根据期望频率和时钟频率计算N值
*
* 根据期望频率和时钟频率计算N值，公式为 Freq = 1/(N/4 * 1/CLK) = 4*CLK/N
*
* @param desiredFreq 期望频率(Hz)
* @param clkFreq 时钟频率(Hz)
* @return 计算出的N值
*/
static U32 sgpioCalculateNValue(U32 desiredFreq, U32 clkFreq)
{
    if (desiredFreq == 0) {
        LOGE("error: desiredFreq can't set to 0\n");
        return 0;
    }

    /**
     * 根据期望频率和时钟频率计算N值
     * 公式: Freq = 1/(N/4 * 1/CLK) = 4*CLK/N
     * 所以: N = 4*CLK/Freq
     */
    U64 nValue = (U64)4 * clkFreq / desiredFreq;

    if (nValue > 0xFFFFFFFF) {
        LOGE("Error: N value overflow\n");
        return 0;
    }
    LOGD("N value: CLK=%uHz, desiredFreq=%uHz, N=%llu\n", clkFreq, desiredFreq, nValue);

    return (U32)nValue;
}

/**
* @brief 配置SGPIO的闪烁功能
*
* 配置SGPIO的闪烁功能，包括闪烁使能、闪烁模式、输出选择和频率。
*
* @param devId 设备ID
* @param blinkEn 闪烁使能，0 == enable, 1 == disable
* @param blinkMode 闪烁模式,0 == from external input trig, 1 == sio self trigger
* @param blinkOutSel 闪烁输出选择,0 == sgpio0, 1 == sgpio1, 2 == sgpio2, 3 == sgpio3
* @param freq 闪烁频率，单位Hz，范围为1Hz到100kHz
*
* @return 成功返回EXIT_SUCCESS，失败返回错误码
*         -EIO: 设备I/O错误
*         -EINVAL: 无效参数，如频率超出范围或计算N值失败
*/
static S32 sgpioBlinkConfig(DevList_e devId, U8 blinkEn, U8 blinkMode, U8 blinkOutSel, U32 freq)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    U32 nValue;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }
    if (freq < 1 || freq > 100000) {
        LOGE("err: freq %uHz exceed (1Hz - 100kHz)\n", freq);
        return -EINVAL;
    }

    // 计算N值
    nValue = sgpioCalculateNValue(freq, SGPIO_SOURCE_CLK);
    if (nValue == 0) {
        return -EINVAL;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);
    if (sgpioRegs == NULL) {
        return -EIO;
    }

    // 配置blink频率
    sgpioRegs->sioBlinkReg = nValue; // 设置N值, N = 4 * PCLK / Freq
    // 设置闪烁选择寄存器
    sgpioRegs->sioBlinkSel.dword =
        (blinkMode & 0x01) << 4 | (blinkEn & 0x01) << 3 | (blinkOutSel & 0x3);

    LOGD("SGPIO Blink config: freq=%uHz, N value=%u\r\n", freq, nValue);
    return EXIT_SUCCESS;
}

/**
* @brief 设置SGPIO协议模式
*
* 根据传入的模式参数，设置SGPIO协议模式。
*
* @param mode SGPIO协议模式，可选值包括：
*   - SGPIO_STA1005：STA1005协议模式
*   - SGPIO_SFF8485：SFF8485协议模式
*
* @return 成功时返回EXIT_SUCCESS，失败时返回-EINVAL
*/
static S32 sgpioSetProtocol(SgpioProtocol_e mode)
{
    SgpioTopCfg_u *ptrSgpioTopRegs = (SgpioTopCfg_u *)(SGPIO_TOP_CRG_REG_BASE + 0xc680);

    switch (mode) {
    case SGPIO_STA1005:
        ptrSgpioTopRegs->dword &= ~(0x0f);
        break;
    case SGPIO_SFF8485:
        ptrSgpioTopRegs->dword |= 0x0f;
        break;
    default:
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}

static S32 sgpioTurnOffAllLeds(DevList_e devId, SgpioDrvData_s *sgpioDrvData)
{
    for (U8 phyNum = 0; phyNum < sgpioDrvData->sbrCfg.driveNum; phyNum++) {
        if (sgpioSetLedState(devId, phyNum, SGPIO_LED_ERROR, SGPIO_LED_OFF) != EXIT_SUCCESS ||
            sgpioSetLedState(devId, phyNum, SGPIO_LED_ACTIVITY, SGPIO_LED_OFF) != EXIT_SUCCESS ||
            sgpioSetLedState(devId, phyNum, SGPIO_LED_LOCATE, SGPIO_LED_OFF) != EXIT_SUCCESS) {
            LOGE("%s: sgpio set led state failed for phy %d\r\n", __func__, phyNum);
            return -EIO; // 或者具体的错误码
        }
    }
    return EXIT_SUCCESS;
}

/**
* @brief 设置PCIe活动映射到SGPIO控制器
*
* 此函数用于将PCIe活动映射到指定的SGPIO控制器上的某个driver。
*
* @param devId SGPIO控制器ID，枚举类型DevList_e
* @param phyNum PCIe PHY编号
* @param sgpioActivitySel 要映射的SGPIO driver编号
*
* @return 函数执行结果，成功返回EXIT_SUCCESS，失败返回错误码
*         - EINVAL：无效的参数
*         - EIO：获取SGPIO驱动数量失败
*/
S32 sgpioSetPcieActivityMapping(DevList_e devId, U32 phyNum, U8 sgpioActivitySel)
{
    S32 ret = EXIT_SUCCESS;
    U32 deviceCount;
    U32 regAddr;
    U32 regVal;
    U32 sgpioCtrlId;
    U32 regIndex;
    U32 byteOffset;

    if (isDrvInit(devId) == false) {
        return -ENODEV;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        return ret;
    }

    // 1. 验证参数
    sgpioCtrlId = devId - DEVICE_SGPIO0;
    if (sgpioCtrlId >= SGPIO_MAX_NUM) {
        LOGE("Error: Invalid SGPIO controller ID %d\n", sgpioCtrlId);
        ret = -EINVAL;
        goto exit;
    }

    ret = sgpioGetDriveCount(devId, &deviceCount);
    if (ret != EXIT_SUCCESS) {
        LOGE("Error: Failed to get drive count for SGPI0 controller %d\n", sgpioCtrlId);
        ret = -EIO;
        goto exit;
    }

    if (sgpioActivitySel > deviceCount) {
        LOGE("Error: SGPIO driver selection %u is out of range (0-63)\n", sgpioActivitySel);
        ret = -EINVAL;
        goto exit;
    }

    // 2. 计算寄存器地址
    // 每个寄存器4字节，控制4个driver
    regIndex = sgpioActivitySel / 4;
    // 每个driver在寄存器内占8-bit (1 byte)
    byteOffset = sgpioActivitySel % 4;

    // 计算最终的寄存器地址
    regAddr =
        SGPIO_TOP_CRG_MAP_REG_ADDR + (sgpioCtrlId * SGPIO_TOP_CRG_MAP_BLOCK_SIZE) + (regIndex * 4);

    // 3. 读-改-写
    regVal = reg32Read(regAddr);
    // 清除对应的8-bit
    regVal &= ~(0xFF << (byteOffset * 8));
    // 写入新的phyNum
    regVal |= (phyNum & 0xFF) << (byteOffset * 8);
    reg32Write(regAddr, regVal);

    LOGD("Set SGPIO%u, Driver %u -> PHY %u. Reg[0x%08x]=0x%08x\n", sgpioCtrlId, sgpioActivitySel,
        phyNum, regAddr, regVal);

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
* @brief 导出SGPIO控制器的PCIe活动映射表
*
* 该函数将导出SGPIO控制器的PCIe活动映射表，显示每个驱动器（Driver）对应的PHY编号。
*
* @param devId 设备ID，表示要操作的SGPIO控制器。
*
* @return 返回值类型为S32，成功时返回EXIT_SUCCESS，失败时返回相应的错误码。
*/
S32 sgpioDumpPcieActivityMapping(DevList_e devId)
{
    U32 sgpioCtrlId;
    U32 regAddr;
    U32 regVal;
    U32 deviceCount;
    U32 regIndex;
    U32 byteOffset;
    U32 phyNum;
    S32 ret;
    U32 i;

    if (isDrvInit(devId) == false) {
        return -ENODEV;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        return ret;
    }

    // 1. 验证参数
    sgpioCtrlId = devId - DEVICE_SGPIO0;
    if (sgpioCtrlId >= SGPIO_MAX_NUM) {
        LOGE("Error: Invalid SGPIO controller ID %d\n", sgpioCtrlId);
        ret = -EINVAL;
        goto exit;
    }

    ret = sgpioGetDriveCount(devId, &deviceCount);
    if (ret != EXIT_SUCCESS) {
        LOGE("Error: Failed to get drive count for SGPIO controller %d\n", sgpioCtrlId);
        ret = -EIO;
        goto exit;
    }

    // 限制deviceCount不超过最大值
    if (deviceCount > SGPIO_DRIVER_COUNT_MAX) {
        deviceCount = SGPIO_DRIVER_COUNT_MAX;
    }

    // 打印表头
    LOGI("SGPIO%u PCIe Activity Mapping Table (Driver -> PHY):\n", sgpioCtrlId);
    LOGI("Driver\tPHY\n");
    LOGI("------\t---\n");

    // 2. 遍历所有可能的driver (最多64个)
    for (i = 0; i < deviceCount; i++) {
        // 计算寄存器地址
        regIndex = i / 4;
        byteOffset = i % 4;

        regAddr = SGPIO_TOP_CRG_MAP_REG_ADDR + (sgpioCtrlId * SGPIO_TOP_CRG_MAP_BLOCK_SIZE) +
            (regIndex * 4);

        // 读取寄存器值
        regVal = reg32Read(regAddr);

        // 提取对应的PHY编号
        phyNum = (regVal >> (byteOffset * 8)) & 0xFF;

        // 打印映射关系
        LOGI("%u\t%u\n", i, phyNum);
    }

exit:
    devUnlockByDriver(devId);
    return ret;
}

static S32 sgpioHwInit(DevList_e devId, SgpioDrvData_s *sgpioDrvData)
{
    S32 ret = EXIT_SUCCESS;

    if (sgpioDrvData == NULL) {
        LOGE("%s: invalid drv data\r\n", __func__);
        return -EINVAL;
    }

    if (sgpioDrvData->sbrCfg.mode == 2) {
        sgpioSetProtocol(SGPIO_STA1005);
    } else if (sgpioDrvData->sbrCfg.mode == 0 || sgpioDrvData->sbrCfg.mode == 1) {
        ret = sgpioSetProtocol(SGPIO_SFF8485);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: set protocol failed %d\r\n", __func__, ret);
            return ret;
        }

        ret = sgpioSloadEnable(devId);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio sload enable failed %d\r\n", __func__, ret);
            return ret;
        }

        ret = sgpioSetSClock(devId, sgpioDrvData->sbrCfg.cfgClk);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio set sclock failed %d\r\n", __func__, ret);
            return ret;
        }

        /* register isr */
        if (sgpioDrvData->sbrCfg.irqEn == 1) {
            ret = sgpioRegisterIsr(
                devId, sgpioIsr, sgpioDrvData->sbrCfg.irqNum, sgpioDrvData->sbrCfg.irqPrio);
            if (ret != EXIT_SUCCESS) {
                LOGE("%s: sgpio register isr failed %d\r\n", __func__, ret);
                return ret;
            }
        }

        /* 模式选择 */
        if (sgpioDrvData->sbrCfg.mode == 0) {
            /* normal mode */
            ret = sgpioSetGpTxSload(devId, 0x0);
            if (ret != EXIT_SUCCESS) {
                LOGE("%s: sgpio set gp tx sload failed %d\r\n", __func__, ret);
                return ret;
            }
        } else {
            /* GP mode */
            ret = sgpioSetGpTxSload(devId, 0x1);
        }

        ret = sgpioSetCfg1(devId, sgpioDrvData->sbrCfg.cfg1Timing);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio set cfg1 failed %d\r\n", __func__, ret);
            return ret;
        }

        // TODO maybe need sbr
        ret = sgpioDriveActivityMask(devId, 0xffffffff, 0xffffffff);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio drive activity mask failed %d\r\n", __func__, ret);
            return ret;
        }

        ret = sgpioSetDriveCount(devId, sgpioDrvData->sbrCfg.driveNum);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio set drive count failed %d\r\n", __func__, ret);
            return ret;
        }

        ret = sgpioBlinkConfig(devId, sgpioDrvData->sbrCfg.blinkEn, sgpioDrvData->sbrCfg.blinkMode,
            sgpioDrvData->sbrCfg.blinkOutSel, sgpioDrvData->sbrCfg.blinkFreq);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio blink config failed %d\r\n", __func__, ret);
            return ret;
        }
    } else {
        LOGE("%s: invalid sgpio mode %d\r\n", __func__, sgpioDrvData->sbrCfg.mode);
        return -EINVAL;
    }
    return ret;
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
    bool isLocked = false;

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (isDrvInit(devId) == true) {
        return -EBUSY;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }
    isLocked = true;

    sgpioDrvData = (SgpioDrvData_s *)calloc(1, sizeof(SgpioDrvData_s));
    if (sgpioDrvData == NULL) {
        ret = -ENOMEM;
        goto cleanup;
    }

    if (sgpioDevCfgGet(devId, &sgpioDrvData->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto cleanup;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto cleanup;
    }

    if (drvInstall(devId, sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto cleanup;
    }

    // TODO：需要soc提供pinmux配置函数
    sgpioPinmuxCfg();

    ret = sgpioHwInit(devId, sgpioDrvData);
    if (ret != EXIT_SUCCESS) {
        goto cleanup;
    }

    ret = sgpioTurnOffAllLeds(devId, sgpioDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sgpio turn off all leds failed %d\r\n", __func__, ret);
        goto cleanup;
    }

    ret = sgpioEnable(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sgpio enable failed %d\r\n", __func__, ret);
        goto cleanup;
    }

cleanup:
    if (ret != EXIT_SUCCESS) {
        if (isDrvInit(devId)) {
            // 如果驱动已注册，则调用drvUninstall来清理（包括释放sgpioDrvData）
            drvUninstall(devId);
        } else if (sgpioDrvData != NULL) {
            // 如果驱动未注册但内存已分配，则手动释放
            free(sgpioDrvData);
        }
    }

    if (isLocked) {
        devUnlockByDriver(devId);
    }
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

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    ret = sgpioDisable(devId);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
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
 * @brief Get sgpio drive count
 * @param [in] device id
 * @param [out] count, drive count
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 sgpioGetDriveCount(DevList_e devId, U32 *count)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;

    if (isDrvInit(devId) == false) {
        return -ENODEV;
    }

    if (count == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    *count = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr)->sgpioCfg0.fields.sdriveCount;

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
    SgpioReg_s *sgpioRegs = NULL;

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EIO;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (count > sgpioDrvData->sbrCfg.driveNum) {
        ret = -EINVAL;
        goto exit;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }
    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);

    sgpioRegs->sgpioCfg0.fields.sdriveCount = count;
    sgpioRegs->vendorSpec1.fields.sgpioDeviceCnt = count;

    ret = EXIT_SUCCESS;
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief  Set sgpio driver led state directly
 * @param [in] devId sgpio controller id
 * @param [in] phyNum PHY编号
 * @param [in] ledType LED类型 (Error/Locate/Activity)
 * @param [in] ledState LED状态
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 *          -EINVAL : -22
 */
S32 sgpioSetLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, SgpioLedState_e ledState)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 txnIndex, txnOffset;
    U32 currentValue, newValue;
    U32 mask, pattern;

    if (isDrvInit(devId) == false) {
        return -ENODEV;
    }

    // 参数检查
    if (ledType >= SGPIO_LED_MAX || ledState >= SGPIO_LED_STATE_MAX) {
        return -EINVAL;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    if (phyNum >= sgpioDrvData->sbrCfg.driveNum) {
        LOGE("%s: phyNum %d is out of range\r\n", __func__, phyNum);
        return -EINVAL;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    reg = (SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr;

    // 计算driver索引和txn寄存器索引及偏移
    // 每个txn控制4个驱动器
    txnIndex = phyNum / 4;
    txnOffset = phyNum % 4;

    // 获取当前值
    currentValue = reg->txn[txnIndex].dword;

    // 根据LED类型确定掩码和位移
    switch (ledType) {
    case SGPIO_LED_ERROR: // [2:0]
        mask = 0x7 << (txnOffset * 8);
        pattern = sgpioLedPatternMap[SGPIO_LED_ERROR][ledState] << (txnOffset * 8);
        break;
    case SGPIO_LED_LOCATE: // [4:3]
        mask = 0x3 << (3 + txnOffset * 8);
        pattern = sgpioLedPatternMap[SGPIO_LED_LOCATE][ledState] << (3 + txnOffset * 8);
        break;
    case SGPIO_LED_ACTIVITY: // [7:5]
        mask = 0x7 << (5 + txnOffset * 8);
        pattern = sgpioLedPatternMap[SGPIO_LED_ACTIVITY][ledState] << (5 + txnOffset * 8);
        break;
    default:
        ret = -EINVAL;
        goto unlock;
    }

    // 清除原值并设置新值
    newValue = (currentValue & ~mask) | pattern;
    reg->txn[txnIndex].dword = newValue;

unlock:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief  Get sgpio driver led state directly
 * @param [in] devId sgpio controller id
 * @param [in] phyNum PHY编号
 * @param [in] ledType LED类型 (Error/Locate/Activity)
 * @param [out] pLedState LED状态指针
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 *          -EINVAL : -22
 */
S32 sgpioGetLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, SgpioLedState_e *pLedState)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 txnIndex, txnOffset;
    U32 currentValue;
    U32 mask, value;
    U8 pattern;
    int i;

    if (isDrvInit(devId) == false) {
        return -ENODEV;
    }

    // 参数检查
    if (ledType >= SGPIO_LED_MAX || pLedState == NULL) {
        return -EINVAL;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    if (phyNum >= sgpioDrvData->sbrCfg.driveNum) {
        LOGE("%s: phyNum %d is out of range\r\n", __func__, phyNum);
        return -EINVAL;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    reg = (SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr;

    // 计算driver索引和txn寄存器索引及偏移
    // 每个txn控制4个驱动器
    txnIndex = phyNum / 4;
    txnOffset = phyNum % 4;

    // 获取当前值
    currentValue = reg->txn[txnIndex].dword;

    // 根据LED类型确定掩码和位移
    switch (ledType) {
    case SGPIO_LED_ERROR: // [2:0]
        mask = 0x7 << (txnOffset * 8);
        value = (currentValue & mask) >> (txnOffset * 8);
        break;
    case SGPIO_LED_LOCATE: // [4:3]
        mask = 0x3 << (3 + txnOffset * 8);
        value = (currentValue & mask) >> (3 + txnOffset * 8);
        break;
    case SGPIO_LED_ACTIVITY: // [7:5]
        mask = 0x7 << (5 + txnOffset * 8);
        value = (currentValue & mask) >> (5 + txnOffset * 8);
        break;
    default:
        ret = -EINVAL;
        goto unlock;
    }

    // 查找匹配的LED状态
    pattern = (U8)value;
    *pLedState = SGPIO_LED_OFF; // 默认状态

    for (i = 0; i < SGPIO_LED_STATE_MAX; i++) {
        if (sgpioLedPatternMap[ledType][i] == pattern) {
            *pLedState = (SgpioLedState_e)i;
            break;
        }
    }

unlock:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief  Set sgpio general purpose driver led state directly
 * @details This function operates on txgp registers for General Purpose mode.
 *          Each PHY is controlled by 3 bits for error, locate, and activity.
 *          It handles cases where a PHY's 3 bits span across two registers.
 * @param [in] devId sgpio controller id
 * @param [in] phyNum PHY编号
 * @param [in] ledType LED类型 (SGPIO_LED_ERROR/SGPIO_LED_LOCATE/SGPIO_LED_ACTIVITY)
 * @param [in] ledOn  true: turn on, false: turn off
 * @return  EXIT_SUCCESS : 0
 *          -EBUSY : -16
 *          -EINVAL : -22
 */
S32 sgpioSetGpLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, bool ledOn)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 totalBitOffset, startRegIndex, startBitOffset;
    U32 bitInLedType;

    if (isDrvInit(devId) == false) {
        return -ENODEV;
    }

    // 参数检查
    if (phyNum >= SGPIO_DRIVER_COUNT_MAX || ledType >= SGPIO_LED_MAX) {
        return -EINVAL;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    reg = (SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr;

    // 3-bit 布局: [bit0:activity, bit1:locate, bit2:error]
    switch (ledType) {
    case SGPIO_LED_ERROR:
        bitInLedType = 2;
        break;
    case SGPIO_LED_LOCATE:
        bitInLedType = 1;
        break;
    case SGPIO_LED_ACTIVITY:
        bitInLedType = 0;
        break;
    default: // Should not happen
        ret = -EINVAL;
        goto unlock;
    }

    // 计算目标位在192-bit虚拟空间中的总偏移
    totalBitOffset = phyNum * 3 + bitInLedType;

    // 计算目标位所在的寄存器索引和寄存器内偏移
    startRegIndex = totalBitOffset / 32;
    startBitOffset = totalBitOffset % 32;

    // 检查寄存器索引是否越界
    if (startRegIndex >= 6) {
        ret = -EINVAL;
        goto unlock;
    }

    // 读取、修改、写回寄存器
    U32 value = reg->txgp[startRegIndex].dword;
    if (ledOn) {
        value |= (1UL << startBitOffset);
    } else {
        value &= ~(1UL << startBitOffset);
    }
    reg->txgp[startRegIndex].dword = value;

unlock:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief 获取GP模式下指定PHY的LED状态
 * @param [in] devId sgpio控制器ID
 * @param [in] phyNum PHY编号
 * @param [in] ledType LED类型 (SGPIO_LED_ERROR/SGPIO_LED_LOCATE/SGPIO_LED_ACTIVITY)
 * @param [out] pLedOn 指向bool变量，返回true表示点亮，false表示熄灭
 * @return EXIT_SUCCESS: 0
 *         -EBUSY: -16
 *         -EINVAL: -22
 */
S32 sgpioGetGpLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, bool *pLedOn)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 totalBitOffset, regIndex, bitOffset;
    U32 bitInLedType;

    if (isDrvInit(devId) == false) {
        return -ENODEV;
    }

    // 参数检查
    if (pLedOn == NULL || phyNum >= SGPIO_DRIVER_COUNT_MAX || ledType >= SGPIO_LED_MAX) {
        return -EINVAL;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    reg = (SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr;

    // 3-bit布局: [bit0:activity, bit1:locate, bit2:error]
    switch (ledType) {
    case SGPIO_LED_ERROR:
        bitInLedType = 2;
        break;
    case SGPIO_LED_LOCATE:
        bitInLedType = 1;
        break;
    case SGPIO_LED_ACTIVITY:
        bitInLedType = 0;
        break;
    default:
        ret = -EINVAL;
        goto unlock;
    }

    totalBitOffset = phyNum * 3 + bitInLedType;
    regIndex = totalBitOffset / 32;
    bitOffset = totalBitOffset % 32;

    if (regIndex >= 6) {
        ret = -EINVAL;
        goto unlock;
    }

    U32 value = reg->txgp[regIndex].dword;
    *pLedOn = (value & (1UL << bitOffset)) ? true : false;

unlock:
    devUnlockByDriver(devId);
    return ret;
}
